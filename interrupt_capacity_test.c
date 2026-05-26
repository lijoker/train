#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "fsm_interrupt_demo.h"

typedef enum {
    ARRIVAL_PERIODIC = 0,
    ARRIVAL_BURST,
    ARRIVAL_POISSON
} ArrivalMode;

typedef struct {
    const char *name;
    ArrivalMode mode;
    double target_irq_per_sec;
    double duration_sec;
    int work_steps;
    int burst_size;
    uint32_t random_seed;
} BenchmarkConfig;

typedef struct {
    uint64_t generated;
    uint64_t handled;
    uint64_t dropped;
    uint64_t max_pending;
    double wall_time_sec;
    double handled_per_sec;
    double avg_isr_us;
    double max_isr_us;
    bool pass_3000;
} BenchmarkResult;

typedef struct {
    ArrivalMode mode;
    uint64_t base_interval_ns;
    int burst_size;
    int burst_index;
    uint32_t rng_state;
} ArrivalGenerator;

static uint64_t monotonic_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static double ns_to_us(uint64_t ns) {
    return (double)ns / 1000.0;
}

static double ns_to_s(uint64_t ns) {
    return (double)ns / 1000000000.0;
}

static uint32_t lcg_next(uint32_t *state) {
    *state = (*state * 1664525U) + 1013904223U;
    return *state;
}

static void arrival_init(ArrivalGenerator *gen, const BenchmarkConfig *cfg) {
    double base_interval = 1000000000.0 / cfg->target_irq_per_sec;
    gen->mode = cfg->mode;
    gen->base_interval_ns = (uint64_t)(base_interval > 1.0 ? base_interval : 1.0);
    gen->burst_size = cfg->burst_size > 1 ? cfg->burst_size : 1;
    gen->burst_index = 0;
    gen->rng_state = cfg->random_seed == 0U ? 1U : cfg->random_seed;
}

static uint64_t arrival_next_delta_ns(ArrivalGenerator *gen) {
    if (gen->mode == ARRIVAL_PERIODIC) {
        return gen->base_interval_ns;
    }

    if (gen->mode == ARRIVAL_BURST) {
        gen->burst_index++;
        if (gen->burst_index < gen->burst_size) {
            return 0ULL;
        }
        gen->burst_index = 0;
        return gen->base_interval_ns * (uint64_t)gen->burst_size;
    }

    {
        double lambda = 1.0 / (double)gen->base_interval_ns;
        double uniform = ((double)(lcg_next(&gen->rng_state) + 1U)) / 4294967297.0;
        double delta = -log(uniform) / lambda;
        if (delta < 1.0) {
            delta = 1.0;
        }
        return (uint64_t)delta;
    }
}

static bool run_capacity_benchmark(const BenchmarkConfig *cfg, BenchmarkResult *out) {
    ArrivalGenerator gen;
    InterruptFSM fsm = {STATE_IDLE, false};
    InterruptServiceStats stats = {0};
    uint64_t pending = 0;
    uint64_t total_isr_ns = 0;
    uint64_t max_isr_ns = 0;
    uint64_t queue_limit = 1000000ULL;
    uint64_t start_ns;
    uint64_t end_ns;
    uint64_t next_arrival_ns;

    if (cfg == NULL || out == NULL || cfg->target_irq_per_sec <= 0.0 ||
        cfg->duration_sec <= 0.0 || cfg->work_steps <= 0) {
        return false;
    }

    arrival_init(&gen, cfg);
    start_ns = monotonic_now_ns();
    end_ns = start_ns + (uint64_t)(cfg->duration_sec * 1000000000.0);
    next_arrival_ns = start_ns;

    out->generated = 0;
    out->handled = 0;
    out->dropped = 0;
    out->max_pending = 0;
    out->avg_isr_us = 0.0;
    out->max_isr_us = 0.0;
    out->pass_3000 = false;

    while (true) {
        uint64_t now_ns = monotonic_now_ns();

        while (next_arrival_ns <= now_ns && next_arrival_ns < end_ns) {
            out->generated++;
            if (pending < queue_limit) {
                pending++;
                if (pending > out->max_pending) {
                    out->max_pending = pending;
                }
            } else {
                out->dropped++;
            }
            next_arrival_ns += arrival_next_delta_ns(&gen);
        }

        if (pending > 0) {
            uint64_t irq_start;
            uint64_t irq_end;

            fsm.irq_level = true;
            irq_start = monotonic_now_ns();
            (void)simulate_interrupt_and_handle(&fsm, &stats, cfg->name, cfg->work_steps);
            irq_end = monotonic_now_ns();

            pending--;
            out->handled++;
            total_isr_ns += (irq_end - irq_start);
            if ((irq_end - irq_start) > max_isr_ns) {
                max_isr_ns = irq_end - irq_start;
            }
            continue;
        }

        if (next_arrival_ns >= end_ns) {
            break;
        }
    }

    out->wall_time_sec = ns_to_s(monotonic_now_ns() - start_ns);
    if (out->handled > 0) {
        out->avg_isr_us = ns_to_us(total_isr_ns) / (double)out->handled;
    }
    out->max_isr_us = ns_to_us(max_isr_ns);
    out->handled_per_sec = out->handled / out->wall_time_sec;
    out->pass_3000 = (out->handled_per_sec >= 3000.0) && (out->dropped == 0);
    return true;
}

static const char *mode_name(ArrivalMode mode) {
    switch (mode) {
    case ARRIVAL_PERIODIC:
        return "periodic";
    case ARRIVAL_BURST:
        return "burst";
    case ARRIVAL_POISSON:
        return "poisson";
    default:
        return "unknown";
    }
}

static void print_result(const BenchmarkConfig *cfg, const BenchmarkResult *res) {
    printf("\n[MODE=%s] target=%.0f irq/s, duration=%.2fs, work_steps=%d\n",
           mode_name(cfg->mode), cfg->target_irq_per_sec, cfg->duration_sec,
           cfg->work_steps);
    printf("generated=%llu handled=%llu dropped=%llu max_pending=%llu\n",
           (unsigned long long)res->generated, (unsigned long long)res->handled,
           (unsigned long long)res->dropped, (unsigned long long)res->max_pending);
    printf("handled_rate=%.2f irq/s avg_isr=%.3f us max_isr=%.3f us\n",
           res->handled_per_sec, res->avg_isr_us, res->max_isr_us);
    printf("CPU_3000_PLUS=%s\n", res->pass_3000 ? "PASS" : "FAIL");
}

int main(void) {
    const BenchmarkConfig configs[] = {
        {"periodic_case", ARRIVAL_PERIODIC, 3500.0, 2.0, 200, 1, 1U},
        {"burst_case", ARRIVAL_BURST, 3500.0, 2.0, 200, 8, 1U},
        {"poisson_case", ARRIVAL_POISSON, 3500.0, 2.0, 200, 1, 20260526U},
    };
    bool all_pass = true;
    size_t i;

    printf("Interrupt capacity benchmark (goal: >=3000 irq/s)\n");
    for (i = 0; i < sizeof(configs) / sizeof(configs[0]); ++i) {
        BenchmarkResult result;
        if (!run_capacity_benchmark(&configs[i], &result)) {
            fprintf(stderr, "Failed to run benchmark for mode=%s\n",
                    mode_name(configs[i].mode));
            return EXIT_FAILURE;
        }
        print_result(&configs[i], &result);
        all_pass = all_pass && result.pass_3000;
    }

    if (!all_pass) {
        fprintf(stderr, "\nBenchmark failed: CPU cannot sustain 3000+ irq/s in all modes.\n");
        return EXIT_FAILURE;
    }

    printf("\nBenchmark passed: CPU can respond to 3000+ irq/s in all configured modes.\n");
    return EXIT_SUCCESS;
}
