#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fsm_interrupt_demo.h"

#ifndef FSM_ISR_TRACE
#define FSM_ISR_TRACE 0
#endif

#ifndef ISR_MEMCPY_CHUNK_BYTES
#define ISR_MEMCPY_CHUNK_BYTES 256
#endif

#ifndef XDMA_BUFFER_BYTES
#define XDMA_BUFFER_BYTES 1024
#endif

#ifndef XDMA_COPY_BYTES_PER_REQ
#define XDMA_COPY_BYTES_PER_REQ 128
#endif

#ifndef XDMA_LATENCY_CYCLES
#define XDMA_LATENCY_CYCLES 1
#endif

#ifdef FSM_DEMO_NO_MAIN
#define DEMO_STATIC static __attribute__((unused))
#else
#define DEMO_STATIC static
#endif

typedef struct {
    bool reg_mode;
    bool hw_trigger;
    bool sw_trigger;
    bool cfg_done;
    bool dma_ack;
    bool flow_ctl_en;
    bool feof;
    bool fsync;
    int sw_cfg_cnt;
    int reg_sw_cfg_num;
    int sw_flow_ctl_dly_cnt;
    int reg_sw_flow_ctl_dly_num;
} Inputs;

typedef struct {
    const char *name;
    Inputs in;
    bool clear_irq_before_step;
} Cycle;

typedef struct {
    bool dma_ack;
    size_t copied_bytes;
} XdmaTickStatus;

typedef struct {
    unsigned char src[XDMA_BUFFER_BYTES];
    unsigned char dst[XDMA_BUFFER_BYTES];
    size_t next_offset;
    size_t copy_bytes_per_req;
    unsigned int latency_cycles;
    unsigned int ticks_left;
    bool busy;
    unsigned long total_reqs;
    unsigned long total_copied_bytes;
} XdmaEngine;

typedef struct {
    State prev_state;
    State next_state;
    bool irq_rise;
    bool irq_level;
    bool req_issued;
    bool dma_ack_seen;
    size_t copied_bytes;
    const char *reason;
} StepResult;

DEMO_STATIC const char *state_name(State s) {
    switch (s) {
    case STATE_IDLE:
        return "IDLE";
    case STATE_WAIT_ACK_HW:
        return "WAIT_ACK_HW";
    case STATE_WAIT_ACK_SW:
        return "WAIT_ACK_SW";
    case STATE_WAIT_PIPE_FEOF:
        return "WAIT_PIPE_FEOF";
    case STATE_WAIT_START_ACK:
        return "WAIT_START_ACK";
    case STATE_CFG_END_SW:
        return "CFG_END_SW";
    default:
        return "UNKNOWN";
    }
}

DEMO_STATIC Inputs sig_default(void) {
    Inputs in = {0};
    in.reg_sw_cfg_num = 1;
    in.reg_sw_flow_ctl_dly_num = 0;
    return in;
}

DEMO_STATIC void fsm_init(InterruptFSM *fsm) {
    fsm->state = STATE_IDLE;
    fsm->irq_level = false;
}

static void fsm_clear_irq(InterruptFSM *fsm) {
    fsm->irq_level = false;
}

DEMO_STATIC bool fsm_raise_irq(InterruptFSM *fsm) {
    if (fsm->irq_level) {
        return false;
    }
    fsm->irq_level = true;
    return true;
}

DEMO_STATIC void xdma_init(XdmaEngine *xdma) {
    size_t i;
    memset(xdma, 0, sizeof(*xdma));
    xdma->copy_bytes_per_req = XDMA_COPY_BYTES_PER_REQ;
    xdma->latency_cycles = XDMA_LATENCY_CYCLES;
    for (i = 0; i < XDMA_BUFFER_BYTES; ++i) {
        xdma->src[i] = (unsigned char)(i & 0xFFU);
    }
}

DEMO_STATIC bool xdma_issue_req(XdmaEngine *xdma) {
    if (xdma->busy) {
        return false;
    }
    xdma->busy = true;
    xdma->ticks_left = xdma->latency_cycles;
    xdma->total_reqs++;
    return true;
}

DEMO_STATIC XdmaTickStatus xdma_tick(XdmaEngine *xdma) {
    XdmaTickStatus tick = {false, 0};
    size_t chunk;
    size_t remain;
    if (!xdma->busy) {
        return tick;
    }

    if (xdma->ticks_left > 0U) {
        xdma->ticks_left--;
        if (xdma->ticks_left > 0U) {
            return tick;
        }
    }

    remain = XDMA_BUFFER_BYTES - xdma->next_offset;
    chunk = xdma->copy_bytes_per_req;
    if (chunk > remain) {
        chunk = remain;
    }
    memcpy(&xdma->dst[xdma->next_offset], &xdma->src[xdma->next_offset], chunk);
    xdma->total_copied_bytes += chunk;
    xdma->next_offset += chunk;
    if (xdma->next_offset >= XDMA_BUFFER_BYTES) {
        xdma->next_offset = 0;
    }
    xdma->busy = false;
    tick.dma_ack = true;
    tick.copied_bytes = chunk;
    return tick;
}

/*
 * Reusable ISR simulation API.
 * Other functions can call this to emulate:
 * 1) entering interrupt context
 * 2) handling some work (including memcpy-like data movement)
 * 3) clearing interrupt
 */
bool simulate_interrupt_and_handle(InterruptFSM *fsm, InterruptServiceStats *stats,
                                   const char *caller_name, int work_steps) {
    int i;
    unsigned char src[ISR_MEMCPY_CHUNK_BYTES];
    unsigned char dst[ISR_MEMCPY_CHUNK_BYTES];
    volatile unsigned int isr_dummy_acc = 0U;

    if (fsm == NULL || stats == NULL) {
        return false;
    }
    if (!fsm->irq_level) {
        return false;
    }

    for (i = 0; i < ISR_MEMCPY_CHUNK_BYTES; ++i) {
        src[i] = (unsigned char)(i ^ 0x5AU);
        dst[i] = 0U;
    }

    stats->enter_count++;
    if (FSM_ISR_TRACE) {
        printf("[ISR] enter from %s, work_steps=%d\n",
               caller_name == NULL ? "unknown" : caller_name, work_steps);
    }

    if (work_steps <= 0) {
        work_steps = 1;
    }
    for (i = 0; i < work_steps; ++i) {
        memcpy(dst, src, sizeof(src));
        isr_dummy_acc += (unsigned int)dst[(unsigned int)i % sizeof(dst)];
        src[(unsigned int)i % sizeof(src)] ^= (unsigned char)i;
        stats->handled_steps++;
    }
    (void)isr_dummy_acc;

    fsm_clear_irq(fsm);
    stats->clear_count++;
    if (FSM_ISR_TRACE) {
        printf("[ISR] handled and irq cleared\n");
    }
    return true;
}

DEMO_STATIC bool app_poll_and_service_irq(InterruptFSM *fsm, InterruptServiceStats *stats,
                                          const char *caller_name) {
    return simulate_interrupt_and_handle(fsm, stats, caller_name, 3);
}

DEMO_STATIC StepResult fsm_step(InterruptFSM *fsm, XdmaEngine *xdma, const Inputs *sig,
                                bool dma_ack, size_t copied_bytes) {
    StepResult ret;
    ret.prev_state = fsm->state;
    ret.next_state = fsm->state;
    ret.irq_rise = false;
    ret.irq_level = fsm->irq_level;
    ret.req_issued = false;
    ret.dma_ack_seen = dma_ack;
    ret.copied_bytes = copied_bytes;
    ret.reason = "hold";

    switch (fsm->state) {
    case STATE_IDLE:
        if (sig->reg_mode && sig->hw_trigger && sig->cfg_done) {
            if (xdma_issue_req(xdma)) {
                ret.next_state = STATE_WAIT_ACK_HW;
                ret.req_issued = true;
                ret.reason = "HW mode: cfg_done then req -> WAIT_ACK_HW";
            } else {
                ret.reason = "HW mode: xdma busy, req blocked";
            }
        } else if ((!sig->reg_mode) && sig->sw_trigger) {
            if (xdma_issue_req(xdma)) {
                ret.next_state = STATE_WAIT_ACK_SW;
                ret.req_issued = true;
                ret.reason = "SW mode: sw_trigger then req -> WAIT_ACK_SW";
            } else {
                ret.reason = "SW mode: xdma busy, req blocked";
            }
        } else {
            ret.reason = "IDLE wait trigger";
        }
        break;

    case STATE_WAIT_ACK_HW:
        if (dma_ack) {
            ret.next_state = STATE_IDLE;
            ret.reason = "HW dma_ack received -> IDLE";
            ret.irq_rise = fsm_raise_irq(fsm);
        } else {
            ret.reason = "WAIT_ACK_HW: wait dma_ack";
        }
        break;

    case STATE_WAIT_ACK_SW:
        if (!dma_ack) {
            ret.reason = "WAIT_ACK_SW: wait dma_ack";
        } else if (sig->flow_ctl_en) {
            ret.next_state = STATE_WAIT_PIPE_FEOF;
            ret.reason = "SW dma_ack + flow_ctl_en=1 -> WAIT_PIPE_FEOF";
        } else {
            ret.next_state = STATE_CFG_END_SW;
            ret.reason = "SW dma_ack + flow_ctl_en=0 -> CFG_END_SW";
        }
        break;

    case STATE_WAIT_PIPE_FEOF:
        if (!sig->feof) {
            ret.reason = "WAIT_PIPE_FEOF: wait feof";
        } else if (sig->sw_flow_ctl_dly_cnt < sig->reg_sw_flow_ctl_dly_num) {
            ret.reason = "WAIT_PIPE_FEOF: wait flow_ctl delay";
        } else if (xdma_issue_req(xdma)) {
            ret.next_state = STATE_WAIT_START_ACK;
            ret.req_issued = true;
            ret.reason = "feof + delay met -> req -> WAIT_START_ACK";
        } else {
            ret.reason = "WAIT_PIPE_FEOF: xdma busy, req blocked";
        }
        break;

    case STATE_WAIT_START_ACK:
        if (dma_ack) {
            ret.next_state = STATE_CFG_END_SW;
            ret.reason = "WAIT_START_ACK dma_ack -> CFG_END_SW";
        } else {
            ret.reason = "WAIT_START_ACK: wait dma_ack";
        }
        break;

    case STATE_CFG_END_SW:
        if (sig->sw_cfg_cnt >= sig->reg_sw_cfg_num) {
            ret.next_state = STATE_IDLE;
            ret.reason = "SW cfg completed -> IDLE";
            ret.irq_rise = fsm_raise_irq(fsm);
        } else if (sig->fsync) {
            if (xdma_issue_req(xdma)) {
                ret.next_state = STATE_WAIT_ACK_SW;
                ret.req_issued = true;
                ret.reason = "SW not done + fsync -> req -> WAIT_ACK_SW";
            } else {
                ret.reason = "CFG_END_SW: xdma busy, req blocked";
            }
        } else {
            ret.reason = "CFG_END_SW: wait fsync or completion";
        }
        break;

    default:
        ret.reason = "invalid state";
        break;
    }

    fsm->state = ret.next_state;
    ret.irq_level = fsm->irq_level;
    return ret;
}

DEMO_STATIC bool run_scenario(const char *scenario_name, const Cycle *cycles,
                              size_t cycle_count, const int *expected_rise_cycles,
                              size_t expected_count, unsigned int expected_irq_service_count,
                              unsigned long expected_req_count) {
    InterruptFSM fsm;
    XdmaEngine xdma;
    InterruptServiceStats irq_stats = {0};
    int actual_rise_cycles[32] = {0};
    size_t actual_count = 0;
    size_t i;

    fsm_init(&fsm);
    xdma_init(&xdma);

    printf("\n=== Scenario: %s ===\n", scenario_name);
    printf("cycle | event                  | prev_state      -> next_state      | "
           "ack req irq  copied | reason\n");
    printf("-------------------------------------------------------------------------------------------\n");

    for (i = 0; i < cycle_count; ++i) {
        StepResult res;
        XdmaTickStatus tick;
        Inputs in = cycles[i].in;
        char irq_mark = '0';

        if (cycles[i].clear_irq_before_step) {
            (void)app_poll_and_service_irq(&fsm, &irq_stats, cycles[i].name);
        }

        tick = xdma_tick(&xdma);
        in.dma_ack = in.dma_ack || tick.dma_ack;
        res = fsm_step(&fsm, &xdma, &in, in.dma_ack, tick.copied_bytes);
        if (res.irq_rise) {
            if (actual_count >= (sizeof(actual_rise_cycles) / sizeof(actual_rise_cycles[0]))) {
                fprintf(stderr, "actual_rise_cycles buffer overflow\n");
                return false;
            }
            actual_rise_cycles[actual_count++] = (int)i;
        }

        if (res.irq_rise) {
            irq_mark = 'R';
        } else if (res.irq_level) {
            irq_mark = '1';
        }

        printf("%5zu | %-22s | %-15s -> %-15s |  %c   %c   %c  %6zu | %s\n", i,
               cycles[i].name, state_name(res.prev_state), state_name(res.next_state),
               res.dma_ack_seen ? '1' : '0', res.req_issued ? '1' : '0', irq_mark,
               res.copied_bytes, res.reason);
    }

    if (actual_count != expected_count) {
        fprintf(stderr, "[FAIL] %s: expected %zu irq rise(s), actual %zu\n",
                scenario_name, expected_count, actual_count);
        return false;
    }

    for (i = 0; i < expected_count; ++i) {
        if (actual_rise_cycles[i] != expected_rise_cycles[i]) {
            fprintf(stderr,
                    "[FAIL] %s: irq rise mismatch at index %zu, expected=%d, actual=%d\n",
                    scenario_name, i, expected_rise_cycles[i], actual_rise_cycles[i]);
            return false;
        }
    }

    if (irq_stats.enter_count != expected_irq_service_count) {
        fprintf(stderr,
                "[FAIL] %s: expected irq service count=%u, actual=%u\n",
                scenario_name, expected_irq_service_count, irq_stats.enter_count);
        return false;
    }

    if (xdma.total_reqs != expected_req_count) {
        fprintf(stderr,
                "[FAIL] %s: expected xdma req count=%lu, actual=%lu\n",
                scenario_name, expected_req_count, xdma.total_reqs);
        return false;
    }

    if (xdma.total_copied_bytes == 0UL) {
        fprintf(stderr, "[FAIL] %s: xdma copied bytes is zero\n", scenario_name);
        return false;
    }

    printf("[PASS] IRQ rise cycles = [");
    for (i = 0; i < actual_count; ++i) {
        if (i > 0) {
            printf(", ");
        }
        printf("%d", actual_rise_cycles[i]);
    }
    printf("]\n");
    printf("[PASS] IRQ service count = %u, handled_steps = %u\n",
           irq_stats.enter_count, irq_stats.handled_steps);
    printf("[PASS] XDMA req count = %lu, copied bytes = %lu\n",
           xdma.total_reqs, xdma.total_copied_bytes);
    return true;
}

DEMO_STATIC bool scenario_hw_trigger(void) {
    Inputs in0 = sig_default();
    Inputs in1 = sig_default();
    Inputs in2 = sig_default();
    Inputs in3 = sig_default();
    const int expected[] = {2};

    in1.reg_mode = true;
    in1.hw_trigger = true;
    in1.cfg_done = true;

    in2.reg_mode = true;

    const Cycle cycles[] = {
        {"idle", in0, false},
        {"hw trigger + cfg_done", in1, false},
        {"xdma copy done ack", in2, false},
        {"sw clear irq", in3, true},
    };

    return run_scenario("HW mode", cycles, sizeof(cycles) / sizeof(cycles[0]), expected,
                        sizeof(expected) / sizeof(expected[0]), 1U, 1UL);
}

DEMO_STATIC bool scenario_sw_no_flow_control(void) {
    Inputs in0 = sig_default();
    Inputs in1 = sig_default();
    Inputs in2 = sig_default();
    Inputs in3 = sig_default();
    Inputs in4 = sig_default();
    const int expected[] = {3};

    in1.reg_mode = false;
    in1.sw_trigger = true;

    in2.reg_mode = false;
    in2.flow_ctl_en = false;

    in3.reg_mode = false;
    in3.sw_cfg_cnt = 1;
    in3.reg_sw_cfg_num = 1;

    const Cycle cycles[] = {
        {"idle", in0, false},
        {"sw trigger", in1, false},
        {"xdma ack flow_ctl=0", in2, false},
        {"cfg end -> idle", in3, false},
        {"sw clear irq", in4, true},
    };

    return run_scenario("SW mode (flow_ctl_en=0)", cycles,
                        sizeof(cycles) / sizeof(cycles[0]), expected,
                        sizeof(expected) / sizeof(expected[0]), 1U, 1UL);
}

DEMO_STATIC bool scenario_sw_with_flow_control_multi_cfg(void) {
    Inputs in0 = sig_default();
    Inputs in1 = sig_default();
    Inputs in2 = sig_default();
    Inputs in3 = sig_default();
    Inputs in4 = sig_default();
    Inputs in5 = sig_default();
    Inputs in6 = sig_default();
    Inputs in7 = sig_default();
    Inputs in8 = sig_default();
    Inputs in9 = sig_default();
    const int expected[] = {8};

    in0.reg_mode = false;
    in0.reg_sw_cfg_num = 2;

    in1.reg_mode = false;
    in1.sw_trigger = true;
    in1.reg_sw_cfg_num = 2;

    in2.reg_mode = false;
    in2.flow_ctl_en = true;
    in2.reg_sw_cfg_num = 2;

    in3.reg_mode = false;
    in3.reg_sw_cfg_num = 2;
    in3.feof = false;

    in4.reg_mode = false;
    in4.reg_sw_cfg_num = 2;
    in4.feof = true;
    in4.sw_flow_ctl_dly_cnt = 2;
    in4.reg_sw_flow_ctl_dly_num = 2;

    in5.reg_mode = false;
    in5.reg_sw_cfg_num = 2;

    in6.reg_mode = false;
    in6.reg_sw_cfg_num = 2;
    in6.sw_cfg_cnt = 1;
    in6.fsync = true;

    in7.reg_mode = false;
    in7.reg_sw_cfg_num = 2;
    in7.flow_ctl_en = false;

    in8.reg_mode = false;
    in8.reg_sw_cfg_num = 2;
    in8.sw_cfg_cnt = 2;

    const Cycle cycles[] = {
        {"idle", in0, false},
        {"sw trigger cfg#1", in1, false},
        {"ack cfg#1 + flow_ctl=1", in2, false},
        {"wait feof", in3, false},
        {"feof + delay met", in4, false},
        {"wait_start_ack", in5, false},
        {"cfg_end not done + fsync", in6, false},
        {"ack cfg#2 + flow_ctl=0", in7, false},
        {"cfg done -> idle", in8, false},
        {"sw clear irq", in9, true},
    };

    return run_scenario("SW mode (flow_ctl_en=1 multi-cfg)", cycles,
                        sizeof(cycles) / sizeof(cycles[0]), expected,
                        sizeof(expected) / sizeof(expected[0]), 1U, 3UL);
}

#ifndef FSM_DEMO_NO_MAIN
int main(void) {
    bool ok = true;

    ok = scenario_hw_trigger() && ok;
    ok = scenario_sw_no_flow_control() && ok;
    ok = scenario_sw_with_flow_control_multi_cfg() && ok;

    if (!ok) {
        fprintf(stderr, "\nOne or more scenarios failed.\n");
        return EXIT_FAILURE;
    }

    printf("\nAll demo scenarios passed.\n");
    return EXIT_SUCCESS;
}
#endif
