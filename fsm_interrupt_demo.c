#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    STATE_IDLE = 0,
    STATE_WAIT_ACK_HW,
    STATE_WAIT_ACK_SW,
    STATE_WAIT_PIPE_FEOF,
    STATE_WAIT_START_ACK,
    STATE_CFG_END_SW
} State;

typedef struct {
    bool reg_mode;
    bool hw_trigger;
    bool sw_trigger;
    bool cfg_done;
    bool dma_ack;
    bool flow_ctl_en;
    bool pipe_busy;
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
    State prev_state;
    State next_state;
    bool irq_rise;
    bool irq_level;
    const char *reason;
} StepResult;

typedef struct {
    State state;
    bool irq_level;
} InterruptFSM;

typedef struct {
    unsigned int enter_count;
    unsigned int handled_steps;
    unsigned int clear_count;
} InterruptServiceStats;

static const char *state_name(State s) {
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

static Inputs sig_default(void) {
    Inputs in = {0};
    in.reg_sw_cfg_num = 1;
    return in;
}

static void fsm_init(InterruptFSM *fsm) {
    fsm->state = STATE_IDLE;
    fsm->irq_level = false;
}

static void fsm_clear_irq(InterruptFSM *fsm) {
    fsm->irq_level = false;
}

static bool fsm_raise_irq(InterruptFSM *fsm) {
    if (fsm->irq_level) {
        return false;
    }
    fsm->irq_level = true;
    return true;
}

/*
 * Reusable ISR simulation API.
 * Other functions can call this to emulate:
 * 1) entering interrupt context
 * 2) handling some work
 * 3) clearing interrupt
 */
bool simulate_interrupt_and_handle(InterruptFSM *fsm, InterruptServiceStats *stats,
                                   const char *caller_name, int work_steps) {
    int i;

    if (fsm == NULL || stats == NULL || caller_name == NULL) {
        return false;
    }
    if (!fsm->irq_level) {
        return false;
    }

    stats->enter_count++;
    printf("[ISR] enter from %s, work_steps=%d\n", caller_name, work_steps);

    if (work_steps <= 0) {
        work_steps = 1;
    }
    for (i = 0; i < work_steps; ++i) {
        stats->handled_steps++;
    }

    fsm_clear_irq(fsm);
    stats->clear_count++;
    printf("[ISR] handled and irq cleared\n");
    return true;
}

static bool app_poll_and_service_irq(InterruptFSM *fsm, InterruptServiceStats *stats,
                                     const char *caller_name) {
    return simulate_interrupt_and_handle(fsm, stats, caller_name, 3);
}

static StepResult fsm_step(InterruptFSM *fsm, const Inputs *sig) {
    StepResult ret;
    ret.prev_state = fsm->state;
    ret.next_state = fsm->state;
    ret.irq_rise = false;
    ret.irq_level = fsm->irq_level;
    ret.reason = "hold";

    switch (fsm->state) {
    case STATE_IDLE:
        if (sig->reg_mode && sig->hw_trigger && sig->cfg_done) {
            ret.next_state = STATE_WAIT_ACK_HW;
            ret.reason = "reg_mode & hw_trigger & cfg_done";
        } else if ((!sig->reg_mode) && sig->sw_trigger) {
            ret.next_state = STATE_WAIT_ACK_SW;
            ret.reason = "~reg_mode & sw_trigger";
        } else {
            ret.reason = "idle wait trigger";
        }
        break;

    case STATE_WAIT_ACK_HW:
        if (sig->dma_ack) {
            ret.next_state = STATE_IDLE;
            ret.reason = "dma_ack -> HW flow completed";
            ret.irq_rise = fsm_raise_irq(fsm);
        } else {
            ret.reason = "~dma_ack";
        }
        break;

    case STATE_WAIT_ACK_SW:
        if (!sig->dma_ack) {
            ret.reason = "~dma_ack";
        } else if (sig->flow_ctl_en) {
            ret.next_state = STATE_WAIT_PIPE_FEOF;
            ret.reason = "dma_ack & flow_ctl_en";
        } else {
            ret.next_state = STATE_CFG_END_SW;
            ret.reason = "dma_ack & ~flow_ctl_en";
        }
        break;

    case STATE_WAIT_PIPE_FEOF:
        if (sig->pipe_busy) {
            ret.reason = "pipe_busy";
        } else if ((!sig->dma_ack) &&
                   (sig->sw_flow_ctl_dly_cnt >= sig->reg_sw_flow_ctl_dly_num)) {
            ret.next_state = STATE_WAIT_START_ACK;
            ret.reason = "~pipe_busy & ~dma_ack & delay_met";
        } else {
            ret.reason = "wait pipe empty / delay";
        }
        break;

    case STATE_WAIT_START_ACK:
        if (sig->dma_ack) {
            ret.next_state = STATE_CFG_END_SW;
            ret.reason = "dma_ack";
        } else {
            ret.reason = "~dma_ack";
        }
        break;

    case STATE_CFG_END_SW:
        if (sig->sw_cfg_cnt >= sig->reg_sw_cfg_num) {
            ret.next_state = STATE_IDLE;
            ret.reason = "sw_cfg_cnt >= reg_sw_cfg_num";
            ret.irq_rise = fsm_raise_irq(fsm);
        } else if ((sig->sw_cfg_cnt < sig->reg_sw_cfg_num) && (!sig->dma_ack) &&
                   sig->fsync) {
            ret.next_state = STATE_WAIT_ACK_SW;
            ret.reason = "sw_cfg_cnt < reg_sw_cfg_num & ~dma_ack & fsync";
        } else {
            ret.reason = "wait next cfg or completion";
        }
        break;

    default:
        break;
    }

    fsm->state = ret.next_state;
    ret.irq_level = fsm->irq_level;
    return ret;
}

static bool run_scenario(const char *scenario_name, const Cycle *cycles,
                         size_t cycle_count, const int *expected_rise_cycles,
                         size_t expected_count, unsigned int expected_irq_service_count) {
    InterruptFSM fsm;
    InterruptServiceStats irq_stats = {0};
    int actual_rise_cycles[32] = {0};
    size_t actual_count = 0;
    size_t i;

    fsm_init(&fsm);

    printf("\n=== Scenario: %s ===\n", scenario_name);
    printf("cycle | event                  | prev_state      -> next_state      | "
           "irq | reason\n");
    printf("-----------------------------------------------------------------------"
           "--------\n");

    for (i = 0; i < cycle_count; ++i) {
        StepResult res;
        char irq_mark = '0';

        if (cycles[i].clear_irq_before_step) {
            (void)app_poll_and_service_irq(&fsm, &irq_stats, cycles[i].name);
        }

        res = fsm_step(&fsm, &cycles[i].in);
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

        printf("%5zu | %-22s | %-15s -> %-15s |  %c  | %s\n", i, cycles[i].name,
               state_name(res.prev_state), state_name(res.next_state), irq_mark,
               res.reason);
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

    printf("[PASS] IRQ rise cycles = [");
    for (i = 0; i < actual_count; ++i) {
        if (i > 0) {
            printf(", ");
        }
        printf("%d", actual_rise_cycles[i]);
    }
    printf("]\n");
    if (irq_stats.enter_count != expected_irq_service_count) {
        fprintf(stderr,
                "[FAIL] %s: expected irq service count=%u, actual=%u\n",
                scenario_name, expected_irq_service_count, irq_stats.enter_count);
        return false;
    }
    printf("[PASS] IRQ service count = %u, handled_steps = %u\n",
           irq_stats.enter_count, irq_stats.handled_steps);

    return true;
}

static bool scenario_hw_trigger(void) {
    Inputs in0 = sig_default();
    Inputs in1 = sig_default();
    Inputs in2 = sig_default();
    Inputs in3 = sig_default();
    Inputs in4 = sig_default();
    const int expected[] = {3};

    in1.reg_mode = true;
    in1.hw_trigger = true;
    in1.cfg_done = true;

    in2.reg_mode = true;
    in2.cfg_done = true;

    in3.reg_mode = true;
    in3.cfg_done = true;
    in3.dma_ack = true;

    const Cycle cycles[] = {
        {"idle", in0, false},
        {"hw trigger", in1, false},
        {"wait dma ack", in2, false},
        {"dma ack", in3, false},
        {"sw clear irq", in4, true},
    };

    return run_scenario("HW trigger flow", cycles, sizeof(cycles) / sizeof(cycles[0]),
                        expected, sizeof(expected) / sizeof(expected[0]), 1U);
}

static bool scenario_sw_no_flow_control(void) {
    Inputs in0 = sig_default();
    Inputs in1 = sig_default();
    Inputs in2 = sig_default();
    Inputs in3 = sig_default();
    Inputs in4 = sig_default();
    Inputs in5 = sig_default();
    const int expected[] = {4};

    in1.sw_trigger = true;
    in1.reg_mode = false;

    in2.reg_mode = false;

    in3.reg_mode = false;
    in3.dma_ack = true;
    in3.flow_ctl_en = false;

    in4.reg_mode = false;
    in4.sw_cfg_cnt = 1;
    in4.reg_sw_cfg_num = 1;
    in4.dma_ack = false;

    const Cycle cycles[] = {
        {"idle", in0, false},
        {"sw trigger", in1, false},
        {"wait dma ack", in2, false},
        {"dma ack, no flow", in3, false},
        {"all cfg done", in4, false},
        {"sw clear irq", in5, true},
    };

    return run_scenario("SW flow (no flow control)", cycles,
                        sizeof(cycles) / sizeof(cycles[0]), expected,
                        sizeof(expected) / sizeof(expected[0]), 1U);
}

static bool scenario_sw_with_flow_control_multi_cfg(void) {
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

    in0.reg_sw_cfg_num = 2;

    in1.sw_trigger = true;
    in1.reg_mode = false;
    in1.reg_sw_cfg_num = 2;

    in2.reg_mode = false;
    in2.dma_ack = true;
    in2.flow_ctl_en = true;
    in2.reg_sw_cfg_num = 2;

    in3.reg_mode = false;
    in3.pipe_busy = true;
    in3.reg_sw_cfg_num = 2;

    in4.reg_mode = false;
    in4.pipe_busy = false;
    in4.dma_ack = false;
    in4.sw_flow_ctl_dly_cnt = 2;
    in4.reg_sw_flow_ctl_dly_num = 2;
    in4.reg_sw_cfg_num = 2;

    in5.reg_mode = false;
    in5.dma_ack = true;
    in5.reg_sw_cfg_num = 2;

    in6.reg_mode = false;
    in6.sw_cfg_cnt = 1;
    in6.reg_sw_cfg_num = 2;
    in6.dma_ack = false;
    in6.fsync = true;

    in7.reg_mode = false;
    in7.dma_ack = true;
    in7.flow_ctl_en = false;
    in7.reg_sw_cfg_num = 2;

    in8.reg_mode = false;
    in8.sw_cfg_cnt = 2;
    in8.reg_sw_cfg_num = 2;

    const Cycle cycles[] = {
        {"idle", in0, false},
        {"sw trigger cfg#1", in1, false},
        {"dma ack + flow ctl", in2, false},
        {"pipe busy", in3, false},
        {"pipe empty + delay met", in4, false},
        {"start ack", in5, false},
        {"fsync next cfg", in6, false},
        {"cfg#2 dma ack", in7, false},
        {"all cfg done", in8, false},
        {"sw clear irq", in9, true},
    };

    return run_scenario("SW flow (flow control, multi cfg)", cycles,
                        sizeof(cycles) / sizeof(cycles[0]), expected,
                        sizeof(expected) / sizeof(expected[0]), 1U);
}

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
