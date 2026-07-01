#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * M3 flow simulator
 *
 * This program is a software model derived from the three flowcharts.
 * Hardware path, software path, and software flow-control path are treated as
 * branches inside one unified state machine.
 *
 * Assumptions used by the simulator:
 *   - time advances in discrete cycles
 *   - fsync happens at the first cycle of each frame
 *   - teof happens in the middle of each frame
 *   - feof happens at the last cycle of each frame
 *   - one DMA request completes after dma_ack_latency cycles
 *   - one sw_trigger configures sw_cfg_num frames
 *   - hw_skip_frame_num skips the first N trigger opportunities
 *   - hw_cfg_done_max_idx is reported as metadata, matching the diagram note
 */

#define MAX_TEXT 128

typedef enum {
    TRIGGER_FSYNC = 0,
    TRIGGER_TEOF = 1
} TriggerSource;

typedef enum {
    STATE_IDLE = 0,
    STATE_WAIT_ACK_HW,
    STATE_WAIT_ACK_SW,
    STATE_WAIT_PIPE_FEOF,
    STATE_WAIT_START_ACK,
    STATE_CFG_END_SW,
    STATE_ERROR
} SimState;

typedef struct {
    int frame_cycles;
    int dma_ack_latency;
    int initial_dma_busy_cycles;
    int watchdog_cycles;

    int reg_mode_hw;
    int hw_cfg_done;
    int hw_dly_num;
    int hw_cfg_done_max_idx;
    int hw_skip_frame_num;
    TriggerSource hw_trigger_source;
    int hardware_waits_ack;

    int sw_trigger;
    int sw_flow_ctl_en;
    int sw_cfg_num;
    int sw_flow_ctl_dly_num;
    int pipe_busy_cycles;
} SimConfig;

typedef struct {
    SimConfig cfg;
    SimState state;
    int cycle;
    int due_cycle;
    int ack_cycle;
    int dma_busy_until;
    int pipe_busy_until;
    int feof_cycle;
    int skipped_triggers;
    int sw_cfg_cnt;
    int pending_cfg_complete;
    int hw_armed;
    int hw_delay_pending;
    int feof_seen;
    int sw_started;
    int completed;
    char result[MAX_TEXT];
} SimContext;

static const char *reg_mode_name(int reg_mode_hw) {
    return reg_mode_hw ? "hw" : "sw";
}

static bool is_hw_mode(const SimContext *ctx) {
    return ctx->cfg.reg_mode_hw != 0;
}

static bool is_sw_mode(const SimContext *ctx) {
    return ctx->cfg.reg_mode_hw == 0;
}

static bool sw_flow_enabled(const SimContext *ctx) {
    return is_sw_mode(ctx) && ctx->cfg.sw_flow_ctl_en != 0;
}

static const char *state_name(SimState state) {
    switch (state) {
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
        case STATE_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

static const char *trigger_name(TriggerSource source) {
    return source == TRIGGER_FSYNC ? "fsync" : "teof";
}

static void log_cycle(SimContext *ctx, const char *message) {
    printf("[cycle %03d] %-16s %s\n", ctx->cycle, state_name(ctx->state), message);
}

static void complete_and_return_idle(SimContext *ctx, const char *message) {
    ctx->completed = 1;
    ctx->state = STATE_IDLE;
    snprintf(ctx->result, sizeof(ctx->result), "%s", message);
    log_cycle(ctx, message);
}

static void set_error(SimContext *ctx, const char *message) {
    ctx->state = STATE_ERROR;
    snprintf(ctx->result, sizeof(ctx->result), "%s", message);
    log_cycle(ctx, message);
}

static int frame_pos(const SimContext *ctx) {
    return ctx->cycle % ctx->cfg.frame_cycles;
}

static bool event_fsync(const SimContext *ctx) {
    return frame_pos(ctx) == 0;
}

static bool event_teof(const SimContext *ctx) {
    return frame_pos(ctx) == ctx->cfg.frame_cycles / 2;
}

static bool event_feof(const SimContext *ctx) {
    return frame_pos(ctx) == ctx->cfg.frame_cycles - 1;
}

static bool dma_busy(const SimContext *ctx) {
    return ctx->cycle < ctx->dma_busy_until;
}

static void launch_dma(SimContext *ctx, const char *why) {
    ctx->ack_cycle = ctx->cycle + ctx->cfg.dma_ack_latency;
    ctx->dma_busy_until = ctx->ack_cycle;
    char text[MAX_TEXT];
    snprintf(text, sizeof(text), "trigger XDMA (%s), ack expected at cycle %d", why, ctx->ack_cycle);
    log_cycle(ctx, text);
}

static void complete_sw_cfg(SimContext *ctx) {
    ctx->pending_cfg_complete = 0;
    ctx->sw_cfg_cnt++;
    char text[MAX_TEXT];
    snprintf(text, sizeof(text), "CFG_END_SW, sw_cfg_cnt=%d/%d", ctx->sw_cfg_cnt, ctx->cfg.sw_cfg_num);
    log_cycle(ctx, text);
}

static int validate_config(const SimConfig *cfg, char *error_text, size_t error_text_size) {
    if (cfg->frame_cycles < 2) {
        snprintf(error_text, error_text_size, "frame_cycles must be >= 2");
        return -1;
    }
    if (cfg->dma_ack_latency < 1) {
        snprintf(error_text, error_text_size, "dma_ack_latency must be >= 1");
        return -1;
    }
    if (cfg->watchdog_cycles < 0) {
        snprintf(error_text, error_text_size, "watchdog_cycles must be >= 0");
        return -1;
    }
    if (cfg->hw_dly_num < 0) {
        snprintf(error_text, error_text_size, "hw_dly_num must be >= 0");
        return -1;
    }
    if (cfg->hw_cfg_done_max_idx < 0) {
        snprintf(error_text, error_text_size, "hw_cfg_done_max_idx must be >= 0");
        return -1;
    }
    if (cfg->sw_cfg_num < 1) {
        snprintf(error_text, error_text_size, "sw_cfg_num must be >= 1");
        return -1;
    }
    if (cfg->sw_flow_ctl_dly_num < 0) {
        snprintf(error_text, error_text_size, "sw_flow_ctl_dly_num must be >= 0");
        return -1;
    }
    return 0;
}

static void print_banner(const SimConfig *cfg) {
    printf("\n=== M3 unified flow simulation ===\n");
    printf("frame_cycles=%d, dma_ack_latency=%d, initial_dma_busy_cycles=%d, watchdog_cycles=%d(0=auto)\n",
           cfg->frame_cycles,
           cfg->dma_ack_latency,
           cfg->initial_dma_busy_cycles,
           cfg->watchdog_cycles);
    printf("reg_mode=%s, sw_trigger=%d, sw_flow_ctl_en=%d\n",
           reg_mode_name(cfg->reg_mode_hw),
           cfg->sw_trigger,
           cfg->sw_flow_ctl_en);
    printf("hw_cfg_done=%d, hw_dly_num=%d, hw_cfg_done_max_idx=%d, valid_hw_cfg_bits=%d, "
           "hw_skip_frame_num=%d, hw_trigger=%s, hardware_waits_ack=%d\n",
           cfg->hw_cfg_done,
           cfg->hw_dly_num,
           cfg->hw_cfg_done_max_idx,
           cfg->hw_cfg_done_max_idx + 1,
           cfg->hw_skip_frame_num,
           trigger_name(cfg->hw_trigger_source),
           cfg->hardware_waits_ack);
    printf("sw_cfg_num=%d, sw_flow_ctl_dly_num=%d, pipe_busy_cycles=%d\n",
           cfg->sw_cfg_num,
           cfg->sw_flow_ctl_dly_num,
           cfg->pipe_busy_cycles);
}

static void init_context(SimContext *ctx, const SimConfig *cfg) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->state = STATE_IDLE;
    ctx->dma_busy_until = cfg->initial_dma_busy_cycles;
    snprintf(ctx->result, sizeof(ctx->result), "running");
}

static void try_start_hw_dma(SimContext *ctx) {
    if (dma_busy(ctx)) {
        set_error(ctx, "intr: dma_busy_error in hardware branch");
        return;
    }

    launch_dma(ctx, "hardware flow");
    if (ctx->cfg.hardware_waits_ack) {
        ctx->state = STATE_WAIT_ACK_HW;
        return;
    }

    complete_and_return_idle(ctx, "A500 path complete, return IDLE");
}

static void try_start_sw_dma(SimContext *ctx, const char *why, SimState wait_state) {
    if (dma_busy(ctx)) {
        set_error(ctx, "return error: dma busy");
        return;
    }

    launch_dma(ctx, why);
    ctx->state = wait_state;
}

static bool arm_hw_mode(SimContext *ctx) {
    if (!is_hw_mode(ctx) || ctx->hw_armed) {
        return true;
    }

    ctx->hw_armed = 1;
    if (!ctx->cfg.hw_cfg_done) {
        set_error(ctx, "intr: hw_cfg_done_error");
        return false;
    }
    if (ctx->cfg.hw_dly_num > ctx->cfg.frame_cycles) {
        set_error(ctx, "intr: hw_dly_num_error");
        return false;
    }

    {
        char text[MAX_TEXT];
        snprintf(text,
                 sizeof(text),
                 "hardware branch armed, valid cfg_done bits = [0..%d]",
                 ctx->cfg.hw_cfg_done_max_idx);
        log_cycle(ctx, text);
    }

    return true;
}

static void arm_next_feof_wait(SimContext *ctx) {
    ctx->feof_seen = 0;
    ctx->feof_cycle = -1;
    ctx->pipe_busy_until = ctx->cycle + ctx->cfg.pipe_busy_cycles;
    log_cycle(ctx, "enter WAIT_PIPE_FEOF");
}

static void handle_hw_idle(SimContext *ctx) {
    bool trigger_hit;

    if (!arm_hw_mode(ctx)) {
        return;
    }

    if (ctx->hw_delay_pending) {
        if (ctx->cycle < ctx->due_cycle) {
            return;
        }
        ctx->hw_delay_pending = 0;
        log_cycle(ctx, "hardware delay elapsed, trigger XDMA");
        try_start_hw_dma(ctx);
        return;
    }

    trigger_hit = (ctx->cfg.hw_trigger_source == TRIGGER_FSYNC) ? event_fsync(ctx) : event_teof(ctx);
    if (!trigger_hit) {
        return;
    }

    if (ctx->skipped_triggers < ctx->cfg.hw_skip_frame_num) {
        char text[MAX_TEXT];
        ctx->skipped_triggers++;
        snprintf(text,
                 sizeof(text),
                 "skip trigger #%d because hw_skip_frame_num=%d",
                 ctx->skipped_triggers,
                 ctx->cfg.hw_skip_frame_num);
        log_cycle(ctx, text);
        return;
    }

    if (ctx->cfg.hw_dly_num == 0) {
        log_cycle(ctx, "hardware trigger detected in IDLE");
        try_start_hw_dma(ctx);
        return;
    }

    ctx->due_cycle = ctx->cycle + ctx->cfg.hw_dly_num;
    ctx->hw_delay_pending = 1;
    {
        char text[MAX_TEXT];
        snprintf(text,
                 sizeof(text),
                 "hardware trigger detected, wait hw_dly_num=%d cycles until cycle %d",
                 ctx->cfg.hw_dly_num,
                 ctx->due_cycle);
        log_cycle(ctx, text);
    }
}

static void handle_sw_idle(SimContext *ctx) {
    if (!is_sw_mode(ctx) || !ctx->cfg.sw_trigger || ctx->sw_started) {
        return;
    }

    ctx->sw_started = 1;
    try_start_sw_dma(ctx,
                     sw_flow_enabled(ctx) ? "sw_trigger start, enter WAIT_ACK_SW"
                                          : "sw_trigger start, no flow-control",
                     STATE_WAIT_ACK_SW);
}

static void try_restart_sw_burst(SimContext *ctx) {
    if (sw_flow_enabled(ctx)) {
        if (!event_fsync(ctx)) {
            return;
        }
        try_start_sw_dma(ctx, "fsync restart after CFG_END_SW", STATE_WAIT_ACK_SW);
        return;
    }

    try_start_sw_dma(ctx, "next software configuration", STATE_WAIT_ACK_SW);
}

static void step_state_machine(SimContext *ctx) {
    if (!arm_hw_mode(ctx)) {
        return;
    }

    switch (ctx->state) {
        case STATE_IDLE:
            if (is_hw_mode(ctx)) {
                handle_hw_idle(ctx);
            } else {
                handle_sw_idle(ctx);
            }
            return;

        case STATE_WAIT_ACK_HW:
            if (ctx->cycle >= ctx->ack_cycle) {
                complete_and_return_idle(ctx, "intr: dma_cfg_done, return IDLE");
            }
            return;

        case STATE_WAIT_ACK_SW:
            if (ctx->cycle >= ctx->ack_cycle) {
                if (sw_flow_enabled(ctx)) {
                    log_cycle(ctx, "dma_ack received, flow_ctl_en=1, go WAIT_PIPE_FEOF");
                    arm_next_feof_wait(ctx);
                    ctx->state = STATE_WAIT_PIPE_FEOF;
                } else {
                    log_cycle(ctx, "dma_ack received, flow_ctl_en=0, go CFG_END_SW");
                    ctx->pending_cfg_complete = 1;
                    ctx->state = STATE_CFG_END_SW;
                }
            }
            return;

        case STATE_WAIT_PIPE_FEOF:
            if (ctx->cycle < ctx->pipe_busy_until) {
                return;
            }
            if (!ctx->feof_seen && event_feof(ctx)) {
                ctx->feof_seen = 1;
                ctx->feof_cycle = ctx->cycle;
                log_cycle(ctx, "FEOF observed, start sw_flow_ctl_dly_cnt");
                return;
            }
            if (ctx->feof_seen && ctx->cycle >= ctx->feof_cycle + ctx->cfg.sw_flow_ctl_dly_num) {
                try_start_sw_dma(ctx, "flow delay done, trigger restart XDMA", STATE_WAIT_START_ACK);
            }
            return;

        case STATE_WAIT_START_ACK:
            if (ctx->cycle >= ctx->ack_cycle) {
                log_cycle(ctx, "dma_ack received, go CFG_END_SW");
                ctx->pending_cfg_complete = 1;
                ctx->state = STATE_CFG_END_SW;
            }
            return;

        case STATE_CFG_END_SW:
            if (ctx->pending_cfg_complete) {
                complete_sw_cfg(ctx);
            }
            if (ctx->sw_cfg_cnt >= ctx->cfg.sw_cfg_num) {
                complete_and_return_idle(ctx, "intr: sw_last_done, return IDLE");
                return;
            }
            try_restart_sw_burst(ctx);
            return;

        default:
            return;
    }
}

static int compute_wait_watchdog_cycles(const SimConfig *cfg) {
    int base_wait = (cfg->frame_cycles * 2) + cfg->hw_dly_num + cfg->sw_flow_ctl_dly_num +
                    cfg->pipe_busy_cycles + cfg->dma_ack_latency + cfg->initial_dma_busy_cycles + 16;
    return base_wait < 8 ? 8 : base_wait;
}

static int compute_completion_budget_cycles(const SimConfig *cfg) {
    int sw_burst_factor = cfg->sw_cfg_num < 1 ? 1 : cfg->sw_cfg_num;
    int hw_factor = cfg->hw_skip_frame_num + 2;
    int sw_factor = (cfg->sw_flow_ctl_en ? (cfg->frame_cycles * 2) : (cfg->dma_ack_latency + 4)) * sw_burst_factor;
    int budget = cfg->initial_dma_busy_cycles + (cfg->frame_cycles * hw_factor) + sw_factor +
                 cfg->hw_dly_num + cfg->dma_ack_latency * (sw_burst_factor + 2) + 64;
    return budget;
}

static int run_simulation(const SimConfig *cfg) {
    SimContext ctx;
    char error_text[MAX_TEXT];
    int watchdog_limit;
    int completion_budget;
    int stall_cycles = 0;

    if (validate_config(cfg, error_text, sizeof(error_text)) != 0) {
        fprintf(stderr, "Invalid config: %s\n", error_text);
        return 1;
    }

    print_banner(cfg);
    init_context(&ctx, cfg);
    watchdog_limit = cfg->watchdog_cycles == 0 ? compute_wait_watchdog_cycles(cfg) : cfg->watchdog_cycles;
    completion_budget = compute_completion_budget_cycles(cfg);

    ctx.cycle = 0;
    while (!ctx.completed && ctx.state != STATE_ERROR) {
        SimState prev_state = ctx.state;
        int prev_sw_cfg_cnt = ctx.sw_cfg_cnt;
        int prev_pending_cfg = ctx.pending_cfg_complete;
        int prev_skipped_triggers = ctx.skipped_triggers;
        int prev_hw_delay_pending = ctx.hw_delay_pending;
        int prev_feof_seen = ctx.feof_seen;
        int prev_ack_cycle = ctx.ack_cycle;
        int prev_due_cycle = ctx.due_cycle;
        bool progressed;

        if (ctx.cycle > completion_budget) {
            set_error(&ctx, "completion budget exceeded before returning to IDLE");
            break;
        }

        step_state_machine(&ctx);

        progressed = ctx.completed || ctx.state == STATE_ERROR || ctx.state != prev_state ||
                     ctx.sw_cfg_cnt != prev_sw_cfg_cnt || ctx.pending_cfg_complete != prev_pending_cfg ||
                     ctx.skipped_triggers != prev_skipped_triggers || ctx.hw_delay_pending != prev_hw_delay_pending ||
                     ctx.feof_seen != prev_feof_seen || ctx.ack_cycle != prev_ack_cycle ||
                     ctx.due_cycle != prev_due_cycle;

        if (progressed) {
            stall_cycles = 0;
        } else {
            stall_cycles++;
            if (stall_cycles > watchdog_limit) {
                set_error(&ctx, "watchdog timeout: no state-machine progress");
                break;
            }
        }

        if (!ctx.completed && ctx.state != STATE_ERROR) {
            ctx.cycle++;
        }
    }

    if (!ctx.completed && ctx.state != STATE_ERROR) {
        snprintf(ctx.result, sizeof(ctx.result), "simulation ended without completion");
        log_cycle(&ctx, ctx.result);
        return 1;
    }

    if (ctx.completed && ctx.state != STATE_IDLE) {
        snprintf(ctx.result, sizeof(ctx.result), "completed but final state is not IDLE");
        log_cycle(&ctx, ctx.result);
        return 1;
    }

    if (ctx.completed) {
        char text[MAX_TEXT];
        snprintf(text, sizeof(text), "final state check passed: %s", state_name(ctx.state));
        log_cycle(&ctx, text);
    }

    printf("result: %s\n", ctx.result);
    return ctx.completed ? 0 : 1;
}

static void set_default_config(SimConfig *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->frame_cycles = 16;
    cfg->dma_ack_latency = 3;
    cfg->initial_dma_busy_cycles = 0;
    cfg->watchdog_cycles = 0;

    cfg->reg_mode_hw = 0;
    cfg->hw_cfg_done = 1;
    cfg->hw_dly_num = 2;
    cfg->hw_cfg_done_max_idx = 2;
    cfg->hw_skip_frame_num = 1;
    cfg->hw_trigger_source = TRIGGER_FSYNC;
    cfg->hardware_waits_ack = 1;

    cfg->sw_trigger = 0;
    cfg->sw_flow_ctl_en = 0;
    cfg->sw_cfg_num = 3;
    cfg->sw_flow_ctl_dly_num = 2;
    cfg->pipe_busy_cycles = 1;
}

static int parse_int_arg(const char *name, const char *value, int *target) {
    char *endptr = NULL;
    long parsed = strtol(value, &endptr, 10);
    if (value[0] == '\0' || (endptr != NULL && *endptr != '\0')) {
        fprintf(stderr, "Invalid value for %s: %s\n", name, value);
        return -1;
    }
    *target = (int)parsed;
    return 0;
}

static int parse_reg_mode(const char *text, int *reg_mode_hw) {
    if (strcmp(text, "hw") == 0) {
        *reg_mode_hw = 1;
        return 0;
    }
    if (strcmp(text, "sw") == 0) {
        *reg_mode_hw = 0;
        return 0;
    }
    return -1;
}

static int parse_trigger(const char *text, TriggerSource *source) {
    if (strcmp(text, "fsync") == 0) {
        *source = TRIGGER_FSYNC;
        return 0;
    }
    if (strcmp(text, "teof") == 0) {
        *source = TRIGGER_TEOF;
        return 0;
    }
    return -1;
}

static void print_usage(const char *program) {
    printf("Usage:\n");
    printf("  %s                 Run built-in demo cases\n", program);
    printf("  %s [options]\n", program);
    printf("  %s --self-test\n", program);
    printf("\nCommon options:\n");
    printf("  --frame-cycles N\n");
    printf("  --dma-ack-latency N\n");
    printf("  --initial-dma-busy-cycles N\n");
    printf("  --watchdog-cycles N   (0 uses auto derived threshold)\n");
    printf("  --reg-mode hw|sw\n");
    printf("\nHardware options:\n");
    printf("  --hw-cfg-done 0|1\n");
    printf("  --hw-delay N\n");
    printf("  --hw-cfg-done-max-idx N\n");
    printf("  --hw-skip-frame N\n");
    printf("  --trigger fsync|teof\n");
    printf("  --no-wait-ack      Use A500-like path\n");
    printf("\nSoftware options:\n");
    printf("  --sw-trigger 0|1\n");
    printf("  --flow-ctl 0|1\n");
    printf("  --sw-cfg-num N\n");
    printf("  --flow-delay N\n");
    printf("  --pipe-busy-cycles N\n");
}

static int parse_args(int argc, char **argv, SimConfig *cfg) {
    int i;

    set_default_config(cfg);

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--no-wait-ack") == 0) {
            cfg->hardware_waits_ack = 0;
            continue;
        }
        if (i + 1 >= argc) {
            fprintf(stderr, "Missing value after %s\n", argv[i]);
            return -1;
        }
        if (strcmp(argv[i], "--frame-cycles") == 0) {
            if (parse_int_arg("--frame-cycles", argv[++i], &cfg->frame_cycles) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--reg-mode") == 0) {
            if (parse_reg_mode(argv[++i], &cfg->reg_mode_hw) != 0) {
                fprintf(stderr, "Unknown reg-mode: %s\n", argv[i]);
                return -1;
            }
        } else if (strcmp(argv[i], "--dma-ack-latency") == 0) {
            if (parse_int_arg("--dma-ack-latency", argv[++i], &cfg->dma_ack_latency) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--initial-dma-busy-cycles") == 0) {
            if (parse_int_arg("--initial-dma-busy-cycles", argv[++i], &cfg->initial_dma_busy_cycles) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--watchdog-cycles") == 0) {
            if (parse_int_arg("--watchdog-cycles", argv[++i], &cfg->watchdog_cycles) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--hw-cfg-done") == 0) {
            if (parse_int_arg("--hw-cfg-done", argv[++i], &cfg->hw_cfg_done) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--hw-delay") == 0) {
            if (parse_int_arg("--hw-delay", argv[++i], &cfg->hw_dly_num) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--hw-cfg-done-max-idx") == 0) {
            if (parse_int_arg("--hw-cfg-done-max-idx", argv[++i], &cfg->hw_cfg_done_max_idx) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--hw-skip-frame") == 0) {
            if (parse_int_arg("--hw-skip-frame", argv[++i], &cfg->hw_skip_frame_num) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--trigger") == 0) {
            if (parse_trigger(argv[++i], &cfg->hw_trigger_source) != 0) {
                fprintf(stderr, "Unknown trigger source: %s\n", argv[i]);
                return -1;
            }
        } else if (strcmp(argv[i], "--sw-cfg-num") == 0) {
            if (parse_int_arg("--sw-cfg-num", argv[++i], &cfg->sw_cfg_num) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--sw-trigger") == 0) {
            if (parse_int_arg("--sw-trigger", argv[++i], &cfg->sw_trigger) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--flow-ctl") == 0) {
            if (parse_int_arg("--flow-ctl", argv[++i], &cfg->sw_flow_ctl_en) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--flow-delay") == 0) {
            if (parse_int_arg("--flow-delay", argv[++i], &cfg->sw_flow_ctl_dly_num) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--pipe-busy-cycles") == 0) {
            if (parse_int_arg("--pipe-busy-cycles", argv[++i], &cfg->pipe_busy_cycles) != 0) {
                return -1;
            }
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return -1;
        }
    }

    return 0;
}

static int run_demo_cases(void) {
    SimConfig cfg;
    int rc = 0;

    set_default_config(&cfg);
    cfg.reg_mode_hw = 1;
    cfg.sw_trigger = 0;
    rc |= run_simulation(&cfg);

    set_default_config(&cfg);
    cfg.reg_mode_hw = 0;
    cfg.sw_trigger = 1;
    cfg.sw_flow_ctl_en = 0;
    cfg.sw_cfg_num = 4;
    rc |= run_simulation(&cfg);

    set_default_config(&cfg);
    cfg.reg_mode_hw = 0;
    cfg.sw_trigger = 1;
    cfg.sw_flow_ctl_en = 1;
    cfg.sw_cfg_num = 3;
    cfg.sw_flow_ctl_dly_num = 2;
    cfg.pipe_busy_cycles = 1;
    rc |= run_simulation(&cfg);

    return rc;
}

static int run_idle_return_self_tests(void) {
    SimConfig cfg;
    int failures = 0;
    int rc;

    printf("\n=== Self-test: start at IDLE and return to IDLE ===\n");

    set_default_config(&cfg);
    cfg.reg_mode_hw = 1;
    cfg.sw_trigger = 0;
    rc = run_simulation(&cfg);
    printf("[self-test] hw-branch: %s\n", rc == 0 ? "PASS" : "FAIL");
    failures += (rc != 0);

    set_default_config(&cfg);
    cfg.reg_mode_hw = 0;
    cfg.sw_trigger = 1;
    cfg.sw_flow_ctl_en = 0;
    cfg.sw_cfg_num = 2;
    rc = run_simulation(&cfg);
    printf("[self-test] sw-no-flow: %s\n", rc == 0 ? "PASS" : "FAIL");
    failures += (rc != 0);

    set_default_config(&cfg);
    cfg.reg_mode_hw = 0;
    cfg.sw_trigger = 1;
    cfg.sw_flow_ctl_en = 1;
    cfg.sw_cfg_num = 2;
    cfg.sw_flow_ctl_dly_num = 1;
    rc = run_simulation(&cfg);
    printf("[self-test] sw-flow: %s\n", rc == 0 ? "PASS" : "FAIL");
    failures += (rc != 0);

    if (failures != 0) {
        printf("[self-test] FAILED: %d case(s)\n", failures);
        return 1;
    }

    printf("[self-test] ALL PASSED\n");
    return 0;
}

int main(int argc, char **argv) {
    SimConfig cfg;

    if (argc == 1) {
        print_usage(argv[0]);
        printf("\nRunning demo cases...\n");
        return run_demo_cases();
    }

    if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0) {
        print_usage(argv[0]);
        return 0;
    }

    if (strcmp(argv[1], "--self-test") == 0) {
        return run_idle_return_self_tests();
    }

    if (parse_args(argc, argv, &cfg) != 0) {
        print_usage(argv[0]);
        return 1;
    }

    return run_simulation(&cfg);
}
