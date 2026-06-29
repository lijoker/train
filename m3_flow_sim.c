#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * M3 flow simulator
 *
 * This program is a software model derived from the three flowcharts:
 *   1) hardware mode
 *   2) software mode without flow control
 *   3) software mode with flow control
 *
 * Assumptions used by the simulator:
 *   - time advances in discrete cycles
 *   - fsync happens at the first cycle of each frame
 *   - teof happens in the middle of each frame
 *   - feof happens at the last cycle of each frame
 *   - one DMA request completes after dma_ack_latency cycles
 *   - sw_trigger starts a burst of sw_cfg_num configurations
 *   - hw_skip_frame_num skips the first N trigger opportunities
 *   - hw_cfg_done_max_idx is reported as metadata, matching the diagram note
 */

#define MAX_TEXT 128

typedef enum {
    MODE_HW = 0,
    MODE_SW_NO_FLOW = 1,
    MODE_SW_FLOW = 2
} SimMode;

typedef enum {
    TRIGGER_FSYNC = 0,
    TRIGGER_TEOF = 1
} TriggerSource;

typedef enum {
    STATE_IDLE = 0,
    STATE_WAIT_HW_TRIGGER,
    STATE_WAIT_HW_DELAY,
    STATE_WAIT_ACK_HW,
    STATE_WAIT_ACK_SW,
    STATE_WAIT_PRE_FEOF,
    STATE_WAIT_START_ACK,
    STATE_CFG_END_SW,
    STATE_DONE,
    STATE_ERROR
} SimState;

typedef struct {
    SimMode mode;
    int frame_cycles;
    int dma_ack_latency;
    int initial_dma_busy_cycles;
    int max_cycles;

    int hw_cfg_done;
    int hw_dly_num;
    int hw_cfg_done_max_idx;
    int hw_skip_frame_num;
    TriggerSource hw_trigger_source;
    int hardware_waits_ack;

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
    int feof_seen;
    char result[MAX_TEXT];
} SimContext;

static const char *mode_name(SimMode mode) {
    switch (mode) {
        case MODE_HW:
            return "hw";
        case MODE_SW_NO_FLOW:
            return "sw0";
        case MODE_SW_FLOW:
            return "sw1";
        default:
            return "unknown";
    }
}

static const char *state_name(SimState state) {
    switch (state) {
        case STATE_IDLE:
            return "IDLE";
        case STATE_WAIT_HW_TRIGGER:
            return "WAIT_HW_TRIGGER";
        case STATE_WAIT_HW_DELAY:
            return "WAIT_HW_DELAY";
        case STATE_WAIT_ACK_HW:
            return "WAIT_ACK_HW";
        case STATE_WAIT_ACK_SW:
            return "WAIT_ACK_SW";
        case STATE_WAIT_PRE_FEOF:
            return "WAIT_PRE_FEOF";
        case STATE_WAIT_START_ACK:
            return "WAIT_START_ACK";
        case STATE_CFG_END_SW:
            return "CFG_END_SW";
        case STATE_DONE:
            return "DONE";
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

static void set_done(SimContext *ctx, const char *message) {
    ctx->state = STATE_DONE;
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
    if (cfg->max_cycles < 1) {
        snprintf(error_text, error_text_size, "max_cycles must be >= 1");
        return -1;
    }
    if (cfg->mode == MODE_HW) {
        if (cfg->hw_dly_num < 0) {
            snprintf(error_text, error_text_size, "hw_dly_num must be >= 0");
            return -1;
        }
        if (cfg->hw_cfg_done_max_idx < 0) {
            snprintf(error_text, error_text_size, "hw_cfg_done_max_idx must be >= 0");
            return -1;
        }
    } else {
        if (cfg->sw_cfg_num < 1) {
            snprintf(error_text, error_text_size, "sw_cfg_num must be >= 1");
            return -1;
        }
        if (cfg->sw_flow_ctl_dly_num < 0) {
            snprintf(error_text, error_text_size, "sw_flow_ctl_dly_num must be >= 0");
            return -1;
        }
    }
    return 0;
}

static void print_banner(const SimConfig *cfg) {
    printf("\n=== M3 flow simulation: mode=%s ===\n", mode_name(cfg->mode));
    printf("frame_cycles=%d, dma_ack_latency=%d, initial_dma_busy_cycles=%d, max_cycles=%d\n",
           cfg->frame_cycles,
           cfg->dma_ack_latency,
           cfg->initial_dma_busy_cycles,
           cfg->max_cycles);
    if (cfg->mode == MODE_HW) {
        printf("hw_cfg_done=%d, hw_dly_num=%d, hw_cfg_done_max_idx=%d, valid_hw_cfg_bits=%d, "
               "hw_skip_frame_num=%d, hw_trigger=%s, hardware_waits_ack=%d\n",
               cfg->hw_cfg_done,
               cfg->hw_dly_num,
               cfg->hw_cfg_done_max_idx,
               cfg->hw_cfg_done_max_idx + 1,
               cfg->hw_skip_frame_num,
               trigger_name(cfg->hw_trigger_source),
               cfg->hardware_waits_ack);
    } else {
        printf("sw_cfg_num=%d, sw_flow_ctl_dly_num=%d, pipe_busy_cycles=%d\n",
               cfg->sw_cfg_num,
               cfg->sw_flow_ctl_dly_num,
               cfg->pipe_busy_cycles);
    }
}

static void init_context(SimContext *ctx, const SimConfig *cfg) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;
    ctx->state = cfg->mode == MODE_HW ? STATE_WAIT_HW_TRIGGER : STATE_IDLE;
    ctx->dma_busy_until = cfg->initial_dma_busy_cycles;
    snprintf(ctx->result, sizeof(ctx->result), "running");
}

static void try_start_hw_dma(SimContext *ctx) {
    if (dma_busy(ctx)) {
        set_error(ctx, "intr: dma_busy_error in hardware mode");
        return;
    }

    launch_dma(ctx, "hardware flow");
    if (ctx->cfg.hardware_waits_ack) {
        ctx->state = STATE_WAIT_ACK_HW;
        return;
    }

    set_done(ctx, "A500 path: do not wait dma_ack, return IDLE");
}

static void step_hw(SimContext *ctx) {
    bool trigger_hit = false;

    if (!ctx->hw_armed) {
        ctx->hw_armed = 1;
        if (!ctx->cfg.hw_cfg_done) {
            set_error(ctx, "intr: hw_cfg_done_error");
            return;
        }
        if (ctx->cfg.hw_dly_num > ctx->cfg.frame_cycles) {
            set_error(ctx, "intr: hw_dly_num_error");
            return;
        }

        char text[MAX_TEXT];
        snprintf(text,
                 sizeof(text),
                 "hardware mode armed, valid cfg_done bits = [0..%d]",
                 ctx->cfg.hw_cfg_done_max_idx);
        log_cycle(ctx, text);
    }

    switch (ctx->state) {
        case STATE_WAIT_HW_TRIGGER:
            trigger_hit = (ctx->cfg.hw_trigger_source == TRIGGER_FSYNC) ? event_fsync(ctx) : event_teof(ctx);
            if (!trigger_hit) {
                return;
            }

            if (ctx->skipped_triggers < ctx->cfg.hw_skip_frame_num) {
                ctx->skipped_triggers++;
                char text[MAX_TEXT];
                snprintf(text,
                         sizeof(text),
                         "skip trigger #%d because hw_skip_frame_num=%d",
                         ctx->skipped_triggers,
                         ctx->cfg.hw_skip_frame_num);
                log_cycle(ctx, text);
                return;
            }

            if (ctx->cfg.hw_dly_num == 0) {
                log_cycle(ctx, "trigger detected, no extra delay");
                try_start_hw_dma(ctx);
                return;
            }

            ctx->due_cycle = ctx->cycle + ctx->cfg.hw_dly_num;
            ctx->state = STATE_WAIT_HW_DELAY;
            {
                char text[MAX_TEXT];
                snprintf(text,
                         sizeof(text),
                         "trigger detected, wait hw_dly_num=%d cycles until cycle %d",
                         ctx->cfg.hw_dly_num,
                         ctx->due_cycle);
                log_cycle(ctx, text);
            }
            return;

        case STATE_WAIT_HW_DELAY:
            if (ctx->cycle < ctx->due_cycle) {
                return;
            }
            try_start_hw_dma(ctx);
            return;

        case STATE_WAIT_ACK_HW:
            if (ctx->cycle >= ctx->ack_cycle) {
                set_done(ctx, "intr: dma_cfg_done");
            }
            return;

        default:
            return;
    }
}

static void try_start_sw_dma(SimContext *ctx, const char *why, SimState wait_state) {
    if (dma_busy(ctx)) {
        set_error(ctx, "return error: dma busy");
        return;
    }

    launch_dma(ctx, why);
    ctx->state = wait_state;
}

static void step_sw_no_flow(SimContext *ctx) {
    switch (ctx->state) {
        case STATE_IDLE:
            try_start_sw_dma(ctx, "sw flow without flow-control", STATE_WAIT_ACK_SW);
            return;

        case STATE_WAIT_ACK_SW:
            if (ctx->cycle >= ctx->ack_cycle) {
                log_cycle(ctx, "dma_ack received");
                ctx->pending_cfg_complete = 1;
                ctx->state = STATE_CFG_END_SW;
            }
            return;

        case STATE_CFG_END_SW:
            if (ctx->pending_cfg_complete) {
                complete_sw_cfg(ctx);
            }
            if (ctx->sw_cfg_cnt >= ctx->cfg.sw_cfg_num) {
                set_done(ctx, "intr: sw_last_done");
                return;
            }
            ctx->state = STATE_IDLE;
            return;

        default:
            return;
    }
}

static void arm_next_feof_wait(SimContext *ctx) {
    ctx->feof_seen = 0;
    ctx->feof_cycle = -1;
    ctx->pipe_busy_until = ctx->cycle + ctx->cfg.pipe_busy_cycles;
    log_cycle(ctx, "enter WAIT_PRE_FEOF");
}

static void step_sw_flow(SimContext *ctx) {
    switch (ctx->state) {
        case STATE_IDLE:
            try_start_sw_dma(ctx, "sw flow first start", STATE_WAIT_ACK_SW);
            return;

        case STATE_WAIT_ACK_SW:
            if (ctx->cycle >= ctx->ack_cycle) {
                log_cycle(ctx, "dma_ack received for current configuration");
                ctx->pending_cfg_complete = 1;
                ctx->state = STATE_CFG_END_SW;
            }
            return;

        case STATE_CFG_END_SW:
            if (ctx->pending_cfg_complete) {
                complete_sw_cfg(ctx);
            }
            if (ctx->sw_cfg_cnt >= ctx->cfg.sw_cfg_num) {
                set_done(ctx, "intr: sw_last_done");
                return;
            }
            arm_next_feof_wait(ctx);
            ctx->state = STATE_WAIT_PRE_FEOF;
            return;

        case STATE_WAIT_PRE_FEOF:
            if (ctx->cycle < ctx->pipe_busy_until) {
                return;
            }
            if (!ctx->feof_seen && event_feof(ctx)) {
                ctx->feof_seen = 1;
                ctx->feof_cycle = ctx->cycle;
                log_cycle(ctx, "FEOF observed");
                return;
            }
            if (ctx->feof_seen && ctx->cycle >= ctx->feof_cycle + ctx->cfg.sw_flow_ctl_dly_num) {
                try_start_sw_dma(ctx, "sw flow after feof delay", STATE_WAIT_START_ACK);
            }
            return;

        case STATE_WAIT_START_ACK:
            if (ctx->cycle >= ctx->ack_cycle) {
                log_cycle(ctx, "dma_ack received after flow-controlled restart");
                ctx->pending_cfg_complete = 1;
                ctx->state = STATE_CFG_END_SW;
            }
            return;

        default:
            return;
    }
}

static int run_simulation(const SimConfig *cfg) {
    SimContext ctx;
    char error_text[MAX_TEXT];

    if (validate_config(cfg, error_text, sizeof(error_text)) != 0) {
        fprintf(stderr, "Invalid config: %s\n", error_text);
        return 1;
    }

    print_banner(cfg);
    init_context(&ctx, cfg);

    for (ctx.cycle = 0; ctx.cycle < cfg->max_cycles; ++ctx.cycle) {
        if (ctx.state == STATE_DONE || ctx.state == STATE_ERROR) {
            break;
        }

        switch (cfg->mode) {
            case MODE_HW:
                step_hw(&ctx);
                break;
            case MODE_SW_NO_FLOW:
                step_sw_no_flow(&ctx);
                break;
            case MODE_SW_FLOW:
                step_sw_flow(&ctx);
                break;
            default:
                fprintf(stderr, "Unsupported mode\n");
                return 1;
        }
    }

    if (ctx.state != STATE_DONE && ctx.state != STATE_ERROR) {
        snprintf(ctx.result, sizeof(ctx.result), "timeout at cycle %d", cfg->max_cycles);
        log_cycle(&ctx, ctx.result);
        return 1;
    }

    printf("result: %s\n", ctx.result);
    return ctx.state == STATE_DONE ? 0 : 1;
}

static void set_default_config(SimConfig *cfg, SimMode mode) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->mode = mode;
    cfg->frame_cycles = 16;
    cfg->dma_ack_latency = 3;
    cfg->initial_dma_busy_cycles = 0;
    cfg->max_cycles = 128;

    cfg->hw_cfg_done = 1;
    cfg->hw_dly_num = 2;
    cfg->hw_cfg_done_max_idx = 2;
    cfg->hw_skip_frame_num = 1;
    cfg->hw_trigger_source = TRIGGER_FSYNC;
    cfg->hardware_waits_ack = 1;

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

static int parse_mode(const char *text, SimMode *mode) {
    if (strcmp(text, "hw") == 0) {
        *mode = MODE_HW;
        return 0;
    }
    if (strcmp(text, "sw0") == 0) {
        *mode = MODE_SW_NO_FLOW;
        return 0;
    }
    if (strcmp(text, "sw1") == 0) {
        *mode = MODE_SW_FLOW;
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
    printf("  %s <mode> [options]\n", program);
    printf("\nModes:\n");
    printf("  hw   hardware mode\n");
    printf("  sw0  software mode, sw_flow_ctl_en=0\n");
    printf("  sw1  software mode, sw_flow_ctl_en=1\n");
    printf("\nCommon options:\n");
    printf("  --frame-cycles N\n");
    printf("  --dma-ack-latency N\n");
    printf("  --initial-dma-busy-cycles N\n");
    printf("  --max-cycles N\n");
    printf("\nHardware options:\n");
    printf("  --hw-cfg-done 0|1\n");
    printf("  --hw-delay N\n");
    printf("  --hw-cfg-done-max-idx N\n");
    printf("  --hw-skip-frame N\n");
    printf("  --trigger fsync|teof\n");
    printf("  --no-wait-ack      Use A500-like path\n");
    printf("\nSoftware options:\n");
    printf("  --sw-cfg-num N\n");
    printf("  --flow-delay N\n");
    printf("  --pipe-busy-cycles N\n");
}

static int parse_args(int argc, char **argv, SimConfig *cfg) {
    SimMode mode;
    int i;

    if (parse_mode(argv[1], &mode) != 0) {
        fprintf(stderr, "Unknown mode: %s\n", argv[1]);
        return -1;
    }
    set_default_config(cfg, mode);

    for (i = 2; i < argc; ++i) {
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
        } else if (strcmp(argv[i], "--dma-ack-latency") == 0) {
            if (parse_int_arg("--dma-ack-latency", argv[++i], &cfg->dma_ack_latency) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--initial-dma-busy-cycles") == 0) {
            if (parse_int_arg("--initial-dma-busy-cycles", argv[++i], &cfg->initial_dma_busy_cycles) != 0) {
                return -1;
            }
        } else if (strcmp(argv[i], "--max-cycles") == 0) {
            if (parse_int_arg("--max-cycles", argv[++i], &cfg->max_cycles) != 0) {
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

    set_default_config(&cfg, MODE_HW);
    rc |= run_simulation(&cfg);

    set_default_config(&cfg, MODE_SW_NO_FLOW);
    cfg.sw_cfg_num = 4;
    rc |= run_simulation(&cfg);

    set_default_config(&cfg, MODE_SW_FLOW);
    cfg.sw_cfg_num = 3;
    cfg.sw_flow_ctl_dly_num = 2;
    cfg.pipe_busy_cycles = 1;
    rc |= run_simulation(&cfg);

    return rc;
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

    if (parse_args(argc, argv, &cfg) != 0) {
        print_usage(argv[0]);
        return 1;
    }

    return run_simulation(&cfg);
}
