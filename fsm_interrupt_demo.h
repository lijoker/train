#ifndef FSM_INTERRUPT_DEMO_H
#define FSM_INTERRUPT_DEMO_H

#include <stdbool.h>

typedef enum {
    STATE_IDLE = 0,
    STATE_WAIT_ACK_HW,
    STATE_WAIT_ACK_SW,
    STATE_WAIT_PIPE_FEOF,
    STATE_WAIT_START_ACK,
    STATE_CFG_END_SW
} State;

typedef struct {
    State state;
    bool irq_level;
} InterruptFSM;

typedef struct {
    unsigned int enter_count;
    unsigned int handled_steps;
    unsigned int clear_count;
} InterruptServiceStats;

bool simulate_interrupt_and_handle(InterruptFSM *fsm, InterruptServiceStats *stats,
                                   const char *caller_name, int work_steps);

#endif
