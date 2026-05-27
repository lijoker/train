#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "fsm_interrupt_demo.h"

static void test_irq_not_pending_should_not_enter_isr(void) {
    InterruptFSM fsm = {STATE_IDLE, false};
    InterruptServiceStats stats = {0};
    bool entered = simulate_interrupt_and_handle(&fsm, &stats, "test_no_irq", 3);

    assert(!entered);
    assert(!fsm.irq_level);
    assert(stats.enter_count == 0U);
    assert(stats.handled_steps == 0U);
    assert(stats.clear_count == 0U);
}

static void test_irq_pending_should_enter_handle_and_clear(void) {
    InterruptFSM fsm = {STATE_IDLE, true};
    InterruptServiceStats stats = {0};
    bool entered = simulate_interrupt_and_handle(&fsm, &stats, "test_irq", 4);

    assert(entered);
    assert(!fsm.irq_level);
    assert(stats.enter_count == 1U);
    assert(stats.handled_steps == 4U);
    assert(stats.clear_count == 1U);
}

static void test_non_positive_work_steps_should_fallback_to_one(void) {
    InterruptFSM fsm = {STATE_IDLE, true};
    InterruptServiceStats stats = {0};
    bool entered = simulate_interrupt_and_handle(&fsm, &stats, "test_work_step", 0);

    assert(entered);
    assert(!fsm.irq_level);
    assert(stats.enter_count == 1U);
    assert(stats.handled_steps == 1U);
    assert(stats.clear_count == 1U);
}

int main(void) {
    test_irq_not_pending_should_not_enter_isr();
    test_irq_pending_should_enter_handle_and_clear();
    test_non_positive_work_steps_should_fallback_to_one();

    printf("fsm_interrupt_api_test: all tests passed.\n");
    return 0;
}
