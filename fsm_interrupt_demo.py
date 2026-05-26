"""State-machine demo for interrupt response testing.

This demo implements the FSM from the provided diagram and runs
cycle-by-cycle scenarios to verify interrupt behavior.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import List


class State(Enum):
    IDLE = auto()
    WAIT_ACK_HW = auto()
    WAIT_ACK_SW = auto()
    WAIT_PIPE_FEOF = auto()
    WAIT_START_ACK = auto()
    CFG_END_SW = auto()


@dataclass(frozen=True)
class Inputs:
    reg_mode: bool = False
    hw_trigger: bool = False
    sw_trigger: bool = False
    cfg_done: bool = False
    dma_ack: bool = False
    flow_ctl_en: bool = False
    pipe_busy: bool = False
    fsync: bool = False
    sw_cfg_cnt: int = 0
    reg_sw_cfg_num: int = 1
    sw_flow_ctl_dly_cnt: int = 0
    reg_sw_flow_ctl_dly_num: int = 0


@dataclass
class Cycle:
    name: str
    inputs: Inputs
    clear_irq_before_step: bool = False


@dataclass
class StepResult:
    prev_state: State
    next_state: State
    irq_rise: bool
    irq_level: bool
    reason: str


class InterruptFSM:
    """Cycle-accurate FSM with a level interrupt output."""

    def __init__(self) -> None:
        self.state = State.IDLE
        self.irq_level = False

    def clear_irq(self) -> None:
        self.irq_level = False

    def step(self, sig: Inputs) -> StepResult:
        prev = self.state
        nxt = prev
        reason = "hold"
        irq_rise = False

        if prev == State.IDLE:
            if sig.reg_mode and sig.hw_trigger and sig.cfg_done:
                nxt = State.WAIT_ACK_HW
                reason = "reg_mode & hw_trigger & cfg_done"
            elif (not sig.reg_mode) and sig.sw_trigger:
                nxt = State.WAIT_ACK_SW
                reason = "~reg_mode & sw_trigger"
            else:
                reason = "idle wait trigger"

        elif prev == State.WAIT_ACK_HW:
            if sig.dma_ack:
                nxt = State.IDLE
                reason = "dma_ack -> HW flow completed"
                irq_rise = self._raise_irq()
            else:
                reason = "~dma_ack"

        elif prev == State.WAIT_ACK_SW:
            if not sig.dma_ack:
                reason = "~dma_ack"
            elif sig.flow_ctl_en:
                nxt = State.WAIT_PIPE_FEOF
                reason = "dma_ack & flow_ctl_en"
            else:
                nxt = State.CFG_END_SW
                reason = "dma_ack & ~flow_ctl_en"

        elif prev == State.WAIT_PIPE_FEOF:
            if sig.pipe_busy:
                reason = "pipe_busy"
            elif (not sig.dma_ack) and (
                sig.sw_flow_ctl_dly_cnt >= sig.reg_sw_flow_ctl_dly_num
            ):
                nxt = State.WAIT_START_ACK
                reason = "~pipe_busy & ~dma_ack & delay_met"
            else:
                reason = "wait pipe empty / delay"

        elif prev == State.WAIT_START_ACK:
            if sig.dma_ack:
                nxt = State.CFG_END_SW
                reason = "dma_ack"
            else:
                reason = "~dma_ack"

        elif prev == State.CFG_END_SW:
            if sig.sw_cfg_cnt >= sig.reg_sw_cfg_num:
                nxt = State.IDLE
                reason = "sw_cfg_cnt >= reg_sw_cfg_num"
                irq_rise = self._raise_irq()
            elif sig.sw_cfg_cnt < sig.reg_sw_cfg_num and (not sig.dma_ack) and sig.fsync:
                nxt = State.WAIT_ACK_SW
                reason = "sw_cfg_cnt < reg_sw_cfg_num & ~dma_ack & fsync"
            else:
                reason = "wait next cfg or completion"

        self.state = nxt
        return StepResult(prev, nxt, irq_rise, self.irq_level, reason)

    def _raise_irq(self) -> bool:
        if self.irq_level:
            return False
        self.irq_level = True
        return True


def _sig(**kwargs: object) -> Inputs:
    """Create input vector with defaults."""
    return Inputs(**kwargs)


def run_scenario(name: str, cycles: List[Cycle], expected_irq_rise_cycles: List[int]) -> None:
    fsm = InterruptFSM()
    irq_rise_cycles: List[int] = []

    print(f"\n=== Scenario: {name} ===")
    print(
        "cycle | event                  | prev_state      -> next_state      | "
        "irq | reason"
    )
    print("-" * 95)

    for cycle_idx, cycle in enumerate(cycles):
        if cycle.clear_irq_before_step:
            fsm.clear_irq()

        result = fsm.step(cycle.inputs)
        if result.irq_rise:
            irq_rise_cycles.append(cycle_idx)

        irq_mark = "R" if result.irq_rise else ("1" if result.irq_level else "0")
        print(
            f"{cycle_idx:>5} | {cycle.name:<22} | "
            f"{result.prev_state.name:<15} -> {result.next_state.name:<15} | "
            f"{irq_mark:^3} | {result.reason}"
        )

    if irq_rise_cycles != expected_irq_rise_cycles:
        raise AssertionError(
            f"{name}: irq rise cycles mismatch, expected={expected_irq_rise_cycles}, "
            f"actual={irq_rise_cycles}"
        )
    print(f"[PASS] IRQ rise cycles = {irq_rise_cycles}")


def scenario_hw_trigger() -> None:
    cycles = [
        Cycle("idle", _sig()),
        Cycle("hw trigger", _sig(reg_mode=True, hw_trigger=True, cfg_done=True)),
        Cycle("wait dma ack", _sig(reg_mode=True, cfg_done=True)),
        Cycle("dma ack", _sig(reg_mode=True, cfg_done=True, dma_ack=True)),
        Cycle("sw clear irq", _sig(), clear_irq_before_step=True),
    ]
    run_scenario("HW trigger flow", cycles, expected_irq_rise_cycles=[3])


def scenario_sw_no_flow_control() -> None:
    cycles = [
        Cycle("idle", _sig()),
        Cycle("sw trigger", _sig(sw_trigger=True, reg_mode=False)),
        Cycle("wait dma ack", _sig(reg_mode=False)),
        Cycle("dma ack, no flow", _sig(reg_mode=False, dma_ack=True, flow_ctl_en=False)),
        Cycle(
            "all cfg done",
            _sig(reg_mode=False, sw_cfg_cnt=1, reg_sw_cfg_num=1, dma_ack=False),
        ),
        Cycle("sw clear irq", _sig(), clear_irq_before_step=True),
    ]
    run_scenario("SW flow (no flow control)", cycles, expected_irq_rise_cycles=[4])


def scenario_sw_with_flow_control_multi_cfg() -> None:
    cycles = [
        Cycle("idle", _sig(reg_sw_cfg_num=2)),
        Cycle("sw trigger cfg#1", _sig(sw_trigger=True, reg_mode=False, reg_sw_cfg_num=2)),
        Cycle(
            "dma ack + flow ctl",
            _sig(reg_mode=False, dma_ack=True, flow_ctl_en=True, reg_sw_cfg_num=2),
        ),
        Cycle("pipe busy", _sig(reg_mode=False, pipe_busy=True, reg_sw_cfg_num=2)),
        Cycle(
            "pipe empty + delay met",
            _sig(
                reg_mode=False,
                pipe_busy=False,
                dma_ack=False,
                sw_flow_ctl_dly_cnt=2,
                reg_sw_flow_ctl_dly_num=2,
                reg_sw_cfg_num=2,
            ),
        ),
        Cycle("start ack", _sig(reg_mode=False, dma_ack=True, reg_sw_cfg_num=2)),
        Cycle(
            "fsync next cfg",
            _sig(reg_mode=False, sw_cfg_cnt=1, reg_sw_cfg_num=2, dma_ack=False, fsync=True),
        ),
        Cycle("cfg#2 dma ack", _sig(reg_mode=False, dma_ack=True, flow_ctl_en=False, reg_sw_cfg_num=2)),
        Cycle("all cfg done", _sig(reg_mode=False, sw_cfg_cnt=2, reg_sw_cfg_num=2)),
        Cycle("sw clear irq", _sig(), clear_irq_before_step=True),
    ]
    run_scenario("SW flow (flow control, multi cfg)", cycles, expected_irq_rise_cycles=[8])


def main() -> None:
    scenario_hw_trigger()
    scenario_sw_no_flow_control()
    scenario_sw_with_flow_control_multi_cfg()
    print("\nAll demo scenarios passed.")


if __name__ == "__main__":
    main()
