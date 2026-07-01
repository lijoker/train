# M3 flow simulator

`m3_flow_sim.c` is a standalone C simulator derived from the three flowcharts.

The simulator now treats hardware path, software path, and software flow-control path as **branches inside one unified state machine**, not as three separate modes.
The key control bits are:

- `reg_mode`: `hw` or `sw`
- `sw_trigger`: software trigger enable
- `sw_flow_ctl_en`: software flow-control branch enable

The current implementation is aligned to the unified state-machine diagram, with these main states:

- `IDLE`
- `WAIT_ACK_HW`
- `WAIT_ACK_SW`
- `WAIT_PIPE_FEOF`
- `WAIT_START_ACK`
- `CFG_END_SW`

Hardware trigger detection and `hw_dly_num` waiting are both handled inside `IDLE`, which matches the idea that the hardware branch only leaves `IDLE` after its trigger condition is satisfied.
For completion, the simulator now requires a closed loop: it starts in `IDLE` and must return to `IDLE`.
Instead of relying on a fixed `max_cycles` loop bound, completion is now checked with:

- a progress watchdog (`watchdog_cycles`, or auto-derived when set to `0`)
- a scenario-derived completion budget based on frame/ack/flow-control parameters

## Build

```bash
gcc -std=c11 -Wall -Wextra -O2 m3_flow_sim.c -o m3_flow_sim
```

## Run demo cases

If no argument is given, the program prints help and then runs three built-in branch examples:

```bash
./m3_flow_sim
```

Run built-in self-tests that verify "start in IDLE and end in IDLE":

```bash
./m3_flow_sim --self-test
```

## Custom simulation

### 1. Hardware branch

```bash
./m3_flow_sim \
  --reg-mode hw \
  --frame-cycles 16 \
  --dma-ack-latency 3 \
  --hw-cfg-done 1 \
  --hw-delay 2 \
  --hw-cfg-done-max-idx 2 \
  --hw-skip-frame 1 \
  --trigger fsync
```

If you want the A500-like path that does not wait for `dma_ack`:

```bash
./m3_flow_sim --reg-mode hw --no-wait-ack
```

### 2. Software branch, `sw_flow_ctl_en=0`

```bash
./m3_flow_sim \
  --reg-mode sw \
  --sw-trigger 1 \
  --flow-ctl 0 \
  --sw-cfg-num 4 \
  --dma-ack-latency 3
```

### 3. Software branch, `sw_flow_ctl_en=1`

```bash
./m3_flow_sim \
  --reg-mode sw \
  --sw-trigger 1 \
  --flow-ctl 1 \
  --sw-cfg-num 3 \
  --flow-delay 2 \
  --pipe-busy-cycles 1
```

## Parameter mapping

- `--frame-cycles`: number of cycles in one frame
- `--dma-ack-latency`: how many cycles later the DMA ack arrives
- `--initial-dma-busy-cycles`: initial busy time of DMA, used to emulate a busy error path
- `--watchdog-cycles`: no-progress watchdog threshold (`0` means auto-derived from scenario timing)
- `--reg-mode hw|sw`: selects the hardware or software branch in the unified state machine
- `--hw-cfg-done`: corresponds to `hw_cfg_done`
- `--hw-delay`: corresponds to `hw_dly_num`
- `--hw-cfg-done-max-idx`: corresponds to `hw_cfg_done_max_idx`
- `--hw-skip-frame`: corresponds to `hw_skip_frame_num`
- `--trigger fsync|teof`: hardware trigger source
- `--sw-trigger`: corresponds to `sw_trigger`
- `--flow-ctl`: corresponds to `sw_flow_ctl_en`
- `--sw-cfg-num`: corresponds to `sw_cfg_num`
- `--flow-delay`: corresponds to `sw_flow_ctl_dly_num`
- `--pipe-busy-cycles`: how long `pipe_busy` remains asserted before the FEOF wait logic can proceed

## Modeling assumptions

Because the flowcharts do not fully define every timing detail, the simulator uses these assumptions:

- time is modeled in discrete cycles
- `fsync` happens on the first cycle of each frame
- `teof` happens in the middle of each frame
- `feof` happens on the last cycle of each frame
- each XDMA trigger completes after `dma_ack_latency` cycles
- one `sw_trigger` starts a burst and completes `sw_cfg_num` configurations
- `hw_skip_frame_num` skips the first N trigger opportunities

This makes the program suitable for software-level behavior simulation and state trace verification.
