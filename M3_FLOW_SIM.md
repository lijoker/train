# M3 flow simulator

`m3_flow_sim.c` is a standalone C simulator derived from the three flowcharts:

1. hardware mode
2. software mode with `sw_flow_ctl_en=0`
3. software mode with `sw_flow_ctl_en=1`

The command line still offers three presets (`hw`, `sw0`, `sw1`), but internally the simulator now uses **one unified state machine**.  
The preset only changes which transitions are enabled and which parameters are meaningful.

## Build

```bash
gcc -std=c11 -Wall -Wextra -O2 m3_flow_sim.c -o m3_flow_sim
```

## Run demo cases

If no argument is given, the program prints help and then runs three built-in preset cases:

```bash
./m3_flow_sim
```

## Custom simulation

### 1. Hardware mode

```bash
./m3_flow_sim hw \
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
./m3_flow_sim hw --no-wait-ack
```

### 2. Software mode, `sw_flow_ctl_en=0`

```bash
./m3_flow_sim sw0 \
  --sw-cfg-num 4 \
  --dma-ack-latency 3
```

### 3. Software mode, `sw_flow_ctl_en=1`

```bash
./m3_flow_sim sw1 \
  --sw-cfg-num 3 \
  --flow-delay 2 \
  --pipe-busy-cycles 1
```

## Parameter mapping

- `--frame-cycles`: number of cycles in one frame
- `--dma-ack-latency`: how many cycles later the DMA ack arrives
- `--initial-dma-busy-cycles`: initial busy time of DMA, used to emulate a busy error path
- `--hw-cfg-done`: corresponds to `hw_cfg_done`
- `--hw-delay`: corresponds to `hw_dly_num`
- `--hw-cfg-done-max-idx`: corresponds to `hw_cfg_done_max_idx`
- `--hw-skip-frame`: corresponds to `hw_skip_frame_num`
- `--trigger fsync|teof`: hardware trigger source
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
