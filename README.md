# DDS

A Direct Digital Synthesizer (DDS) IP core for FPGA, configured over a **16-bit
SPI slave port** (mode 0) and driving **one or two 14-bit parallel DACs**, with
an optional **dual 12-bit ADC capture front end and per-channel closed-loop
amplitude control**. Written in plain Verilog-2001 — no vendor IP, no Block
Memory Generator, no `.coe` files; all memories are inferred BRAM.

Designed to be driven by an MCU. The companion firmware for an STM32F103, with
a UART command line for the PC, lives in [`MCU/STM32F1`](../../MCU/STM32F1).

## Features

- 32-bit frequency tuning word: `fout = FWORD × f_dds / 2^32`
  (23.3 mHz resolution at 100 MHz)
- Waveforms: **sine / cosine** (4096-point quarter-wave LUT, 16384 effective
  points per period), **square** with 16-bit programmable duty cycle,
  **triangle**, **ramp**, and a **user-defined** 4096 × 14-bit waveform RAM
  loaded over SPI
- Frequency control modes:
  - **FIXED** — phase-continuous retune on a single frame
  - **SWEEP** — hardware linear chirp (start/stop/delta/rate), single,
    repeating, or up/down; software or external-pin trigger
  - **PROFILE** — 4 pre-programmed frequency+phase profiles, hopped by register
    or by the `hop_sel[1:0]` input pins (FSK/PSK with zero CPU load)
- 16-bit phase offset (≈ 0.0055° resolution)
- Digital amplitude scaling 0 … 1.0 (Q1.15 multiplier — finer than the DAC's own
  LSB at every setting)
- Programmable DC offset: 14-bit signed, saturating, added after the amplitude
  scaler — waveform + bias, or a pure DC level output
- Output conditioning: 14-bit or 12-bit (MSB-aligned, rounded), offset binary or
  two's complement, polarity invert, mid-scale idle when disabled
- **Atomic frame commit**: everything a SPI frame writes takes effect together,
  on the rising edge of chip select. A 32-bit frequency word written as two
  16-bit halves never puts an intermediate frequency on the DAC.
- **Two channels share one chip select** — the channel is a bit in the command
  word
- **Single clock domain**: the SPI pins are oversampled, so there is no CDC to
  constrain and no second bus clock — and the optional ADC keeps it that way
  (its clock is a `dds_clk`/2 ODDR pattern, data comes back on a clock enable)
- **ADC feedback** (optional, `HAS_ADC`): dual AD9226-style 12-bit parallel
  capture at 50 MSPS, hardware Vpp / AC-RMS / mean / min / max measurement per
  window, and an integrating amplitude loop per channel (dead band,
  anti-windup, lock/saturation status, bumpless open/close). Which ADC feeds
  which channel's loop is a register; the captured streams are exported for
  any other fabric consumer.
- Interrupt output per channel (sweep done / wrap, update done)

## SPI interface

Mode 0 (CPOL = 0, CPHA = 0), MSB first, 16-bit words, CS active low —
bit-for-bit what an STM32 SPI master produces with `DATASIZE_16BIT`,
`POLARITY_LOW`, `PHASE_1EDGE` and a GPIO chip select.

```
CS↓ [CMD] [D0] [D1] ... CS↑        CS↑ commits the whole frame atomically

CMD = | RW(1) | CH(1) | ADDR(14) |     RW: 1 = write, 0 = read
                                       ADDR auto-increments per data word

write:  D0 -> ADDR, D1 -> ADDR+1, ...
read:   one turnaround word after CMD, then data:
          master  [CMD] [dummy] [dummy] [dummy]
          slave   [0]   [0]     [ADDR]  [ADDR+1]
```

The pins are oversampled in `dds_clk`, which requires `f_sck ≤ dds_clk / 6`
(9 MHz SCK against a 100 MHz `dds_clk` gives 11× margin).

## Repository layout

```
rtl/                portable Verilog-2001, no vendor primitives, simulatable
  dds_top.v         the IP core - instantiate this (SPI + 1..2 channels + ADC)
  dds_spi_slave.v   SPI mode-0 slave, oversampled; drives the internal bus
  dds_channel.v     one channel: registers + datapath + waveform RAM + loop
  dds_regs.v        16-bit register file, shadow/commit logic
  dds_core.v        phase accumulator, sweep engine, waveforms, output stage
  dds_sin_lut.v     quarter-wave sine ROM (inferred BRAM)
  dds_sin_lut.mem   ROM init data for $readmemh (add to the Vivado project!)
  dds_wave_ram.v    user waveform dual-port RAM (inferred BRAM)
  dds_adc_if.v      AD9226 capture: clock pattern, capture timing, bus fixes
  dds_meas.v        windowed Vpp / AC-RMS / mean / min / max measurement
  dds_isqrt.v       bit-serial square root for the RMS path
  dds_agc.v         integrating amplitude controller (dead band, anti-windup)
  dds_agc_regs.v    amplitude-loop registers (block 0x40, per channel)
  dds_glob_regs.v   ADC front-end registers (block 0x80, global)
board/              board-level, Xilinx primitives - this is the synthesis top
  dds_board_top.v   PLL + reset sync + IOB registers + ODDR clock forwarding
                    (DAC clocks out, ADC clock out, ADC capture flops in)
  DDS_Constraints.xdc  reference pins + ADC pin template
tb/
  tb_dds.sv         self-checking testbench (bit-accurate SPI master model,
                    behavioural AD9226 + analog chain for the loop tests)
scripts/
  gen_sin_lut.py    regenerates dds_sin_lut.mem
Doc/
  DDS_Datasheet.md  full datasheet: frame format, register map, programming
                    examples, integration and constraints
  DDS_Tutorial_zh.md 中文入门教程: DDS 原理、每个功能、扫频/FSK、闭环幅度控制、上手步骤
  Vivado_Build.md   how to build it: top module, IP, .mem, XDC, bring-up order
  Application Note/AN01/  application note for the AD9764 DAC module
  Application Note/AN02_AD9226_Module.md  application note for the ADC module
                    and the feedback loop hardware (wiring, traps, calibration)
  dds.pdf           theory reference (ADI DDS tutorial) the design follows
```

**`dds_top` is the IP core, not the synthesis top.** It has no PLL, no clock
forwarding and no I/O primitives on purpose, so it stays board-independent and
simulates under Icarus. The synthesis top is `board/dds_board_top.v`, which adds
the 100 MHz PLL, the reset synchronizer, IOB output registers and the ODDR that
forwards the DAC clock. See [Doc/Vivado_Build.md](Doc/Vivado_Build.md).

## Quick start

Instantiate `dds_top`, tie `dds_clk` to the DAC sample clock (100 MHz), wire the
four SPI pins to the MCU, and connect the DAC buses:

```verilog
dds_top #(
    .NUM_CH(2), .SIN_LUT_FILE("dds_sin_lut.mem")
) u_dds (
    .dds_clk(clk100), .dds_rstn(rstn),
    .spi_sck(sck), .spi_cs_n(cs_n), .spi_mosi(mosi),
    .spi_miso(miso), .spi_miso_oe(),          // leave open for a dedicated pin
    .ch0_sweep_trig(1'b0), .ch0_hop_sel(2'b00),   // optional hardware pacing
    .ch1_sweep_trig(1'b0), .ch1_hop_sel(2'b00),
    .ch0_dac_data(dac0_data), .ch0_dac_valid(dac0_valid),
    .ch1_dac_data(dac1_data), .ch1_dac_valid(dac1_valid),
    .dds_irq(dds_irq)
);
```

Generate a 1 MHz full-scale sine on channel 0 (f_dds = 100 MHz):

```
frame: 0x8004, 0x5C29, 0x028F      // FWORD = 1e6 * 2^32 / 100e6, one atomic frame
frame: 0x8002, 0x0001              // CTRL: enable, sine, offset binary (AD9764)
```

Or, from the STM32 firmware's UART console:

```
dds> freq 1M
dds> wave sine
dds> amp 2Vpp            # real volts (or a bare 0..1 for a plain scale)
dds> on
dds> agc target 2Vpp     # ...or let the hardware loop hold it there
dds> agc on
```

The [datasheet](Doc/DDS_Datasheet.md) has the complete register map and worked
frame sequences for sweeps, FSK profiles, duty-cycle squares, DC bias, and
loading user-defined waveforms.

## Simulation

Requires Icarus Verilog:

```
cd tb
iverilog -g2012 -o tb_dds.vvp tb_dds.sv ../rtl/*.v
vvp tb_dds.vvp        # 75 self-checks -> "ALL TESTS PASSED"
```

The testbench drives the real SPI pins with a bit-accurate mode-0 master at
9 MHz — the same waveform the STM32 firmware produces — and models the AD9226
module (inverted transfer, output delay) behind a model of the analog feedback
chain, so the closed-loop amplitude tests run end to end. Add `-DSCK_FAST` to
rerun the suite at the documented `dds_clk / 6` limit (16 MHz). A `tb_dds.vcd`
waveform dump is produced.

## Resource usage (7-series estimate)

Per channel: ~4 × BRAM18 (sine LUT + wave RAM), 1 × DSP48 (amplitude
multiplier), roughly 600 LUT / 900 FF; `HAS_ADC` adds ~1 DSP48 and
~350 LUT / 400 FF per channel for the measurement + loop. The shared SPI slave
adds ~80 LUT / 120 FF. Comfortably exceeds 150 MHz on Artix-7 speed grade −1.
