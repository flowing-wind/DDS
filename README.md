# DDS

A Direct Digital Synthesizer (DDS) IP core for FPGA, configured entirely
through an **APB slave interface** (32-bit registers) and driving a **12/14-bit
parallel DAC**. Written in plain Verilog-2001 — no vendor IP, no Block Memory
Generator, no `.coe` files; all memories are inferred BRAM.

Designed to live on an in-FPGA APB fabric next to other small IP cores
(SPI, timers, user logic) and to be paced by them in hardware where useful.

## Features

- 32-bit frequency tuning word: `fout = FWORD × f_dds / 2^32`
  (11.6 mHz resolution at 50 MHz)
- Waveforms: **sine / cosine** (4096-point quarter-wave LUT, 16384 effective
  points per period), **square** with 16-bit programmable duty cycle,
  **triangle**, **ramp**, and a **user-defined** 4096 × 14-bit waveform RAM
  loaded over APB
- Frequency control modes:
  - **FIXED** — phase-continuous retune on a single register write
  - **SWEEP** — hardware linear/piecewise chirp (start/stop/delta/rate),
    single, repeating, or up/down; software or external-pin trigger
  - **PROFILE** — 4 pre-programmed frequency+phase profiles, hopped by
    register or by the `hop_sel[1:0]` input pins (FSK/PSK with zero CPU load)
- 16-bit phase offset (≈ 0.0055° resolution)
- Digital amplitude scaling 0 … 1.0 (Q1.16 multiplier; peak voltage is set in
  the external DAC/filter chain)
- Programmable DC offset: 14-bit signed, saturating, added after the
  amplitude scaler — waveform + bias, or a pure DC level output

- Output conditioning: 14-bit or 12-bit (MSB-aligned, rounded), offset-binary
  or two's-complement coding, polarity invert, mid-scale idle when disabled
- All parameters double-buffered and applied **atomically and
  phase-continuously**; automatic or explicit update strobe
- Interrupt output (sweep done / wrap, update done)
- APB clock and DAC sample clock may be the **same clock or fully
  asynchronous** — CDC is handled inside

## Repository layout

```
rtl/
  dds_top.v        top level — instantiate this
  dds_regs.v       APB register file (PCLK domain)
  dds_core.v       phase accumulator, sweep engine, waveforms, output stage
  dds_sin_lut.v    quarter-wave sine ROM (inferred BRAM)
  dds_sin_lut.mem  ROM init data for $readmemh (add to the Vivado project)
  dds_wave_ram.v   user waveform dual-port RAM (inferred BRAM)
  dds_cdc.v        clock-domain-crossing helpers
tb/
  tb_dds.sv         self-checking testbench (Icarus Verilog)
scripts/
  gen_sin_lut.py   regenerates dds_sin_lut.mem
Doc/
  DDS_Datasheet.md full datasheet: register map, programming examples,
                   integration and constraints
  AN01_AD9764_Module.md  application note for one specific target DAC board
  dds.pdf          theory reference (ADI DDS tutorial) the design follows
```

## Quick start

Instantiate `dds_top`, connect the APB port, tie `dds_clk` to your DAC sample
clock (it may simply be `PCLK`), and wire `dac_data` to the DAC:

```verilog
dds_top u_dds (
    .PCLK(pclk), .PRESETn(presetn),
    .PADDR(paddr[14:0]), .PSEL(psel), .PENABLE(penable),
    .PWRITE(pwrite), .PWDATA(pwdata), .PRDATA(prdata), .PREADY(pready),
    .dds_clk(dac_clk), .dds_rstn(dac_rstn),
    .sweep_trig(1'b0), .hop_sel(2'b00),      // optional hardware pacing
    .dac_data(dac_data), .dac_valid(dac_valid),
    .dds_irq(dds_irq)
);
```

Generate a 1 MHz full-scale sine (f_dds = 50 MHz):

```
APB write 0x0C = 0x051EB852   // FWORD = 1e6 * 2^32 / 50e6
APB write 0x04 = 0x00002001   // CTRL: enable, sine, two's complement
```

The [datasheet](Doc/DDS_Datasheet.md) contains the complete register map and
worked APB sequences for sweeps, FSK profiles, duty-cycle squares, and
loading user-defined waveforms.

## Simulation

Requires Icarus Verilog:

```
cd tb
iverilog -g2012 -o tb_dds.vvp tb_dds.sv ../rtl/*.v
vvp tb_dds.vvp        # 26 self-checks -> "ALL TESTS PASSED"
```

Add `-DASYNC_CLKS` to rerun the same suite with a 125 MHz `dds_clk`
asynchronous to the 50 MHz `PCLK`. A `tb_dds.vcd` waveform dump is produced.

## Resource usage (7-series estimate)

~4 × BRAM18 (sine LUT + wave RAM), 1 × DSP48 (amplitude multiplier),
roughly 600 LUT / 900 FF. Comfortably exceeds 150 MHz on Artix-7 speed -1.
