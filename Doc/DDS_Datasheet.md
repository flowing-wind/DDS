# DDS_TOP — Direct Digital Synthesizer IP Core with SPI Interface

A Verilog-2001 DDS IP core for FPGA: **one 16-bit SPI slave port** (mode 0)
configuring **one or two independent channels**, each driving a **14-bit
parallel DAC**. No vendor IP, no `.coe` files; all memories are inferred BRAM.

Companion firmware: `MCU/STM32F1` (STM32F103 master + UART command line).

| | |
|---|---|
| Version | 2.1 (`ID` = 0x4453, `VERSION` = 0x0210) |
| Bus | SPI slave, mode 0 (CPOL=0, CPHA=0), 16-bit words, MSB first |
| Channels | 1 or 2 (`NUM_CH`), selected by a bit in the command word — one chip select for both |
| Clock | Single domain: `dds_clk` (100 MHz recommended). SPI pins are oversampled. |
| DAC | 14-bit (or 12-bit MSB-aligned), offset binary or two's complement |
| ADC | optional (`HAS_ADC`): dual 12-bit parallel capture at `dds_clk`/2, for the amplitude loop and export |
| Frequency | 32-bit tuning word, 23.3 mHz resolution at 100 MHz |
| Waveforms | sine, cosine, square (programmable duty), triangle, ramp, user RAM (4096 × 14) |
| Modes | fixed, hardware linear sweep/chirp, 4-profile frequency+phase hopping |
| Amplitude | open-loop Q1.15 scale, or closed-loop on a measured Vpp / RMS per channel |

---

## 1. Features

- **32-bit frequency tuning word** — `fout = FWORD × f_dds / 2^32`
  (23.3 mHz resolution at 100 MHz)
- **Waveforms**: sine / cosine from a 4096-point quarter-wave LUT (16384
  effective points per period), square with 16-bit programmable duty cycle,
  triangle, ramp, and a user-defined 4096 × 14-bit waveform RAM loaded over SPI
- **Frequency control modes**
  - **FIXED** — phase-continuous retune on a single frame
  - **SWEEP** — hardware linear chirp (start/stop/delta/rate), single,
    repeating, or up/down; software or external-pin trigger
  - **PROFILE** — 4 pre-programmed frequency+phase profiles, hopped by register
    or by the `hop_sel[1:0]` input pins (FSK/PSK with zero CPU load)
- **16-bit phase offset** (≈ 0.0055° resolution)
- **Digital amplitude scaling** 0 … 1.0 (Q1.15 multiplier)
- **Programmable DC offset**: 14-bit signed, saturating, added after the
  amplitude scaler — waveform + bias, or a pure DC level output
- **Output conditioning**: 14-bit or 12-bit (MSB-aligned, rounded), offset
  binary or two's complement, polarity invert, mid-scale idle when disabled
- **Atomic frame commit** — every parameter a frame writes takes effect
  together, on the rising edge of chip select. A 32-bit frequency word written
  as two 16-bit halves never puts an intermediate frequency on the DAC.
- **Single clock domain** — no CDC to constrain, no second bus clock; with
  `HAS_ADC` the ADC clock is generated as a `dds_clk`/2 ODDR pattern and the
  returning data is captured with a clock enable, so this stays true
- **ADC capture front end** (optional, `HAS_ADC`): dual AD9226-style 12-bit
  parallel inputs at 50 MSPS, with programmable capture phase, bus reversal
  and sign correction; the captured streams are exported for other fabric users
- **Closed-loop amplitude control per channel**: a hardware measurement unit
  (Vpp, AC-RMS, mean, min/max over a programmable window) and an integrating
  controller that drives the amplitude multiplier toward a target, with dead
  band, anti-windup clamps and lock/saturation status — source-selectable
  between the two ADCs, fully register-controlled
- Interrupt output per channel (sweep done / wrap, update done)

---

## 2. Block Diagram and Theory of Operation

```
  SPI pins                    dds_clk domain (everything)
 ─────────────────────────────────────────────────────────────────────
  SCK  ─┐
  CS_N ─┼─> oversample ─> dds_spi_slave ─> internal bus ─┬─> channel 0
  MOSI ─┘   (2-FF sync)   CMD: RW|CH|ADDR                └─> channel 1
  MISO <────────────────  addr auto-increment                  │
                          CS rise = frame_end (commit)         │
                                                               v
   ┌───────────────────────────────────────────────────────────────┐
   │  shadow registers ──commit on frame_end──> active registers   │
   │                                                    │          │
   │                  ┌─────────────────────────────────┘          │
   │                  v                                            │
   │        ┌────────────────────┐                                 │
   │        │ frequency source   │                                 │
   │        │  FIXED : FWORD     │                                 │
   │        │  SWEEP : FSTART +  │                                 │
   │        │   freq-accumulator │                                 │
   │        │  PROF  : PROFn     │                                 │
   │        └───────┬────────────┘                                 │
   │                v                                              │
   │       32-bit phase accumulator  + (POW << 16)                 │
   │                v                                              │
   │        ┌─ ¼-wave sine LUT (4096×13)                           │
   │        ├─ square (phase < DUTY)                               │
   │        ├─ triangle (folded phase)                             │
   │        ├─ ramp (phase MSBs)                                   │
   │        └─ WAVE RAM playback port (4096×14)                    │
   │                v  14-bit signed                               │
   │           × AMP (Q1.15, 0…1.0)                                │
   │           + OFFSET (14-bit signed)                            │
   │                v  round + saturate                            │
   │           output stage: 14/12-bit, offset-bin / 2's comp, INV │
   │                v                                              │
   │           dac_data[13:0], dac_valid                           │
   └───────────────────────────────────────────────────────────────┘
```

### 2.1 Tuning equation

The phase accumulator adds the effective frequency word every `dds_clk` cycle
(Doc/dds.pdf, Sec. 1/3):

```
fout        = FWORD × f_dds / 2^32
FWORD       = round(fout × 2^32 / f_dds)
resolution  = f_dds / 2^32
```

At **f_dds = 100 MHz**: resolution = 23.3 mHz, and 1 MHz → FWORD =
`0x028F_5C29`. Keep `fout < f_dds/2` (Nyquist); the AD9764 module's 32 MHz
reconstruction filter (see AN01) wants `fout ≤ ~20 MHz` anyway, which leaves
≥ 5 samples per cycle.

### 2.2 Waveform phase mapping

All waveforms derive from the offset phase `ph = phase + (POW << 16)`;
`ph = 0` is the period start.

| Waveform | Value at ph = 0 | Definition |
|---|---|---|
| Sine     | 0, rising | ¼-wave LUT, quadrant = `ph[31:30]`, addr = `ph[29:18]` |
| Cosine   | +FS       | sine advanced 90° (POW + 0x4000 internally) |
| Square   | +FS       | +FS while `ph[31:16] < DUTY`, else −FS |
| Triangle | −FS, rising | folded `ph`, peak +FS at half period |
| Ramp     | −FS, rising | `ph[31:18]` as signed, wraps at period end |
| User     | RAM[0]    | RAM address = `ph[31:20]` |

The LUT stores `sin((i+0.5)/4096 × 90°) × 8191`; the half-LSB offset makes the
quadrant symmetry exact, so the reconstructed sine has no DC offset and peaks
at ±8191.

### 2.3 Amplitude path

`sample(14-bit signed) × AMP(Q1.15) + OFFSET`, rounded to nearest and saturated
to 14 bits (Doc/dds.pdf, Fig. 11.4). **AMP = 0x8000 is exactly 1.0** and a
bit-exact pass-through; values above 0x8000 are clamped to 1.0 by the register
file (a gain above full scale could only clip). The OFFSET add shares the
rounding stage, so it costs no extra latency. One DSP48 implements the multiply.

15 fractional bits of gain against a 14-bit DAC means the amplitude step is
always below the DAC's own LSB — the 16-bit register loses nothing.

### 2.4 Pipeline latency

Phase accumulator to `dac_data`: **6 dds_clk cycles**, constant for every
waveform and setting. Only matters if you align the output with external events.

---

## 3. Ports

```verilog
dds_top #(
    .NUM_CH       (2),                  // 1 or 2
    .SIN_LUT_FILE ("dds_sin_lut.mem"),
    .HAS_ADC      (1)                   // 0 removes the ADC + amplitude loops
) u_dds (
    .dds_clk(clk100), .dds_rstn(rstn),
    .spi_sck(sck), .spi_cs_n(cs_n), .spi_mosi(mosi),
    .spi_miso(miso), .spi_miso_oe(miso_oe),
    .ch0_sweep_trig(1'b0), .ch0_hop_sel(2'b00),
    .ch1_sweep_trig(1'b0), .ch1_hop_sel(2'b00),
    .ch0_dac_data(dac0), .ch0_dac_valid(dv0),
    .ch1_dac_data(dac1), .ch1_dac_valid(dv1),
    // ADC front end: board ODDR + IOB capture flops, see section 6.5
    .adc_clk_d1(ad1), .adc_clk_d2(ad2), .adc_capt_en(ace),
    .adc0_data(a0), .adc0_otr(1'b0), .adc1_data(a1), .adc1_otr(1'b0),
    // captured streams, free for any other fabric consumer
    .adc0_smp(), .adc0_smp_otr(), .adc1_smp(), .adc1_smp_otr(),
    .adc_smp_valid(),
    .dds_irq(irq)
);
```

| Port | Dir | Width | Description |
|---|---|---|---|
| `dds_clk`        | in  | 1  | The clock. DAC sample clock, SPI oversampling clock, register clock. |
| `dds_rstn`       | in  | 1  | Active-low reset; assert asynchronously, release synchronously. |
| `spi_sck`        | in  | 1  | SPI clock (asynchronous, oversampled). `f_sck ≤ dds_clk / 6`. |
| `spi_cs_n`       | in  | 1  | Chip select, active low. Its rising edge commits the frame. |
| `spi_mosi`       | in  | 1  | Master → slave data. |
| `spi_miso`       | out | 1  | Slave → master data. |
| `spi_miso_oe`    | out | 1  | High while a frame is active — for a tri-stated/shared MISO net. Ignore for a dedicated pin. |
| `chN_sweep_trig` | in  | 1  | Rising edge starts a sweep when `SWEEP_CTRL.EXT_TRIG = 1`. Synchronized internally. |
| `chN_hop_sel`    | in  | 2  | Profile select when `PROF_CTRL.EXT_SEL = 1`. Synchronized + 2-cycle agreement filter. |
| `chN_dac_data`   | out | 14 | Parallel DAC data, registered. |
| `chN_dac_valid`  | out | 1  | High while the output is live (CTRL.EN and the pipeline is full). |
| `dds_irq`        | out | 2  | One level interrupt per channel, `|(IRQ_STAT & IRQ_EN)`. |
| `adc_clk_d1/d2`  | out | 1  | ADC clock as an ODDR bit pattern (`HAS_ADC`). Board top plays it through one ODDR per ADC clock pin (ACLK, BCLK) — same pattern, same phase. |
| `adc_capt_en`    | out | 1  | Clock enable for the board-level IOB input flops that capture the ADC buses. |
| `adcN_data`      | in  | 12 | Captured ADC bus, `[11]` = ADC MSB. **The AD9226 module numbers its bus backwards — silk D0 is the MSB.** See AN02. |
| `adcN_otr`       | in  | 1  | ADC over-range flag; tie 0 if unwired. |
| `adcN_smp`       | out | 12 | Captured sample, signed, centred, sign-corrected — exported for other fabric modules. |
| `adcN_smp_otr`   | out | 1  | Over-range, aligned with `adcN_smp`. |
| `adc_smp_valid`  | out | 1  | One pulse per ADC sample period (every 2nd `dds_clk`), common to both ADCs. |

With `NUM_CH = 1`, the channel-1 outputs are tied off and frames addressed to
channel 1 are ignored. With `HAS_ADC = 0` the ADC ports are tied off, the
amplitude-loop and global register blocks vanish from the map (reads return 0),
and the core is exactly the pre-2.1 design.

**Resource routing philosophy.** The DACs, ADCs and loops are deliberately not
hard-wired to each other, because on a real bench they get mixed and matched:

- *Which ADC feeds a channel's loop* is the `AGC_CTRL.SRC` register — any DDS
  channel can watch either ADC, both channels may watch the same one.
- *No feedback at all* is simply `AGC_CTRL.EN = 0` (the reset state): the AMP
  register drives the datapath directly and the ADCs are ignored.
- *An ADC needed by some other module* taps `adcN_smp` / `adc_smp_valid`; the
  export is always live, loop or no loop.
- *A DAC driven by another module* is a mux outside this core; the DDS cannot
  observe that, so leave that channel's loop off.

---

## 4. SPI Protocol

**Mode 0** (CPOL = 0, CPHA = 0): MOSI/MISO change on the falling edge of SCK,
both ends sample on the rising edge. MSB first, 16-bit words, CS active low.

### 4.1 Frame format

```
       CS_N ‾‾‾\____________________________________________/‾‾‾‾
                | CMD (16) | D0 (16) | D1 (16) | ... |
                                                          ^
                                                          commit
CMD:  15   14   13                                     0
     ┌────┬────┬──────────────────────────────────────┐
     │ RW │ CH │              ADDR[13:0]              │
     └────┴────┴──────────────────────────────────────┘
      1=wr  channel        word address, auto-increments
```

- **Write**: `D0 → ADDR`, `D1 → ADDR+1`, … Any number of data words.
- **Read**: the word right after CMD is a **turnaround word** (MISO reads
  0x0000); read data starts in the third word:

```
  master:  [CMD] [dummy]  [dummy]   [dummy]   ...
  slave:   [0]   [0]      [ADDR]    [ADDR+1]  ...
```

The address pointer runs one word ahead of MISO, so the register or BRAM read
always has a full word time (16 SCK) to settle.

### 4.2 The commit point

**The rising edge of CS applies everything the frame wrote.** Config writes land
in shadow registers; the datapath latches them as one atomic set at frame end.
Consequences worth relying on:

- A 32-bit frequency word written as `FWORD_L` + `FWORD_H` in one frame
  produces **no intermediate frequency** on the DAC.
- "Write the sweep config **and** the START bit in one frame" always starts the
  sweep with the config from that same frame — the commit is ordered before the
  start strobe.
- `UPDATE.AUTO = 1` (reset default) commits every frame that touched config.
  With `AUTO = 0`, shadow writes accumulate across frames until a frame writes
  `UPDATE.UPD = 1` — that is how you retune both channels or many parameters at
  a single instant.

### 4.3 Timing

The SPI pins are **oversampled in the `dds_clk` domain** (2-FF synchronizers,
edge detection) — there are no SCK-clocked flops and no BUFG on a slow external
clock. The one rule this imposes:

```
f_sck  ≤  dds_clk / 6
```

At `dds_clk` = 100 MHz that allows up to 16.6 MHz. The STM32F103 firmware runs
SPI1 at **9 MHz** (72 MHz / 8), i.e. 11× oversampling. The testbench passes at
both 9 MHz and the 16 MHz limit.

MISO is updated right after the rising edge that completes a word — a full half
SCK period before the master samples it — so no extra setup constraint applies
at the master.

Keep CS low for the whole frame and raise it between frames. Back-to-back words
inside a frame need no gap.

---

## 5. Register Map

16-bit registers, **word-addressed** (ADDR is a word index, not a byte offset).
When `ADDR[13] = 0`, `ADDR[12:6]` selects a register block and `ADDR[5:0]` the
register inside it; the waveform RAM occupies `ADDR[13] = 1` (0x2000–0x2FFF).

| Block | Addresses | Scope | Contents |
|---|---|---|---|
| 0 | 0x00–0x3F | per channel (CMD[14]) | the core registers below |
| 1 | 0x40–0x7F | per channel (CMD[14]) | amplitude loop (§5.14), if `HAS_ADC` |
| 2 | 0x80–0xBF | global — CMD[14] ignored | ADC front end (§5.15), if `HAS_ADC` |
| — | 0x2000–0x2FFF | per channel | user waveform RAM |

Each channel has its own complete copy of blocks 0 and 1, selected by CMD[14].
Block 2 configures the single shared ADC capture unit; asking which channel it
belongs to has no answer, so it answers identically under either channel bit.

Reserved bits read 0 and should be written 0.
**W1SC** = write 1, self-clearing (reads 0). **W1C** = write 1 to clear.

| Addr | Name | Access | Reset | Function |
|---|---|---|---|---|
| 0x00 | `ID`            | RO  | 0x4453 | Identification ("DS") |
| 0x01 | `VERSION`       | RO  | 0x0210 | Major.minor |
| 0x02 | `CTRL`          | RW  | 0x0000 | Enable, waveform, mode, output format |
| 0x03 | `STATUS`        | RO  | —      | Sweep and profile state |
| 0x04 | `FWORD_L`       | RW  | 0x0000 | Tuning word [15:0] |
| 0x05 | `FWORD_H`       | RW  | 0x0000 | Tuning word [31:16] |
| 0x06 | `POW`           | RW  | 0x0000 | Phase offset, 65536 = 360° |
| 0x07 | `AMP`           | RW  | 0x8000 | Amplitude, Q1.15 (0x8000 = 1.0, clamped) |
| 0x08 | `DUTY`          | RW  | 0x8000 | Square duty, 65536 = 100 % |
| 0x09 | `OFFSET`        | RW  | 0x0000 | DC offset, 14-bit signed |
| 0x0A | `UPDATE`        | RW  | 0x0002 | Commit control |
| 0x0B | `SWEEP_CTRL`    | RW  | 0x0000 | Sweep mode, trigger source, start/abort |
| 0x0C | `SWEEP_FSTART_L`| RW  | 0x0000 | Sweep start frequency [15:0] |
| 0x0D | `SWEEP_FSTART_H`| RW  | 0x0000 | … [31:16] |
| 0x0E | `SWEEP_FSTOP_L` | RW  | 0x0000 | Sweep stop frequency [15:0] |
| 0x0F | `SWEEP_FSTOP_H` | RW  | 0x0000 | … [31:16] |
| 0x10 | `SWEEP_FDELTA_L`| RW  | 0x0000 | Frequency step [15:0] |
| 0x11 | `SWEEP_FDELTA_H`| RW  | 0x0000 | … [31:16] |
| 0x12 | `SWEEP_RATE_L`  | RW  | 0x0001 | dds_clk cycles per step [15:0] |
| 0x13 | `SWEEP_RATE_H`  | RW  | 0x0000 | … [31:16] |
| 0x14 | `PROF_CTRL`     | RW  | 0x0000 | Profile select and source |
| 0x15 | `PROF0_FWORD_L` | RW  | 0x0000 | Profile 0 tuning word [15:0] |
| 0x16 | `PROF0_FWORD_H` | RW  | 0x0000 | … [31:16] |
| 0x17…0x1C | `PROF1…3_FWORD_L/H` | RW | 0x0000 | Profiles 1–3, same layout |
| 0x1D | `PROF0_POW`     | RW  | 0x0000 | Profile 0 phase offset |
| 0x1E…0x20 | `PROF1…3_POW` | RW | 0x0000 | Profiles 1–3 phase offsets |
| 0x21 | `IRQ_EN`        | RW  | 0x0000 | Interrupt enables |
| 0x22 | `IRQ_STAT`      | W1C | 0x0000 | Interrupt flags |
| 0x40 | `AGC_CTRL`      | RW  | 0x0000 | Loop enable, hold, clear, source, metric |
| 0x41 | `AGC_STATUS`    | RO  | —      | Locked / saturated / over-range |
| 0x42 | `AGC_TARGET`    | RW  | 0x0000 | Setpoint, ADC codes |
| 0x43 | `AGC_KI`        | RW  | 0x0600 | Integrator gain, Q8.8 (reset = 6.0) |
| 0x44 | `AGC_TOL`       | RW  | 0x0002 | Dead band, ADC codes |
| 0x45 | `AGC_WIN`       | RW  | 0x0010 | log2 window length in ADC samples (4–24) |
| 0x46 | `AGC_AMP_MIN`   | RW  | 0x0000 | Integrator clamp, Q1.15 |
| 0x47 | `AGC_AMP_MAX`   | RW  | 0x8000 | Integrator clamp, Q1.15 |
| 0x48 | `AGC_AMP`       | RO  | —      | Amplitude actually scaling the waveform |
| 0x49 | `AGC_VPP`       | RO  | —      | Measured peak-to-peak, ADC codes |
| 0x4A | `AGC_RMS`       | RO  | —      | Measured AC RMS (DC removed), ADC codes |
| 0x4B | `AGC_MEAN`      | RO  | —      | Measured DC mean, signed, sign-extended |
| 0x4C | `AGC_VMIN`      | RO  | —      | Window minimum, signed |
| 0x4D | `AGC_VMAX`      | RO  | —      | Window maximum, signed |
| 0x80 | `GLOB_ID`       | RO  | 0x4147 | "AG" if `HAS_ADC`, else 0 — probe this |
| 0x81 | `ADC_CFG`       | RW  | 0x0201 | ADC clock enable, capture phase, bus fixes |
| 0x82 | `ADC0_RAW`      | RO  | —      | Live ADC0 sample, signed, sign-extended |
| 0x83 | `ADC1_RAW`      | RO  | —      | Live ADC1 sample |
| 0x2000–0x2FFF | `WAVE_RAM` | RW | — | User waveform, 4096 × 14-bit signed |

### 5.1 CTRL (0x02)

| Bit | Name | Access | Description |
|---|---|---|---|
| 0     | `EN`      | RW   | 1 = run. 0 = phase accumulator frozen, DAC held at mid-scale. |
| 1     | `SRST`    | W1SC | Soft reset: zeroes the phase accumulator, stops any sweep, clears IRQ flags. |
| 3:2   | —         | —    | Reserved |
| 6:4   | `WAVE`    | RW   | 0 sine, 1 cosine, 2 square, 3 triangle, 4 ramp, 5 user RAM |
| 7     | —         | —    | Reserved |
| 9:8   | `FMODE`   | RW   | 0 FIXED, 1 SWEEP, 2 PROFILE |
| 11:10 | —         | —    | Reserved |
| 12    | `OUT12`   | RW   | 1 = 12-bit output, rounded and MSB-aligned into `dac_data[13:2]` |
| 13    | `FMT_2C`  | RW   | 1 = two's complement, 0 = offset binary (what most current-output DACs want) |
| 14    | `INV`     | RW   | 1 = invert the waveform (negate before coding) |

`SRST` is a strobe: it acts on the frame commit and reads back 0.

### 5.2 STATUS (0x03, read-only)

| Bit | Name | Description |
|---|---|---|
| 0   | `SWEEP_ACTIVE` | A sweep is running |
| 1   | `SWEEP_DIR`    | 1 = currently sweeping down (UPDOWN mode) |
| 3:2 | `ACTIVE_PROF`  | Profile currently driving the accumulator |

### 5.3 FWORD (0x04/0x05)

The FIXED-mode tuning word; see the tuning equation (Sec. 2.1). Write both
halves **in one frame** so they commit together.

### 5.4 POW (0x06)

Phase offset added to the accumulator output: `65536 = 360°`, resolution
0.0055°. Changing POW shifts the phase without disturbing the frequency.

### 5.5 AMP (0x07)

Q1.15 amplitude: `0x8000 = 1.0` (full scale), `0x4000 = 0.5`, `0x0000` = silent.
Writes above 0x8000 are clamped to 0x8000. The peak output voltage is set by the
external DAC/filter chain; this only scales digitally.

### 5.6 DUTY (0x08)

Square-wave duty cycle: `65536 = 100 %`, so `0x8000` = 50 %, `0x4000` = 25 %.
Affects the square waveform only.

### 5.7 OFFSET (0x09)

14-bit signed DC offset (−8192 … +8191), added after the amplitude scaler and
saturated. `AMP = 0` plus a non-zero OFFSET outputs a **pure DC level** — handy
for a programmable bias.

### 5.8 UPDATE (0x0A)

| Bit | Name | Access | Description |
|---|---|---|---|
| 0 | `UPD`  | W1SC | Commit the shadow registers at this frame's end |
| 1 | `AUTO` | RW   | 1 (default) = commit at the end of every frame that wrote config |

### 5.9 SWEEP_CTRL (0x0B)

| Bit | Name | Access | Description |
|---|---|---|---|
| 1:0 | `MODE`     | RW   | 0 SINGLE (run once, hold at FSTOP), 1 SAW (repeat from FSTART), 2 UPDOWN (ping-pong) |
| 2   | `EXT_TRIG` | RW   | 1 = start on the rising edge of `sweep_trig` instead of the START bit |
| 8   | `START`    | W1SC | Start / restart the sweep |
| 9   | `ABORT`    | W1SC | Stop the sweep and return to FSTART |

The sweep engine ramps **upward only**: `FSTOP > FSTART`. It adds `FDELTA` to
the tuning word every `RATE` dds_clk cycles, so

```
steps    = ceil((FSTOP − FSTART) / FDELTA)
duration = steps × RATE / f_dds
```

The sweep only advances while `CTRL.EN = 1`, and `FMODE` must be SWEEP.

### 5.10 SWEEP_RATE (0x12/0x13)

dds_clk cycles per frequency step. 0 is treated as 1. 32 bits, so a single
sweep can run from microseconds to tens of seconds.

### 5.11 PROF_CTRL (0x14)

| Bit | Name | Access | Description |
|---|---|---|---|
| 1:0 | `SEL`      | RW | Active profile 0–3 (when EXT_SEL = 0) |
| 4   | `EXT_SEL`  | RW | 1 = take the profile number from the `hop_sel[1:0]` pins |

With `EXT_SEL = 1` another block (or a pin) can hop frequency/phase at the
sample rate with no SPI traffic at all — FSK/PSK modulation with zero CPU load.

### 5.12 IRQ_EN (0x21) / IRQ_STAT (0x22)

| Bit | Name | Description |
|---|---|---|
| 0 | `SWEEP_DONE` | A SINGLE sweep reached FSTOP |
| 1 | `SWEEP_WRAP` | A SAW sweep restarted, or an UPDOWN sweep reversed |
| 2 | `UPD_DONE`   | The datapath latched a new configuration |

`IRQ_STAT` is write-1-to-clear; `dds_irq[ch] = |(IRQ_STAT & IRQ_EN)`. A
simultaneous event wins over a clear, so no event is ever lost.

### 5.13 WAVE_RAM (0x2000–0x2FFF)

4096 × 14-bit signed samples, one full period. Only the low 14 bits of each
written word are stored; readback returns them zero-extended. Playback address
is `ph[31:20]`, so the table is traversed once per period at any frequency.

Writing the RAM while `WAVE = user` is live is allowed — the DAC simply plays
the new samples as they land. Load the table with the channel disabled or on a
different waveform if you need a clean switch.

### 5.14 Amplitude-loop block (0x40–0x4D, per channel, `HAS_ADC` only)

Unlike block 0, these are **plain registers with no shadow/commit stage** and
they ignore `UPDATE.AUTO`. The atomic-frame machinery exists to protect values
the datapath consumes every clock (a torn FWORD is audible); the loop reads its
configuration once per measurement window, milliseconds apart, and every field
fits one 16-bit write that cannot tear. `AGC_CTRL.CLR` is the one exception —
it is a strobe deferred to the CS rising edge, so
"clear + retarget + enable written in one frame" does what it reads like.

**AGC_CTRL (0x40)**

| Bit | Name | Description |
|---|---|---|
| 0   | `EN`     | 1 = the loop drives the amplitude; the AMP register (0x07) is overridden but keeps its value. 0 = AMP register rules. |
| 1   | `HOLD`   | 1 = freeze the integrator but keep driving its last output. For "lock, then stop hunting" schemes. |
| 2   | `CLR`    | W1SC. Restart the measurement window and reload the integrator from the AMP register. |
| 5:4 | `SRC`    | 00 = ADC0, 01 = ADC1, 1x = no input (measurement parked). |
| 8   | `METRIC` | 0 = regulate Vpp, 1 = regulate AC RMS. |

**AGC_STATUS (0x41, RO)**

| Bit | Name | Description |
|---|---|---|
| 0 | `LOCKED` | Last window's error was inside `AGC_TOL`. |
| 1 | `SAT_HI` | Integrator railed at `AGC_AMP_MAX` — target unreachable (too high, cable off, wrong SRC…). |
| 2 | `SAT_LO` | Railed at `AGC_AMP_MIN`. |
| 3 | `OTR`    | The ADC flagged over-range during the last window: the input clipped, measurements are lies, and the loop is regulating a clipped copy. Reduce the signal or the chain gain. |

**Semantics worth knowing**

- *Bumpless both ways.* On EN 0→1 (and on CLR) the integrator loads the AMP
  register value, so closing the loop does not step the output. While EN = 1
  the datapath follows the loop live; the instant EN drops it is back on the
  AMP register — which still holds whatever was last written to it.
- *`AGC_AMP` (0x48) is the truth.* It always reads what is actually scaling
  the waveform: the loop's value when closed, the AMP register when open.
- *Dead band.* Inside `AGC_TOL` the integrator does not move at all. Without
  it, the loop would dither the amplitude by one step forever around the
  target; `LOCKED` is simply "currently inside the band".
- *Anti-windup.* The clamps bound the integrator **state**, not just the
  output. An unreachable target therefore parks at the rail and recovers the
  moment the error changes sign, instead of unwinding a huge accumulated error
  first. `AGC_AMP_MIN` also doubles as a "never fully mute" floor if the
  downstream stage misbehaves at zero drive.
- *The measurements are always live* (they do not need EN): `AGC_VPP` /
  `AGC_RMS` / `AGC_MEAN` / `AGC_VMIN` / `AGC_VMAX` update once per window even
  with the loop open, so the block doubles as a crude voltmeter. A new
  `AGC_WIN` value takes effect at the next window boundary — write `CLR` to
  make it now (the power-on window is 2^16 samples ≈ 1.3 ms).

### 5.15 Global block (0x80–0x83, `HAS_ADC` only)

**ADC_CFG (0x81)**, reset 0x0201 — the reset value is correct for the AD9226
module wired per the reference XDC; touch it only for bring-up or odd wiring:

| Bit | Name | Description |
|---|---|---|
| 0   | `EN`      | Run the ADC clock and capture. 0 parks the ADC clock low. |
| 5:4 | `CLKPH`   | Forwarded-clock phase in 5 ns steps. Moves the ADC's edge without moving the capture edge — i.e. walks the capture point across the 20 ns sample period. 0 = 20 ns effective delay (nominal). See `dds_adc_if.v` for the timing derivation. |
| 8   | `BITSWAP` | Reverse the 12-bit bus. The XDC already un-reverses the module's backwards silk screen, so this stays 0 — it exists to rescue a board wired the other way without recutting traces. |
| 9   | `INV`     | Negate the sample. **Reset = 1** because the AD9226 module's front end inverts (D = 2048 − Vin/5 × 2048); with INV = 1 a positive volt at the SMA reads positive here. |

`ADC0_RAW` / `ADC1_RAW` are live single samples (no averaging — a few codes of
jitter is the ADC's noise floor, not a fault). They are the bring-up window
into the analog chain: grounded input ⇒ ~0; a known DC ⇒ the volts-per-code
constant, directly.

---

## 6. Functional Description

### 6.1 Configuration update mechanism

```
 SPI write ──> shadow regs ──┐
                             │  frame_end (CS rising)
 UPDATE.AUTO=1 ──────────────┼──> cfg_apply ──> active regs in the datapath
 or UPDATE.UPD=1 ────────────┘                  (one cycle, all together)
```

Because the commit is one cycle wide and takes every parameter at once, a
retune is **phase-continuous**: the accumulator keeps its phase and only the
increment changes. Nothing glitches, and the DAC never sees a torn 32-bit word.

Soft reset and the sweep START/ABORT strobes are deferred to the same commit
point, which is what makes "config + START in one frame" behave sanely.

### 6.2 Frequency modes

- **FIXED** — the accumulator is driven by `FWORD`.
- **SWEEP** — driven by `FSTART + freq_offset`, where the frequency accumulator
  adds `FDELTA` every `RATE` cycles up to `FSTOP` (tutorial Fig. 11.3). Mode
  SINGLE stops and raises `SWEEP_DONE`; SAW jumps back to FSTART and raises
  `SWEEP_WRAP`; UPDOWN reverses at both ends.
- **PROFILE** — driven by `PROFn_FWORD`, with `PROFn_POW` as the phase offset.
  Switching profiles changes frequency **and** phase in one sample period.

Changing `FMODE` away from SWEEP resets the sweep engine.

### 6.3 Output stage and DAC connection

```
14-bit signed sample
   │  OUT12=1 → round to 12 bits, saturate, place in dac_data[13:2]
   │  FMT_2C=0 → invert the MSB (offset binary)
   │  INV=1    → negate first
   v
dac_data[13:0]   (mid-scale when disabled: 0x2000 offset binary / 0x0000 2's comp)
```

For the AD9764 module in AN01: **offset binary, 14-bit** (`FMT_2C = 0`,
`OUT12 = 0`). See that application note for the data-to-clock timing (forward
the DAC clock with an ODDR so data is stable at the DAC's rising edge).

### 6.4 Spectral notes (from Doc/dds.pdf)

- Phase truncation (32-bit accumulator → 12-bit LUT address after the
  quadrant bits) is the dominant spur mechanism; with 12 address bits the
  worst-case spur sits near −78 dBc, below the 14-bit DAC's own quantization
  floor (≈ −86 dBc SFDR ideal, less in practice).
- Images appear at `k × f_dds ± fout`. Choosing `dds_clk` = 100 MHz puts the
  first image of a 10 MHz output at 90 MHz, far inside the AD9764 module's
  32 MHz filter stopband — this is the whole reason for 100 MHz over 50 MHz.
- The `sin(x)/x` roll-off of the DAC's zero-order hold is −0.9 dB at
  `0.2 × f_dds` and −3.9 dB at `0.5 × f_dds`. Compensate in the analog chain if
  you need flatness above ~20 MHz.

### 6.5 ADC capture and the amplitude loop (`HAS_ADC`)

```
                 ┌────────────────────────────── dds_top ─────────────────────────┐
 AD9226 ─ CLK <──┤ ODDR pattern (dds_clk/2, CLKPH-shiftable)   dds_adc_if         │
 module          │                                                │               │
        ─ D  ──> │ IOB flops (CE = capt_en, board level) ──> bitswap/centre/inv   │
                 │                                                │               │
                 │                     adcN_smp + adc_smp_valid ──┼──> exported   │
                 │                                                v               │
                 │   per channel:   dds_meas ──> vpp / rms / mean / min / max     │
                 │   (SRC-selected)     │  once per 2^WIN samples                 │
                 │                      v                                         │
                 │                 dds_agc:  amp += (TARGET − meas) × KI          │
                 │                      │    dead band, state clamps             │
                 │                      v                                         │
                 │              amplitude override into the s4 multiplier         │
                 └────────────────────────────────────────────────────────────────┘
```

**Sampling.** The AD9226 tops out at 65 MSPS, so `dds_clk` (100 MHz) cannot
clock it directly. The capture unit runs it at **`dds_clk`/2 = 50 MSPS**: the
ADC clock leaves as an ODDR bit pattern and the returning bus is captured in
IOB flops on a clock **enable** — every flop in the design stays on `dds_clk`,
preserving the single-clock/zero-CDC property. The last 30 % of sample rate
would cost an async FIFO and a set of CDC constraints; an amplitude loop that
averages thousands of samples per update gets nothing for that price.

**Measurement.** Over a window of 2^`AGC_WIN` samples, `dds_meas` tracks
min/max (→ Vpp) and accumulates Σx and Σx² (→ mean and variance). RMS is
computed as `sqrt(E[x²] − E[x]²)` — the **AC** RMS — because any DC in the
chain (op-amp offset, or the DDS's own OFFSET register) would otherwise be
folded into "amplitude" and the loop would shrink the real signal to make room
for a DC term it cannot influence. The square root is a 12-cycle bit-serial
unit; no divider anywhere, the window division is a shift.

**The window must span ≥ 1 output period, and several is better.** The
hardware cannot enforce this (it never sees FWORD) — it is the driver's
contract. At 50 MSPS: `AGC_WIN = 16` ⇒ 1.31 ms, good to ~5 kHz; each −1 halves
the window, each +1 doubles it (max 24 ⇒ 335 ms, ~10 Hz).

**Vpp vs RMS.** Vpp is waveform-agnostic and matches a scope, but it is a
two-sample statistic: above ~1 MHz out (≲ 50 samples/period) min/max reads
systematically low and the loop pushes the true amplitude high to compensate.
RMS averages the whole window and stays honest to the top of the band, but the
target depends on the waveform (`Vpp = 2√2·RMS` sine, `2·RMS` square,
`2√3·RMS` triangle). Rule of thumb: **Vpp below ~1 MHz, RMS above**.

**The controller** is a pure integrator — the plant is memoryless over one
window, so integral-only gives zero steady-state error with a single knob.
Loop gain = `KI × k`, where the plant gain `k = M_fs / 32768` (measured
amplitude in ADC codes at AMP = full scale, over the AMP range). On the
reference chain (6 Vpp DA → ×0.5 → ADC at 2.441 mV/code) `M_fs ≈ 1229` Vpp
codes ⇒ `k ≈ 1/26.7` ⇒ **unity loop gain at KI ≈ 26.7**. The reset KI = 6.0
puts the gain near 0.22: the error decays ×0.78 per window, settling in ~20
windows (≈ 26 ms at the default window) with no overshoot. If the loop rings,
KI is too high for your chain; if it crawls, raise it toward a quarter of your
measured unity value.

**Amplitude resolution.** AMP is Q1.15: 32769 steps of 1/32768 (30.5 ppm of
full scale, 183 µV at 6 Vpp). The DAC quantizes at 1/8192 of full scale
(366 µV), so AMP is 4× finer than what the DAC can express — the loop's
resolution limit is the DAC (and the ADC's 2.44 mV codes at the measurement
side), never the multiplier.

---

## 7. Programming Examples

Frames are written as `CMD, D0, D1, …`. `W(ch,addr)` = `0x8000 | ch<<14 | addr`.

The CTRL words below use **offset-binary** output (bit 13 = 0), which is what
the AD9764 module and most current-output DACs want, and what the core powers
up as. For a two's-complement DAC, OR in bit 13 (`| 0x2000`). Getting this
wrong does not silence the DAC — it inverts every sample's MSB, and the sine
comes out chopped in half with a half-scale jump each half period.

### 7.1 Identify the core

```
frame: 0x0000, 0x0000, 0x0000      // read ch0 ADDR 0x00 (CMD, turnaround, data)
       → third word reads 0x4453
```

### 7.2 1 MHz full-scale sine on channel 0 (f_dds = 100 MHz)

```
frame: 0x8004, 0x5C29, 0x028F      // FWORD = 0x028F5C29, both halves atomically
frame: 0x8002, 0x0001              // CTRL: EN, sine, FIXED (offset binary)
```

### 7.3 Retune to 2.5 MHz at half amplitude, +90° phase

```
frame: 0x8004, 0x6666, 0x0666      // FWORD = 0x06666666
frame: 0x8007, 0x4000              // AMP = 0.5
frame: 0x8006, 0x4000              // POW = 90 degrees
```

Each frame commits on its own CS edge; the retune is phase-continuous.

### 7.4 100 kHz square, 30 % duty, offset-binary 12-bit DAC

```
frame: 0x8004, 0x8937, 0x0041      // FWORD = 0x00418937 (100 kHz @ 100 MHz)
frame: 0x8008, 0x4CCD              // DUTY = 30 %
frame: 0x8002, 0x1021              // CTRL: EN | OUT12, square, FIXED, offset binary
```

### 7.5 Sine on a DC bias, or a pure DC level

```
frame: 0x8009, 0x0FA0              // OFFSET = +4000
frame: 0x8007, 0x4000              // AMP = 0.5  → sine of half amplitude around +4000
frame: 0x8007, 0x0000              // AMP = 0    → flat DC at +4000
```

### 7.6 Hardware sweep: 100 kHz → 1 MHz in 10 ms, repeating

```
frame: 0x800C, 0x8937, 0x0041      // FSTART = 0x00418937 (100 kHz)
frame: 0x800E, 0x5C29, 0x028F      // FSTOP  = 0x028F5C29 (1 MHz)
frame: 0x8010, 0x09AA, 0x0000      // FDELTA = 2474
frame: 0x8012, 0x0040, 0x0000      // RATE   = 64 cycles/step  → 15625 steps = 10.0 ms
frame: 0x8002, 0x0101              // CTRL: EN, sine, SWEEP (offset binary)
frame: 0x800B, 0x0101              // SWEEP_CTRL: SAW | START
```

The last frame arms the mode and fires START in one commit. The firmware's
`dds_sweep_config()` computes FDELTA/RATE for you and lands within 0.05 % of the
requested duration.

### 7.7 FSK driven by pins, no CPU in the loop

```
frame: 0x8015, 0x5C29, 0x028F      // PROF0 = 1 MHz
frame: 0x8017, 0xB852, 0x051E      // PROF1 = 2 MHz
frame: 0x8014, 0x0010              // PROF_CTRL: EXT_SEL (follow hop_sel pins)
frame: 0x8002, 0x0201              // CTRL: EN, sine, PROFILE (offset binary)
```

`hop_sel[1:0]` now selects the frequency at the sample rate.

### 7.8 Load and play a user waveform

```
frame: 0xA000, s0, s1, s2, ...     // WAVE_RAM base, address auto-increments
frame: 0x8002, 0x0051              // CTRL: EN, WAVE = user, FIXED (offset binary)
```

Samples are 14-bit two's complement (−8192 … +8191) in the low bits of each
word. One frame can carry as many samples as you like; the firmware splits the
4096-entry table into chunks of 33.

### 7.9 Retune two channels at exactly the same instant

```
frame: 0x800A, 0x0000              // ch0: AUTO = 0
frame: 0xC00A, 0x0000              // ch1: AUTO = 0     (CMD bit14 = channel 1)
frame: 0x8004, ...., ....          // ch0: new FWORD into the shadow
frame: 0xC004, ...., ....          // ch1: new FWORD into the shadow
frame: 0x800A, 0x0001              // ch0: UPD  → commits
frame: 0xC00A, 0x0001              // ch1: UPD  → commits
```

The two commits are still two frames apart (a few µs at 9 MHz SCK). For a
sample-exact simultaneous change, use PROFILE mode and tie both channels'
`hop_sel` pins together.

### 7.10 Close the amplitude loop on channel 0

Reference chain: ch0's DA output reaches ADC0 at ×0.5, ADC LSB = 2.441 mV, so
a 4 Vpp target at the DA is 2 Vpp at the ADC = 819 codes.

```
frame: 0x8004, 0x5C29, 0x028F      // 1 MHz sine, as in 7.2
frame: 0x8002, 0x0001              // CTRL: EN
frame: 0x8042, 0x0333              // AGC_TARGET = 819 codes (4 Vpp at the DA)
frame: 0x8045, 0x0010              // AGC_WIN = 2^16 samples = 1.31 ms (default)
frame: 0x8040, 0x0005              // AGC_CTRL: EN + CLR (fresh window, bumpless)
   ... the loop settles in ~20 windows ...
frame: 0x0041, 0x0000, 0x0000      // read AGC_STATUS -> bit0 LOCKED
frame: 0x0048, 0x0000, 0x0000      // read AGC_AMP    -> ~0x5355
frame: 0x0049, 0x0000, 0x0000      // read AGC_VPP    -> ~819
```

To release: `0x8040, 0x0000` — the datapath is back on the AMP register (which
still holds its last written value) on the very next sample.

Channel 1 regulating from the same ADC0 with the RMS metric instead:

```
frame: 0xC042, 0x0122              // ch1 AGC_TARGET = 290 codes RMS
frame: 0xC040, 0x0105              // ch1 AGC_CTRL: EN + CLR + SRC=ADC0 + METRIC=RMS
```

Both loops run independently — different windows, metrics and targets, no
arbitration, because each channel owns a complete measurement + controller
pair.

---

## 8. Integration

### 8.1 File list

```
rtl/
  dds_top.v         top level - instantiate this (SPI + 1..2 channels + ADC)
  dds_spi_slave.v   SPI mode-0 slave, oversampled; drives the internal bus
  dds_channel.v     one channel: registers + datapath + waveform RAM + loop
  dds_regs.v        16-bit register file, shadow/commit logic
  dds_core.v        phase accumulator, sweep engine, waveforms, output stage
  dds_sin_lut.v     quarter-wave sine ROM (inferred BRAM)
  dds_sin_lut.mem   ROM init data for $readmemh (add to the Vivado project)
  dds_wave_ram.v    user waveform dual-port RAM (inferred BRAM)
  dds_adc_if.v      AD9226 capture: clock pattern, capture enable, bus fixes
  dds_meas.v        per-window Vpp / AC-RMS / mean / min / max
  dds_isqrt.v       bit-serial square root for the RMS path
  dds_agc.v         the integrating amplitude controller
  dds_agc_regs.v    register block 1 (0x40), per channel
  dds_glob_regs.v   register block 2 (0x80), ADC front-end config
tb/
  tb_dds.sv         self-checking testbench with a bit-accurate SPI master
                    and a behavioural AD9226 + analog-chain model
scripts/
  gen_sin_lut.py    regenerates dds_sin_lut.mem
```

### 8.2 Clocking and reset

- One clock: `dds_clk`. Run it at **100 MHz** (AN01 explains why) from the
  board's 50 MHz oscillator via PLL/MMCM.
- The SPI pins are asynchronous inputs and are synchronized inside. The only
  requirement is `f_sck ≤ dds_clk / 6`.
- `dds_rstn` is active-low; assert asynchronously, release synchronously to
  `dds_clk` (standard PLL-locked reset practice).
- Forward the DAC clock from `dds_clk` with an ODDR primitive so `dac_data` is
  stable at the DAC's rising edge (AN01 §4).

### 8.3 Timing constraints

There is no clock-domain crossing left to constrain — the register file and the
datapath share `dds_clk`. Just tell the tool the SPI inputs are asynchronous so
it does not try to time them against a clock that does not exist:

```tcl
set_false_path -from [get_ports {spi_sck spi_cs_n spi_mosi}]
set_false_path -to   [get_ports {spi_miso spi_miso_oe}]
set_false_path -from [get_ports {ch*_sweep_trig ch*_hop_sel[*]}]
```

The synchronizer flops handle metastability; the protocol tolerates the
resulting 2–3 cycle latency by design.

The ADC pins (`HAS_ADC`) are **not** false paths — they are synchronous to the
forwarded ADC clock, and the capture-phase analysis lives in `dds_adc_if.v`'s
header. For the bring-up class of designs here, registering them in IOB flops
(reference `board/dds_board_top.v`) and sweeping `ADC_CFG.CLKPH` over its four
settings replaces formal input constraints; if you want the tool to check it,
write `set_input_delay` against a generated clock on the `adc_clk` port.

### 8.4 Resource estimate (7-series, per channel)

~4 × BRAM18 (sine LUT + wave RAM), 1 × DSP48 (amplitude multiplier), roughly
600 LUT / 900 FF. The shared SPI slave adds ~80 LUT / 120 FF. `HAS_ADC` adds,
per channel, 1 × DSP48 (the Σx² multiplier; the mean-square and error products
map into the same or fabric logic) and ~350 LUT / 400 FF for the accumulators,
square root and controller, plus ~40 LUT shared for the capture unit — no
extra BRAM. Comfortably exceeds 150 MHz on Artix-7 speed grade −1.

### 8.5 Simulation

```
cd tb
iverilog -g2012 -o tb_dds.vvp tb_dds.sv ../rtl/*.v
vvp tb_dds.vvp                    # 75 self-checks -> "ALL TESTS PASSED"
```

The testbench includes a behavioural model of the AD9226 module (inverted
transfer function, TOD data delay) fed by a model of the reference analog
chain (DA → ×0.5 → ADC0, → ×0.25 → ADC1), so the closed-loop tests exercise
convergence, lock, saturation, source switching and the RMS metric end to end.

Add `-DSCK_FAST` to rerun the whole suite with SCK at the documented
`dds_clk / 6` limit (16 MHz). A `tb_dds.vcd` waveform dump is produced.

---

## 9. Known limitations

- The sweep engine ramps upward only (`FSTOP > FSTART`). For a downward chirp,
  use UPDOWN mode and trigger on the reversal, or reprogram FSTART/FSTOP.
- `hop_sel` and `sweep_trig` are sampled in `dds_clk`, so pin-driven hops carry
  2–3 cycles of latency (20–30 ns at 100 MHz).
- The user waveform RAM is a single 4096-entry table per channel; there is no
  double buffering, so rewriting it during playback is audible.
- A read frame costs one turnaround word; there is no way to read a register in
  a two-word frame.
- Reads return the **shadow** register value, which with `AUTO = 0` may not yet
  be what the datapath is using.
- **Channels cannot be started on the same clock edge from SPI.** Two frames
  commit microseconds apart, so after enabling both channels their accumulators
  sit a constant `N × FWORD` apart (constant, but not zero, and it scales with
  FWORD). For a phase-exact I/Q pair, either null the offset once with POW at
  your operating frequency, or add a sync signal in your wrapper that releases
  both channels' `dds_rstn` — or both `CTRL.EN`s — on one edge. See AN01 §9.
- **The amplitude loop trusts its window.** It cannot know the output
  frequency, so a window shorter than one output period measures a slice of
  the waveform and the loop chases the slice — set `AGC_WIN` per §6.5.
- The loop regulates whatever the selected ADC sees. Sweeps and FSK hops move
  the frequency through the analog chain's frequency response, and the loop
  will "correct" that response into the drive amplitude at its window rate —
  usually what you want for levelling, occasionally a surprise. `HOLD` freezes
  the integrator when it is not.
- Vpp measurement degrades above roughly `f_sample / 50` (≈ 1 MHz at 50 MSPS);
  switch the metric to RMS there (§6.5).

---

## 10. Revision history

| Version | Change |
|---|---|
| 0x0210 | Optional ADC feedback (`HAS_ADC`): dual AD9226 capture at `dds_clk`/2 (single-clock, ODDR pattern + capture enable), per-channel Vpp/AC-RMS/mean/min/max measurement, per-channel integrating amplitude loop with dead band, anti-windup and lock/saturation status, amplitude override into the datapath (AMP register semantics unchanged), register blocks 1 (0x40, per channel) and 2 (0x80, global), captured-stream export. |
| 0x0200 | SPI (mode 0, 16-bit) replaces APB. 16-bit register map, dual channel on one chip select, single clock domain, atomic frame commit, AMP moved to Q1.15, SWEEP_RATE widened to 32 bits. |
| 0x0110 | APB version: 32-bit registers, DC offset, dual-clock CDC. |
