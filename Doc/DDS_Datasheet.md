# DDS_TOP — Direct Digital Synthesizer IP Core with APB Interface

**Datasheet, Rev 1.0 — 2026-07-07**

| | |
|---|---|
| Top-level module | `dds_top` ([rtl/dds_top.v](../rtl/dds_top.v)) |
| Bus interface    | APB (32-bit data, style-compatible with `spi_regs.v`) |
| Output           | 12/14-bit parallel DAC data |
| HDL              | Verilog-2001 |
| Verification     | Self-checking testbench ([tb/tb_dds.v](../tb/tb_dds.v)), Icarus Verilog; passes with PCLK = dds_clk and with fully asynchronous clocks |
| Theory reference | *A Technical Tutorial on Digital Signal Synthesis*, Analog Devices 1999 (`Doc/dds.pdf`) |

---

## 1. Features

- 32-bit phase accumulator; output frequency `fout = FWORD × f_dds / 2^32`,
  tuning resolution `f_dds / 2^32` (11.6 mHz at f_dds = 50 MHz).
- Six waveforms: **sine**, **cosine**, **square** (programmable duty cycle,
  16-bit resolution), **triangle**, **ramp** (sawtooth), **user-defined**
  (4096-point × 14-bit RAM, loaded and read back over APB).
- Sine/cosine from a 4096-point quarter-wave LUT (16384 effective points per
  period, 14-bit phase index). Worst-case phase-truncation spur ≈ −84 dBc —
  at/below the quantization floor of a 14-bit DAC, so no dithering or Taylor
  correction is needed at this output resolution.
- **Phase offset**: 16-bit, resolution 360°/65536 ≈ 0.0055°.
- **Digital amplitude scaling** 0 … 1.0 (Q1.16 multiplier ahead of the output
  stage). The absolute peak voltage is set externally in the DAC/filter chain.
- Three frequency-control modes:
  - **FIXED** — one tuning word, glitch-free (phase-continuous) retuning.
  - **SWEEP** — linear/piecewise-linear chirp between two frequencies
    (frequency accumulator + ramp timing logic); single, repeating-sawtooth,
    or up/down modes; software or hardware trigger.
  - **PROFILE** — 4 pre-programmed frequency + phase profiles, selected by
    register or by external pins (direct FSK/PSK from another IP block).
- Atomic, glitch-free parameter updates: all settings are double-buffered and
  transferred to the sample-clock domain in a single cycle (manual or
  automatic update).
- Output conditioning: 14-bit or 12-bit (MSB-aligned), offset-binary or
  two's-complement coding, polarity invert, mid-scale idle when disabled.
- Interrupt output (sweep done / sweep wrap / update done) and hardware
  pacing inputs (`sweep_trig`, `hop_sel[1:0]`) for interaction with other
  IP (SPI, user logic) without CPU involvement.
- Two clock domains with internal CDC: APB `PCLK` and sample clock `dds_clk`
  may be the same PLL output or fully asynchronous.

---

## 2. Block Diagram and Theory of Operation

```
            PCLK domain                │              dds_clk domain
                                       │
 APB ──> shadow registers ──capture──> │ ──sync──> active registers
  │            │            (t_* set,  │            (loaded atomically)
  │            │             UPDATE)   │
  │            └── STATUS/IRQ <──sync─ │ ── events (done/wrap/update)
  │                                    │
  └──> WAVE RAM port A (write/read)    │        ┌────────────────────┐
                                       │        │ frequency source    │
                                       │        │  FIXED : FWORD      │
                                       │        │  SWEEP : FSTART +   │
                                       │        │   freq-accumulator  │
                                       │        │  PROF  : PROFn      │
                                       │        └───────┬────────────┘
                                       │                v
                                       │   ┌── 32-bit phase accumulator
                                       │   │        + (POW << 16)
                                       │   │            v
                                       │   │   ┌─ ¼-wave sine LUT (4096×13)
                                       │   │   ├─ square (phase < DUTY)
                                       │   │   ├─ triangle (folded phase)
                                       │   │   ├─ ramp (phase MSBs)
                                       │   │   └─ WAVE RAM port B (4096×14)
                                       │   │            v  14-bit signed
                                       │   │      × AMP (Q1.16, 0…1.0)
                                       │   │            v  round + saturate
                                       │   │      output stage: 14/12-bit,
                                       │   │      offset-bin / 2's comp, INV
                                       │   │            v
                                       │   └──> dac_data[13:0], dac_valid
```

### 2.1 Tuning equation

The phase accumulator adds the effective frequency word every `dds_clk`
cycle (Doc/dds.pdf, Sec. 1/3):

```
fout        = FWORD × f_dds / 2^32
FWORD       = round(fout × 2^32 / f_dds)
resolution  = f_dds / 2^32
```

Example, f_dds = 50 MHz: 1 MHz → FWORD = round(2^32/50) = 85 899 346 =
`0x051E_B852`. Keep `fout < f_dds/2` (Nyquist); with the intended
≤ 10 MHz outputs from a 50 MHz clock there is ≥ 5× oversampling, which keeps
the DAC reconstruction filter easy.

### 2.2 Waveform phase mapping

All waveforms are derived from the offset phase `ph = phase + (POW << 16)`.
`ph = 0` is the period start.

| Waveform | Value at ph = 0 | Definition |
|---|---|---|
| Sine     | 0, rising       | ¼-wave LUT, quadrant = ph[31:30], addr = ph[29:18] |
| Cosine   | +FS             | sine with +90° (POW + 0x4000 internally) |
| Square   | +FS             | +FS while ph[31:16] < DUTY, else −FS |
| Triangle | −FS, rising     | folded ph, peak +FS at half period |
| Ramp     | −FS, rising     | ph[31:18] as signed, wraps at period end |
| User     | RAM[0]          | RAM address = ph[31:20] |

The LUT stores `sin((i+0.5)/4096 × 90°) × 8191`; the half-LSB offset makes
the quadrant symmetry exact, so the reconstructed sine has no DC offset and
peaks at ±8191.

### 2.3 Amplitude path

`sample(14-bit signed) × AMP(Q1.16)`, rounded to nearest and saturated to
14 bits (Doc/dds.pdf, Fig. 11.4). AMP = 0x10000 is exactly 1.0 and is a true
identity (bit-exact pass-through). One DSP48 implements the multiply.

### 2.4 Pipeline latency

Phase accumulator to `dac_data`: **6 dds_clk cycles**. Constant for all
waveforms and settings; only matters if you align the output with external
events.

---

## 3. Ports

| Port | Dir | Width | Clock | Description |
|---|---|---|---|---|
| `PCLK`      | in  | 1  | —       | APB clock |
| `PRESETn`   | in  | 1  | async   | APB domain reset, active low |
| `PADDR`     | in  | 15 | PCLK    | Byte address (see map; bit 14 selects WAVE RAM) |
| `PSEL`      | in  | 1  | PCLK    | APB select |
| `PENABLE`   | in  | 1  | PCLK    | APB enable (access phase) |
| `PWRITE`    | in  | 1  | PCLK    | 1 = write |
| `PWDATA`    | in  | 32 | PCLK    | Write data |
| `PRDATA`    | out | 32 | PCLK    | Read data |
| `PREADY`    | out | 1  | PCLK    | Tied high — every access completes in one cycle |
| `dds_clk`   | in  | 1  | —       | Sample clock (= DAC clock). May equal PCLK. |
| `dds_rstn`  | in  | 1  | async   | Sample domain reset, active low |
| `sweep_trig`| in  | 1  | async   | Sweep start, rising edge (when SWEEP_CTRL.EXT_TRIG_EN=1). Synchronized internally. |
| `hop_sel`   | in  | 2  | async   | Profile select (when PROF_CTRL.EXT_SEL_EN=1). Synchronized + 2-cycle agreement filter. |
| `dac_data`  | out | 14 | dds_clk | Parallel DAC data, registered |
| `dac_valid` | out | 1  | dds_clk | High while output is live (CTRL.EN, pipeline filled) |
| `dds_irq`   | out | 1  | PCLK    | Level interrupt, `|(IRQ_STAT & IRQ_EN)` |

Parameter `SIN_LUT_FILE` (default `"dds_sin_lut.mem"`): path of the sine LUT
init file for `$readmemh`.

---

## 4. Clocking and Reset

- **Single-clock use (recommended to start):** drive `PCLK` and `dds_clk`
  from the same PLL output (e.g. 50 MHz from the on-board oscillator via
  PLL/MMCM). The internal CDC logic is harmless in this case and no extra
  timing constraints are needed.
- **Dual-clock use:** any frequency relationship is allowed. Constrain the
  crossings (Section 9.2). The double-buffer handshake guarantees the active
  configuration set changes in exactly one `dds_clk` cycle.
- Signals ≤ 10 MHz from a 50 MHz `dds_clk` gives ≥ 5 samples/cycle; if you
  later want cleaner high-frequency sines, raise only `dds_clk` (e.g.
  100–150 MHz) — the design is a short pipeline and closes timing easily.
- Both resets are active-low and may be asserted asynchronously; release them
  synchronously to their domains (normal PLL-locked reset practice).
- The DAC clock pin should be forwarded from `dds_clk` with an ODDR output
  buffer; sample `dac_data` in the DAC on the edge its datasheet specifies.

---

## 5. Register Map

32-bit registers, byte-addressed. Register block decodes `PADDR[7:2]` when
`PADDR[14] = 0`; the waveform RAM occupies `PADDR[14] = 1` (0x4000–0x7FFF).
Reserved bits read 0 and must be written 0. **W1SC** = write-1, self-clearing
(reads 0). **W1C** = write 1 to clear.

| Offset | Name | Access | Reset | Function |
|---|---|---|---|---|
| 0x00 | ID           | RO  | 0x4453_0100 | "DS" + version 1.0.0 |
| 0x04 | CTRL         | RW  | 0x0000_0000 | Enable, waveform, frequency mode, output format |
| 0x08 | STATUS       | RO  | 0x0000_0000 | Sweep/profile/update status |
| 0x0C | FWORD        | RW  | 0x0000_0000 | Frequency tuning word (FIXED mode) |
| 0x10 | POW          | RW  | 0x0000_0000 | Phase offset word |
| 0x14 | AMP          | RW  | 0x0001_0000 | Amplitude scale (reset = 1.0) |
| 0x18 | DUTY         | RW  | 0x0000_8000 | Square duty threshold (reset = 50 %) |
| 0x1C | UPDATE       | RW  | 0x0000_0002 | Update strobe / auto-update (reset: AUTO on) |
| 0x20 | SWEEP_CTRL   | RW  | 0x0000_0000 | Sweep mode, trigger source, START/ABORT |
| 0x24 | SWEEP_FSTART | RW  | 0x0000_0000 | Sweep start tuning word (f1) |
| 0x28 | SWEEP_FSTOP  | RW  | 0x0000_0000 | Sweep stop tuning word (f2 ≥ f1) |
| 0x2C | SWEEP_FDELTA | RW  | 0x0000_0000 | Frequency increment per ramp step |
| 0x30 | SWEEP_RATE   | RW  | 0x0000_0001 | dds_clk cycles per ramp step |
| 0x34 | PROF_CTRL    | RW  | 0x0000_0000 | Profile select source & index |
| 0x38 | PROF0_FWORD  | RW  | 0x0000_0000 | Profile 0 tuning word |
| 0x3C | PROF1_FWORD  | RW  | 0x0000_0000 | Profile 1 tuning word |
| 0x40 | PROF2_FWORD  | RW  | 0x0000_0000 | Profile 2 tuning word |
| 0x44 | PROF3_FWORD  | RW  | 0x0000_0000 | Profile 3 tuning word |
| 0x48 | PROF_POW10   | RW  | 0x0000_0000 | [15:0] profile 0 phase, [31:16] profile 1 phase |
| 0x4C | PROF_POW32   | RW  | 0x0000_0000 | [15:0] profile 2 phase, [31:16] profile 3 phase |
| 0x50 | IRQ_EN       | RW  | 0x0000_0000 | Interrupt enables |
| 0x54 | IRQ_STAT     | W1C | 0x0000_0000 | Interrupt flags |
| 0x4000–0x7FFC | WAVE_RAM | RW | undefined | 4096 × 32-bit words; sample in [13:0]. Entry n at 0x4000 + 4n |

### 5.1 CTRL (0x04)

| Bits | Field | Description |
|---|---|---|
| 0     | EN        | 1 = run. 0 = phase/frequency accumulators freeze (state kept), output forced to mid-scale, `dac_valid` low. **Takes effect immediately**, without an UPDATE. |
| 1     | SRST      | W1SC. Soft reset: clears the phase accumulator, sweep state and IRQ_STAT. Register contents are kept. |
| 6:4   | WAVE_SEL  | 0 sine · 1 cosine · 2 square · 3 triangle · 4 ramp · 5 user RAM · 6–7 reserved |
| 9:8   | FREQ_MODE | 0 FIXED · 1 SWEEP · 2 PROFILE · 3 reserved |
| 12    | OUT_WIDTH | 0 = 14-bit; 1 = 12-bit, MSB-aligned: the rounded 12-bit result drives `dac_data[13:2]`, `dac_data[1:0]` = 0. Connect a 12-bit DAC to `dac_data[13:2]`. |
| 13    | OUT_FMT   | 0 = offset binary (mid-scale 0x2000); 1 = two's complement. |
| 14    | OUT_INV   | 1 = invert output polarity (saturating negate). |

All CTRL fields except EN and SRST travel through the double-buffer (Sec. 6.1).

### 5.2 STATUS (0x08, read-only)

| Bits | Field | Description |
|---|---|---|
| 0   | SWEEP_ACTIVE | Sweep running. |
| 1   | SWEEP_DIR    | 0 = up, 1 = down (up/down mode). |
| 3:2 | ACTIVE_PROF  | Profile currently applied. Informational; bits are synchronized independently and may tear for one read during a hop. |
| 4   | UPD_PENDING  | 1 = a shadow→active transfer is in flight. Poll until 0 before relying on new settings (or use the UPD_DONE interrupt). |

### 5.3 FWORD (0x0C) — see tuning equation, Sec. 2.1.

### 5.4 POW (0x10)

| Bits | Field | Description |
|---|---|---|
| 15:0 | POW | Phase offset = POW/65536 × 360°. 0x4000 = 90°, 0x8000 = 180°. Applies in FIXED and SWEEP modes (PROFILE mode uses the per-profile phase). |

### 5.5 AMP (0x14)

| Bits | Field | Description |
|---|---|---|
| 16:0 | AMP | Q1.16 amplitude: gain = AMP/65536. 0x00000 = 0 (output at code mid-scale), 0x08000 = 0.5, 0x10000 = 1.0 (exact, bit-transparent). Writes above 0x10000 are stored clamped to 0x10000. |

### 5.6 DUTY (0x18)

| Bits | Field | Description |
|---|---|---|
| 15:0 | DUTY | Square high-time fraction = DUTY/65536 (resolution ≈ 0.0015 %). 0x0000 = constant −FS; exactly 100 % is not reachable (max 65535/65536). Ignored for other waveforms. |

### 5.7 UPDATE (0x1C)

| Bits | Field | Description |
|---|---|---|
| 0 | UPD  | W1SC. Captures all shadow settings and transfers them atomically to the dds_clk domain. |
| 1 | AUTO | 1 (reset default) = every write to a configuration register triggers the capture automatically. 0 = writes accumulate until UPD is written. |

### 5.8 SWEEP_CTRL (0x20)

| Bits | Field | Description |
|---|---|---|
| 1:0 | MODE        | 0 = single (f1→f2, then hold f2, SWEEP_DONE) · 1 = sawtooth repeat (wrap to f1, SWEEP_WRAP per lap) · 2 = up/down (f1→f2→f1→…, SWEEP_WRAP per reversal) · 3 reserved |
| 2   | EXT_TRIG_EN | 1 = start on `sweep_trig` rising edge instead of the START bit. |
| 8   | START       | W1SC. Software start (FREQ_MODE must be SWEEP). Restarting an active sweep restarts it from f1. |
| 9   | ABORT       | W1SC. Stops the sweep; output returns to f1. |

**Sweep behaviour** (Doc/dds.pdf Fig. 11.3): an internal frequency
accumulator starts at 0 and adds FDELTA every RATE `dds_clk` cycles; the
phase-accumulator input is FSTART + accumulator, clamped so it never exceeds
FSTOP. Requirements: FSTOP ≥ FSTART, FDELTA ≥ 1. Timing:

```
steps          = ceil((FSTOP − FSTART) / FDELTA)
sweep duration = steps × RATE / f_dds
```

Writing FDELTA mid-sweep (it re-arrives via the normal update mechanism)
changes the slope without restarting — piecewise-linear/nonlinear chirps.
Selecting FREQ_MODE ≠ SWEEP force-stops any active sweep.

### 5.9 SWEEP_RATE (0x30)

| Bits | Field | Description |
|---|---|---|
| 23:0 | RATE | dds_clk cycles between frequency steps. 0 and 1 both mean every cycle. Max 16 777 215 (0.34 s per step at 50 MHz). |

### 5.10 PROF_CTRL (0x34)

| Bits | Field | Description |
|---|---|---|
| 1:0 | SEL        | Active profile when EXT_SEL_EN = 0. |
| 4   | EXT_SEL_EN | 1 = active profile follows `hop_sel[1:0]`. Profile switches are phase-continuous. In PROFILE mode the phase offset comes from the selected profile's POW field, enabling PSK as well as FSK. |

### 5.11 IRQ_EN (0x50) / IRQ_STAT (0x54)

Identical layout. A flag sets regardless of the enable; the enable only gates
`dds_irq`. Clear by writing 1 to the flag (or CTRL.SRST clears all).

| Bit | Flag | Set when |
|---|---|---|
| 0 | SWEEP_DONE | Single-mode sweep reached FSTOP. |
| 1 | SWEEP_WRAP | Repeating sweep wrapped / up-down sweep reversed. |
| 2 | UPD_DONE   | A configuration transfer was applied in the dds_clk domain. |

### 5.12 WAVE_RAM (0x4000–0x7FFC)

4096 words; bits [13:0] hold one two's-complement sample at DAC full scale
(−8192…+8191); bits [31:14] are ignored and read 0. Playback address is
`ph[31:20]`, so the RAM holds exactly one waveform period. Reads and writes
are allowed at any time with zero wait states; to avoid output glitches,
reload the RAM while WAVE_SEL ≠ 5 or EN = 0. Contents are **not** cleared by
reset and are undefined at power-up — initialize before selecting the user
waveform. AMP scaling and output formatting apply to RAM samples exactly as
to the built-in waveforms.

---

## 6. Functional Description

### 6.1 Configuration update mechanism

Every APB write lands in a *shadow* register immediately (and reads back from
there). The datapath, however, only sees the *active* set in the `dds_clk`
domain, which is reloaded — all fields at once, in one cycle — when a capture
event crosses the domain boundary:

1. Write with AUTO = 1, or write UPDATE.UPD = 1.
2. One PCLK cycle later the shadows are captured into a transfer set and a
   toggle crosses to `dds_clk` (STATUS.UPD_PENDING = 1).
3. The core loads the active registers, flips an acknowledge toggle back.
4. UPD_PENDING clears; IRQ_STAT.UPD_DONE sets.

A request arriving while a transfer is in flight is remembered and re-issued
once, so the final state always matches the last write; nothing is lost.
Latency is ~4 PCLK + ~4 dds_clk cycles. Frequency, phase, amplitude and
waveform changes are **phase-continuous**: the phase accumulator is never
reset by an update (only by CTRL.SRST or hardware reset).

Recommended driver pattern for multi-register changes (e.g. a whole sweep
setup): write UPDATE = 0 (AUTO off), program everything, write UPDATE = 1
(UPD), then either poll STATUS.UPD_PENDING = 0 or wait for UPD_DONE before
issuing START. Single-register tweaks (retune FWORD, change AMP) are safe
with AUTO on — one write, done.

START/ABORT/SRST strobes are ordered after the configuration capture
internally, so a single SWEEP_CTRL write carrying mode bits *and* START (with
AUTO on) applies the mode first. Still, separating configuration from START
is the cleaner driver style.

### 6.2 Frequency modes

**FIXED.** Output frequency = FWORD. Retune by writing FWORD; the transition
is instantaneous and phase-continuous (Doc/dds.pdf Sec. 3 — suited to
hopping/GMSK-style signalling at APB speed).

**SWEEP.** See 5.8. The chirp is generated entirely in hardware; the CPU
only arms it. With EXT_TRIG_EN another IP block (timer, SPI event, radar
frame pulse on `sweep_trig`) starts sweeps with cycle-level determinism.

**PROFILE.** Four {FWORD, POW} pairs are pre-programmed; hopping between
them costs zero APB traffic when `hop_sel` pins drive the selection —
maximum hop rate is limited only by the input synchronizer (~4 dds_clk
cycles). 2-FSK uses profiles 0/1 on `hop_sel[0]`; 4-FSK/QPSK uses all four.

### 6.3 Output stage and DAC connection

Processing order: waveform sample → INV → × AMP → round/saturate →
12-bit rounding (if OUT_WIDTH) → coding (OUT_FMT) → register → `dac_data`.

- **14-bit DAC:** connect `dac_data[13:0]` directly.
- **12-bit DAC:** set OUT_WIDTH = 1, connect the DAC to `dac_data[13:2]`.
  The 14→12 conversion is round-to-nearest with saturation (not truncation),
  preserving ~0.25 LSB of accuracy.
- When EN = 0, `dac_data` holds mid-scale in the selected coding (0x2000
  offset-binary / 0x0000 two's-complement) and `dac_valid` is low — the DAC
  rests at 0 V differential.

### 6.4 Spectral notes (from Doc/dds.pdf)

- Aliased images appear at `k·f_dds ± fout` (Sec. 2); the analog
  reconstruction filter after the DAC must attenuate them. At ≥ 5×
  oversampling the first image is ≥ 3× fout away — an easy filter.
- sin(x)/x droop at fout = 10 MHz, f_dds = 50 MHz is ≈ −0.58 dB; compensate
  in the DAC filter if flatness matters.
- Phase truncation spurs ≤ −84 dBc (14-bit phase index); DAC quantization
  dominates. Clock jitter on `dds_clk` translates directly to output phase
  noise — use a clean PLL output, not a ring-oscillator clock.

---

## 7. APB Interface and Programming Examples

### 7.1 Protocol

Standard APB write: **setup** cycle (PSEL = 1, PENABLE = 0, address/data/
PWRITE valid) followed by one **access** cycle (PENABLE = 1). PREADY is tied
high, so every transfer — including WAVE_RAM — completes in the access cycle
with zero wait states (RAM reads are pre-fetched during setup). The core is
compatible with any APB3/APB4 master; PSTRB/PPROT/PSLVERR are not used.

```
PCLK    ──┐_┌─┐_┌─┐_┌─
PSEL    ___┌────────┐___
PENABLE _______┌────┐___
PADDR   ───X  addr   X──
PWDATA  ───X  data   X──   (writes)
PRDATA  ───────X data X─   (reads, valid in access cycle)
```

All examples below assume f_dds = 50 MHz and a base address of 0 (add your
fabric's base offset).

### 7.2 Generate a 1 MHz sine, full amplitude

```
FWORD = round(1e6 × 2^32 / 50e6) = 0x051E_B852

write 0x0C, 0x051EB852     // FWORD          (AUTO on: applied immediately)
write 0x04, 0x00002001     // CTRL: EN, sine, FIXED, 14-bit, two's complement
```

### 7.3 Retune to 2.5 MHz, half amplitude, +90° phase

```
write 0x0C, 0x0CCCCCCD     // FWORD = 2.5e6 × 2^32 / 50e6
write 0x14, 0x00008000     // AMP  = 0.5
write 0x10, 0x00004000     // POW  = 90°
```
Each write is applied on the fly, phase-continuously.

### 7.4 100 kHz square wave, 30 % duty, on a 12-bit offset-binary DAC

```
write 0x0C, 0x0083126F     // FWORD = 100e3 × 2^32 / 50e6
write 0x18, 0x00004CCD     // DUTY  = 0.30 × 65536
write 0x04, 0x00001021     // CTRL: EN, square, 12-bit, offset binary
```

### 7.5 Hardware-timed sweep: 100 kHz → 1 MHz in 10 ms, repeating

```
write 0x1C, 0x00000000     // UPDATE: AUTO off — program the set atomically
write 0x24, 0x0083126F     // FSTART = 100 kHz
write 0x28, 0x051EB852     // FSTOP  = 1 MHz
write 0x30, 50             // RATE: one step per µs (50 cycles @ 50 MHz)
write 0x2C, 7731           // FDELTA = (FSTOP−FSTART)/10000 steps
write 0x20, 0x00000001     // SWEEP_CTRL: sawtooth repeat mode
write 0x04, 0x00002101     // CTRL: EN, sine, SWEEP mode
write 0x50, 0x00000003     // IRQ_EN: DONE+WRAP
write 0x1C, 0x00000001     // UPDATE.UPD — commit everything
read  0x08 until bit4==0   // wait UPD_PENDING clear
write 0x20, 0x00000101     // START (mode bits unchanged)
...
// each lap: IRQ fires, handler does:
read  0x54                 //   IRQ_STAT, see bit1 (SWEEP_WRAP)
write 0x54, 0x00000002     //   W1C
```

### 7.6 FSK driven by another IP block, no CPU in the loop

```
write 0x38, FWORD_space    // PROF0_FWORD
write 0x3C, FWORD_mark     // PROF1_FWORD
write 0x34, 0x00000010     // PROF_CTRL: EXT_SEL_EN
write 0x04, 0x00002201     // CTRL: EN, sine, PROFILE mode
// wire the data line (e.g. from your SPI/user logic) to hop_sel[0]
```

### 7.7 Load and play a user-defined waveform

```
write 0x04, 0x00002000     // EN=0 while loading (optional but glitch-free)
for n in 0..4095:
    write 0x4000 + 4*n, sample[n] & 0x3FFF   // 14-bit two's complement
read  0x4000 + 4*k         // optional readback verify
write 0x0C, FWORD          // playback rate: RAM sweeps once per output period
write 0x04, 0x00002051     // CTRL: EN, user wave
```

### 7.8 Interrupt-driven update confirmation

```
write 0x50, 0x00000004     // IRQ_EN: UPD_DONE
write 0x0C, new_fword      // AUTO update
// on dds_irq: read 0x54, write 0x54 = 0x4 — new frequency is now live
```

---

## 8. Integration

### 8.1 File list

| File | Content |
|---|---|
| `rtl/dds_top.v`      | Top level (instantiate this) |
| `rtl/dds_regs.v`     | APB register file, PCLK-side CDC |
| `rtl/dds_core.v`     | Datapath, sweep engine, dds_clk-side CDC |
| `rtl/dds_sin_lut.v`  | Quarter-wave sine ROM |
| `rtl/dds_sin_lut.mem`| ROM init data (regenerate: `scripts/gen_sin_lut.py`) |
| `rtl/dds_wave_ram.v` | User waveform dual-port RAM |
| `rtl/dds_cdc.v`      | Toggle-pulse and 2-FF synchronizers |
| `tb/tb_dds.v`        | Self-checking testbench |

Add `dds_sin_lut.mem` to the Vivado project (as a design source or in the
simulation/synthesis search path), or set the `SIN_LUT_FILE` parameter to an
absolute path.

### 8.2 Timing constraints (only when PCLK ≠ dds_clk)

The transfer-register buses are quasi-static when sampled (guarded by the
toggle handshake); constrain them as such and mark the synchronizers:

```tcl
# synchronizer flops
set_property ASYNC_REG TRUE [get_cells -hier -regex .*u_sync_.*/sync_reg.*]

# quasi-static config bus: bound skew to one destination period
set_max_delay -datapath_only \
  -from [get_cells u_dds/u_regs/t_*_reg*] \
  -to   [get_cells u_dds/u_core/a_*_reg*] \
  [get_property PERIOD [get_clocks -of [get_ports dds_clk]]]
```

With PCLK and dds_clk tied to the same PLL output, skip both — everything is
synchronous.

### 8.3 Resource estimate (7-series)

| Resource | Count | Use |
|---|---|---|
| BRAM18 | 4 | sine LUT (4096×13), wave RAM (4096×14) |
| DSP48  | 1 | amplitude multiplier |
| LUT/FF | ≈ 600 / ≈ 900 | accumulators, sweep engine, registers |

Fmax on Artix-7 (-1): comfortably > 150 MHz on `dds_clk` (longest paths are
the 32-bit adds, all single-level).

### 8.4 Simulation

```
cd tb
iverilog -g2001 -o tb_dds.vvp tb_dds.v ../rtl/*.v
vvp tb_dds.vvp                          # 21 checks, "ALL TESTS PASSED"
iverilog -g2001 -DASYNC_CLKS ...        # same suite with 125 MHz async dds_clk
```

A VCD (`tb_dds.vcd`) is dumped for waveform inspection.

---

## 9. Known limitations

- FSTOP < FSTART or FDELTA = 0 in SWEEP mode is not rejected in hardware;
  the sweep clamps immediately / never advances. Program sane values.
- WAVE_RAM is uninitialized at power-up.
- ACTIVE_PROF in STATUS may momentarily tear during a hop (read-only,
  informational).
- Exactly 100 % square duty is unreachable (65535/65536 max); use AMP = 0 +
  OUT_INV or a profile trick if a DC level is ever needed.

## 10. Revision history

| Rev | Date | Notes |
|---|---|---|
| 1.0 | 2026-07-07 | Initial release. Verified with Icarus Verilog, sync and async clock configurations. |
