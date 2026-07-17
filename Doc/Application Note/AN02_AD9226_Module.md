# AN02 — ADC Feedback with the Dual AD9226 Module

**Application note, Rev 1.0 — 2026-07-17. Supplement to
[DDS_Datasheet.md](../DDS_Datasheet.md) §5.14/5.15/6.5 and a companion to
[AN01](AN01/AN01_AD9764_Module.md).**

This note covers everything specific to the ADC hardware behind the core's
`HAS_ADC` option: the *dual AD9226 high-speed ADC module* (Lingzhi Electronics,
V4.1 LQFP / V3.0 SSOP — electrically identical), used as the feedback path for
closed-loop amplitude control. Vendor documentation lives in
[`双路AD9226（12ADC-65M）模块资料V4.1/`](./双路AD9226（12ADC-65M）模块资料V4.1/).

## 1. Module summary

Signal chain per channel:

```
SMA input (±5 V, 50 Ω) → attenuator (÷5, inverting) → conditioning (1–3 V)
  → AD9226 (12-bit, ≤ 65 MSPS, parallel) → 12-bit bus + OTR on the header
```

| Property | Value | Consequence |
|---|---|---|
| Channels | 2 (AD1 / AD2), each own bus + clock | One capture unit, both sampled together |
| Resolution | 12 bit over −5 … +5 V | 2.441 mV / code |
| Max sample rate | 65 MSPS | Run at `dds_clk`/2 = 50 MSPS (§3) |
| Transfer | **D = 2048 − Vin/5 × 2048** — inverting! | `ADC_CFG.INV = 1` (reset default) undoes it |
| Bus numbering | **Backwards**: silk D0 = MSB … D11 = LSB | Un-reversed in the XDC (§4) |
| Data timing | valid 3.5–7 ns (TOD) after CLK rise | Capture point analysis in §5 |
| Input impedance | 50 Ω | A source that assumes a high-Z load sees its voltage halved |
| Logic level | 3.3 V | Matches the FPGA bank |
| Supply | +5 V, no over-voltage protection | Same low-ripple rule as the DA module |

## 2. The two traps, up front

These two facts cost the most bring-up time, so they come before anything else:

**Trap 1 — the bus is numbered backwards.** The AD9226 datasheet calls its MSB
"BIT1" and its LSB "BIT12", and the module's silk screen follows the chip:
**header pin D0 is the MSB and D11 is the LSB** — the exact opposite of every
other convention. The reference XDC template maps silk D0 → `adc0_data[11]`
down to silk D11 → `adc0_data[0]`, so inside the FPGA the bus is normal and
`ADC_CFG.BITSWAP` stays 0. The bit-reversal of a mis-wired bus is not subtle
noise, it is complete garbage that still *moves* with the input — the
signature check in §6 catches it in one read.

**Trap 2 — the front end inverts.** `D = 2048 − Vin/5 × 2048`: +5 V in reads
code 0, −5 V reads 4095. `ADC_CFG.INV` (reset = 1) negates the centred sample
so that a positive volt at the SMA reads as a positive number everywhere in
the core. Vpp and RMS would survive a wrong sign; the `AGC_MEAN` readout, DC
calibration and anything phase-sensitive would not.

## 3. Clocking: 50 MSPS from the 100 MHz core clock

The module's maximum is 65 MSPS and `dds_clk` is 100 MHz, so the core clocks
both ADCs at **`dds_clk`/2 = 50 MSPS**. The divided clock is produced as an
ODDR bit pattern by `dds_adc_if` and leaves the fabric through ODDRs in
`dds_board_top` — the same "clocks leave through clock resources, never
fabric routing" rule as the DAC clocks (AN01 §4). ACLK and BCLK are separate
header pins, so the board top provides `adc0_clk` and `adc1_clk`, one ODDR
each; both ODDRs are fed the identical bit pattern and launch from the same
`dds_clk` edge, so the two ADCs sample in lockstep and one `CLKPH` setting
times both.

Why not 65 MSPS from a second PLL output: that is a genuine second clock
domain — an async FIFO on the data path and CDC constraints on everything it
touches — for 30 % more samples that an amplitude loop, which averages
thousands of samples per update, cannot use. The whole design keeps exactly
one clock; the loop keeps that property.

What 50 MSPS means for the loop:

- Nyquist at 25 MHz covers the DA module's entire useful band (its filter
  cuts off at 32 MHz, and AN01 already recommends staying below ~20 MHz out).
- The Vpp metric is sample-starved above ~1 MHz output (< 50 samples/period)
  and reads systematically low; switch `AGC_CTRL.METRIC` to RMS there
  (datasheet §6.5 has the numbers).

## 4. Wiring

| dds_board_top port | Module pin | Note |
|---|---|---|
| `adc0_clk` | ACLK | 50 MHz; same pattern as BCLK, same phase |
| `adc1_clk` | BCLK | |
| `adc0_data[11]` … `adc0_data[0]` | AD1 silk D0 … D11 | **reversed on purpose** — silk D0 is the MSB |
| `adc0_otr` | ATR | over-range flag; optional, tie the port 0 if unwired |
| `adc1_data[11]` … `adc1_data[0]` | AD2 silk D0 … D11 | same reversal |
| `adc1_otr` | BTR | optional |
| GND | GND | common ground with the FPGA **and** the signal source — the vendor manual insists, correctly |

A pin-constraint template with the mapping spelled out per bit sits at the end
of [`board/DDS_Constraints.xdc`](../../board/DDS_Constraints.xdc); fill in the
`PACKAGE_PIN`s for your wiring and uncomment it.

Input side: the module input is 50 Ω. If a signal source drives it directly,
set the source to expect a 50 Ω load or its set amplitude arrives halved (the
vendor manual's warning). In the closed-loop chain here the buffer stage
drives it, and that halving is part of the calibrated `DDS_FB_GAIN`.

## 5. Capture timing and ADC_CFG.CLKPH

The AD9226 puts data on the bus **TOD = 3.5 … 7 ns after the rising edge** of
its clock. With ±2 ns of ribbon delay each way, the data for edge *t* is
stable at the FPGA pins in roughly **[t+11, t+27.5] ns** — a ~16 ns window in
the 20 ns sample period.

The capture flops (IOB, clock-enabled, in `dds_board_top`) always load on the
same `dds_clk` edge; what moves is the **forwarded clock**. `ADC_CFG.CLKPH`
shifts the ADC's clock in 5 ns steps, which walks the effective capture delay
across the whole period:

| CLKPH | effective capture delay after the ADC edge | verdict |
|---|---|---|
| 0 | 20 ns | nominal — inside the window with ~9 ns setup / ~7.5 ns hold margin |
| 1 | 15 ns | fine, less hold margin |
| 2 | 10 ns | on the edge of the data-valid window |
| 3 | 5 ns  | data still transitioning — wrong for this module |

**Reset default 0 is correct for a short ribbon.** If a long cable shifts the
window: feed a slow full-scale sine, and step CLKPH through 0–3 while watching
`ADC0_RAW` — wrong phases show sparkle codes (single-bit spikes) or a noisy
`AGC_VPP`. Pick the setting with clean readings; if none is clean the cable is
too long for static timing, shorten it (the escape hatch — a phase-tuned
second PLL output for the capture flops — is documented in `dds_adc_if.v`).

The full slot-by-slot derivation, including why the capture lands 20 ns after
the edge whose data it takes, is in the header of `rtl/dds_adc_if.v`.

## 6. Bring-up checklist

In order; each step isolates a different failure:

1. **Bitstream has the ADC?** `reg 80` → 0x4147 (`GLOB_ID`, "AG"). The MCU
   console prints a warning at boot if not. 0x0000 = built with `HAS_ADC = 0`.
2. **Grounded input reads zero.** Short the SMA (or just terminate it):
   `adc` → both channels within a few codes of 0. A few codes of jitter is the
   noise floor, not a fault.
   - Reads a constant **+2047**? That is the §2 bit-reversal signature (a 0 V
     input's code 2048 = `1000_0000_0000` bit-reversed is 1). The bus is
     wired MSB-for-LSB: fix the XDC, or set `ADC_CFG.BITSWAP = 1` to rescue
     the board in place.
   - Reads ±2047/±2048 stuck? A missing clock — check ACLK/BCLK.
3. **A known DC has the right value and sign.** With the loop chain connected:
   `offset 0.25`, `on`, `wave sine`, `freq 0` — the DA outputs +0.75 V DC.
   `adc` should read `0.75 × FB_GAIN / 2.441 mV` ≈ +154 codes for the
   reference ×0.5 chain. Right value, wrong sign → someone flipped
   `ADC_CFG.INV`.
4. **AC measurement is sane.** `freq 10k`, `amp 1`, `on`, then `meas`:
   Vpp ≈ 1229 codes (6 Vpp × 0.5 / 2.441 mV), RMS ≈ Vpp/2.83 for a sine,
   mean ≈ 0.
5. **Close the loop.** `agc target 4Vpp`, `agc on`, then `agc`: LOCKED within
   tens of windows, `AGC_VPP` on target. `SAT_HI` instead = the target is
   unreachable — wrong SRC, dead cable, or a target above the chain's range.
6. **Check OTR is quiet.** `meas` flagging OVER-RANGE means the ADC input
   exceeds ±5 V and every number is a lie — reduce the amplitude or the chain
   gain before trusting anything.

## 7. Calibration

The FPGA loop works entirely in ADC codes; **volts exist only in the MCU
firmware**, as three constants in `dds.h`:

| Constant | Reference value | Meaning |
|---|---|---|
| `DDS_DA_FS_VPP` | 6.0 | DA open-circuit full swing (AN01 §5; re-measure if the reference jumper/pot moves) |
| `DDS_FB_GAIN` | 0.5 | DA volts → ADC-input volts through buffer + matching + amplifier |
| `DDS_ADC_FS_VPP` | 10.0 | AD9226 module span, fixed by the hardware |

Two-step calibration, five minutes with a multimeter:

1. **DA scale** (AN01 §6): `amp 0`, `reg 09 1000` (OFFSET = +4096), measure
   V_dc at the DA SMA with a high-impedance meter.
   `DDS_DA_FS_VPP = V_dc / 4096 × 16384`.
2. **Feedback gain**: with the same DC applied and the chain connected, read
   `adc` → N codes. `DDS_FB_GAIN = N × 2.441 mV / V_dc`.

Update the two macros, rebuild the firmware — done. The bitstream is not
involved. After calibration, `amp 2.0V`, `agc target 4Vpp` and the `meas`
voltages all refer to the DA SMA **open-circuit**; into a 50 Ω load every
number halves (AN01 §7).

The reference chain (`FB_GAIN = 0.5`) uses only 3 Vpp of the ADC's 10 Vpp
span — about 10.3 effective bits, 1229 of 4095 codes at full DA swing. That is
plenty for a level loop (the dead band is ±2 codes ≈ ±10 mV), but if the
buffer chain is ever redesigned, aiming closer to `FB_GAIN ≈ 1.2` would use
~7 Vpp of the span and better the measurement SNR by ~8 dB. Keep at least
25 % headroom below OTR.

## 8. Using the ADCs for other things

The captured streams leave `dds_top` as `adc0_smp` / `adc1_smp` (12-bit
signed, centred, sign-corrected) with a shared `adc_smp_valid` strobe at
50 MSPS — always live, whether any loop uses them or not. A scope-capture
block, a digital filter or a demodulator taps them directly in the `dds_clk`
domain with no CDC.

Corollary: if another fabric module *drives* one of the DA channels through an
external mux, the DDS cannot see that channel's output — leave that channel's
`AGC_CTRL.EN` off, and its measurement registers still work as a voltmeter on
whichever ADC its `SRC` selects.
