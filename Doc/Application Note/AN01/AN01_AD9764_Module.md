# AN01 — Using DDS_TOP with the AD9764 High-Speed DA Module

**Application note, Rev 1.0 — 2026-07-07. Supplement to [DDS_Datasheet.md](DDS_Datasheet.md).**

This note covers everything specific to the target DAC hardware: the
*Lingzhi Electronics (凌智电子) dual-channel AD9764 module, Bessel-filter
version* (user manual V2.0, schematic V1.1). The core datasheet deliberately
describes only the IP itself; board-level wiring, coding, clocking, voltage
scaling and analog limitations of this module live here.

## 1. Module summary

Signal chain per channel:

```
AD9764 (14-bit, 125 MSPS, complementary current outputs IOUTA/IOUTB)
  → OPA690 differential-to-single-ended stage (×2.44)
  → 7th-order 32 MHz Bessel LC low-pass filter
  → OPA690 output amplifier (×3.44)
  → 50 Ω series matching resistor → SMA
```

Key facts that drive the integration choices below:

| Property | Value | Consequence |
|---|---|---|
| Channels | 2, fully independent (U3/U6), each with own D[13:0] + CLK on the 34-pin header | Two `dds_top` instances for dual output |
| Input coding | Straight (natural) binary: code 0 = −FS, 8192 = 0 V, 16383 = +FS | Use **CTRL.OUT_FMT = 0** (offset binary, the reset default) |
| Logic level | 3.3 V | Match the FPGA bank voltage |
| Output range | −3 V … +3 V, max ≈ 6 Vpp, **DC-coupled end to end** | DC offsets pass through; the OFFSET register works as a true bias |
| Output impedance | 50 Ω source | Amplitude halves into a 50 Ω load (see §6) |
| Filter | 32 MHz cutoff Bessel, designed for a 100 MHz DAC clock | Run `dds_clk` at 100 MHz (see §3) |
| Data timing | Latched on CLK rising edge, ≥ 2 ns data setup | Forward an inverted/phase-shifted clock (see §4) |
| Supply | 5 V (max!), reverse-polarity protected only | Low-ripple supply strongly preferred |

## 2. Wiring and core configuration

| DDS_TOP signal | Module pin | Note |
|---|---|---|
| `dac_data[13:0]` | DB13 (MSB) … DB0 (LSB) | Direct, one channel |
| `dds_clk` (forwarded, see §4) | CLKA / CLKB | One clock per channel |
| — | SLEEP, REFLO, FS ADJ | Handled on-board, not on the header |

Required register settings for this module:

```
CTRL.OUT_FMT   = 0   // straight binary (reset default — do NOT set two's complement)
CTRL.OUT_WIDTH = 0   // 14-bit mode, full bus connected
```

With EN = 0 the core drives mid-scale 0x2000, which this module converts to
0 V — the correct idle level.

Code-to-voltage mapping (high-impedance load, full-scale reference setting):

```
Vout ≈ (code − 8192) / 8192 × (Vpp_fullscale / 2)
```

e.g. at the 6 Vpp maximum: 1 digital LSB ≈ 366 µV, OFFSET = +2048 ≈ +0.75 V.

## 3. Clocking: run dds_clk at 100 MHz

The on-board reconstruction filter is designed for a 100 MHz DAC clock
(cutoff = 0.32 × 100 MHz = 32 MHz). This matters more than it looks:

- At **f_dds = 100 MHz**, a 10 MHz output places the first alias image at
  90 MHz — deep in the filter stopband. Clean output.
- At **f_dds = 50 MHz**, the same 10 MHz output places the image at 40 MHz —
  barely above the 32 MHz corner, and a Bessel filter rolls off *slowly*
  (flat delay is bought with a gentle magnitude slope). The image leaks
  through and the waveform visibly jitters. The module manual reports
  exactly this above 20 MHz even at full clock.

Recommended PLL plan from the 50 MHz oscillator: 100 MHz → `dds_clk` (and
DAC CLK), 50 MHz → `PCLK`. The core's internal CDC supports this directly;
apply the two XDC constraints from datasheet §8.2. Remember FWORD values
scale with f_dds: at 100 MHz, 1 MHz → FWORD = 0x028F5C29.

## 4. Data-to-clock timing at the DAC

The AD9764 latches D[13:0] on the rising edge of CLK and needs ≥ 2 ns setup
(the module manual dedicates a section to glitches caused by violating
this). Robust arrangement:

1. Register `dac_data` in IOB flip-flops (`IOB=TRUE` on the output FFs, or
   let Vivado pack them — `dac_data` is already a registered output).
2. Forward the DAC clock with an ODDR primitive driven by `dds_clk`
   **inverted** (D1=0, D2=1), so the DAC sees the rising edge half a period
   after the data changes: ~5 ns setup / 5 ns hold at 100 MHz, with margin
   for board skew. A PLL phase-shifted clock output works equally well if
   you prefer to fine-tune.
3. Keep the ribbon cable short; the module already has series termination
   resistors on all data lines.

## 5. Amplitude (Vpp) control — two stages

Total output amplitude = **hardware full-scale × AMP register**.

**Hardware full-scale** (what AMP = 1.0 means in volts) is set by the
AD9764 reference, selected by jumpers J3/J5:

- Jumper fitted (default): internal 1.2 V reference → ≈ 6 Vpp full scale.
- Jumper moved to the potentiometer position: VR1/VR2 adjust the reference
  0.1–1.25 V → ≈ 0.5–6 Vpp.
- **Programmable option:** the J2/J4 pin labeled "3" on silk is an external
  reference input (0.1–1.25 V). Drive it from a filtered FPGA PWM DAC or any
  auxiliary DAC and the full-scale voltage becomes software-controlled too —
  no potentiometer touching.

**Division of labour:** the AMP register is fast, fine-grained and glitch-
free, but scaling down digitally costs resolution (AMP = 0.1 leaves ~10.7
effective bits and raises the relative quantization floor). Use the hardware
reference for coarse range setting and AMP for fine/dynamic control (AM,
soft ramp-up/down, calibration).

**Calibration hint:** near the top of the band the module's Bessel filter
droops (−1 dB around 10–14 MHz per the manual) plus the inherent sin(x)/x
droop. If absolute Vpp accuracy matters across frequency, build a small
frequency→AMP correction table in the driver.

## 6. DC offset behaviour

The module's entire analog path is DC-coupled (op-amps direct, LC filter
passes DC, no transformer or AC-coupling capacitor), with a ±3 V output
window — so the core's OFFSET register (0x58) produces a genuine DC bias at
the SMA connector:

- Waveform + bias: keep |AMP·full-scale| + |offset| ≤ 8191 codes to stay out
  of digital saturation, which corresponds to staying inside ±3 V analog.
- Pure DC level: AMP = 0, OFFSET = level. The module then acts as a slow
  programmable voltage source (−3 V … +3 V at 366 µV/LSB at full reference).
- There is no offset adjustment on the board itself; all bias must come from
  the digital side, which is exactly what OFFSET provides.
- The 50 Ω halving in §7 applies to DC levels too.

**OFFSET is a fraction of the hardware full scale, not an absolute voltage.**
The DAC reference scales the entire transfer function, so:

```
V_offset = OFFSET / 8192 × (Vpp_fullscale / 2)
```

OFFSET = 2048 gives +25 % of the positive half-range — +0.75 V when the
reference is at the 6 Vpp maximum, but only +0.25 V if the potentiometer has
been turned down to 2 Vpp. AMP does **not** scale the offset (it is added
after the multiplier), but the hardware reference scales waveform and offset
together.

**One-point calibration:** since Vpp_fullscale depends on the pot/reference
setting (and a few % of resistor/op-amp tolerance even at the default
jumper), measure it once per reference setting: write AMP = 0,
OFFSET = 4096, read the DC voltage V_m at the SMA with a high-impedance
meter, then `V_LSB = V_m / 4096`. This single constant calibrates both the
OFFSET and AMP voltage scales, because they share the same DAC transfer.
Redo it if the potentiometer or external reference changes.

## 7. Loads, measurement, and electrical cautions

- **50 Ω halving:** the output has a 50 Ω series resistor. Into a 50 Ω
  terminated scope/load you see **half** the open-circuit amplitude (3 Vpp
  max instead of 6 Vpp); on a 1 MΩ scope input you see the full swing. Decide
  which load you calibrate for and be consistent.
- Use 50 Ω coax (the vendor tests with RG316); avoid long unshielded leads
  after the SMA.
- **Supply:** 5 V absolute — the module has reverse-polarity protection but
  no over-voltage protection. A linear regulator or well-filtered rail is
  strongly preferred; switching-supply ripple degrades the analog floor.
- The module separates analog and digital grounds internally; connect its
  supply ground solidly to the FPGA board ground and keep the data ribbon
  short.

## 8. Waveform expectations through the Bessel filter

- **Sine:** clean to ~10 MHz (−1 dB) / ~25 MHz (−3 dB). Above ~20 MHz the
  unfiltered image makes the display jitter — avoid, or accept it knowingly.
- **Square / triangle:** the Bessel filter's flat group delay means **no
  overshoot or ringing** — edges are simply slew-limited by the 32 MHz
  cutoff (≈ 11 ns 10–90 % rise). 1 MHz squares look textbook; above ~3–5 MHz
  the corners visibly round. Duty-cycle accuracy from the DUTY register is
  unaffected.
- **User RAM waveforms:** same rules — spectral content above ~25 MHz will
  be attenuated/distorted, so band-limit what you load.

## 9. Dual-channel use

The module carries two independent AD9764s. Instantiate two `dds_top` cores
(each takes a 32 KB address window) sharing `dds_clk`, with separate
`dac_data`/CLK wiring per channel. For a phase-locked I/Q pair (one sine,
one cosine at the same FWORD): the two cores' phase accumulators start from
the same reset, but separate APB SRST writes land a few cycles apart, giving
a fixed, calculable phase difference of `FWORD × Δt`. Either compensate it
with one core's POW register after measuring, or hold both cores in EN = 0,
configure identically, then enable — the accumulators were both cleared by
the common `dds_rstn`, so they leave reset phase-aligned as long as neither
was enabled in between.

## 10. Bring-up checklist

1. Power the module from a clean 5 V rail; verify −5 V and 3.3 V rails.
2. Jumpers J3/J5 fitted (internal reference) for first light.
3. `PCLK` = 50 MHz, `dds_clk` = 100 MHz, ODDR-inverted clock to CLKA.
4. Write FWORD = 0x028F5C29 (1 MHz @ 100 MHz), CTRL = 0x0000_0001
   (EN, sine, 14-bit, offset binary — all other fields default).
5. Expect ≈ 6 Vpp sine on a 1 MΩ scope input (≈ 3 Vpp into 50 Ω).
6. Then exercise AMP, OFFSET, DUTY and the sweep — all pure register writes,
   examples in datasheet §7.
