# Building the DDS in Vivado (Zynq-7020, PL only)

Target: Zynq-7020, **PL only** — no PS7, no block design. Vivado will warn that
the Zynq processing system is unused; that is expected. The bitstream loads over
JTAG and runs on its own, with no FSBL and nothing running on the ARM cores.

Vivado runs in the eda-box container: `vivado-launch.sh` (see
`dotfile/eda/vivado.md`).

## What is the top module?

**`board/dds_board_top.v`** — not `dds_top`.

`dds_top` is the IP core. It deliberately contains no PLL, no clock forwarding
and no I/O primitives, so it stays board-independent and simulatable with Icarus.
Synthesizing it directly as top does "work", but the result cannot run on a
board: `dds_clk` becomes an external input pin with no 100 MHz to drive it, and
the DAC never gets a clock.

```
board/dds_board_top.v        <- set this as top
 ├── clk_wiz_0 (PLL IP)      <- board oscillator -> 100 MHz dds_clk
 ├── reset synchronizer      <- async assert, sync release, held until PLL lock
 ├── rtl/dds_top.v           <- the IP core (NUM_CH = 2, HAS_ADC = 1)
 │    ├── dds_spi_slave.v
 │    ├── dds_channel.v × 2  (dds_regs + dds_core + dds_wave_ram + dds_sin_lut
 │    │                        + dds_meas/dds_isqrt/dds_agc/dds_agc_regs)
 │    ├── dds_adc_if.v       <- AD9226 capture timing + data conditioning
 │    └── dds_glob_regs.v    <- ADC front-end registers
 ├── IOB output registers    <- dac_data leaves with matched clock-to-out delay
 ├── ODDR × 2                <- forwards an inverted dds_clk to CLKA / CLKB
 ├── ODDR × 2                <- 50 MHz ADC clocks (dds_clk/2 pattern) to ACLK/BCLK
 └── IOB input registers     <- ADC data captured at the pad (CE-gated)
```

## Files to add to the project

| Add as | Files |
|---|---|
| Design sources | `rtl/*.v` **and** `board/dds_board_top.v` |
| Design sources (!) | **`rtl/dds_sin_lut.mem`** |
| Constraints | your XDC (not written yet — pins are board-specific) |
| IP | `clk_wiz_0`, generated in the IP catalog |

> **`dds_sin_lut.mem` must be added to the project as a design source.** It is
> read by `$readmemh` at elaboration; if Vivado cannot see it, synthesis
> silently produces a sine ROM full of X (which becomes all-zeros in the
> bitstream) and the DAC outputs a flat line. Nothing errors out. If your sine
> is dead flat but square/triangle work, this is why.

## The PLL

Generate `clk_wiz_0` in the IP catalog:

- **Input**: your board's PL oscillator frequency, from the pin your XDC names.
- **Output**: one clock, **100.000 MHz** → `dds_clk`.
- **Reset**: active-low (`resetn`), and keep the `locked` output.

100 MHz is not arbitrary: the AD9764 module's reconstruction filter is designed
for a 100 MHz DAC clock (AN01 §3), and the STM32 firmware's `DDS_CLK_HZ` says
100000000. Change one of the three and you must change all three.

The instance in `dds_board_top.v` already matches the generated IP's port list
(`clk_in1` / `resetn` / `clk_out1` / `locked`) — nothing to edit there. What the
port list does **not** tell you, and what the design depends on, is inside the IP
configuration: **`clk_in1` must be your board's actual oscillator frequency, and
`clk_out1` must be 100.000 MHz.** Get the input frequency wrong and the PLL still
locks — it just produces the wrong output frequency, and every generated signal
comes out scaled by the same ratio, with nothing anywhere reporting an error.

## Constraints (XDC)

A working XDC for the reference board (Zynq-7020) is bundled at
[`board/DDS_Constraints.xdc`](../board/DDS_Constraints.xdc) — add it to the
project. It currently pins every port and sets `LVCMOS33` on all of them (the
AD9764 module is a 3.3 V part, so the DAC buses and SPI pins must sit on a 3.3 V
I/O bank).

**Adapting it to another board:** change the `PACKAGE_PIN` assignments to match
your wiring. Keep `LVCMOS33` unless your bank voltage differs.

**ADC pins:** the AD9226 module's pins are a commented template at the end of
the bundled XDC — fill in your `PACKAGE_PIN`s and uncomment. Mind the mapping
direction: the module's silk screen is numbered backwards (silk **D0 is the
MSB** → `adc0_data[11]`); the template spells it out per bit, and
[AN02](Application%20Note/AN02_AD9226_Module.md) explains why and how to
verify it in one register read.

**Recommended additions** — the bundled XDC pins the design but does not yet
constrain timing. For a robust build, add:

```tcl
# --- the oscillator: one create_clock, everything else derives from the PLL ---
create_clock -period <T> -name sys_clk [get_ports sys_clk]

# --- SPI: asynchronous inputs, oversampled in the dds_clk domain. ---
# The synchronizers handle metastability; there is no setup/hold relationship
# to time here, so tell the tool not to invent one.
set_false_path -from [get_ports {spi_sck spi_cs_n spi_mosi}]
set_false_path -to   [get_ports spi_miso]
set_false_path -from [get_ports ext_rst]
```

Without `create_clock`, the tool has no clock definition to check against and
the SPI inputs may be flagged as unconstrained; both are harmless to the current
low-speed bring-up but should be added before trusting timing closure.

## Flow

1. `vivado-launch.sh`, create an RTL project, part = your Zynq-7020 device.
2. Add the sources and the `.mem` above; generate `clk_wiz_0`.
3. Set **`dds_board_top`** as top (right-click → Set as Top).
4. Add the XDC.
5. Run Synthesis → Implementation → Generate Bitstream.
6. Open Hardware Manager, program the device over JTAG.

## Bring-up order

Do these in order — each step's failure has a completely different cause, and
skipping ahead wastes hours:

1. **Is the PLL locked?** Route `locked` to an LED, or check it in the ILA.
   Nothing works without it, and a wrong oscillator frequency in the clk_wiz
   config fails exactly here.
2. **Does the SPI link answer?** On the STM32 console, `id` must print
   `ch0 ID 0x4453 VERSION 0x0210` for both channels. If it returns `0x0000` or
   garbage, the problem is the four SPI wires or the common ground — not the DAC.
   (An ILA on `spi_cs_n` / `spi_sck` / `spi_mosi` settles it in one shot.)
3. **Does the DAC move?** `freq 1M`, `wave sine`, `fmt ob`, `on`. Offset binary
   is mandatory for this module (AN01 §2).
4. Only then worry about signal quality, images and the filter.
5. **ADC / amplitude loop** (if wired): follow the bring-up checklist in
   [AN02 §6](Application%20Note/AN02_AD9226_Module.md) — `reg 80` (GLOB_ID),
   grounded-input `adc` read, known-DC sign check, `meas`, then `agc on`. Each
   step isolates a different wiring or configuration fault.

A dead-flat sine with working square/triangle means the sine LUT is empty — see
the `.mem` warning above.
