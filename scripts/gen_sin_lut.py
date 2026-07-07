# Generates the quarter-wave sine LUT init file for dds_sin_lut.v
# 4096 points covering the first quadrant, sampled at (i+0.5)/4096 * 90 deg.
# The half-LSB offset makes the quarter-wave symmetry exact (no duplicated
# endpoint samples, no DC offset after quadrant reconstruction).
# Word format: 13-bit unsigned magnitude, 0..8191 (full scale of a 14-bit
# two's-complement output sample).

import math

DEPTH = 4096
AMPL = 8191

with open("../rtl/dds_sin_lut.mem", "w", newline="\n") as f:
    for i in range(DEPTH):
        v = round(math.sin((i + 0.5) / DEPTH * math.pi / 2) * AMPL)
        f.write(f"{v:04x}\n")

print(f"wrote {DEPTH} entries, max={AMPL}")
