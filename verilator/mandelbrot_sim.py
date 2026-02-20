#!/usr/bin/env python3
"""
mandelbrot_sim.py  –  Software model of the Verilog Mandelbrot renderer

Uses the EXACT same:
  - 4.23 signed fixed-point arithmetic (integer math, same overflow behavior)
  - escape condition bounds  (|z|^2 > 4.0, |zr_next| > 2.0, |zi_next| > 2.0,
                              iter == ITER_MAX)
  - signed_mult truncation   (keep bits [53] and [48:23] of 54-bit product)
  - color scheme             (same bucket thresholds as color_scheme module)
  - coordinate scan          (x left-to-right, y top-to-bottom)

Run:
    python mandelbrot_sim.py
    python mandelbrot_sim.py --width 320 --height 240   # faster preview
    python mandelbrot_sim.py --iter 200                  # fewer iterations

Output:  mandelbrot.png
"""

import argparse
import time
from pathlib import Path

# ---------------------------------------------------------------
# 4.23 Fixed-Point Constants  (1.0 = 2^23 = 8388608)
# ---------------------------------------------------------------
FRAC_BITS  = 23
SCALE      = 1 << FRAC_BITS   # 8388608
BITS       = 27                # total bits (signed)
SIGN_MASK  = 1 << (BITS - 1)  # 0x4000000
MASK       = (1 << BITS) - 1  # 0x7FFFFFF  (27-bit mask)

def to_fixed(f):
    """Python float → 27-bit signed fixed-point integer"""
    return int(f * SCALE)

def sign_extend_27(v):
    """Force a 27-bit two's-complement value into Python signed int"""
    v = v & MASK
    if v & SIGN_MASK:
        v -= (1 << BITS)
    return v

def signed_mult(a, b):
    """
    Mirrors the signed_mult Verilog module exactly:
      wire [53:0] mult_out = a * b;
      out = {mult_out[53], mult_out[48:23]}   // 27-bit result
    This is a 4.23 × 4.23 → 8.46, keeping the Q4.23 slice.
    """
    # Ensure both are in Python signed range first
    a = sign_extend_27(a)
    b = sign_extend_27(b)
    product = a * b                     # full 54-bit signed product
    # Truncate to 27 bits: take sign bit [53] and bits [48:23]
    result = (product >> FRAC_BITS)     # arithmetic right shift by 23
    return sign_extend_27(result)

def arithmetic_shift_left_1(v):
    """<<< 1 on a 27-bit signed value (same as *2, with saturation to 27 bits)"""
    return sign_extend_27(v << 1)

# ---------------------------------------------------------------
# Escape condition  (matches DE1_SoC_Computer.v exactly)
# ---------------------------------------------------------------
ESCAPE_MAG_SQ  = to_fixed(4.0)   # 27'h2000000 = 33554432
ESCAPE_BOUND   = to_fixed(2.0)   # 27'h1000000 = 16777216

def mandelbrot_pixel(c_r, c_i, iter_max):
    """
    Simulate one iterator instance.
    Returns iter_count (same value the Verilog outputs).
    """
    zr = 0
    zi = 0
    zr_sq = 0
    zi_sq = 0

    for count in range(iter_max):
        # Combinational (mirrors always @(*) wires)
        zr_sq_next = signed_mult(zr, zr)
        zi_sq_next = signed_mult(zi, zi)
        zr_zi      = signed_mult(zr, zi)
        zr_next    = sign_extend_27(zr_sq_next - zi_sq_next + c_r)
        zi_next    = sign_extend_27(arithmetic_shift_left_1(zr_zi) + c_i)
        z_mag_sq   = sign_extend_27(zr_sq_next + zi_sq_next)

        # Escape check (mirrors assign escape_condition = ...)
        if (z_mag_sq   >  ESCAPE_MAG_SQ  or
            zi_next    >  ESCAPE_BOUND   or
            zi_next    < -ESCAPE_BOUND   or
            zr_next    >  ESCAPE_BOUND   or
            zr_next    < -ESCAPE_BOUND):
            # iter_count increments one more in CALC before going to DONE
            return count + 1

        # Register update (mirrors always @(posedge clk) CALC case)
        zr    = zr_next
        zi    = zi_next
        zr_sq = zr_sq_next
        zi_sq = zi_sq_next

    return iter_max   # Did not escape → inside set

# ---------------------------------------------------------------
# Color scheme  (mirrors color_scheme module exactly)
# ---------------------------------------------------------------
def color_scheme(count, iter_max):
    """Returns 8-bit color value: [7:5]=R [4:2]=G [1:0]=B"""
    if   count >= iter_max:              return 0b_000_000_00  # black (inside set)
    elif count >= iter_max >> 1:         return 0b_011_001_00
    elif count >= iter_max >> 2:         return 0b_011_001_00
    elif count >= iter_max >> 3:         return 0b_101_010_01
    elif count >= iter_max >> 4:         return 0b_011_001_01
    elif count >= iter_max >> 5:         return 0b_001_001_01
    elif count >= iter_max >> 6:         return 0b_011_010_10
    elif count >= iter_max >> 7:         return 0b_010_100_10
    elif count >= iter_max >> 8:         return 0b_010_100_10
    else:                                return 0b_010_100_10

def pixel_to_rgb(byte_val):
    """Expand 8-bit hardware color to (R,G,B) each 0-255"""
    r3 = (byte_val >> 5) & 0x07
    g3 = (byte_val >> 2) & 0x07
    b2 = (byte_val >> 0) & 0x03
    # Bit-replicate to fill 8 bits (matches VGA driver expansion)
    R = (r3 << 5) | (r3 << 2) | (r3 >> 1)
    G = (g3 << 5) | (g3 << 2) | (g3 >> 1)
    B = (b2 << 6) | (b2 << 4) | (b2 << 2) | b2
    return (R, G, B)

# ---------------------------------------------------------------
# Full-frame render
# ---------------------------------------------------------------
def render(width, height, iter_max, pixel_increment, x_start, y_start):
    pixels = []
    total  = width * height
    report_every = max(1, height // 20)   # progress every 5%

    t0 = time.time()
    for row in range(height):
        c_i = sign_extend_27(y_start - row * pixel_increment)
        for col in range(width):
            c_r = sign_extend_27(x_start + col * pixel_increment)
            count = mandelbrot_pixel(c_r, c_i, iter_max)
            raw   = color_scheme(count, iter_max)
            pixels.append(pixel_to_rgb(raw))

        if row % report_every == 0:
            elapsed = time.time() - t0
            pct = 100.0 * (row + 1) / height
            eta = elapsed / max(row + 1, 1) * (height - row - 1)
            print(f"  row {row+1:4d}/{height}  ({pct:5.1f}%)  "
                  f"elapsed={elapsed:.1f}s  eta={eta:.1f}s")

    elapsed = time.time() - t0
    print(f"Render complete: {width}x{height} in {elapsed:.2f}s")
    return pixels

# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Software model of the Verilog Mandelbrot renderer")
    parser.add_argument("--width",  type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--iter",   type=int, default=1000,
                        help="ITER_MAX (default 1000)")
    parser.add_argument("--output", default="mandelbrot.png")
    parser.add_argument("--hex",    action="store_true",
                        help="Also write mem_dump.hex (for comparison)")
    args = parser.parse_args()

    # Match DE1_SoC_Computer.v instantiation exactly
    PIXEL_INC = 39000                   # 27'sd39000
    X_START   = to_fixed(-2.0)         # -27'sd16777216
    Y_START   = 9340500                 # centered: 479*39000/2

    print(f"=== Mandelbrot Software Model ===")
    print(f"Resolution:  {args.width} x {args.height}")
    print(f"ITER_MAX:    {args.iter}")
    print(f"x range:     [{X_START/SCALE:.4f}, "
          f"{(X_START + args.width*PIXEL_INC)/SCALE:.4f}]")
    print(f"y range:     [{Y_START/SCALE:.4f}, "
          f"{(Y_START - args.height*PIXEL_INC)/SCALE:.4f}]")
    print()

    pixels = render(args.width, args.height, args.iter,
                    PIXEL_INC, X_START, Y_START)

    # Write PNG
    try:
        from PIL import Image
        img = Image.new("RGB", (args.width, args.height))
        img.putdata(pixels)
        img.save(args.output)
        print(f"Saved: {args.output}")
    except ImportError:
        # Fallback: PPM
        ppm = Path(args.output).with_suffix(".ppm")
        with open(ppm, "wb") as f:
            f.write(f"P6\n{args.width} {args.height}\n255\n".encode())
            for r, g, b in pixels:
                f.write(bytes([r, g, b]))
        print(f"Saved: {ppm}  (install Pillow for PNG: pip install Pillow)")

    # Optionally dump hex
    if args.hex:
        flat = []
        for row in range(args.height):
            c_i = sign_extend_27(Y_START - row * PIXEL_INC)
            for col in range(args.width):
                c_r = sign_extend_27(X_START + col * PIXEL_INC)
                count = mandelbrot_pixel(c_r, c_i, args.iter)
                flat.append(color_scheme(count, args.iter))
        with open("mem_dump.hex", "w") as f:
            for v in flat:
                f.write(f"{v:02x}\n")
        print("Saved: mem_dump.hex")

if __name__ == "__main__":
    main()
