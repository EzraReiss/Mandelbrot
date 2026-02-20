#!/usr/bin/env python3
"""
render_to_png.py  –  Convert ModelSim Mandelbrot memory dump to PNG

Usage:
    python render_to_png.py [--input mem_dump.hex] [--output mandelbrot.png]
                            [--width 640] [--height 480]

The 8-bit pixel format is identical to the FPGA hardware:
    bits [7:5] = red   (3 bits)
    bits [4:2] = green (3 bits)
    bits [1:0] = blue  (2 bits)

Expanded to 8-bit per channel:
    R = {r[2:0], r[2:0], r[1:0]}  (replicate bits for smooth gradients)
    G = {g[2:0], g[2:0], g[1:0]}
    B = {b[1:0], b[1:0], b[1:0], b[1:0]}
"""

import argparse
import sys
from pathlib import Path

def expand_channel(val, bits):
    """Scale a `bits`-wide value to 8 bits by bit-replication."""
    if bits == 3:
        # replicate: b2 b1 b0  b2 b1 b0  b1 b0
        return (val << 5) | (val << 2) | (val >> 1)
    elif bits == 2:
        # replicate: b1 b0  b1 b0  b1 b0  b1 b0
        return (val << 6) | (val << 4) | (val << 2) | val
    return val

def pixel_to_rgb(byte_val):
    """Convert 8-bit hardware pixel to (R, G, B) tuple each 0-255."""
    r3 = (byte_val >> 5) & 0x07
    g3 = (byte_val >> 2) & 0x07
    b2 = (byte_val >> 0) & 0x03
    return (expand_channel(r3, 3),
            expand_channel(g3, 3),
            expand_channel(b2, 2))

def main():
    parser = argparse.ArgumentParser(description="Convert Mandelbrot memory dump to PNG")
    parser.add_argument("--input",  default="mem_dump.hex",     help="Input hex dump file")
    parser.add_argument("--output", default="mandelbrot.png",   help="Output PNG filename")
    parser.add_argument("--width",  type=int, default=640,      help="Image width in pixels")
    parser.add_argument("--height", type=int, default=480,      help="Image height in pixels")
    args = parser.parse_args()

    infile = Path(args.input)
    if not infile.exists():
        print(f"ERROR: Input file '{infile}' not found.")
        print("Make sure you ran ModelSim first to generate the dump.")
        sys.exit(1)

    # Try to import Pillow; fall back to writing raw PPM if not installed
    try:
        from PIL import Image
        use_pil = True
    except ImportError:
        use_pil = False
        print("Pillow not found – will output a PPM file instead.")
        print("Install Pillow with:  pip install Pillow")

    # Read pixel data
    pixels = []
    with open(infile, "r") as f:
        for line in f:
            line = line.strip()
            if line:
                pixels.append(int(line, 16))

    expected = args.width * args.height
    if len(pixels) < expected:
        print(f"WARNING: Only {len(pixels)} pixels in dump, expected {expected}. Padding with black.")
        pixels.extend([0] * (expected - len(pixels)))
    elif len(pixels) > expected:
        print(f"INFO: Truncating dump from {len(pixels)} to {expected} pixels.")
        pixels = pixels[:expected]

    rgb_pixels = [pixel_to_rgb(p) for p in pixels]

    if use_pil:
        # Create image using Pillow
        img = Image.new("RGB", (args.width, args.height))
        img.putdata(rgb_pixels)
        img.save(args.output)
        print(f"Saved PNG: {args.output}  ({args.width}x{args.height})")
    else:
        # Fallback: write PPM (viewable in most image viewers)
        ppm_out = Path(args.output).with_suffix(".ppm")
        with open(ppm_out, "wb") as f:
            header = f"P6\n{args.width} {args.height}\n255\n"
            f.write(header.encode())
            for r, g, b in rgb_pixels:
                f.write(bytes([r, g, b]))
        print(f"Saved PPM: {ppm_out}  ({args.width}x{args.height})")
        print("(Install Pillow to get PNG output: pip install Pillow)")

if __name__ == "__main__":
    main()
