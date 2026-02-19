# Mandelbrot Simulation & PNG Export

## Files
| File | Purpose |
|------|---------|
| `mandelbrot_tb.v` | Verilog testbench — simulates the full Mandelbrot render |
| `sim.tcl` | ModelSim compile & run script |
| `render_to_png.py` | Convert the memory dump to a PNG image |

## How to Run

### 1. Run the Simulation (ModelSim)
```
Open ModelSim → File → Change Directory → select this folder (signaltap/)
In Transcript: do sim.tcl
```
Wait for `=== Testbench DONE ===` in the transcript.
This generates **`mem_dump.hex`** in the signaltap folder.

**Full 640×480 render time estimate:** ~400M clock cycles. At 100MHz sim speed
in ModelSim this may take 5–20 minutes depending on your machine.

> **Tip for fast iteration:** To test quickly, reduce resolution in the top of
> `mandelbrot_tb.v` by changing the `` `define SIM_X_PIXELS `` and
> `` `define SIM_Y_PIXELS `` values (e.g. 64×48).

### 2. Generate the PNG (Python)
```bash
# Install Pillow once (needed for PNG output)
pip install Pillow

# Convert the dump
python render_to_png.py
```
This produces **`mandelbrot.png`** in the same folder.

Optional arguments:
```
python render_to_png.py --input mem_dump.hex --output out.png --width 640 --height 480
```

If Pillow is not installed, a `.ppm` file is produced instead (viewable in
GIMP, IrfanView, Preview, etc.).

## Color Format
The 8-bit pixel format is the same as the FPGA hardware:
- `[7:5]` → Red (3 bits)
- `[4:2]` → Green (3 bits)  
- `[1:0]` → Blue (2 bits)
