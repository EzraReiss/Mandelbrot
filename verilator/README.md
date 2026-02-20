# Verilator Mandelbrot Simulation

Much faster than ModelSim — Verilator compiles Verilog to native C++.

## Requirements

```bash
# Install Verilator (choose one):
#   Windows: use MSYS2 MinGW64 shell, then:
pacman -S mingw-w64-x86_64-verilator
#   Ubuntu/WSL:
sudo apt install verilator
#   Mac:
brew install verilator

# Install Python PNG library (once):
pip install Pillow
```

## Run Everything

```bash
# From this folder (verilator/) in MSYS2/WSL/Linux:
make
```

This does three things:
1. **Verilate** — converts the Verilog to C++
2. **Compile** — builds a native binary with `-O3`
3. **Simulate** — runs until `done`, writes `mem_dump.hex`
4. **PNG** — converts to `mandelbrot.png`

## Step by Step

```bash
make sim      # just compile
make run      # just run (writes mem_dump.hex)
make png      # just convert to PNG
make clean    # delete build artifacts
```

## Speed

| Resolution | Approx sim time |
|-----------|----------------|
| 64 × 48   | < 1 second |
| 640 × 480 | 30–120 seconds |

Compared to ModelSim which can take many minutes for the same workload.

## Source files

| File | Purpose |
|------|---------|
| `sim_main.cpp` | C++ simulation harness |
| `Makefile` | Build system |
| `render_to_png.py` | Hex dump → PNG |
| `../VGA_to_M10k/verilog/DE1_SoC_Computer.v` | RTL source |
