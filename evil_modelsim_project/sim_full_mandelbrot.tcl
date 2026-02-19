# sim_full_mandelbrot.tcl
# Full Mandelbrot renderer simulation (640x480, all modules from DE1_SoC_Computer.v)
#
# This uses a SEPARATE library "work_mandelbrot" so it does NOT conflict
# with the existing simlink_iterator.v / tb_iterator simulation.
#
# How to use in ModelSim:
#   File > Change Directory > select evil_modelsim_project/
#   In Transcript: do sim_full_mandelbrot.tcl
#
# After done: python render_to_png.py  → mandelbrot.png

# ---- Create a separate library so we don't clash with simlink_iterator ----
vlib work_mandelbrot
vmap work_mandelbrot work_mandelbrot

# Source files
set verilog_src "../VGA_to_M10k/verilog/DE1_SoC_Computer.v"
set tb_src      "mandelbrot_tb.v"

echo "=== Compiling full Mandelbrot simulation ==="
echo "Compiling $verilog_src into work_mandelbrot ..."
vlog -work work_mandelbrot -vlog01compat $verilog_src

echo "Compiling $tb_src into work_mandelbrot ..."
vlog -work work_mandelbrot -vlog01compat $tb_src

# ---- Simulate from the mandelbrot library ----
echo "Starting simulation (work_mandelbrot.mandelbrot_tb)..."
vsim -t 1ns -novopt work_mandelbrot.mandelbrot_tb

# Add useful waveform signals
add wave -divider "Control"
add wave -hex sim:/mandelbrot_tb/reset
add wave -hex sim:/mandelbrot_tb/clk
add wave -hex sim:/mandelbrot_tb/done

add wave -divider "Memory Write"
add wave -hex sim:/mandelbrot_tb/mem_we
add wave -unsigned sim:/mandelbrot_tb/mem_write_address
add wave -hex sim:/mandelbrot_tb/mem_write_data

add wave -divider "DUT Internals"
add wave -hex sim:/mandelbrot_tb/dut/current_state
add wave -unsigned sim:/mandelbrot_tb/dut/pixel_x
add wave -unsigned sim:/mandelbrot_tb/dut/pixel_y
add wave -hex sim:/mandelbrot_tb/dut/curr_x
add wave -hex sim:/mandelbrot_tb/dut/curr_y

add wave -divider "Iterator"
add wave -hex sim:/mandelbrot_tb/dut/iterator_in_val
add wave -hex sim:/mandelbrot_tb/dut/iterator_in_rdy
add wave -hex sim:/mandelbrot_tb/dut/iterator_out_val
add wave -hex sim:/mandelbrot_tb/dut/iterator_out_rdy
add wave -unsigned sim:/mandelbrot_tb/dut/iterator_iter_count
add wave -hex sim:/mandelbrot_tb/dut/iterator_escape_condition

echo "Running – progress prints every 5M cycles in Transcript..."
run -all

echo "=== Done! Run: python render_to_png.py ==="
