// System-level testbench: one mandelbrot_top + M10K on a small grid.
// Driven by sim_system.cpp via Verilator.
// Pass defines: -DITER_MAX=100 -DX_PIXEL_MAX=7 -DY_PIXEL_MAX=7 -DCOLS_PER_BANK=8

module tb_system_top (
    input clk,
    input reset,
    input signed [26:0] x_start,
    input signed [26:0] y_start,
    input signed [26:0] pixel_increment_x,
    input signed [26:0] pixel_increment_y,
    input [9:0] iter_max,
    input is_high_resolution,

    output done,

    input [18:0] read_address,
    output [9:0] read_data
);

    wire [9:0]  write_data;
    wire [18:0] write_address;
    wire        we;

    mandelbrot_top #(
        .ITERATOR_ID(0),
        .NUM_ITERATORS(1)
    ) dut (
        .reset(reset),
        .clk(clk),
        .x_start(x_start),
        .y_start(y_start),
        .pixel_increment_x(pixel_increment_x),
        .pixel_increment_y(pixel_increment_y),
        .iter_max(iter_max),
        .is_high_resolution(is_high_resolution),
        .done(done),
        .mem_write_data(write_data),
        .mem_write_address(write_address),
        .mem_we(we)
    );

    M10K_1000_8 #(
        .MEM_DEPTH(64),
        .DATA_WIDTH(10)
    ) mem (
        .q(read_data),
        .d(write_data),
        .write_address(write_address),
        .read_address(read_address),
        .we(we),
        .clk(clk)
    );

endmodule
