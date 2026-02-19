// =============================================================
// mandelbrot_tb.v  –  Full-frame Mandelbrot simulation testbench
// Compatible with ModelSim 10.5b (Verilog-2001, no SystemVerilog)
//
// After simulation completes, mem_dump.hex is written (one byte
// per line, hex).  Run render_to_png.py to generate a PNG image.
// =============================================================

`timescale 1ns/1ps

// Override defines here to trade off speed vs. resolution
`define SIM_X_PIXELS   640
`define SIM_Y_PIXELS   480
`define SIM_MEM_MAX    (`SIM_X_PIXELS * `SIM_Y_PIXELS)

// Must match DE1_SoC_Computer.v
`define ITER_MAX 1000

module mandelbrot_tb;

// ---------------------------------------------------------------
// Clock & reset
// ---------------------------------------------------------------
reg clk;
reg reset;

localparam CLK_PERIOD = 10; // 10 ns = 100 MHz
initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// ---------------------------------------------------------------
// DUT signals
// ---------------------------------------------------------------
wire        done;
wire        start;
wire [7:0]  mem_write_data;
wire [18:0] mem_write_address;
wire        mem_we;

// ---------------------------------------------------------------
// Framebuffer (mirror writes so we can dump after done)
// ---------------------------------------------------------------
reg [7:0] framebuffer [0:`SIM_MEM_MAX-1];
integer i;
initial begin
    for (i = 0; i < `SIM_MEM_MAX; i = i + 1)
        framebuffer[i] = 8'h00;
end

always @(posedge clk) begin
    if (mem_we && (mem_write_address < `SIM_MEM_MAX))
        framebuffer[mem_write_address] <= mem_write_data;
end

// ---------------------------------------------------------------
// DUT  –  mandelbrot_top from DE1_SoC_Computer.v
// ---------------------------------------------------------------
mandelbrot_top dut (
    .reset          (reset),
    .clk            (clk),
    // 4.23 fixed point: -2.0 = -16777216,  1.0 = 8388608
    .x_start        (-27'sd16777216),
    .y_start        ( 27'sd8388608),
    .pixel_increment( 27'sd39000),   // ~0.00464 per pixel
    .start          (start),
    .done           (done),
    .mem_write_data (mem_write_data),
    .mem_write_address(mem_write_address),
    .mem_we         (mem_we)
);

// ---------------------------------------------------------------
// Coordination flag between initial blocks
// ---------------------------------------------------------------
reg sim_finished;
initial sim_finished = 0;

// ---------------------------------------------------------------
// Timeout watchdog  (runs in parallel with main block)
// 400 M cycles @ 100 MHz = ~4 seconds of simulated time
// ---------------------------------------------------------------
initial begin : timeout_block
    repeat(400000000) @(posedge clk);
    if (!sim_finished) begin
        $display("*** TIMEOUT after 400M cycles – aborting ***");
        $finish;
    end
end

// ---------------------------------------------------------------
// Main stimulus + memory dump
// ---------------------------------------------------------------
integer fd;
integer px;

initial begin : main_block
    $display("=== Mandelbrot Testbench START ===");
    $display("Resolution: %0d x %0d,  ITER_MAX: %0d",
             `SIM_X_PIXELS, `SIM_Y_PIXELS, `ITER_MAX);

    // Assert reset for 4 cycles
    reset = 1;
    repeat(4) @(posedge clk);
    @(negedge clk);
    reset = 0;
    $display("Reset released at t=%0t ns", $time);

    // Wait for mandelbrot_top to signal done
    @(posedge done);
    $display("DONE asserted at t=%0t ns", $time);
    sim_finished = 1;

    // Let the last memory write settle
    repeat(4) @(posedge clk);

    // Dump framebuffer to hex file
    fd = $fopen("mem_dump.hex", "w");
    if (fd == 0) begin
        $display("ERROR: Cannot open mem_dump.hex");
        $finish;
    end
    for (px = 0; px < `SIM_MEM_MAX; px = px + 1)
        $fwrite(fd, "%02x\n", framebuffer[px]);
    $fclose(fd);

    $display("Written mem_dump.hex (%0d pixels)", `SIM_MEM_MAX);
    $display("Run:  python render_to_png.py");
    $display("=== Testbench DONE ===");
    $finish;
end

// ---------------------------------------------------------------
// Progress ticker  – prints every 5M cycles
// ---------------------------------------------------------------
integer tick_cnt;
initial begin : ticker_block
    tick_cnt = 0;
    forever begin
        repeat(5000000) @(posedge clk);
        tick_cnt = tick_cnt + 5000000;
        $display("[%0d M clk]  addr = %0d / %0d  (%.1f%%)",
                 tick_cnt / 1000000,
                 mem_write_address,
                 `SIM_MEM_MAX,
                 100.0 * mem_write_address / `SIM_MEM_MAX);
    end
end

endmodule
