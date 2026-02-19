// =============================================================
// mandelbrot_tb.v  –  Full-frame Mandelbrot simulation testbench
//
// After simulation completes, the pixel memory is written to
// "mem_dump.hex" (one byte per line, hex).  Run render_to_png.py
// to convert it to a PNG image.
//
// Simulation time warning: at ITER_MAX=1000 and 640x480 pixels the
// worst case is ~307M clock cycles.  Use a fast clock and/or reduce
// the defines below for quick iteration.
// =============================================================

`timescale 1ns/1ps

// ---------------------------------------------------------------
// Override parameters for simulation speed.
// Change these to match what you want to simulate.
// ---------------------------------------------------------------
`define SIM_ITER_MAX  1000
`define SIM_X_PIXELS  640
`define SIM_Y_PIXELS  480
`define SIM_MEM_MAX   (`SIM_X_PIXELS * `SIM_Y_PIXELS)

module mandelbrot_tb;

// ---------------------------------------------------------------
// Clock & reset
// ---------------------------------------------------------------
reg clk;
reg reset;

localparam CLK_PERIOD = 10; // 10 ns → 100 MHz
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
// Framebuffer (mirror in testbench so we can dump it)
// ---------------------------------------------------------------
reg [7:0] framebuffer [`SIM_MEM_MAX - 1 : 0];

integer i;
initial begin
    for (i = 0; i < `SIM_MEM_MAX; i = i + 1)
        framebuffer[i] = 8'h00;
end

// Capture writes on each clock edge
always @(posedge clk) begin
    if (mem_we && mem_write_address < `SIM_MEM_MAX)
        framebuffer[mem_write_address] <= mem_write_data;
end

// ---------------------------------------------------------------
// DUT instantiation
// ---------------------------------------------------------------
mandelbrot_top dut (
    .reset(reset),
    .clk(clk),

    // Standard Mandelbrot view: x ∈ [-2, 1], y ∈ [1, -1.23]
    // 4.23 fixed point: 1.0 = 2^23 = 8388608
    .x_start(-27'sd16777216),   // -2.0
    .y_start( 27'sd8388608),    //  1.0
    .pixel_increment(27'sd39000), // ≈ 0.00464 per pixel

    .start(start),
    .done(done),

    .mem_write_data(mem_write_data),
    .mem_write_address(mem_write_address),
    .mem_we(mem_we)
);

// ---------------------------------------------------------------
// Stimulus & dump
// ---------------------------------------------------------------
integer fd;
integer px;

initial begin
    $display("=== Mandelbrot Testbench START ===");
    $display("Resolution: %0d x %0d,  ITER_MAX: %0d",
             `SIM_X_PIXELS, `SIM_Y_PIXELS, `SIM_ITER_MAX);

    // Assert reset for 4 cycles
    reset = 1;
    repeat(4) @(posedge clk);
    @(negedge clk); reset = 0;
    $display("Reset released at t=%0t", $time);

    // Wait for done signal (with a generous timeout)
    fork
        begin
            wait(done == 1'b1);
            $display("DONE asserted at t=%0t", $time);
        end
        begin
            // Timeout: 400M cycles should be enough for 640x480 x 1000 iters
            repeat(400_000_000) @(posedge clk);
            $display("TIMEOUT reached – simulation aborted.");
            $finish;
        end
    join_any
    disable fork;

    // Give one extra cycle for the last write to settle
    repeat(4) @(posedge clk);

    // ---- Dump framebuffer to hex file ----
    fd = $fopen("mem_dump.hex", "w");
    if (fd == 0) begin
        $display("ERROR: Could not open mem_dump.hex for writing");
        $finish;
    end

    for (px = 0; px < `SIM_MEM_MAX; px = px + 1) begin
        $fwrite(fd, "%02x\n", framebuffer[px]);
    end

    $fclose(fd);
    $display("Framebuffer written to mem_dump.hex (%0d pixels)", `SIM_MEM_MAX);
    $display("Run:  python render_to_png.py  to generate the PNG.");
    $display("=== Testbench DONE ===");
    $finish;
end

// Optional: periodic progress ticker
integer tick_count;
initial begin
    tick_count = 0;
    forever begin
        repeat(5_000_000) @(posedge clk);
        tick_count = tick_count + 5_000_000;
        $display("[%0d M clk] addr=%0d / %0d  (%.1f%%)",
                 tick_count/1_000_000,
                 mem_write_address,
                 `SIM_MEM_MAX,
                 100.0 * mem_write_address / `SIM_MEM_MAX);
    end
end

endmodule
