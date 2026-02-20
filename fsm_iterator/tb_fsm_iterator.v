`timescale 1ns/1ps
`define ITER_MAX 1000

module tb_fsm_iterator;

    // Inputs
    reg reset;
    reg clk;
    reg in_val;
    reg signed [26:0] in_c_r;
    reg signed [26:0] in_c_i;
    reg out_rdy;

    // Outputs
    wire in_rdy;
    wire [$clog2(`ITER_MAX):0] iter_count;
    wire escape_condition;
    wire out_val;

    // Instantiate the Unit Under Test (UUT)
    fsm_iterator uut (
        .reset(reset), 
        .clk(clk), 
        .in_val(in_val), 
        .in_rdy(in_rdy), 
        .in_c_r(in_c_r), 
        .in_c_i(in_c_i), 
        .iter_count(iter_count), 
        .escape_condition(escape_condition), 
        .out_val(out_val), 
        .out_rdy(out_rdy)
    );

    // ---------------------------------------------------------
    // Clock Generation (100 MHz)
    // ---------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; 
    end

    // ---------------------------------------------------------
    // Helper Task: Send a coordinate and wait for result
    // ---------------------------------------------------------
    // ---------------------------------------------------------
    // Helper Task: Send a coordinate and wait for result
    // ---------------------------------------------------------
    task test_point;
        input signed [26:0] c_r;
        input signed [26:0] c_i;
        begin
            // 1. Wait until DUT is ready for new data
            while (!in_rdy) @(posedge clk);
            
            // 2. Drive inputs using NON-BLOCKING assignments
            in_c_r <= c_r;
            in_c_i <= c_i;
            in_val <= 1'b1;
            
            // 3. Wait one clock cycle for the handshake to register
            @(posedge clk);
            in_val <= 1'b0; // De-assert using NON-BLOCKING assignment

            // 4. Wait for DUT to finish calculation
            while (!out_val) @(posedge clk);
            
            // 5. Print results (converting Q4.23 back to float for the console)
            $display("Tested C = (%f, %f) | Iterations: %0d | Escaped: %0b", 
                     $itor(c_r) / 8388608.0, 
                     $itor(c_i) / 8388608.0, 
                     iter_count, 
                     escape_condition);
            
            // 6. Acknowledge the output using NON-BLOCKING assignments
            out_rdy <= 1'b1;
            @(posedge clk);
            out_rdy <= 1'b0;
            
            // Add a small gap between tests in the waveform
            repeat(5) @(posedge clk);
        end
    endtask

    // ---------------------------------------------------------
    // Main Test Stimulus
    // ---------------------------------------------------------
    initial begin
        // Setup VCD dumping for GTKWave viewing
        $dumpfile("mandelbrot.vcd");
        $dumpvars(0, tb_fsm_iterator);

        // Initialize Inputs
        reset = 1;
        in_val = 0;
        in_c_r = 0;
        in_c_i = 0;
        out_rdy = 0;

        // Wait 100 ns for global reset to finish
        #100;
        reset = 0;
        repeat(5) @(posedge clk);

        $display("--- Starting Mandelbrot FSM Tests ---");

        // Test Case 1: C = 0 + 0i (Deep inside the set, should hit ITER_MAX)
        // 0 in Q4.23 fixed-point is 27'h0000000
        test_point(27'h0000000, 27'h0000000);

        // Test Case 2: C = -2.0 + 0i (Tip of the tail, should hit ITER_MAX)
        // +2.0 is 27'h1000000. Two's complement for -2.0 is 27'h7000000
        test_point(27'h7000000, 27'h0000000);

        // Test Case 3: C = 1.0 + 1.0i (Outside the set, escapes quickly)
        // +1.0 in Q4.23 fixed-point is 27'h0800000
        test_point(27'h0800000, 27'h0800000);

        // Test Case 4: C = -0.5 + 0.5i (Edge of the main cardioid)
        // +0.5 is 27'h0400000. Two's complement for -0.5 is 27'h7C00000
        test_point(27'h7C00000, 27'h0400000);



        test_point(27'h7F33333, 27'h0533333);
        $display("--- Tests Complete ---");
        $finish;
    end

endmodule