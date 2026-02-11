`timescale 1ns / 1ps

module tb_iterator;

    // --- Inputs to UUT ---
    reg reset;
    reg clk;
    reg in_val;
    reg signed [26:0] in_c_r;
    reg signed [26:0] in_c_i;
    reg out_rdy;

    // --- Outputs from UUT ---
    wire in_rdy;
    // Note: Adjust width to match your module [$clog2(1000):0] which is 10 bits
    wire signed [10:0] iter_count; 
    wire out_val;

    // --- Instantiate the Unit Under Test (UUT) ---
    iterator uut (
        .reset(reset), 
        .clk(clk), 
        .in_val(in_val), 
        .in_rdy(in_rdy), 
        .in_c_r(in_c_r), 
        .in_c_i(in_c_i), 
        .iter_count(iter_count), 
        .out_val(out_val), 
        .out_rdy(out_rdy)
    );

    // --- Clock Generation (100 MHz) ---
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // --- Helper Function: Real -> 4.23 Fixed Point ---
    // This allows us to write "1.0" in the testbench and get 27'h800000
    function signed [26:0] to_fixed;
        input real value;
        begin
            to_fixed = $rtoi(value * (1 << 23));
        end
    endfunction

    // --- Test Stimulus ---
    initial begin
        // 1. Initialize
        reset = 1;
        in_val = 0;
        in_c_r = 0;
        in_c_i = 0;
        out_rdy = 0;

        // 2. Global Reset
        #100;
        reset = 0;
        #20;

        // -----------------------------------------------------------
        // TEST CASE 1: Origin (0 + 0i)
        // Expected: Should stay in set forever -> Max Iterations (1000)
        // -----------------------------------------------------------
        wait(in_rdy); // Wait for module to be ready
        @(posedge clk);
        $display("TC1: Testing C = 0.0 + 0.0i (Should hit MAX Iters)");
        
        in_c_r = to_fixed(0.0);
        in_c_i = to_fixed(0.0);
        in_val = 1;
        
        @(posedge clk);
        in_val = 0; // Release data valid

        // Wait for calculation to finish
        wait(out_val); 
        $display("TC1 Result: Iterations = %d", iter_count);
        
        // Handshake: Acknowledge output
        @(posedge clk);
        out_rdy = 1;
        @(posedge clk);
        out_rdy = 0;
        
        #50;

        // -----------------------------------------------------------
        // TEST CASE 2: Distinctly Outside (1.0 + 0.0i)
        // Expected: Escape very quickly (iter_count ~ 1 or 2)
        // -----------------------------------------------------------
        wait(in_rdy);
        @(posedge clk);
        $display("TC2: Testing C = 1.0 + 0.0i (Should escape quickly)");
        
        in_c_r = to_fixed(1.0);
        in_c_i = to_fixed(0.0);
        in_val = 1;
        
        @(posedge clk);
        in_val = 0;

        wait(out_val);
        $display("TC2 Result: Iterations = %d", iter_count);

        @(posedge clk);
        out_rdy = 1;
        @(posedge clk);
        out_rdy = 0;

        #50;

        // -----------------------------------------------------------
        // TEST CASE 3: Seahorse Valley Point (-0.75 + 0.1i)
        // Expected: Takes some iterations, but eventually escapes
        // -----------------------------------------------------------
        wait(in_rdy);
        @(posedge clk);
        $display("TC3: Testing C = -0.75 + 0.1i (Medium iterations)");
        
        in_c_r = to_fixed(-0.75);
        in_c_i = to_fixed(0.1);
        in_val = 1;
        
        @(posedge clk);
        in_val = 0;

        wait(out_val);
        $display("TC3 Result: Iterations = %d", iter_count);

        @(posedge clk);
        out_rdy = 1;
        @(posedge clk);
        out_rdy = 0;

        #100;
        $stop;
    end

endmodule