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
    wire escape_condition;
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
	.escape_condition(escape_condition), 
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
//End AI Usage 

/////////////////////////////////////////////////
//// signed mult of 4.23 format 2'comp////////////
//////////////////////////////////////////////////

module signed_mult (out, a, b);
	output 	signed  [26:0]	out;
	input 	signed	[26:0] 	a;
	input 	signed	[26:0] 	b;
	// intermediate full bit length
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 4.23 fixed point
	assign out = {mult_out[53], mult_out[48:23]};
endmodule
//////////////////////////////////////////////////

`define ITER_MAX 1000

module iterator (
	input reset,
	input clk,
	input in_val,
	output reg in_rdy,

	input signed  [26:0] in_c_r,
	input signed  [26:0] in_c_i,

	output reg signed [$clog2(`ITER_MAX):0] iter_count,
	output escape_condition,
	output reg out_val,
	input out_rdy
);
	
	localparam [1:0] IDLE = 2'b00,
	                 CALC = 2'b01,
	                 DONE = 2'b10;
	reg  [1:0] current_state;
	reg [1:0] next_state;

	reg signed [26:0] zi, zr, zr_sq, zi_sq, c_r, c_i;
	//wire escape_condition;
	wire signed [26:0] zr_next, zi_next, z_mag_sq, zr_sq_next, zi_sq_next, zr_zi;

	always @(posedge clk) begin
		case (current_state)
			IDLE: begin
				if (in_val) begin
					c_r <= in_c_r;
					c_i <= in_c_i;
					zi <= 27'sd0;
					zr <= 27'sd0;
					zr_sq <= 27'sd0;
					zi_sq <= 27'sd0;
					iter_count <= 0;
				end
			end
			CALC: begin
				zi <= zi_next;
				zr <= zr_next;
				zr_sq <= zr_sq_next;
				zi_sq <= zi_sq_next;
				iter_count <= iter_count + 1;
			end
			DONE: begin
				// Stay in DONE until reset
			end
		endcase
		if (reset) begin
			current_state <= IDLE;
		end
		else
		current_state <= next_state;
		
	end	

	always @(*) begin
		// Default assignments to prevent inferred latches
		next_state = current_state;
		in_rdy = 1'b0;
		out_val = 1'b0;

		case (current_state)
			IDLE: begin
				in_rdy = 1'b1;
				if (in_val) begin
					next_state = CALC;
				end
			end
			CALC: begin
				if (escape_condition) begin
					next_state = DONE;
				end
			end
			DONE: begin
				out_val = 1'b1;
				if (out_rdy) begin
					next_state = IDLE;
				end
			end
			default: begin
				next_state = IDLE;
				in_rdy = 1'b1;
			end
		endcase
	end



	
	assign escape_condition = z_mag_sq > $signed(27'h2000000) 
							|| iter_count == `ITER_MAX - 1
							|| zi_next > $signed(27'h1000000) 
							|| zi_next < $signed(-27'h1000000) 
							|| zr_next > $signed(27'h1000000) 
							|| zr_next < $signed(-27'h1000000); 

	signed_mult mult_zr_zr(zr_sq_next, zr, zr);
	signed_mult mult_zi_zi(zi_sq_next, zi, zi);
	signed_mult mult_zr_zi(zr_zi, zr, zi);

	assign zr_next = zr_sq_next - zi_sq_next + c_r;
	assign zi_next = (zr_zi <<< 1) + c_i;

	assign z_mag_sq = zr_sq_next + zi_sq_next;
endmodule

