
module signed_mult (
    output signed [26:0] out,
    input  signed [26:0] a,
    input  signed [26:0] b
);
    // intermediate full bit length
    wire signed [53:0] mult_out;
    assign mult_out = a * b;
    // select bits for 4.23 fixed point
    assign out = {mult_out[53], mult_out[48:23]};
endmodule

module fsm_iterator (
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
    // state machine encoding 
    localparam [2:0] IDLE   = 3'b000,
                     CALC_1 = 3'b001,
                     CALC_2 = 3'b010,
                     CALC_3 = 3'b011,
                     DONE   = 3'b100;
                     
    reg [2:0] current_state = IDLE;
    reg [2:0] next_state;

    // internal registers for iterative calculations
    reg signed [26:0] zi, zr, c_r, c_i;
    reg signed [26:0] zr_sq_reg, zi_sq_reg;
    wire signed [26:0] zr_next, zi_next, z_mag_sq;


   // instantiate ONE multiplier 
    reg  signed [26:0] mult_in_a, mult_in_b;
    wire signed [26:0] mult_out;

    signed_mult single_mult(mult_out, mult_in_a, mult_in_b);

    always @(posedge clk) begin
        case (current_state)
            IDLE: begin
                if (in_val) begin
                    c_r <= in_c_r;
                    c_i <= in_c_i;
                    zi <= 27'sd0;
                    zr <= 27'sd0;
                    zr_sq_reg <= 27'sd0;
                    zi_sq_reg <= 27'sd0;
                    iter_count <= 0;
                end
            end
            CALC_1: begin
                zr_sq_reg <= mult_out;
            end
            CALC_2: begin
                zi_sq_reg <= mult_out;
            end
            CALC_3: begin
                zi <= zi_next;
                zr <= zr_next;
                iter_count <= iter_count + 1;
            end
            DONE: begin
                // do nothing
            end
        endcase

        // update state
        if (reset) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // next state and output logic
    always @(*) begin
        next_state = current_state;
        in_rdy = 1'b0;
        out_val = 1'b0;

        case (current_state)
            IDLE: begin
                in_rdy = 1'b1;
                if (in_val) begin
                    next_state = CALC_1;
                end
            end
            CALC_1: begin
                next_state = CALC_2;
            end
            CALC_2: begin
                next_state = CALC_3;
            end
            CALC_3: begin
                if (escape_condition) begin
                    next_state = DONE;
                end else begin
                    next_state = CALC_1;
                end
            end
            DONE: begin
                out_val = 1'b1;
                if (out_rdy) begin
                    next_state = IDLE;
                end
            end
        endcase
    end

    // multiplier input mux based on state
    always @(*) begin
        case (current_state)
            CALC_1: begin
                mult_in_a = zr;
                mult_in_b = zr;
            end
            CALC_2: begin
                mult_in_a = zi;
                mult_in_b = zi;
            end
            CALC_3, DONE: begin 
                mult_in_a = zr;
                mult_in_b = zi;
            end
            default: begin
                mult_in_a = 27'sd0;
                mult_in_b = 27'sd0;
            end
        endcase
    end

    // combinational arithmetic
    assign zr_next = zr_sq_reg - zi_sq_reg + c_r;
    assign zi_next = (mult_out <<< 1) + c_i;  
    assign z_mag_sq = zr_sq_reg + zi_sq_reg;

    assign escape_condition = (z_mag_sq > $signed(27'h2000000)) 
                            || (iter_count == `ITER_MAX - 1)
                            || (zi_next > $signed(27'h1000000)) 
                            || (zi_next < $signed(-27'h1000000)) 
                            || (zr_next > $signed(27'h1000000)) 
                            || (zr_next < $signed(-27'h1000000));

endmodule