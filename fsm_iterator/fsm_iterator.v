
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
                            || z_mag_sq[26]
                            || (iter_count == `ITER_MAX - 1)
                            || (zr > $signed(27'h1000000))
                            || (zr < $signed(-27'h1000000))
                            || (zi > $signed(27'h1000000))
                            || (zi < $signed(-27'h1000000));
endmodule

module signed_mult_18_bit (
    output signed [17:0] out,
    input  signed [17:0] a,
    input  signed [17:0] b
);
    // intermediate full bit length
    wire signed [35:0] mult_out;
    assign mult_out = a * b;
    // select bits for 4.14 fixed point
    assign out = {mult_out[35], mult_out[30:14]};
endmodule


module full_precision_mult_18_bit (
    output signed [35:0] out,
    input  signed [17:0] a,
    input  signed [17:0] b
);
    assign out = a * b;
endmodule


module karatsuba_multiplier (
    input clk,
    input reset, 
    
    //Is it 18 bit multiply or 27 bit multiply?
    input is_high_resolution,
    

    //assume on the caller side a and b are wires from a register
    input signed [26:0] a,
    input signed [26:0] b,
    output reg signed [26:0] out

    input in_val,
    output reg out_val,
    output reg in_rdy,
);


    //input registers
    reg is_high_resolution_reg;


    localparam [2:0] state_idle = 3'b000,
                    state_high_pres_calc_0 = 3'b010,
                    state_high_pres_calc_1 = 3'b011,
                    state_high_pres_calc_2 = 3'b100;


    reg [2:0] current_state = state_idle;
    reg [2:0] next_state;

    wire signed [17:0] low_res_mult_in_a, low_res_mult_in_b;
    wire signed [17:0] low_res_mult_out;

    reg signed [17:0] fpm_in_a, fpm_in_b;
    reg signed [35:0] fpm_out;

    full_precision_mult_18_bit full_precision_mult(fpm_out, fpm_in_a, fpm_in_b);

    assign low_res_mult_in_a = a[17:0];
    assign low_res_mult_in_b = b[17:0];
    assign low_res_mult_out = {8'b0, fpm_out[35], fpm_out[30:14]};
    //karatsuba multiplication initialization registers
    wire sign_a;
    wire sign_b;
    wire sign_out;

    assign sign_a = a[26];
    assign sign_b = b[26];
    assign sign_out = sign_a ^ sign_b;

    wire [26:0] abs_a;
    wire [26:0] abs_b;

    assign abs_a = sign_a ? (~a + 1) : a;
    assign abs_b = sign_b ? (~b + 1) : b;

    wire [16:0] AL;
    wire [9:0] AH;
    wire [16:0] BL;
    wire [9:0] BH;

    assign AL = abs_a[16:0];
    assign AH = abs_a[26:17];
    assign BL = abs_b[16:0];
    assign BH = abs_b[26:17];

    wire [17:0] AL_extended, AH_extended, BL_extended, BH_extended;
    assign AH_extended = {8'b0, AH};
    assign BH_extended = {8'b0, BH};

    assign AL_extended = {1'b0, AL};
    assign BL_extended = {1'b0, BL};


    wire [17:0] SA, SB;
    assign SA = AL + AH;
    assign SB = BL + BH;

    reg [35:0] Z0, Z2, M;
    reg [35:0] Z0_reg, Z2_reg, M_reg;

    wire [35:0] Z2_shifted;
    assign Z2_shifted = Z2_reg << 34;

    wire signed [37:0] Z1;
    assign Z1 = M_reg - Z0_reg - Z2_reg;




//Mealy state machine
always @(posedge clk) begin
    if (reset) begin
        
    end
    end else begin
        case (current_state) 
            
        endcase
    end


//Mealy outputs as a function of state
always @(*) begin
    if (reset) begin

    end else begin
        case (current_state)
            fpm_in_a = 0;
            fpm_in_b = 0;
            out = 0;
            next_state = state_idle;
            in_rdy = 1'b0;
            out_val = 1'b0;
            state_idle: begin
                if (in_val && !is_high_resolution) begin
                    out_val = 1'b1;
                    in_rdy = 1'b1;
                    fpm_in_a = low_res_mult_in_a;
                    fpm_in_b = low_res_mult_in_b;
                    out = low_res_mult_out;
                    next_state = state_idle;
                end else if (in_val && is_high_resolution) begin
                    out_val = 1'b0;
                    in_rdy = 1'b0;
                    out = 27'sd0; //default output since we don't have a valid output
                    fpm_in_a = AH_extended;
                    fpm_in_b = BH_extended;
                    Z0 = fpm_out;
                    next_state = state_high_pres_calc_0;
                end else begin
                    out_val = 1'b0;
                    in_rdy = 1'b1;
                    out = 27'sd0; //default output since we don't have a valid output
                    next_state = state_idle;
                end
            end
            state_high_pres_calc_0: begin
                out_val = 1'b0;
                in_rdy = 1'b0;
                out = 27'sd0; //default output since we don't have a valid output
                fpm_in_a = AL_extended;
                fpm_in_b = BL_extended;
                Z2 = fpm_out;
                next_state = state_high_pres_calc_1;
            end
            state_high_pres_calc_1: begin
                out_val = 1'b0;
                in_rdy = 1'b0;
                out = 27'sd0;
                fpm_in_a = SA;
                fpm_in_b = SB;
                Z1 = fpm_out;
                next_state = state_high_pres_calc_2; 
            end
            state_high_pres_calc_2: begin
                out_val = 1'b1;
                in_rdy = 1'b1;
                fpm_in_a = SA;
                fpm_in_b = SB;
                M = fpm_out;
                Z1 = M - Z0 - Z2;
                out = 27'sd0; //need to update this

                next_state = state_idle;
            end
        endcase
    end
end
endmodule