
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


// Variable-precision multiplier using one 18x18 DSP block.
// Low-res (is_high_resolution=0): combinational Q4.14 multiply, 0 extra cycles.
// High-res (is_high_resolution=1): Karatsuba Q4.23 multiply, 3 extra cycles.
// Caller must hold a, b, is_high_resolution stable while in_rdy is low.
module karatsuba_multiplier (
    input clk,
    input reset,
    input is_high_resolution,

    input signed [26:0] a,
    input signed [26:0] b,
    output reg signed [26:0] out,

    input in_val,
    input out_rdy,
    output reg out_val,
    output reg in_rdy
);

    localparam [1:0] IDLE   = 2'b00,
                     CALC_0 = 2'b01,
                     CALC_1 = 2'b10,
                     CALC_2 = 2'b11;

    reg [1:0] current_state;
    reg [1:0] next_state;

    // Single 18x18 signed multiplier (maps to one DSP block)
    reg  signed [17:0] fpm_in_a, fpm_in_b;
    wire signed [35:0] fpm_out;
    full_precision_mult_18_bit fpm(fpm_out, fpm_in_a, fpm_in_b);

    // ---- Low-res path: truncate Q4.23 → Q4.14, multiply, widen back ----
    wire signed [17:0] lo_a = a[26:9];
    wire signed [17:0] lo_b = b[26:9];
    wire signed [17:0] lo_out = {fpm_out[35], fpm_out[30:14]};

    // ---- Karatsuba: sign handling ----
    wire sign_out = a[26] ^ b[26];
    wire [26:0] abs_a = a[26] ? (~a + 1) : a;
    wire [26:0] abs_b = b[26] ? (~b + 1) : b;

    // Split at bit 16 so that SA/SB never overflow bit 17 of signed 18-bit input.
    // k=16: AL is 16-bit, AH is 11-bit. SA max = 65535+2047 = 67582 < 2^17. Safe.
    wire [15:0] AL = abs_a[15:0];
    wire [10:0] AH = abs_a[26:16];
    wire [15:0] BL = abs_b[15:0];
    wire [10:0] BH = abs_b[26:16];

    wire [17:0] AL_ext = {2'b0, AL};
    wire [17:0] AH_ext = {7'b0, AH};
    wire [17:0] BL_ext = {2'b0, BL};
    wire [17:0] BH_ext = {7'b0, BH};
    wire [17:0] SA_ext = {2'b0, AL} + {7'b0, AH};
    wire [17:0] SB_ext = {2'b0, BL} + {7'b0, BH};

    // ---- Registered partial products ----
    reg [35:0] Z2_reg;  // AH * BH
    reg [35:0] Z0_reg;  // AL * BL
    reg [35:0] M_reg;   // (AL+AH) * (BL+BH)

    // ---- Karatsuba reconstruction in 54-bit unsigned space ----
    // result = Z0 + Z1*2^16 + Z2*2^32,  where Z1 = M - Z0 - Z2
    wire [53:0] Z0_w = {18'b0, Z0_reg};
    wire [53:0] Z2_w = {18'b0, Z2_reg};
    wire [53:0] M_w  = {18'b0, M_reg};
    wire [53:0] Z1_w = M_w - Z0_w - Z2_w;
    wire [53:0] p_mag = Z0_w + (Z1_w << 16) + (Z2_w << 32);

    // Negate full product BEFORE truncation to match 2's complement rounding
    wire [53:0] p_signed = sign_out ? (~p_mag + 1'b1) : p_mag;
    wire signed [26:0] hi_out = {p_signed[53], p_signed[48:23]};

    // ---- Registered block: state update + capture multiply results ----
    always @(posedge clk) begin
        if (reset) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
            case (current_state)
                IDLE: begin
                    if (in_val && is_high_resolution)
                        Z2_reg <= fpm_out;
                end
                CALC_0:
                    Z0_reg <= fpm_out;
                CALC_1:
                    M_reg  <= fpm_out;
                default: ;
            endcase
        end
    end

    // ---- Combinational block: next state, mux, handshake, output ----
    always @(*) begin
        next_state = current_state;
        fpm_in_a   = 18'sd0;
        fpm_in_b   = 18'sd0;
        out        = 27'sd0;
        out_val    = 1'b0;
        in_rdy     = 1'b0;

        case (current_state)
            IDLE: begin
                in_rdy = 1'b1;
                if (in_val && !is_high_resolution) begin
                    fpm_in_a = lo_a;
                    fpm_in_b = lo_b;
                    out      = {lo_out, 9'b0};
                    out_val  = 1'b1;
                end else if (in_val && is_high_resolution) begin
                    fpm_in_a   = AH_ext;
                    fpm_in_b   = BH_ext;
                    next_state = CALC_0;
                end
            end
            CALC_0: begin
                fpm_in_a   = AL_ext;
                fpm_in_b   = BL_ext;
                next_state = CALC_1;
            end
            CALC_1: begin
                fpm_in_a   = SA_ext;
                fpm_in_b   = SB_ext;
                next_state = CALC_2;
            end
            CALC_2: begin
                out     = hi_out;
                out_val = 1'b1;
                if (out_rdy)
                    next_state = IDLE;
            end
        endcase
    end
endmodule