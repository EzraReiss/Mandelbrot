// Compute modules extracted from DE1_SoC_Computer.v for Verilator testing.
// Keep in sync with the versions in DE1_SoC_Computer.v.

module M10K_1000_8 #(
    parameter MEM_DEPTH = 307200,
    parameter DATA_WIDTH = 10
)(
    output reg [DATA_WIDTH-1:0] q,
    input [DATA_WIDTH-1:0] d,
    input [18:0] write_address, read_address,
    input we, clk
);
    // Simple array for simulation (no sub-block partitioning needed)
    reg [DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    always @(posedge clk) begin
        if (we)
            mem[write_address] <= d;
        q <= mem[read_address];
    end
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

    reg  signed [17:0] fpm_in_a, fpm_in_b;
    wire signed [35:0] fpm_out;
    full_precision_mult_18_bit fpm(fpm_out, fpm_in_a, fpm_in_b);

    wire signed [17:0] lo_a = a[26:9];
    wire signed [17:0] lo_b = b[26:9];
    wire signed [17:0] lo_out = {fpm_out[35], fpm_out[30:14]};

    wire sign_out = a[26] ^ b[26];
    wire [26:0] abs_a = a[26] ? (~a + 1) : a;
    wire [26:0] abs_b = b[26] ? (~b + 1) : b;

    wire [15:0] AL = abs_a[15:0];
    wire [10:0] AH = abs_a[26:16];
    wire [15:0] BL = abs_b[15:0];
    wire [10:0] BH = abs_b[26:16];

    wire [17:0] AH_ext = {7'b0, AH};
    wire [17:0] BH_ext = {7'b0, BH};
    wire [17:0] AL_ext = {2'b0, AL};
    wire [17:0] BL_ext = {2'b0, BL};
    wire [17:0] SA_ext = {2'b0, AL} + {7'b0, AH};
    wire [17:0] SB_ext = {2'b0, BL} + {7'b0, BH};

    reg [21:0] Z2_reg;
    reg [31:0] Z0_reg;
    reg [33:0] M_reg;

    wire [53:0] Z0_w = {22'b0, Z0_reg};
    wire [53:0] Z2_w = {32'b0, Z2_reg};
    wire [53:0] M_w  = {20'b0, M_reg};
    wire [53:0] Z1_w = M_w - Z0_w - Z2_w;
    wire [53:0] p_mag = Z0_w + (Z1_w << 16) + (Z2_w << 32);

    wire [53:0] p_signed = sign_out ? (~p_mag + 1'b1) : p_mag;
    wire signed [26:0] hi_out = {p_signed[53], p_signed[48:23]};

    always @(posedge clk) begin
        if (reset) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
            case (current_state)
                IDLE: begin
                    if (in_val && is_high_resolution)
                        Z2_reg <= fpm_out[21:0];
                end
                CALC_0: Z0_reg <= fpm_out[31:0];
                CALC_1: M_reg  <= fpm_out[33:0];
                default: ;
            endcase
        end
    end

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

module fsm_iterator (
    input reset,
    input clk,
    input in_val,
    output reg in_rdy,
    input is_high_resolution,

    input signed [26:0] in_c_r,
    input signed [26:0] in_c_i,
    input [9:0] iter_max,
    output reg [9:0] iter_count,
    output escape_condition,
    output reg out_val,
    input out_rdy
);
    localparam [2:0] IDLE   = 3'b000,
                     CALC_1 = 3'b001,
                     CALC_2 = 3'b010,
                     CALC_3 = 3'b011,
                     DONE   = 3'b100;

    reg [2:0] current_state;
    reg [2:0] next_state;

    reg signed [26:0] zi, zr, c_r, c_i;
    reg signed [26:0] zr_sq_reg, zi_sq_reg;
    wire signed [26:0] zr_next, zi_next, z_mag_sq;

    reg  signed [26:0] kara_a, kara_b;
    wire signed [26:0] kara_out;
    reg  kara_in_val, kara_out_rdy;
    wire kara_out_val, kara_in_rdy;

    karatsuba_multiplier mult (
        .clk(clk), .reset(reset),
        .is_high_resolution(is_high_resolution),
        .a(kara_a), .b(kara_b), .out(kara_out),
        .in_val(kara_in_val), .out_rdy(kara_out_rdy),
        .out_val(kara_out_val), .in_rdy(kara_in_rdy)
    );

    always @(posedge clk) begin
        if (reset) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
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
                CALC_1: if (kara_out_val) zr_sq_reg <= kara_out;
                CALC_2: if (kara_out_val) zi_sq_reg <= kara_out;
                CALC_3: begin
                    if (kara_out_val) begin
                        zi <= zi_next;
                        zr <= zr_next;
                        iter_count <= iter_count + 1;
                    end
                end
                default: ;
            endcase
        end
    end

    always @(*) begin
        next_state = current_state;
        in_rdy = 1'b0;
        out_val = 1'b0;
        case (current_state)
            IDLE: begin
                in_rdy = 1'b1;
                if (in_val) next_state = CALC_1;
            end
            CALC_1: if (kara_out_val) next_state = CALC_2;
            CALC_2: if (kara_out_val) next_state = CALC_3;
            CALC_3: begin
                if (kara_out_val) begin
                    if (escape_condition) next_state = DONE;
                    else                  next_state = CALC_1;
                end
            end
            DONE: begin
                out_val = 1'b1;
                if (out_rdy) next_state = IDLE;
            end
        endcase
    end

    always @(*) begin
        kara_a = 27'sd0; kara_b = 27'sd0;
        kara_in_val = 1'b0; kara_out_rdy = 1'b0;
        case (current_state)
            CALC_1: begin kara_a = zr; kara_b = zr; kara_in_val = kara_in_rdy; kara_out_rdy = 1'b1; end
            CALC_2: begin kara_a = zi; kara_b = zi; kara_in_val = kara_in_rdy; kara_out_rdy = 1'b1; end
            CALC_3: begin kara_a = zr; kara_b = zi; kara_in_val = kara_in_rdy; kara_out_rdy = 1'b1; end
            default: ;
        endcase
    end

    assign zr_next = zr_sq_reg - zi_sq_reg + c_r;
    assign zi_next = (kara_out <<< 1) + c_i;
    assign z_mag_sq = zr_sq_reg + zi_sq_reg;

    assign escape_condition = (z_mag_sq > $signed(27'h2000000))
                            || z_mag_sq[26]
                            || (iter_count == iter_max - 1)
                            || (zr > $signed(27'h1000000))
                            || (zr < $signed(-27'h1000000))
                            || (zi > $signed(27'h1000000))
                            || (zi < $signed(-27'h1000000));
endmodule

module mandelbrot_top #(
    parameter ITERATOR_ID   = 0,
    parameter NUM_ITERATORS = 1
) (
    input reset, input clk,
    input signed [26:0] x_start, y_start,
    input signed [26:0] pixel_increment_x, pixel_increment_y,
    input [9:0] iter_max,
    input is_high_resolution,
    output reg done,
    output [9:0] mem_write_data,
    output reg [18:0] mem_write_address,
    output reg mem_we
);
    localparam [1:0] CALC = 2'b01, DONE = 2'b10;

    reg [9:0] iter_max_reg;
    reg       hires_reg;
    reg [1:0] current_state, next_state;
    reg signed [26:0] curr_x, curr_y;
    reg [9:0] pixel_x, pixel_y, next_pixel_x, next_pixel_y;
    reg signed [26:0] next_x, next_y;

    reg iterator_reset, iterator_in_val, iterator_out_rdy;
    wire iterator_in_rdy, iterator_out_val;
    wire [9:0] iterator_iter_count;
    wire iterator_escape_condition;

    assign mem_write_data = iterator_iter_count;

    fsm_iterator iter1 (
        .reset(iterator_reset), .clk(clk),
        .in_val(iterator_in_val), .in_rdy(iterator_in_rdy),
        .is_high_resolution(hires_reg),
        .in_c_r(curr_x), .in_c_i(curr_y),
        .iter_count(iterator_iter_count), .iter_max(iter_max_reg),
        .escape_condition(iterator_escape_condition),
        .out_val(iterator_out_val), .out_rdy(iterator_out_rdy)
    );

    always @(posedge clk) begin
        if (reset) begin
            done <= 0; current_state <= CALC;
            curr_x <= x_start; curr_y <= y_start;
            pixel_x <= 0; pixel_y <= 0;
            iterator_reset <= 1; iter_max_reg <= iter_max;
            hires_reg <= is_high_resolution;
            mem_write_address <= 0; mem_we <= 0;
            iterator_in_val <= 0; iterator_out_rdy <= 0;
        end else begin
            iterator_reset <= 0;
            current_state <= next_state;
            case (current_state)
                CALC: begin
                    if (iterator_in_rdy && !iterator_in_val) begin
                        iterator_in_val <= 1; iterator_out_rdy <= 0; mem_we <= 0;
                        if (mem_write_address != 0 || pixel_x != 0 || pixel_y != 0)
                            mem_write_address <= mem_write_address + 1;
                    end
                    else if (iterator_in_val) begin
                        iterator_in_val <= 0; mem_we <= 0;
                    end
                    else if (iterator_out_val) begin
                        iterator_out_rdy <= 1; mem_we <= 1;
                        curr_x <= next_x; curr_y <= next_y;
                        pixel_x <= next_pixel_x; pixel_y <= next_pixel_y;
                    end
                    else begin
                        iterator_in_val <= 0; iterator_out_rdy <= 1; mem_we <= 0;
                    end
                end
                DONE: begin done <= 1; mem_we <= 0; end
            endcase
        end
    end

    always @(*) begin
        if (pixel_x == `X_PIXEL_MAX) begin
            next_pixel_x = 0; next_pixel_y = pixel_y + 1;
            next_x = x_start; next_y = curr_y - pixel_increment_y;
        end else begin
            next_pixel_x = pixel_x + 1; next_pixel_y = pixel_y;
            next_x = curr_x + pixel_increment_x; next_y = curr_y;
        end
    end

    always @(*) begin
        next_state = current_state;
        case (current_state)
            CALC: if (iterator_out_val && pixel_x == `X_PIXEL_MAX && pixel_y == `Y_PIXEL_MAX)
                      next_state = DONE;
            DONE:    next_state = DONE;
            default: next_state = CALC;
        endcase
    end
endmodule
