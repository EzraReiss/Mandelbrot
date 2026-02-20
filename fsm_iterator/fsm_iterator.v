
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
    localparam [1:0] IDLE = 2'b00,
                     CALC = 2'b01,
                     DONE = 2'b10;
    
    localparam [1:0] CALC_1 = 2'b00,
                     CALC_2 = 2'b01,
                     CALC_3 = 2'b10;

    reg [1:0] current_state, next_state;
    reg [1:0] calc_state, next_calc_state;

    reg signed [26:0] zr, zi, c_r, c_i;
    reg signed [26:0] zr_zi_reg, zr_sq_reg;
    reg signed [26:0] zr_next, zi_next;
    reg signed [27:0] z_mag_sq; 
    
    reg signed [26:0] a, b;
    wire signed [26:0] y;

    reg escaped_reg;
    assign escape_condition = escaped_reg;

    signed_mult mult_a_b_y (
        .out(y),
        .a(a),
        .b(b)
    );

    // Fixed typos, removed semicolon, removed invalid underscores
    wire done_condition = z_mag_sq > $signed(27'h2000000) 
                            || iter_count == `ITER_MAX - 1
                            || zi_next > $signed( 27'h1000000) 
                            || zi_next < $signed(-27'h1000000) 
                            || zr_next > $signed( 27'h1000000) 
                            || zr_next < $signed(-27'h1000000)
                            || zr_zi_reg > $signed({4'sd3, 23'h800000})
                            || zr_zi_reg < $signed({-4'sd3, 23'h800000}); 

    always @ (posedge clk) begin
        if (reset) begin
            current_state <= IDLE;
            calc_state    <= CALC_1;
            iter_count    <= 0;
            in_rdy        <= 1'b0;
            out_val       <= 1'b0;
            zr            <= 0;
            zi            <= 0;
            c_r           <= 0;
            c_i           <= 0;
            zr_sq_reg     <= 0;
            zr_zi_reg     <= 0;
            escaped_reg   <= 0;
        end
        else begin
            current_state <= next_state;
            calc_state    <= next_calc_state;      
            
            case (current_state)
                IDLE: begin
                    in_rdy <= 1'b1;
                    if (in_val && in_rdy) begin
                        in_rdy      <= 1'b0;
                        out_val     <= 1'b0;
                        c_r         <= in_c_r;
                        c_i         <= in_c_i;
                        zr          <= 0;
                        zi          <= 0;
                        iter_count  <= 0;
                        escaped_reg <= 0;
                    end
                end

                CALC: begin
                    if (calc_state == CALC_1) begin
                        zr_zi_reg <= y;
                    end 
                    else if (calc_state == CALC_2) begin
                        zr_sq_reg <= y;
                    end 
                    else if (calc_state == CALC_3) begin
                        if (done_condition) begin
                            // FIX: Replaced undeclared variable with correct logic
                            escaped_reg <= (iter_count != `ITER_MAX - 1);
                        end else begin
                            zr         <= zr_next;
                            zi         <= zi_next;
                            iter_count <= iter_count + 1;
                        end
                    end
                end

                DONE: begin
                    out_val <= 1'b1;
                    if (out_rdy && out_val) begin
                        out_val <= 1'b0;
                    end
                end
            endcase
        end
    end

    always @ (*) begin
        next_state      = current_state;
        next_calc_state = calc_state;
        a = 0;
        b = 0;
        zr_next  = zr;
        zi_next  = zi;
        z_mag_sq = 0;

        case (current_state)
            IDLE: begin
                if (in_val && in_rdy) begin
                    next_state      = CALC;
                    next_calc_state = CALC_1;
                end
            end

            CALC: begin
                case (calc_state) 
                    CALC_1: begin
                        a = zr;
                        b = zi;
                        next_calc_state = CALC_2;
                    end

                    CALC_2: begin
                        a = zr;
                        b = zr;
                        next_calc_state = CALC_3;
                    end

                    CALC_3: begin
                        a = zi;
                        b = zi;
                        
                        z_mag_sq = $signed({zr_sq_reg[26], zr_sq_reg}) + $signed({y[26], y});
                        zr_next  = zr_sq_reg - y + c_r;
                        zi_next  = (zr_zi_reg <<< 1) + c_i;

                        if (done_condition) begin
                            next_state = DONE;
                            next_calc_state = CALC_1; // FIX: Added calc state reset
                        end
                        else begin
                            next_calc_state = CALC_1;
                        end
                    end
                endcase
            end
            
            DONE: begin
                if (out_rdy && out_val) begin
                    next_state = IDLE;
                end
            end
        endcase
    end
endmodule