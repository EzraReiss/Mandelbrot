module tb_karatsuba_top (
    input clk,
    input reset,
    input signed [26:0] a,
    input signed [26:0] b,
    input in_val,
    input out_rdy,

    output signed [26:0] ref_out,

    output signed [26:0] kara_out,
    output kara_out_val,
    output kara_in_rdy
);

    signed_mult ref_mult (
        .out(ref_out),
        .a(a),
        .b(b)
    );

    karatsuba_multiplier kara (
        .clk(clk),
        .reset(reset),
        .is_high_resolution(1'b1),
        .a(a),
        .b(b),
        .out(kara_out),
        .in_val(in_val),
        .out_rdy(out_rdy),
        .out_val(kara_out_val),
        .in_rdy(kara_in_rdy)
    );

endmodule
