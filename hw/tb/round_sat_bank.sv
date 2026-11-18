// Test harness for the round_sat G3 gate (not synthesised): one 12-bit input into round_sat
// instances covering SHIFT = 1 and the widest shift, saturating and non-saturating outputs,
// and OUT_W at its bound. The parameter sets must match rvec round_sat (ROUND_SAT_SETS).

`default_nettype none

module round_sat_bank (
    input  wire logic signed [11:0] in,
    output logic signed [11:0]      o_s1_w12,   // SHIFT 1,  OUT_W 12 (bound, never saturates)
    output logic signed [9:0]       o_s1_w10,   // SHIFT 1,  OUT_W 10 (saturates)
    output logic signed [8:0]       o_s4_w9,    // SHIFT 4,  OUT_W 9  (bound)
    output logic signed [7:0]       o_s4_w8,
    output logic signed [5:0]       o_s4_w6,
    output logic signed [3:0]       o_s8_w4,
    output logic signed [1:0]       o_s11_w2,   // SHIFT 11, OUT_W 2  (bound)
    output logic signed [0:0]       o_s11_w1
);
    round_sat #(.IN_W(12), .SHIFT(1),  .OUT_W(12)) u0 (.in, .out(o_s1_w12));
    round_sat #(.IN_W(12), .SHIFT(1),  .OUT_W(10)) u1 (.in, .out(o_s1_w10));
    round_sat #(.IN_W(12), .SHIFT(4),  .OUT_W(9))  u2 (.in, .out(o_s4_w9));
    round_sat #(.IN_W(12), .SHIFT(4),  .OUT_W(8))  u3 (.in, .out(o_s4_w8));
    round_sat #(.IN_W(12), .SHIFT(4),  .OUT_W(6))  u4 (.in, .out(o_s4_w6));
    round_sat #(.IN_W(12), .SHIFT(8),  .OUT_W(4))  u5 (.in, .out(o_s8_w4));
    round_sat #(.IN_W(12), .SHIFT(11), .OUT_W(2))  u6 (.in, .out(o_s11_w2));
    round_sat #(.IN_W(12), .SHIFT(11), .OUT_W(1))  u7 (.in, .out(o_s11_w1));
endmodule

`default_nettype wire
