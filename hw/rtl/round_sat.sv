// Narrowing of a signed integer: drop SHIFT low bits with convergent rounding (half to even),
// then saturate to OUT_W bits. Bit-exact with radio_phy::fixed::requantize(.., HalfEven,
// Saturate) as used by rx_front_fixed::narrow (the golden model's only narrowing rule).
// Combinational; the caller registers the result.

`default_nettype none

module round_sat #(
    parameter int IN_W  = 28,
    parameter int SHIFT = 8,   // >= 1
    parameter int OUT_W = 20
) (
    input  wire logic signed [IN_W-1:0]  in,
    output logic signed [OUT_W-1:0]      out
);
    // floor(in / 2^SHIFT) and the dropped remainder (>= 0, two's complement).
    logic signed [IN_W-SHIFT-1:0] q;
    logic        [SHIFT-1:0]      rem;
    logic                         up;
    logic signed [IN_W-SHIFT:0]   r;     // one guard bit: q + 1 cannot overflow

    localparam logic [SHIFT-1:0] HALF = 1 << (SHIFT - 1);
    localparam logic signed [IN_W-SHIFT:0] MAX = (1 <<< (OUT_W - 1)) - 1;
    localparam logic signed [IN_W-SHIFT:0] MIN = -(1 <<< (OUT_W - 1));

    assign q   = in[IN_W-1:SHIFT];
    assign rem = in[SHIFT-1:0];
    // Round up above half; at exactly half, only when that makes the result even (q odd).
    assign up  = (rem > HALF) || (rem == HALF && q[0]);
    assign r   = {q[IN_W-SHIFT-1], q} + {{(IN_W-SHIFT){1'b0}}, up};

    always_comb begin
        if (r > MAX)      out = MAX[OUT_W-1:0];
        else if (r < MIN) out = MIN[OUT_W-1:0];
        else              out = r[OUT_W-1:0];
    end
endmodule

`default_nettype wire
