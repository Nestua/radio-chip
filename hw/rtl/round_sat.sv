// Narrowing of a signed integer: drop SHIFT low bits with convergent rounding (half to even),
// then saturate to OUT_W bits. Bit-exact with radio_phy::fixed::requantize(.., HalfEven,
// Saturate) as used by rx_front_fixed::narrow (the golden model's only narrowing rule).
// Gate G3: make -C hw round_sat (exhaustive over a 12-bit input, several parameter sets).
// Combinational; the caller registers the result.
//
// Parameters: SHIFT >= 1 and OUT_W <= IN_W - SHIFT + 1 (checked at elaboration). A wider
// output than the rounded value needs is a plain sign extension: do that outside.

`default_nettype none

module round_sat #(
    parameter int IN_W  = 28,
    parameter int SHIFT = 8,
    parameter int OUT_W = 20
) (
    input  wire logic signed [IN_W-1:0]  in,
    output logic signed [OUT_W-1:0]      out
);
    localparam int R_W = IN_W - SHIFT + 1;   // q plus one guard bit: q + 1 cannot overflow

    if (SHIFT < 1 || SHIFT >= IN_W || OUT_W < 1 || OUT_W > R_W) begin : g_bad_params
        $error("round_sat: need 1 <= SHIFT < IN_W and 1 <= OUT_W <= IN_W - SHIFT + 1");
    end

    // floor(in / 2^SHIFT); the dropped remainder is in[SHIFT-1:0] (>= 0, two's complement).
    logic signed [IN_W-SHIFT-1:0] q;
    logic                         half;     // remainder >= 1/2
    logic                         sticky;   // remainder bits below the half bit (> 1/2 if set)
    logic                         up;
    logic signed [R_W-1:0]        r;

    localparam logic signed [R_W-1:0] MAX = R_W'((1 <<< (OUT_W - 1)) - 1);
    localparam logic signed [R_W-1:0] MIN = R_W'(-(1 <<< (OUT_W - 1)));

    assign q    = in[IN_W-1:SHIFT];
    assign half = in[SHIFT-1];
    if (SHIFT > 1) begin : g_sticky
        assign sticky = |in[SHIFT-2:0];
    end else begin : g_no_sticky
        assign sticky = 1'b0;
    end
    // Round up above half; at exactly half, only when that makes the result even (q odd).
    assign up = half && (sticky || q[0]);
    assign r  = {q[IN_W-SHIFT-1], q} + R_W'(up);

    always_comb begin
        if (r > MAX)      out = MAX[OUT_W-1:0];
        else if (r < MIN) out = MIN[OUT_W-1:0];
        else              out = r[OUT_W-1:0];
    end
endmodule

`default_nettype wire
