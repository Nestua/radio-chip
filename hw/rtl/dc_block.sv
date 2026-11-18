// Causal DC blocker of the receiver front: M = m·2^F, e = x·2^F − M, M += e >>> k, y = e·2^−F
// rounded half to even and saturated to Y_BITS; k = min(⌊log2(n + 1)⌋, DC_SHIFT) for the n-th
// sample after reset (a start-up gear: the first sample sets M to x exactly, then the pole
// slows to 2^−DC_SHIFT). Golden model: radio_phy::rx_front_fixed::DcFixed (detector_trace_fixed,
// RxFrontFixed::push step 1). Gate G3: make -C hw det.
//
// Widths: |x·2^F| ≤ 2^(11+F) and M stays within the inputs' range (each update moves it a
// floored fraction of the way to x·2^F), so e needs Y_BITS + F bits; M_W = Y_BITS + F + 1 leaves
// a bit of margin. |e|·2^−F ≤ 4095 + 2^(k−F) < 4095.5, so the saturation is unreachable
// (kept: the model saturates).
//
// Stream: one sample per in_valid, never stalls; y registered at the edge that takes x.

`default_nettype none

module dc_block
    import rx_front_rom_pkg::*;
(
    input  wire logic                     clk,
    input  wire logic                     rst_n,     // synchronous, active low
    input  wire logic                     in_valid,
    input  wire logic signed [Y_BITS-2:0] in_re,     // 12-bit ADC
    input  wire logic signed [Y_BITS-2:0] in_im,
    output logic                          out_valid,
    output logic signed [Y_BITS-1:0]      out_re,
    output logic signed [Y_BITS-1:0]      out_im
);
    localparam int MW = Y_BITS + DC_FRAC + 1;
    localparam int CW = $clog2(DC_SHIFT + 2);      // n + 1 up to 2^DC_SHIFT
    localparam int NW = DC_SHIFT + 1;

    // ---- start-up gear: n counts samples since reset, saturating once k reaches DC_SHIFT
    logic [NW-1:0] n, np1;
    logic [CW-1:0] k;
    assign np1 = n + 1'b1;
    always_comb begin
        k = '0;
        for (int b = 0; b < NW; b++)
            if (np1[b]) k = CW'(b);                // ⌊log2(n + 1)⌋ (np1 ≥ 1)
        if (int'(k) > DC_SHIFT) k = CW'(DC_SHIFT);
    end
    always_ff @(posedge clk) begin
        if (!rst_n)
            n <= '0;
        else if (in_valid && np1 != NW'(1 << DC_SHIFT))
            n <= np1;                               // stops at 2^DC_SHIFT − 1: k = DC_SHIFT
    end

    // ---- e = x·2^F − M, M += e >>> k
    logic signed [MW-1:0] m_re, m_im, e_re, e_im;
    assign e_re = (MW'(in_re) <<< DC_FRAC) - m_re;
    assign e_im = (MW'(in_im) <<< DC_FRAC) - m_im;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_re <= '0;
            m_im <= '0;
        end else if (in_valid) begin
            m_re <= m_re + (e_re >>> k);
            m_im <= m_im + (e_im >>> k);
        end
    end

    // ---- y = e·2^−F, half to even, saturated
    logic signed [Y_BITS-1:0] y_re, y_im;
    round_sat #(.IN_W(MW), .SHIFT(DC_FRAC), .OUT_W(Y_BITS)) u_rs_re (.in(e_re), .out(y_re));
    round_sat #(.IN_W(MW), .SHIFT(DC_FRAC), .OUT_W(Y_BITS)) u_rs_im (.in(e_im), .out(y_im));
    always_ff @(posedge clk) begin
        out_re <= y_re;
        out_im <= y_im;
        if (!rst_n) out_valid <= 1'b0;
        else        out_valid <= in_valid;
    end
endmodule

`default_nettype wire
