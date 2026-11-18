// Detector A of the receiver front with its run counter: lag-16 autocorrelation over
// AUTOCORR_LEN products, C = Σ x(k)·conj(x(k+16)), P1 = Σ|x(k)|², P2 = Σ|x(k+16)|², fire when
// 2^THETA_SHIFT·|C|² ≥ THETA_NUM·P1·P2 (exact, no normalisation); an event when DET_RUN
// consecutive samples fire, the counter restarting after each event, on a non-firing sample and
// on `clear` (the top level's acquisition). Golden model: radio_phy::rx_front_fixed::DetFixed
// and the run counter of detector_trace_fixed / RxFrontFixed::push step 1. Gate G3:
// make -C hw det.
//
// The model adds a sample's terms from n = 16 on (x(k) exists) and subtracts the oldest from
// n = 64 on (k ≥ AUTOCORR_LEN), and may fire from n = 63 on (a full window) when P1, P2 > 0; a
// saturating sample counter reproduces exactly that, so the history needs no reset.
//
// Widths at D = DET_BITS: products < 2^(2D−1) per component, sums of 48 < 2^(2D+5): C needs
// 2D + 6 bits signed, P 2D + 5 unsigned; |C|² < 2^(4D+11), shifted by THETA_SHIFT; THETA_NUM·P1·P2
// < 2^(15 + 4D + 10). Both sides fit CMP_W = 4D + 11 + THETA_SHIFT + 1 bits.
//
// Stream: one sample per in_valid, never stalls. Pipeline: products, sums, the two sides,
// compare and count: fire/event_o registered 4 edges after the edge that takes the sample.
// `clear` resets the counter after the counting of the sample in stage 4, and only on a cycle
// that stage holds a valid sample (v3): the top level asserts it on the cycle sample n is in
// stage 4 to express the model's count reset at apply_pending of sample n (step 1 first).
//
// c_valid/c_re/c_im expose C(n) of the sample in stage 2 (exact, C_W bits) for the provisional
// latch (rx_latch): C is updated at the edge that registers v2, so c_valid = v2 holds it.

`default_nettype none

module det_a
    import rx_front_rom_pkg::*;
(
    input  wire logic                     clk,
    input  wire logic                     rst_n,     // synchronous, active low
    input  wire logic                     in_valid,
    input  wire logic signed [Y_BITS-1:0] in_re,
    input  wire logic signed [Y_BITS-1:0] in_im,
    input  wire logic                     clear,
    output logic                          c_valid,
    output logic signed [2*DET_BITS+5:0]  c_re,      // C_W = 2D + 6
    output logic signed [2*DET_BITS+5:0]  c_im,
    output logic                          out_valid,
    output logic                          fire,
    output logic                          event_o
);
    localparam int D     = DET_BITS;
    localparam int LAG   = STF_PERIOD;
    localparam int LEN   = AUTOCORR_LEN;
    localparam int DEPTH = LEN + LAG;              // x(n−1) … x(n−64)
    localparam int PR_W  = 2 * D + 1;              // one complex product component
    localparam int C_W   = 2 * D + 6;
    localparam int P_W   = 2 * D + 5;
    localparam int CMP_W = 4 * D + 12 + THETA_SHIFT;
    localparam int RW    = $clog2(DET_RUN + 1);

    // ---- D-bit samples (the model's narrow(y, Y_BITS, D))
    logic signed [D-1:0] x_re, x_im;
    if (D == Y_BITS) begin : g_full
        assign x_re = in_re;
        assign x_im = in_im;
    end else begin : g_narrow
        round_sat #(.IN_W(Y_BITS), .SHIFT(Y_BITS - D), .OUT_W(D)) u_re (.in(in_re), .out(x_re));
        round_sat #(.IN_W(Y_BITS), .SHIFT(Y_BITS - D), .OUT_W(D)) u_im (.in(in_im), .out(x_im));
    end

    // ---- history and sample counter
    logic signed [D-1:0] h_re [0:DEPTH-1];
    logic signed [D-1:0] h_im [0:DEPTH-1];
    logic [$clog2(DEPTH+1)-1:0] n;                 // saturates at DEPTH
    always_ff @(posedge clk) begin
        if (in_valid) begin
            h_re[0] <= x_re;
            h_im[0] <= x_im;
            for (int i = 1; i < DEPTH; i++) begin
                h_re[i] <= h_re[i-1];
                h_im[i] <= h_im[i-1];
            end
        end
        if (!rst_n)                          n <= '0;
        else if (in_valid && int'(n) < DEPTH) n <= n + 1'b1;
    end

    // x(k) = x(n−16), x(old) = x(n−64), x(old+16) = x(n−48)
    logic signed [D-1:0] k_re, k_im, o_re, o_im, q_re, q_im;
    assign k_re = h_re[LAG-1];
    assign k_im = h_im[LAG-1];
    assign o_re = h_re[DEPTH-1];
    assign o_im = h_im[DEPTH-1];
    assign q_re = h_re[DEPTH-LAG-1];
    assign q_im = h_im[DEPTH-LAG-1];

    function automatic logic signed [PR_W-1:0] mul(input logic signed [D-1:0] a,
                                                   input logic signed [D-1:0] b);
        logic signed [PR_W-1:0] wa, wb;
        wa = PR_W'(a);
        wb = PR_W'(b);
        return wa * wb;
    endfunction

    // ---- stage 1: products, add/subtract/full-window flags
    logic signed [PR_W:0]  a_re1, a_im1, s_re1, s_im1;   // x(k)·conj(x(n)), x(old)·conj(x(old+16))
    logic [PR_W-1:0]       ak1, an1, so1, sq1;          // |x(k)|², |x(n)|², |x(old)|², |x(old+16)|²
    logic                  add1, sub1, full1, v1;
    always_ff @(posedge clk) begin
        a_re1 <= (PR_W+1)'(mul(k_re, x_re)) + (PR_W+1)'(mul(k_im, x_im));
        a_im1 <= (PR_W+1)'(mul(k_im, x_re)) - (PR_W+1)'(mul(k_re, x_im));
        s_re1 <= (PR_W+1)'(mul(o_re, q_re)) + (PR_W+1)'(mul(o_im, q_im));
        s_im1 <= (PR_W+1)'(mul(o_im, q_re)) - (PR_W+1)'(mul(o_re, q_im));
        ak1   <= PR_W'(mul(k_re, k_re) + mul(k_im, k_im));
        an1   <= PR_W'(mul(x_re, x_re) + mul(x_im, x_im));
        so1   <= PR_W'(mul(o_re, o_re) + mul(o_im, o_im));
        sq1   <= PR_W'(mul(q_re, q_re) + mul(q_im, q_im));
        add1  <= int'(n) >= LAG;
        sub1  <= int'(n) >= DEPTH;
        full1 <= int'(n) >= DEPTH - 1;
        if (!rst_n) v1 <= 1'b0;
        else        v1 <= in_valid;
    end

    // ---- stage 2: running sums
    logic [P_W-1:0]        p1, p2;
    logic                  full2, v2;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            c_re <= '0;
            c_im <= '0;
            p1   <= '0;
            p2   <= '0;
        end else if (v1 && add1) begin
            c_re <= c_re + C_W'(a_re1) - (sub1 ? C_W'(s_re1) : '0);
            c_im <= c_im + C_W'(a_im1) - (sub1 ? C_W'(s_im1) : '0);
            p1   <= p1 + P_W'(ak1) - (sub1 ? P_W'(so1) : '0);
            p2   <= p2 + P_W'(an1) - (sub1 ? P_W'(sq1) : '0);
        end
        full2 <= full1;
        if (!rst_n) v2 <= 1'b0;
        else        v2 <= v1;
    end
    assign c_valid = v2;

    // ---- stage 3: the two sides of the threshold
    logic [CMP_W-1:0] lhs3, rhs3;
    logic             ok3, v3;
    // Natural widths: squares 2·C_W signed, P1·P2 2·P_W unsigned, then the 15-bit constant;
    // both sides extended to CMP_W only for the shift and the compare.
    localparam int SQ_W = 2 * C_W;
    localparam int PP_W = 2 * P_W;
    logic signed [SQ_W-1:0] sq_re, sq_im, wc_re, wc_im;
    logic [PP_W-1:0]        pp, wp1, wp2;
    assign wc_re = SQ_W'(c_re);
    assign wc_im = SQ_W'(c_im);
    assign sq_re = wc_re * wc_re;
    assign sq_im = wc_im * wc_im;
    assign wp1   = PP_W'(p1);
    assign wp2   = PP_W'(p2);
    assign pp    = wp1 * wp2;
    always_ff @(posedge clk) begin
        lhs3 <= (CMP_W'(sq_re) + CMP_W'(sq_im)) << THETA_SHIFT;
        rhs3 <= CMP_W'(THETA_NUM) * CMP_W'(pp);
        ok3  <= full2 && p1 != '0 && p2 != '0;
        if (!rst_n) v3 <= 1'b0;
        else        v3 <= v2;
    end

    // ---- stage 4: compare and run counter
    logic          f4;
    logic [RW-1:0] count;
    assign f4 = ok3 && lhs3 >= rhs3;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            count   <= '0;
            fire    <= 1'b0;
            event_o <= 1'b0;
        end else if (v3) begin
            fire    <= f4;
            event_o <= f4 && int'(count) + 1 >= DET_RUN;
            if (clear || !f4 || int'(count) + 1 >= DET_RUN) count <= '0;
            else                                          count <= count + 1'b1;
        end
        if (!rst_n) out_valid <= 1'b0;
        else        out_valid <= v3;
    end
endmodule

`default_nettype wire
