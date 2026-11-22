// One-symbol LTF correlator of the decision engine: c = Σ_{k<64} x(k)·conj(t(k)) over the 64
// samples ending at a marked one, the full sum (no narrowing). Golden model: the `xc` of
// radio_phy::rx_front_fixed::RxFrontFixed::decide (its confirmation and coherence sums are
// made of these). Gate G3: make -C hw decide (through rx_decide's sums).
//
// Arithmetic: ltf_corr's Hermitian fold without its T1 + T2 sum. t(64 − k) = conj(t(k)) with
// t(0), t(32) real (rvec refuses taps that are not), so for a pair k, m = 64 − k and t = a + jb
//   re: a·(xr_k + xr_m) + b·(xi_k − xi_m)
//   im: a·(xi_k + xi_m) + b·(xr_m − xr_k)
// and slot 0 is the two real taps. Every step is an exact integer sum.
//
// Folded in time (ADR 0008, decision 4: F = 4): 32 slots of four real products on 32
// multipliers over 4 clocks, eight slots a clock.
//
// Stream: every in_valid shifts the sample into the 64-entry window; in_corr with it asks for
// the correlation of the window ending at that sample. A correlation takes the window for the
// four clocks after its sample: the next in_valid must not come sooner than 4 clocks after a
// marked one (the decision's schedule waits F − 1 = 3). out_valid marks out_re/out_im
// LATENCY = 10 clock edges after the edge that takes the marked sample (11 from cycle to
// cycle: radio_phy::rx_decision's F + 7). The window is not reset: the caller marks a sample
// only after 64 of its own.

`default_nettype none

module ltf_corr1
    import rx_front_rom_pkg::*;
#(
    localparam int FULL = Z_BITS + TAP_BITS + 7      // |c| < 64·2^(Z−1)·2^(T−1)·2
) (
    input  wire logic                   clk,
    input  wire logic                   rst_n,     // synchronous, active low
    input  wire logic                   in_valid,
    input  wire logic                   in_corr,
    input  wire logic signed [Z_BITS-1:0] in_re,
    input  wire logic signed [Z_BITS-1:0] in_im,
    output logic                        out_valid,
    output logic signed [FULL-1:0]      out_re,
    output logic signed [FULL-1:0]      out_im
);
    localparam int N     = 64;
    localparam int F     = 4;                 // clocks of products
    localparam int SLOTS = 8;                 // slots per clock
    localparam int S_W   = Z_BITS + 1;        // x ± x
    localparam int P_W   = S_W + TAP_BITS;    // product
    localparam int Q_W   = P_W + 1;           // the two products of a slot's component
    localparam int G_W   = Q_W + 3;           // the eight slots of a clock

    // ---- window: win[0] oldest, win[N-1] newest
    logic signed [Z_BITS-1:0] win_re [N];
    logic signed [Z_BITS-1:0] win_im [N];
    always_ff @(posedge clk) begin
        if (in_valid) begin
            for (int k = 0; k < N - 1; k++) begin
                win_re[k] <= win_re[k+1];
                win_im[k] <= win_im[k+1];
            end
            win_re[N-1] <= in_re;
            win_im[N-1] <= in_im;
        end
    end

    // ---- control: the clock p of the correlation in hand
    logic       run;
    logic [1:0] ph;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            run <= 1'b0;
            ph  <= '0;
        end else if (in_valid && in_corr) begin
            run <= 1'b1;
            ph  <= '0;
        end else if (run) begin
            ph <= ph + 2'd1;
            if (ph == 2'(F - 1)) run <= 1'b0;
        end
    end

    // ---- stage 1: the window entries of this clock's slots. Slot k = 8·ph + s: entries k and
    // m = 64 − k, or 0 and 32 in slot 0.
    logic signed [Z_BITS-1:0] a_re [SLOTS], a_im [SLOTS];   // x(k)
    logic signed [Z_BITS-1:0] c_re [SLOTS], c_im [SLOTS];   // x(m)
    logic [1:0] ph1;
    logic       v1, last1;
    for (genvar s = 0; s < SLOTS; s++) begin : g_sel
        always_ff @(posedge clk) begin
            for (int p = 0; p < F; p++) begin
                if (ph == 2'(p)) begin
                    automatic logic [5:0] k = 6'(SLOTS * p + s);
                    automatic logic [5:0] m = (k == '0) ? 6'(N / 2) : 6'(N - SLOTS * p - s);
                    a_re[s] <= win_re[k];
                    a_im[s] <= win_im[k];
                    c_re[s] <= win_re[m];
                    c_im[s] <= win_im[m];
                end
            end
        end
    end

    // ---- stage 2: Hermitian pair sums and differences, the slots' taps
    logic signed [S_W-1:0]      xr_a [SLOTS], xr_b [SLOTS];
    logic signed [S_W-1:0]      xi_a [SLOTS], xi_b [SLOTS];
    logic signed [TAP_BITS-1:0] t_a [SLOTS], t_b [SLOTS];
    logic                       v2, first2, last2;
    for (genvar s = 0; s < SLOTS; s++) begin : g_fold
        logic [4:0] k;
        assign k = {ph1, 3'(s)};
        always_ff @(posedge clk) begin
            if (k == 5'd0) begin
                // t(0)·x(0) + t(32)·x(32): both taps real.
                xr_a[s] <= S_W'(a_re[s]);
                xr_b[s] <= S_W'(c_re[s]);
                xi_a[s] <= S_W'(a_im[s]);
                xi_b[s] <= S_W'(c_im[s]);
                t_a[s]  <= ltf_tap_re(0);
                t_b[s]  <= ltf_tap_re(N / 2);
            end else begin
                xr_a[s] <= S_W'(a_re[s]) + S_W'(c_re[s]);   // × a
                xr_b[s] <= S_W'(a_im[s]) - S_W'(c_im[s]);   // × b
                xi_a[s] <= S_W'(a_im[s]) + S_W'(c_im[s]);   // × a
                xi_b[s] <= S_W'(c_re[s]) - S_W'(a_re[s]);   // × b
                t_a[s]  <= ltf_tap_re(int'(k));
                t_b[s]  <= ltf_tap_im(int'(k));
            end
        end
    end

    // ---- stage 3: the 32 products
    logic signed [P_W-1:0] pr_a [SLOTS], pr_b [SLOTS];
    logic signed [P_W-1:0] pi_a [SLOTS], pi_b [SLOTS];
    logic                  v3, first3, last3;
    always_ff @(posedge clk) begin
        for (int s = 0; s < SLOTS; s++) begin
            pr_a[s] <= P_W'(xr_a[s]) * P_W'(t_a[s]);
            pr_b[s] <= P_W'(xr_b[s]) * P_W'(t_b[s]);
            pi_a[s] <= P_W'(xi_a[s]) * P_W'(t_a[s]);
            pi_b[s] <= P_W'(xi_b[s]) * P_W'(t_b[s]);
        end
    end

    // ---- stage 4: each slot's two products
    logic signed [Q_W-1:0] q_re [SLOTS], q_im [SLOTS];
    logic                  v4, first4, last4;
    always_ff @(posedge clk) begin
        for (int s = 0; s < SLOTS; s++) begin
            q_re[s] <= Q_W'(pr_a[s]) + Q_W'(pr_b[s]);
            q_im[s] <= Q_W'(pi_a[s]) + Q_W'(pi_b[s]);
        end
    end

    // ---- stage 5: the clock's eight slots
    logic signed [G_W-1:0] g_re, g_im;
    logic                  v5, first5, last5;
    always_ff @(posedge clk) begin
        g_re <= ((G_W'(q_re[0]) + G_W'(q_re[1])) + (G_W'(q_re[2]) + G_W'(q_re[3])))
              + ((G_W'(q_re[4]) + G_W'(q_re[5])) + (G_W'(q_re[6]) + G_W'(q_re[7])));
        g_im <= ((G_W'(q_im[0]) + G_W'(q_im[1])) + (G_W'(q_im[2]) + G_W'(q_im[3])))
              + ((G_W'(q_im[4]) + G_W'(q_im[5])) + (G_W'(q_im[6]) + G_W'(q_im[7])));
    end

    // ---- stage 6: the correlation's sum over its 4 clocks; stage 7: the output register
    logic signed [FULL-1:0] acc_re, acc_im;
    logic                   last6;
    always_ff @(posedge clk) begin
        if (v5) begin
            acc_re <= first5 ? FULL'(g_re) : acc_re + FULL'(g_re);
            acc_im <= first5 ? FULL'(g_im) : acc_im + FULL'(g_im);
        end
        if (last6) begin
            out_re <= acc_re;
            out_im <= acc_im;
        end
    end

    // ---- the valid chain beside the stages
    always_ff @(posedge clk) begin
        ph1 <= ph;
        if (!rst_n) begin
            {v1, v2, v3, v4, v5} <= '0;
            {last1, last2, last3, last4, last5, last6, out_valid} <= '0;
        end else begin
            {v1, v2, v3, v4, v5} <= {run, v1, v2, v3, v4};
            {last1, last2, last3, last4, last5, last6, out_valid} <=
                {run && ph == 2'(F - 1), last1, last2, last3, last4, last5 && v5, last6};
        end
        first2 <= v1 && ph1 == '0;
        first3 <= first2;
        first4 <= first3;
        first5 <= first4;
    end
endmodule

`default_nettype wire
