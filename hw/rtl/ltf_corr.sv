// LTF search correlator of the receiver front: c = Σ_{k<128} z(i+k)·conj(t(k mod 64)), narrowed
// to CORR_BITS (half to even, saturating). Golden model: radio_phy::rx_front_fixed::ltf_corr
// (used by RxFrontFixed::push). Gate G3: make -C hw ltf_corr.
//
// Area: the template is T1 T2 (the same 64 taps twice) and the LTF is real in frequency, so its
// taps are Hermitian, t(64−k) = conj(t(k)) with t(0), t(32) real (IEEE 802.11-2020 §17.3.3;
// rvec refuses to generate taps that are not). Two exact folds cut the 512 real products of
// the direct form to 128 constant multiplications:
//   u(k) = z(i+k) + z(i+k+64),                       k < 64
//   u(k)·conj(t) + u(m)·t, with t = a + jb and m = 64 − k, is
//     re: a·(ur_k + ur_m) + b·(ui_k − ui_m)
//     im: a·(ui_k + ui_m) + b·(ur_m − ur_k)
// Every step is an exact integer sum, so the result equals the model's direct sum bit for bit.
//
// Stream: one sample per in_valid; the block never stalls (no ready). out_valid marks the
// correlation of the window ending at a valid sample, from the 128th sample after reset on,
// registered 5 clocks after the clock edge that takes the sample. The window advances only on
// in_valid; the pipeline every clock.

`default_nettype none

module ltf_corr
    import rx_front_rom_pkg::*;
(
    input  wire logic                        clk,
    input  wire logic                        rst_n,     // synchronous, active low
    input  wire logic                        in_valid,
    input  wire logic signed [Z_BITS-1:0]    in_re,
    input  wire logic signed [Z_BITS-1:0]    in_im,
    output logic                             out_valid,
    output logic signed [CORR_BITS-1:0]      out_re,
    output logic signed [CORR_BITS-1:0]      out_im
);
    localparam int N     = 128;
    localparam int HALF  = 64;
    localparam int U_W   = Z_BITS + 1;        // z + z
    localparam int S_W   = Z_BITS + 2;        // u ± u
    localparam int P_W   = S_W + TAP_BITS;    // constant product
    localparam int TERMS = 64;                // per component: t(0), t(32), 2 × 31 pairs
    localparam int GROUP = 8;
    localparam int G_W   = P_W + 3;           // sum of 8 products
    // Full width of the model's sum, Z + T + 8 bits (|c| ≤ 128·2^(Z−1)·2^(T−1)·2).
    localparam int FULL  = Z_BITS + TAP_BITS + 8;

    // ---- window: win[0] oldest, win[N-1] newest --------------------------------------------
    logic signed [Z_BITS-1:0] win_re [N];
    logic signed [Z_BITS-1:0] win_im [N];
    logic [7:0]               filled;   // valid samples since reset, saturating at N
    logic                     v0;

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

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            filled <= '0;
            v0     <= 1'b0;
        end else begin
            v0 <= in_valid && filled >= 8'(N - 1);
            if (in_valid && filled != 8'(N)) filled <= filled + 8'd1;
        end
    end

    // ---- stage 1: T1 + T2 fold and Hermitian pair sums/differences ------------------------
    logic signed [U_W-1:0] ur [HALF];
    logic signed [U_W-1:0] ui [HALF];
    always_comb begin
        for (int k = 0; k < HALF; k++) begin
            ur[k] = U_W'(win_re[k]) + U_W'(win_re[k+HALF]);
            ui[k] = U_W'(win_im[k]) + U_W'(win_im[k+HALF]);
        end
    end

    // Per component, term index j: j = 0 → t(0); j = 1 → t(32); j = 2p, 2p+1 → pair p+1 (a, b).
    logic signed [S_W-1:0] x_re [TERMS];   // multiplicands of the real part
    logic signed [S_W-1:0] x_im [TERMS];   // multiplicands of the imaginary part
    logic                  v1;

    always_ff @(posedge clk) begin
        x_re[0] <= S_W'(ur[0]);
        x_im[0] <= S_W'(ui[0]);
        x_re[1] <= S_W'(ur[32]);
        x_im[1] <= S_W'(ui[32]);
        for (int k = 1; k < 32; k++) begin
            x_re[2*k]   <= S_W'(ur[k]) + S_W'(ur[HALF-k]);   // × a
            x_re[2*k+1] <= S_W'(ui[k]) - S_W'(ui[HALF-k]);   // × b
            x_im[2*k]   <= S_W'(ui[k]) + S_W'(ui[HALF-k]);   // × a
            x_im[2*k+1] <= S_W'(ur[HALF-k]) - S_W'(ur[k]);   // × b
        end
    end

    // ---- stage 2: constant products ------------------------------------------------------
    logic signed [P_W-1:0] p_re [TERMS];
    logic signed [P_W-1:0] p_im [TERMS];
    logic                  v2;

    for (genvar j = 0; j < TERMS; j++) begin : g_mul
        // Tap index and part for term j (see stage 1).
        localparam int K = (j == 0) ? 0 : (j == 1) ? 32 : j / 2;
        localparam logic signed [TAP_BITS-1:0] C =
            (j >= 2 && j % 2 == 1) ? ltf_tap_im(K) : ltf_tap_re(K);
        always_ff @(posedge clk) begin
            p_re[j] <= P_W'(x_re[j]) * P_W'(C);
            p_im[j] <= P_W'(x_im[j]) * P_W'(C);
        end
    end

    // ---- stage 3: partial sums of 8 --------------------------------------------------------
    logic signed [G_W-1:0] g_re [TERMS/GROUP];
    logic signed [G_W-1:0] g_im [TERMS/GROUP];
    logic                  v3;

    always_ff @(posedge clk) begin
        for (int g = 0; g < TERMS / GROUP; g++) begin
            logic signed [G_W-1:0] sr, si;
            sr = '0;
            si = '0;
            for (int t = 0; t < GROUP; t++) begin
                sr = sr + G_W'(p_re[g*GROUP+t]);
                si = si + G_W'(p_im[g*GROUP+t]);
            end
            g_re[g] <= sr;
            g_im[g] <= si;
        end
    end

    // ---- stage 4: total --------------------------------------------------------------------
    logic signed [FULL-1:0] c_re, c_im;
    logic                   v4;

    always_ff @(posedge clk) begin
        logic signed [FULL-1:0] sr, si;
        sr = '0;
        si = '0;
        for (int g = 0; g < TERMS / GROUP; g++) begin
            sr = sr + FULL'(g_re[g]);
            si = si + FULL'(g_im[g]);
        end
        c_re <= sr;
        c_im <= si;
    end

    // ---- stage 5: keep the top CORR_BITS (half to even, saturating) -------------------------
    logic signed [CORR_BITS-1:0] n_re, n_im;

    round_sat #(.IN_W(FULL), .SHIFT(FULL - CORR_BITS), .OUT_W(CORR_BITS)) u_round_re (
        .in(c_re), .out(n_re)
    );
    round_sat #(.IN_W(FULL), .SHIFT(FULL - CORR_BITS), .OUT_W(CORR_BITS)) u_round_im (
        .in(c_im), .out(n_im)
    );

    always_ff @(posedge clk) begin
        out_re <= n_re;
        out_im <= n_im;
    end

    // ---- valid pipeline --------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            {v1, v2, v3, v4, out_valid} <= '0;
        end else begin
            {v1, v2, v3, v4, out_valid} <= {v0, v1, v2, v3, v4};
        end
    end
endmodule

`default_nettype wire
