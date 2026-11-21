// LTF search correlator of the receiver front: c = Σ_{k<128} z(i+k)·conj(t(k mod 64)), narrowed
// to CORR_BITS (half to even, saturating). Golden model: radio_phy::rx_front_fixed::ltf_corr
// (used by RxFrontFixed::push). Gate G3: make -C hw ltf_corr.
//
// Arithmetic: the template is T1 T2 (the same 64 taps twice) and the LTF is real in frequency,
// so its taps are Hermitian, t(64−k) = conj(t(k)) with t(0), t(32) real (IEEE 802.11-2020
// §17.3.3; rvec refuses to generate taps that are not). Two exact folds cut the 512 real
// products of the direct form to 128:
//   u(k) = z(i+k) + z(i+k+64),                       k < 64
//   u(k)·conj(t) + u(m)·t, with t = a + jb and m = 64 − k, is
//     re: a·(ur_k + ur_m) + b·(ui_k − ui_m)
//     im: a·(ui_k + ui_m) + b·(ur_m − ur_k)
// Every step is an exact integer sum, so the result equals the model's direct sum bit for bit.
//
// Folded in time (ADR 0008, decision 4): a sample comes once in R = 8 clocks at most, so the 128
// products are made by 16 multipliers over 8 clocks. A slot is one pair k = 1 … 31 (four
// products) or the two real taps t(0), t(32) together (slot 0); clock p of a sample takes slots
// 4p … 4p + 3, their window entries through 8-to-1 multiplexers and their taps from a table.
//
// Stream: one sample per in_valid, at most one in 8 clocks; the block never stalls (no ready).
// A sample taken sooner than 8 clocks after the one before abandons that one's correlation (no
// out_valid for it). out_valid marks the correlation of the window ending at a valid sample,
// from the 128th sample after reset on, registered LATENCY = 14 clocks after the clock edge
// that takes the sample. The window advances only on in_valid; the pipeline every clock.

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
    localparam int R     = 8;                 // clocks per sample
    localparam int SLOTS = 4;                 // slots per clock: 4 products each
    localparam int U_W   = Z_BITS + 1;        // z + z
    localparam int S_W   = Z_BITS + 2;        // u ± u
    localparam int P_W   = S_W + TAP_BITS;    // product
    localparam int Q_W   = P_W + 1;           // the two products of a slot's component
    localparam int G_W   = Q_W + 2;           // the four slots of a clock
    // Full width of the model's sum, Z + T + 8 bits (|c| ≤ 128·2^(Z−1)·2^(T−1)·2).
    localparam int FULL  = Z_BITS + TAP_BITS + 8;

    // ---- window: win[0] oldest, win[N-1] newest --------------------------------------------
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

    // ---- control: the clock p of the sample in hand ----------------------------------------
    logic [7:0] filled;   // valid samples since reset, saturating at N
    logic       full;     // the window of the sample in hand holds N samples
    logic       run;
    logic [2:0] ph;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            filled <= '0;
            full   <= 1'b0;
            run    <= 1'b0;
            ph     <= '0;
        end else if (in_valid) begin
            full <= filled >= 8'(N - 1);
            if (filled != 8'(N)) filled <= filled + 8'd1;
            run  <= 1'b1;
            ph   <= '0;
        end else if (run) begin
            ph <= ph + 3'd1;
            if (ph == 3'(R - 1)) run <= 1'b0;
        end
    end

    // ---- stage 1: the window entries of this clock's slots ---------------------------------
    // Slot k = 4·ph + s: entries k, k + 64 (their sum is u(k)) and m, m + 64 with m = 64 − k,
    // or m = 32 in slot 0 (u(0) and u(32), the real taps).
    logic signed [Z_BITS-1:0] a_re [SLOTS], a_im [SLOTS];   // z(k)
    logic signed [Z_BITS-1:0] b_re [SLOTS], b_im [SLOTS];   // z(k + 64)
    logic signed [Z_BITS-1:0] c_re [SLOTS], c_im [SLOTS];   // z(m)
    logic signed [Z_BITS-1:0] d_re [SLOTS], d_im [SLOTS];   // z(m + 64)
    logic [2:0] ph1;
    logic       v1, last1;

    for (genvar s = 0; s < SLOTS; s++) begin : g_sel
        always_ff @(posedge clk) begin
            for (int p = 0; p < R; p++) begin
                if (ph == 3'(p)) begin
                    automatic int k = SLOTS * p + s;
                    automatic int m = (k == 0) ? HALF / 2 : HALF - k;
                    a_re[s] <= win_re[k];
                    a_im[s] <= win_im[k];
                    b_re[s] <= win_re[k+HALF];
                    b_im[s] <= win_im[k+HALF];
                    c_re[s] <= win_re[m];
                    c_im[s] <= win_im[m];
                    d_re[s] <= win_re[m+HALF];
                    d_im[s] <= win_im[m+HALF];
                end
            end
        end
    end

    // ---- stage 2: T1 + T2 fold, Hermitian pair sums and differences, the slots' taps --------
    logic signed [S_W-1:0]      xr_a [SLOTS], xr_b [SLOTS];   // multiplicands of the real part
    logic signed [S_W-1:0]      xi_a [SLOTS], xi_b [SLOTS];   // … of the imaginary part
    logic signed [TAP_BITS-1:0] t_a [SLOTS], t_b [SLOTS];
    logic                       v2, first2, last2;

    for (genvar s = 0; s < SLOTS; s++) begin : g_fold
        logic signed [U_W-1:0] ur_k, ui_k, ur_m, ui_m;
        logic                  real_taps;
        logic [4:0]            k;
        always_comb begin
            ur_k = U_W'(a_re[s]) + U_W'(b_re[s]);
            ui_k = U_W'(a_im[s]) + U_W'(b_im[s]);
            ur_m = U_W'(c_re[s]) + U_W'(d_re[s]);
            ui_m = U_W'(c_im[s]) + U_W'(d_im[s]);
            k    = {ph1, 2'(s)};
            real_taps = k == 5'd0;
        end
        always_ff @(posedge clk) begin
            if (real_taps) begin
                // t(0)·u(0) + t(32)·u(32): both taps real.
                xr_a[s] <= S_W'(ur_k);
                xr_b[s] <= S_W'(ur_m);
                xi_a[s] <= S_W'(ui_k);
                xi_b[s] <= S_W'(ui_m);
                t_a[s]  <= ltf_tap_re(0);
                t_b[s]  <= ltf_tap_re(HALF / 2);
            end else begin
                xr_a[s] <= S_W'(ur_k) + S_W'(ur_m);   // × a
                xr_b[s] <= S_W'(ui_k) - S_W'(ui_m);   // × b
                xi_a[s] <= S_W'(ui_k) + S_W'(ui_m);   // × a
                xi_b[s] <= S_W'(ur_m) - S_W'(ur_k);   // × b
                t_a[s]  <= ltf_tap_re(int'(k));
                t_b[s]  <= ltf_tap_im(int'(k));
            end
        end
    end

    // ---- stage 3: the 16 products ----------------------------------------------------------
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

    // ---- stage 4: each slot's two products -------------------------------------------------
    logic signed [Q_W-1:0] q_re [SLOTS], q_im [SLOTS];
    logic                  v4, first4, last4;

    always_ff @(posedge clk) begin
        for (int s = 0; s < SLOTS; s++) begin
            q_re[s] <= Q_W'(pr_a[s]) + Q_W'(pr_b[s]);
            q_im[s] <= Q_W'(pi_a[s]) + Q_W'(pi_b[s]);
        end
    end

    // ---- stage 5: the clock's four slots ----------------------------------------------------
    logic signed [G_W-1:0] g_re, g_im;
    logic                  v5, first5, last5;

    always_ff @(posedge clk) begin
        g_re <= (G_W'(q_re[0]) + G_W'(q_re[1])) + (G_W'(q_re[2]) + G_W'(q_re[3]));
        g_im <= (G_W'(q_im[0]) + G_W'(q_im[1])) + (G_W'(q_im[2]) + G_W'(q_im[3]));
    end

    // ---- stage 6: the sample's sum over its 8 clocks ----------------------------------------
    logic signed [FULL-1:0] acc_re, acc_im;
    logic                   last6;

    always_ff @(posedge clk) begin
        if (v5) begin
            acc_re <= first5 ? FULL'(g_re) : acc_re + FULL'(g_re);
            acc_im <= first5 ? FULL'(g_im) : acc_im + FULL'(g_im);
        end
    end

    // ---- stage 7: keep the top CORR_BITS (half to even, saturating) -------------------------
    logic signed [CORR_BITS-1:0] n_re, n_im;

    round_sat #(.IN_W(FULL), .SHIFT(FULL - CORR_BITS), .OUT_W(CORR_BITS)) u_round_re (
        .in(acc_re), .out(n_re)
    );
    round_sat #(.IN_W(FULL), .SHIFT(FULL - CORR_BITS), .OUT_W(CORR_BITS)) u_round_im (
        .in(acc_im), .out(n_im)
    );

    always_ff @(posedge clk) begin
        if (last6) begin
            out_re <= n_re;
            out_im <= n_im;
        end
    end

    // ---- the marks that travel with the data -----------------------------------------------
    // v: a clock of a sample; first: its clock 0; last: its clock 7, of a full window. A sample
    // taken early restarts the phase: the one before never reaches its clock 7.
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            {v1, v2, v3, v4, v5}               <= '0;
            {first2, first3, first4, first5}   <= '0;
            {last1, last2, last3, last4, last5, last6, out_valid} <= '0;
            ph1 <= '0;
        end else begin
            ph1 <= ph;
            {v1, v2, v3, v4, v5}             <= {run, v1, v2, v3, v4};
            {first2, first3, first4, first5} <= {v1 && ph1 == 3'd0, first2, first3, first4};
            last1 <= run && ph == 3'(R - 1) && full;
            {last2, last3, last4, last5, last6, out_valid} <=
                {last1, last2, last3, last4, last5, last6};
        end
    end
endmodule

`default_nettype wire
