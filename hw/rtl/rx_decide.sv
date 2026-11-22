// The acquisition decision of the receiver front (ADR 0006, decision 1; ADR 0008, decision
// 4). Golden model: radio_phy::rx_front_fixed::RxFrontFixed::decide for every value
// (SampleTrace::t1 and sums, the FixedAcquisition of push), radio_phy::rx_decision for every
// cycle (its module text is this block's schedule). Gate G3: make -C hw decide (the chain of
// hw/tb/decide_chain.sv, the loop closed).
//
// A decision starts with rx_search's acc_valid (cycle 0 below; w = W) and reads two memories
// that go on being written: the search's energy ring and the history kept here (y(n) at
// address n mod HISTORY, written in the cycle y_valid shows it).
//   0 … w        the grid: the ring at imax … imax + w, oldest first. T1 is the last position
//                whose energy keeps 7/8 of the maximum's; registered in w + 2.
//   c = w + 3    the final coarse sum: Σ v(T1 − 160 + i)·conj(v(T1 − 144 + i)), i < COARSE_LEN,
//                v the history narrowed to EST_BITS (half to even), a product a cycle, both
//                read ports; the sum registered and the CORDIC asked in c + 99.
//   f = c + 96   the fine sum on the same multiplier, port b alone: v(T1 − 28 + i) in
//                f + 2i, v(T1 + 36 + i) in the next, i < FINE_LEN; the CORDIC asked in f + 179.
//   g + 20       (g: the cycle the coarse request is granted) the rotation: y(T1 + m),
//                m = 0 … 126 + w, on port a through a second NCO (phase 0 at m = 0, the step
//                the coarse angle's) into ltf_corr1, one sample a cycle; after each sample
//                that ends a correlated window (m = 63 … 62 + w, 127 … 126 + w) three cycles
//                of wait. The first symbol's w correlations wait for the second's; each pair
//                gives |c1|² + |c2|² to the confirmation and |c1 + c2|² to the coherence; the
//                first 128 rotated samples give the energy.
//   the rules    energy > 0, 2·confirmation ≥ E_L·energy, 2·coherence ≥ E_L·energy.
//   the CFO      coarse = −(coarse angle << 2); the fine angle is the lag-64 step itself;
//                residual = the low 16 bits of (−fine angle − coarse), signed; the word
//                handed on is coarse + residual (no coarse weight: ADR 0007).
//   res_valid    the result registered: 8·w + 268 cycles after acc_valid when the CORDIC is
//                free at both requests, one more for each cycle a latch makes the coarse
//                request wait.
// No decision is made (res_valid at once, no sums) when T1 is less than 160 samples into the
// stream: acc_lapped = 0 and T1 < 160 (the model's checked_sub).
//
// The result is given at sample n + L of the detector's timeline, n = acc_n: dec_valid in
// cycle Y(n + L) + 3 (the cycle det_a takes its clear), `acquired` with it when the rules
// held, acq_t1 (T1 modulo HISTORY; the FFT window starts GI_LEN − W before it) and the three
// phase words. CHECK_TIMING = 1 stops simulation when the result is not registered by then.
//
// The CORDIC is the front's one, behind cordic_arb (requester b): cq_req with cq_re/cq_im is
// held until cq_grant; the coarse angle comes back before the fine request is made.
// A rejected candidate runs its whole schedule (the fine sum is made beside the rotation).

`default_nettype none

module rx_decide
    import rx_front_rom_pkg::*;
#(
    parameter int W            = SEARCH_W,           // GI − B, 1 … GI_LEN
    parameter int L            = DECISION_LATENCY,
    parameter bit CHECK_TIMING = 1'b1,
    localparam int E_W         = 2 * CORR_BITS + 4,                  // the search's energies
    localparam int POS_W       = $clog2(HISTORY),
    localparam int RING        = LTF_SYMBOL_LEN + GI_LEN + 1,
    localparam int RA_W        = $clog2(RING),
    localparam int P           = NCO_PHASE_BITS,
    localparam int S_W         = CORDIC_IN_BITS,                     // the coarse and fine sums
    localparam int C_W         = Z_BITS + TAP_BITS + 7,              // ltf_corr1's output
    localparam int M_W         = 2 * C_W + 8,                        // confirmation, coherence
    localparam int N_W         = 2 * Z_BITS + 7                      // energy of 128 samples
) (
    input  wire logic                     clk,
    input  wire logic                     rst_n,     // synchronous, active low
    // the history's write: the DC blocker's output, every sample
    input  wire logic                     y_valid,
    input  wire logic signed [Y_BITS-1:0] y_re,
    input  wire logic signed [Y_BITS-1:0] y_im,
    // the accepted candidate and the energy ring (rx_search)
    input  wire logic                     acc_valid,
    input  wire logic [POS_W-1:0]         acc_n,
    input  wire logic [POS_W-1:0]         acc_imax,
    input  wire logic                     acc_lapped,
    input  wire logic [RA_W-1:0]          acc_ring,
    input  wire logic [E_W-1:0]           acc_emax,
    output logic                          ring_re,
    output logic [RA_W-1:0]               ring_addr,
    input  wire logic [E_W-1:0]           ring_data,
    // the CORDIC (cordic_arb, requester b)
    output logic                          cq_req,
    output logic signed [S_W-1:0]         cq_re,
    output logic signed [S_W-1:0]         cq_im,
    input  wire logic                     cq_grant,
    input  wire logic                     cq_valid,
    input  wire logic signed [ANGLE_BITS-1:0] cq_angle,
    // the result, at sample acc_n + L
    output logic                          dec_valid,
    output logic                          acquired,
    output logic [POS_W-1:0]              acq_t1,
    output logic signed [P-1:0]           acq_cfo,
    output logic signed [P-1:0]           acq_coarse,
    output logic signed [P-1:0]           acq_fine,
    // the result as registered, and the sums it was taken from (the gate's probes)
    output logic                          res_valid,
    output logic                          res_sums,
    output logic signed [S_W-1:0]         res_coarse_re,
    output logic signed [S_W-1:0]         res_coarse_im,
    output logic [M_W-1:0]                res_confirmation,
    output logic [M_W-1:0]                res_coherence,
    output logic [N_W-1:0]                res_energy,
    output logic signed [S_W-1:0]         res_fine_re,
    output logic signed [S_W-1:0]         res_fine_im
);
    localparam int E      = EST_BITS;
    localparam int A      = ANGLE_BITS;
    localparam int PR_W   = 2 * E + 1;                       // a product's component
    localparam int T1_MIN = LTF_SYMBOL_LEN + COARSE_LEN;     // 160: the coarse window's reach
    localparam int ROT    = 2 * LTF_SYMBOL_LEN + W - 1;      // rotated samples: 127 + w
    localparam int FREADS = 2 * FINE_LEN;
    localparam int GW     = $clog2(GI_LEN + 2);
    localparam int MW     = $clog2(ROT + 1);
    localparam int OW     = (W > 1) ? $clog2(W) : 1;      // a first-symbol correlation's index

    if (W < 1 || W > GI_LEN) begin : g_bad_w
        $error("rx_decide: W must be 1 … GI_LEN");
    end
    if (P != A + 6 || STF_PERIOD != 16 || LTF_SYMBOL_LEN != 64) begin : g_bad_step
        $error("rx_decide: the steps are exact shifts only for P = A + 6, lags 16 and 64");
    end
    if (T1_MIN + GI_LEN + L + 2 * LTF_SYMBOL_LEN + 4 * GI_LEN >= HISTORY) begin : g_bad_hist
        $error("rx_decide: the history does not reach the coarse window");
    end

    // ---- the history
    logic [POS_W-1:0]      hw;                    // the sample at the write port
    logic                  re_a, re_b;
    logic [POS_W-1:0]      ad_a, ad_b;
    logic [2*Y_BITS-1:0]   rd_a, rd_b;
    always_ff @(posedge clk) begin
        if (!rst_n)       hw <= '0;
        else if (y_valid) hw <= hw + 1'b1;
    end
    ram_1w2r #(.W(2 * Y_BITS), .DEPTH(HISTORY)) u_hist (
        .clk,
        .we(y_valid), .waddr(hw), .wdata({y_re, y_im}),
        .re_a, .raddr_a(ad_a), .rdata_a(rd_a),
        .re_b, .raddr_b(ad_b), .rdata_b(rd_b)
    );

    // ---- the candidate, as registered at the strobe
    logic [POS_W-1:0] imax, due;
    logic             lapped, busy;
    logic [RA_W-1:0]  ring0;
    logic [E_W-1:0]   keep;                       // 7/8 of the maximum

    // ---- the grid
    logic [GW-1:0]    gi;                         // reads issued after the strobe's own
    logic             g_run, gd_v, g_done;
    logic [GW-1:0]    gd_k;                       // offset of the word on ring_data
    logic [POS_W:0]   t1x;                        // T1; the top bit: 2^POS_W or more
    logic [POS_W-1:0] t1;
    logic             t1_far;
    logic [RA_W:0]    g_sum;
    logic             early;                      // T1 less than 160 samples into the stream
    assign t1      = t1x[POS_W-1:0];
    assign early   = !t1_far && !t1x[POS_W] && int'(t1) < T1_MIN;
    assign ring_re = acc_valid || g_run;
    assign g_sum   = (RA_W+1)'(ring0) + (RA_W+1)'(gi);
    assign ring_addr = acc_valid ? acc_ring
                     : (int'(g_sum) >= RING ? RA_W'(int'(g_sum) - RING) : g_sum[RA_W-1:0]);

    // ---- the reads of the sums and of the rotation
    logic             co_run, fi_run, ro_run;
    logic [6:0]       ci;                         // coarse product
    logic [7:0]       fi;                         // fine read
    logic [MW-1:0]    rm;                         // rotated sample to read
    logic [1:0]       rwait;
    logic             ro_read;

    function automatic logic correlated(input logic [MW-1:0] m);
        // The windows ending at m = 63 … 62 + w and 64 later.
        int k;
        k = int'(m) % LTF_SYMBOL_LEN;
        return int'(m) >= LTF_SYMBOL_LEN - 1 && (k == LTF_SYMBOL_LEN - 1 || k + 1 < W);
    endfunction

    assign ro_read = ro_run && rwait == '0;
    assign re_a    = co_run || ro_read;
    assign re_b    = co_run || fi_run;
    always_comb begin
        if (co_run) ad_a = t1 - POS_W'(T1_MIN) + POS_W'(ci);
        else        ad_a = t1 + POS_W'(rm);
        if (co_run)      ad_b = t1 - POS_W'(T1_MIN - STF_PERIOD) + POS_W'(ci);
        else if (!fi[0]) ad_b = t1 - POS_W'(GI2_LEN - FINE_GUARD) + POS_W'(fi[7:1]);
        else             ad_b = t1 + POS_W'(LTF_SYMBOL_LEN - GI2_LEN + FINE_GUARD) + POS_W'(fi[7:1]);
    end

    // ---- the multiplier and the two sums
    logic                  d_co, d_f0, d_f1, d_last;      // what the memory shows
    logic                  n_v, n_fine, n_last;
    logic                  p_v, p_fine, p_last;
    logic signed [E-1:0]   na_re, na_im, nb_re, nb_im;
    logic signed [E-1:0]   ea_re, ea_im, eb_re, eb_im;
    logic signed [PR_W-1:0] pr_re, pr_im;
    logic signed [S_W-1:0] cc_re, cc_im, cf_re, cf_im;
    logic                  rq_fine, got_coarse, got_fine;

    round_sat #(.IN_W(Y_BITS), .SHIFT(Y_BITS - E), .OUT_W(E)) u_ea_re (
        .in($signed(rd_a[2*Y_BITS-1:Y_BITS])), .out(ea_re));
    round_sat #(.IN_W(Y_BITS), .SHIFT(Y_BITS - E), .OUT_W(E)) u_ea_im (
        .in($signed(rd_a[Y_BITS-1:0])), .out(ea_im));
    round_sat #(.IN_W(Y_BITS), .SHIFT(Y_BITS - E), .OUT_W(E)) u_eb_re (
        .in($signed(rd_b[2*Y_BITS-1:Y_BITS])), .out(eb_re));
    round_sat #(.IN_W(Y_BITS), .SHIFT(Y_BITS - E), .OUT_W(E)) u_eb_im (
        .in($signed(rd_b[Y_BITS-1:0])), .out(eb_im));

    assign cq_re = rq_fine ? cf_re : cc_re;
    assign cq_im = rq_fine ? cf_im : cc_im;

    // ---- the rotation: the second NCO, the one-symbol correlator
    logic signed [P-1:0]      step, coarse, fine_w;
    logic                     ro_dv;                       // a rotation word on rd_a
    logic                     z_v, c_v;
    logic signed [Z_BITS-1:0] z_re, z_im;
    logic signed [C_W-1:0]    c_re, c_im;
    logic [MW-1:0]            mz;                          // rotated sample at the NCO's output
    logic [GW:0]              oc;                          // correlation at ltf_corr1's output

    nco u_nco (
        .clk, .rst_n(rst_n && !acc_valid),
        .in_valid(ro_dv), .in_re($signed(rd_a[2*Y_BITS-1:Y_BITS])), .in_im($signed(rd_a[Y_BITS-1:0])),
        .in_step(step),
        .out_valid(z_v), .out_re(z_re), .out_im(z_im)
    );
    ltf_corr1 u_corr (
        .clk, .rst_n,
        .in_valid(z_v), .in_corr(correlated(mz)), .in_re(z_re), .in_im(z_im),
        .out_valid(c_v), .out_re(c_re), .out_im(c_im)
    );

    // ---- the tail: squares, sums, rules
    logic signed [C_W-1:0]  c1_re [W], c1_im [W];
    logic signed [C_W-1:0]  ta_re, ta_im, tb_re, tb_im;
    logic signed [C_W:0]    ts_re, ts_im;
    logic [2*C_W-1:0]       qa_re, qa_im, qb_re, qb_im;
    logic [2*C_W+1:0]       qs_re, qs_im;
    logic [2*C_W:0]         m_a, m_b;
    logic [2*C_W+2:0]       m_s;
    logic                   t1_v, t2_v, t3_v, t1_last, t2_last, t3_last, t4_last, t5_last;
    logic [M_W-1:0]         conf, coh;
    logic [N_W-1:0]         en;
    logic [N_W+19:0]        rhs;
    logic                   ok_conf, ok_coh, en_pos, tail_rdy, fine_rdy, res_done, res_acq;
    logic signed [15:0]     resid;
    logic                   fa_v;                         // the residual is registered

    // ---- the result's sample: Y(n + L), three registers to det_a's clear
    logic hit, q1, q2;
    assign hit      = y_valid && busy && hw == due;
    assign acquired = dec_valid && res_acq;
    assign acq_t1     = t1;
    assign acq_cfo    = fine_w;
    assign acq_coarse = coarse;
    assign acq_fine   = fine_w;
    assign res_coarse_re    = cc_re;
    assign res_coarse_im    = cc_im;
    assign res_fine_re      = cf_re;
    assign res_fine_im      = cf_im;
    assign res_confirmation = conf;
    assign res_coherence    = coh;
    assign res_energy       = en;

    always_ff @(posedge clk) begin
        // -- the memories' outputs into the multiplier
        if (d_co) begin
            na_re <= ea_re;
            na_im <= ea_im;
        end else if (d_f0) begin
            na_re <= eb_re;
            na_im <= eb_im;
        end
        if (d_co || d_f1) begin
            nb_re <= eb_re;
            nb_im <= eb_im;
        end
        // v0·conj(v1)
        pr_re <= PR_W'(na_re) * PR_W'(nb_re) + PR_W'(na_im) * PR_W'(nb_im);
        pr_im <= PR_W'(na_im) * PR_W'(nb_re) - PR_W'(na_re) * PR_W'(nb_im);
        n_fine <= d_f1;
        p_fine <= n_fine;
        // -- the tail's registers
        if (c_v && int'(oc) < W) begin
            c1_re[oc[OW-1:0]] <= c_re;
            c1_im[oc[OW-1:0]] <= c_im;
        end
        if (c_v && int'(oc) >= W) begin
            ta_re <= c1_re[OW'(int'(oc) - W)];
            ta_im <= c1_im[OW'(int'(oc) - W)];
            tb_re <= c_re;
            tb_im <= c_im;
            ts_re <= (C_W+1)'(c1_re[OW'(int'(oc) - W)]) + (C_W+1)'(c_re);
            ts_im <= (C_W+1)'(c1_im[OW'(int'(oc) - W)]) + (C_W+1)'(c_im);
        end
        qa_re <= unsigned'((2*C_W)'(ta_re) * (2*C_W)'(ta_re));
        qa_im <= unsigned'((2*C_W)'(ta_im) * (2*C_W)'(ta_im));
        qb_re <= unsigned'((2*C_W)'(tb_re) * (2*C_W)'(tb_re));
        qb_im <= unsigned'((2*C_W)'(tb_im) * (2*C_W)'(tb_im));
        qs_re <= unsigned'((2*C_W+2)'(ts_re) * (2*C_W+2)'(ts_re));
        qs_im <= unsigned'((2*C_W+2)'(ts_im) * (2*C_W+2)'(ts_im));
        m_a <= (2*C_W+1)'(qa_re) + (2*C_W+1)'(qa_im);
        m_b <= (2*C_W+1)'(qb_re) + (2*C_W+1)'(qb_im);
        m_s <= (2*C_W+3)'(qs_re) + (2*C_W+3)'(qs_im);
        rhs <= (N_W+20)'(en) * (N_W+20)'(LTF_TAP_ENERGY);
        ok_conf <= {2'b0, conf, 1'b0} >= (M_W+3)'(rhs);
        ok_coh  <= {2'b0, coh, 1'b0} >= (M_W+3)'(rhs);
        en_pos  <= en != '0;

        if (!rst_n) begin
            busy <= 1'b0;
            {g_run, gd_v, g_done, co_run, fi_run, ro_run} <= '0;
            {d_co, d_f0, d_f1, d_last, n_v, n_last, p_v, p_last, ro_dv} <= '0;
            {t1_v, t2_v, t3_v, t1_last, t2_last, t3_last, t4_last, t5_last} <= '0;
            {cq_req, rq_fine, got_coarse, got_fine, fa_v} <= '0;
            {tail_rdy, fine_rdy, res_done, res_valid, res_sums, res_acq} <= '0;
            {q1, q2, dec_valid} <= '0;
        end else begin
            // -- the strobe: a decision begins
            if (acc_valid) begin
                busy   <= 1'b1;
                imax   <= acc_imax;
                lapped <= acc_lapped;
                due    <= acc_n + POS_W'(L);
                ring0  <= acc_ring;
                keep   <= acc_emax - (acc_emax >> 3);
                gi     <= GW'(1);
                g_run  <= 1'b1;
                gd_k   <= '0;
                cc_re  <= '0;
                cc_im  <= '0;
                cf_re  <= '0;
                cf_im  <= '0;
                conf   <= '0;
                coh    <= '0;
                en     <= '0;
                mz     <= '0;
                oc     <= '0;
                {co_run, fi_run, ro_run, cq_req, rq_fine, got_coarse, got_fine, fa_v} <= '0;
                {tail_rdy, fine_rdy, res_done, res_sums, res_acq} <= '0;
            end else if (g_run) begin
                gi <= gi + 1'b1;
                if (int'(gi) == W) g_run <= 1'b0;
            end

            // -- the grid's words: the last that keeps 7/8 of the maximum
            gd_v   <= ring_re;
            g_done <= gd_v && int'(gd_k) == W;
            if (gd_v) begin
                gd_k <= gd_k + 1'b1;
                if (ring_data >= keep) begin
                    t1x    <= (POS_W+1)'(imax) + (POS_W+1)'(gd_k);
                    t1_far <= lapped;
                end
            end
            // T1 is registered: the coarse sum's reads, or no decision
            if (g_done) begin
                if (early) begin
                    res_done  <= 1'b1;
                end else begin
                    co_run <= 1'b1;
                    ci     <= '0;
                end
            end
            res_valid <= (g_done && early) || (tail_rdy && fine_rdy && !res_done);

            // -- the reads of the two sums
            if (co_run) begin
                ci <= ci + 1'b1;
                if (int'(ci) == COARSE_LEN - 1) begin
                    co_run <= 1'b0;
                    fi_run <= 1'b1;
                    fi     <= '0;
                end
            end
            if (fi_run) begin
                fi <= fi + 1'b1;
                if (int'(fi) == FREADS - 1) fi_run <= 1'b0;
            end
            d_co   <= co_run;
            d_f0   <= fi_run && !fi[0];
            d_f1   <= fi_run && fi[0];
            d_last <= (co_run && int'(ci) == COARSE_LEN - 1) || (fi_run && int'(fi) == FREADS - 1);
            n_v    <= d_co || d_f1;
            n_last <= d_last;
            p_v    <= n_v;
            p_last <= n_last;
            if (p_v && !acc_valid) begin
                if (p_fine) begin
                    cf_re <= cf_re + S_W'(pr_re);
                    cf_im <= cf_im + S_W'(pr_im);
                end else begin
                    cc_re <= cc_re + S_W'(pr_re);
                    cc_im <= cc_im + S_W'(pr_im);
                end
            end

            // -- the CORDIC: the coarse sum, then the fine one
            if (p_v && p_last && !acc_valid) begin
                cq_req  <= 1'b1;
                rq_fine <= p_fine;
            end else if (cq_req && cq_grant) begin
                cq_req <= 1'b0;
            end
            if (cq_valid && busy && !acc_valid) begin
                if (!got_coarse) begin
                    got_coarse <= 1'b1;
                    step       <= P'(cq_angle) <<< 2;
                    coarse     <= -(P'(cq_angle) <<< 2);
                    ro_run     <= 1'b1;
                    rm         <= '0;
                    rwait      <= '0;
                end else if (!got_fine) begin
                    got_fine <= 1'b1;
                    resid    <= 16'(-P'(cq_angle) - coarse);
                    fa_v     <= 1'b1;
                end
            end
            if (fa_v && !acc_valid) begin
                fine_w   <= coarse + P'(resid);
                fine_rdy <= 1'b1;
                fa_v     <= 1'b0;
            end

            // -- the rotation's reads
            if (ro_run && !acc_valid) begin
                if (rwait != '0) begin
                    rwait <= rwait - 1'b1;
                end else begin
                    rm <= rm + 1'b1;
                    if (correlated(rm)) rwait <= 2'd3;
                    if (int'(rm) == ROT - 1) ro_run <= 1'b0;
                end
            end
            ro_dv <= ro_read && !acc_valid;
            if (z_v && !acc_valid) begin
                mz <= mz + 1'b1;
                if (int'(mz) < 2 * LTF_SYMBOL_LEN)
                    en <= en + N_W'(unsigned'((2*Z_BITS)'(z_re) * (2*Z_BITS)'(z_re)))
                             + N_W'(unsigned'((2*Z_BITS)'(z_im) * (2*Z_BITS)'(z_im)));
            end

            // -- the tail
            if (c_v && !acc_valid) oc <= oc + 1'b1;
            t1_v    <= c_v && int'(oc) >= W && !acc_valid;
            t1_last <= c_v && int'(oc) == 2 * W - 1 && !acc_valid;
            t2_v    <= t1_v;
            t2_last <= t1_last;
            t3_v    <= t2_v;
            t3_last <= t2_last;
            t4_last <= t3_last;
            t5_last <= t4_last;
            if (t3_v && !acc_valid) begin
                conf <= conf + M_W'(m_a) + M_W'(m_b);
                coh  <= coh + M_W'(m_s);
            end
            if (t5_last && !acc_valid) tail_rdy <= 1'b1;
            if (tail_rdy && fine_rdy && !res_done && !acc_valid) begin
                res_done <= 1'b1;
                res_sums <= 1'b1;
                res_acq  <= en_pos && ok_conf && ok_coh;
            end

            // -- the result's sample
            q1        <= hit;
            q2        <= q1;
            dec_valid <= q2;
            if (dec_valid) busy <= 1'b0;
        end
    end

    // ---- simulation checks (procedural $error: not for synthesis, see ram_1w1r)
`ifndef SYNTHESIS
    if (CHECK_TIMING) begin : g_check
        always_ff @(posedge clk) if (rst_n) begin
            if (dec_valid && !res_done)
                $error("rx_decide: the result is not registered at its sample (the schedule does not fit)");
            if (acc_valid && busy)
                $error("rx_decide: an acceptance with a decision in flight");
            if (cq_valid && busy && got_coarse && got_fine)
                $error("rx_decide: an angle nobody asked for");
        end
    end
`endif
endmodule

`default_nettype wire
