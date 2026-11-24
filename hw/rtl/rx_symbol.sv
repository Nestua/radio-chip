// The symbol block (ADR 0010, decisions 1 and 6.2; R0-09's data path): per OFDM symbol, the
// products u = Z·Hs* after the packet's shift, the pilots for the tracker, the common phase φ
// from the pilots rotated by the tracker's slope θ, and the soft pass — each data subcarrier
// rotated by −(φ + c·θ), demapped with thresholds scaled by its gain, scaled by the packet's
// quotient, clipped. Golden model: radio_phy::rx_ofdm_fixed::{symbol_pilots,
// symbol_from_bins} (symbol_soft_fixed after its FFT and its tracker step). Gate G3:
// make -C hw rx_symbol. Architecture and widths: docs/gates/rx_symbol.md (G0),
// tests/g0_rx_symbol.rs.
//
// Two slots (the founder's choice, 2026-11-24): two 64-word stores, a symbol's bins written to
// its slot as they come, then the products pass rewrites them in place as u, and the soft pass
// reads them; symbols alternate between the slots. Three engines work in symbol order:
//   bins:     a symbol's 52 used bins into the slot that is empty (the others ignored);
//   products: once a slot is full and the channel is in: 52 reads of Z and Hs (the pilots
//             first), four real products, the shift (half to even), saturation to U bits,
//             written back; pil_valid with the four pilots, their signs removed (U + 1 bits);
//   rotation: once a slot's products are in and its θ has come: the pilots read again, rotated
//             by −c·θ, summed, sent to the CORDIC; φ = angle·2^(P − A) (phi_valid); with the
//             symbol's soft scale in (SIGNAL: BPSK's; data: the data modulation's), the soft
//             pass, a subcarrier a clock from −26 to 26, soft_valid with its N_BPSC values in
//             on-air order (lanes 0 .. N_BPSC − 1, the I axis's first); the slot is empty after
//             its 48th read.
// The phase is kept in P bits: θ's bits above P do not matter (G0).
//
// In: bins with their index from fft64, at most one a clock, a symbol's 52 used bins before the
// next symbol's, each while bins_ok (the slot it goes to is empty or filling; simulation stops
// otherwise: the top level holds the next window until then). The packet's symbols are
// numbered from 0 (SIGNAL, polarity index 0) on the first bin after drop or reset; data symbol
// k is symbol k + 1. chan_valid once a packet with the shift and n = bit length of Σg; the
// channel block's stores answer hs_*/g_* a clock after the read (rx_channel's ports).
// scale_valid with a modulation's q (any order, any time from chan_valid on); mod_valid with the
// data's modulation; theta_valid with the next symbol's θ (in symbol order; one may wait,
// a second while one waits stops simulation).
// The CORDIC: cr_valid with the pilots' sum held until cr_take; the angle comes back with
// ca_valid (the shared iterative CORDIC, hw/rtl/cordic_iter.sv; the top level arbitrates).
// Out: no ready. drop (and rst_n): back to a packet's start, both slots empty; the stores'
// contents left. The top level drops at every packet's end.
//
// Latencies, in clock edges after the edge that takes the last of the triggers (tb_rx_symbol
// checks every one):
//   pil_valid   9 after: the symbol's 52nd used bin, chan_valid, the previous symbol's last
//               product written (48 edges after its pil_valid);
//   cr_valid   10 after: the symbol's last product written, the edge before θ's (θ is used
//               in the cycle it comes), the previous soft pass's last read (41 edges after
//               its first soft_valid);
//   phi_valid   1 after ca_valid (18 after the edge that took the request, cordic_iter's 17);
//   soft_valid  8 after: phi_valid, the scale's edge, the data modulation's (data symbols);
//               then one a clock, 48 of them.

`default_nettype none

module rx_symbol
    import rx_channel_pkg::*;
    import rx_front_rom_pkg::*;
    import rx_symbol_pkg::*;
(
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              drop,
    // bins from fft64
    input  wire                              bin_valid,
    input  wire        [5:0]                 bin_idx,
    input  wire signed [W_BITS-1:0]          bin_re,
    input  wire signed [W_BITS-1:0]          bin_im,
    output logic                             bins_ok,
    // the channel
    input  wire                              chan_valid,
    input  wire        [4:0]                 chan_shift,
    input  wire        [4:0]                 chan_n,
    output logic                             hs_re,
    output logic       [5:0]                 hs_raddr,
    input  wire signed [HS_BITS-1:0]         hs_rdata_re,
    input  wire signed [HS_BITS-1:0]         hs_rdata_im,
    output logic                             g_re,
    output logic       [5:0]                 g_raddr,
    input  wire        [U_BITS-2:0]          g_rdata,
    // the soft scales, the data's modulation
    input  wire                              scale_valid,
    input  wire        [1:0]                 scale_mod,
    input  wire        [SQ_BITS-1:0]         scale_q,
    input  wire                              mod_valid,
    input  wire        [1:0]                 data_mod,
    // the tracker
    output logic                             pil_valid,
    output logic signed [PILOT_BITS-1:0]     pil_re [4],
    output logic signed [PILOT_BITS-1:0]     pil_im [4],
    input  wire                              theta_valid,
    input  wire signed [NCO_PHASE_BITS:0]    theta,
    // the CORDIC
    output logic                             cr_valid,
    output logic signed [CORDIC_IN_BITS-1:0] cr_re,
    output logic signed [CORDIC_IN_BITS-1:0] cr_im,
    input  wire                              cr_take,
    input  wire                              ca_valid,
    input  wire signed [ANGLE_BITS-1:0]      ca_angle,
    // out
    output logic                             phi_valid,
    output logic signed [NCO_PHASE_BITS-1:0] phi,
    output logic                             soft_valid,
    output logic       [5:0]                 soft_j,
    output logic       [1:0]                 soft_mod,
    output logic                             soft_last,
    output logic signed [SOFT_BITS-1:0]      soft_v [6]
);
    localparam int P  = NCO_PHASE_BITS;
    localparam int L  = NCO_LUT_BITS;
    localparam int Q  = NCO_AMP_BITS;
    localparam int A  = ANGLE_BITS;
    localparam int U  = U_BITS;
    localparam int W  = W_BITS;
    localparam int ZH = W + HS_BITS + 1;            // Z·Hs* summed: 35
    localparam int PB = PILOT_BITS;                 // 17
    localparam int RW = PB + Q;                     // a·e^(jφ) summed: 28
    localparam int GW = U - 1;                      // a gain: 15
    localparam int TW = GW + KAPPA_BITS;                // g·κ: 24
    localparam int TH = TW - KAPPA_FRAC + 1;            // a threshold, signed: 15
    localparam int MW = U + 1;                      // a demapper value: 17
    localparam int MG = U + SQ_BITS;                // |m|·q: 36
    localparam int SW = U + 2;                      // the pilots' sum: 18
    localparam logic [P-1:0] HALF = P'(1) << (P - L - 1);

    // ---- polarity, subcarrier tables ------------------------------------------------------
    function automatic logic [5:0] seq_bin(input logic [5:0] k);   // the products' order
        seq_bin = '0;
        for (int i = 0; i < 4; i++)  if (int'(k) == i)     seq_bin = 6'(PILOT_BIN[i]);
        for (int i = 0; i < 48; i++) if (int'(k) == i + 4) seq_bin = 6'(DATA_BIN[i]);
    endfunction
    function automatic logic [5:0] data_bin(input logic [5:0] j);
        data_bin = '0;
        for (int i = 0; i < 48; i++) if (int'(j) == i) data_bin = 6'(DATA_BIN[i]);
    endfunction
    function automatic logic signed [5:0] data_c(input logic [5:0] j);
        data_c = '0;
        for (int i = 0; i < 48; i++) if (int'(j) == i) data_c = 6'(DATA_C[i]);
    endfunction
    function automatic logic signed [5:0] pilot_c(input logic [1:0] k);
        pilot_c = '0;
        for (int i = 0; i < 4; i++) if (int'(k) == i) pilot_c = 6'(PILOT_C[i]);
    endfunction
    // The demapper's constants of a modulation (0 where it has no such fold).
    function automatic logic [KAPPA_BITS-1:0] k_hi(input logic [1:0] m);
        k_hi = '0;
        for (int i = 0; i < 4; i++) if (int'(m) == i) k_hi = KAPPA_BITS'(K_HI[i]);
    endfunction
    function automatic logic [KAPPA_BITS-1:0] k_lo(input logic [1:0] m);
        k_lo = '0;
        for (int i = 0; i < 4; i++) if (int'(m) == i) k_lo = KAPPA_BITS'(K_LO[i]);
    endfunction
    // c·θ modulo 2^P (θ's low P bits).
    function automatic logic [P-1:0] c_theta(input logic signed [5:0] c, input logic [P-1:0] t);
        logic [P-1:0] cc;
        cc      = P'(c);                // sign-extended: c modulo 2^P
        c_theta = cc * t;               // P bits: the product modulo 2^P
    endfunction

    // ---- packet state -------------------------------------------------------------------
    logic             chan_ok;
    logic [4:0]       shift_r, n_r;
    logic [3:0]       q_ok;
    logic [SQ_BITS-1:0] q_val [4];
    logic             mod_ok;
    logic [1:0]       dmod;
    logic             th_full;
    logic [P-1:0]     th_buf;

    // ---- slots --------------------------------------------------------------------------
    typedef enum logic [1:0] {S_EMPTY, S_FILL, S_FULL, S_PROD} slot_t;
    slot_t      st [2];
    logic       sl_neg [2];      // the slot's polarity p_n = −1
    logic       sl_sig [2];      // the slot holds SIGNAL
    logic       wsel, psel, rsel;
    logic [5:0] fill_cnt;
    logic [6:0] wpol;            // the next symbol's polarity index, modulo 127
    logic       wfirst;          // the next symbol is SIGNAL

    assign bins_ok = st[wsel] == S_EMPTY || st[wsel] == S_FILL;
    wire bin_wr  = bin_valid && L_USED[bin_idx] && bins_ok;
    wire bin_end = bin_wr && fill_cnt == 6'd51;

    // ---- the products engine ------------------------------------------------------------
    logic       p_run, p_wait;   // issuing; waiting for the last write
    logic [5:0] p_cnt;
    wire        p_start = !p_run && !p_wait && st[psel] == S_FULL && chan_ok;
    wire        p_issue = p_run;
    wire [5:0]  p_addr  = seq_bin(p_cnt);

    // ---- the rotation engine ------------------------------------------------------------
    typedef enum logic [2:0] {R_IDLE, R_PIL, R_SUM, R_CORD, R_ANG, R_SCALE, R_SOFT} rstate_t;
    rstate_t    rs;
    logic [5:0] r_cnt;
    logic       cur_neg, cur_sig;
    logic [P-1:0] th_cur;
    logic [1:0] cur_mod;
    logic [SQ_BITS-1:0] cur_q;
    logic [4:0] cur_down;
    logic [2:0] pil_seen;
    logic signed [SW-1:0] sum_re, sum_im;
    wire        r_go    = rs == R_IDLE && st[rsel] == S_PROD && (th_full || theta_valid);
    wire        r_pil   = rs == R_PIL;
    wire        r_soft  = rs == R_SOFT;
    wire        r_issue = r_pil || r_soft;
    wire [5:0]  r_addr  = r_pil ? 6'(PILOT_BIN[0]) : data_bin(r_cnt);
    // the pilot's bin by its index (r_addr's for the pilots)
    logic [5:0] pil_addr;
    always_comb begin
        pil_addr = '0;
        for (int i = 0; i < 4; i++) if (int'(r_cnt[1:0]) == i) pil_addr = 6'(PILOT_BIN[i]);
    end
    wire [5:0]  r_raddr = r_pil ? pil_addr : r_addr;
    wire [1:0]  sym_mod = cur_sig ? 2'd0 : dmod;
    wire        scale_in = (cur_sig || mod_ok) && q_ok[sym_mod];

    // ---- the stores ---------------------------------------------------------------------
    logic                 p4_v, p4_last;
    logic                 p4_slot;
    logic [5:0]           p4_addr;
    logic signed [U-1:0]  p4_re, p4_im;
    logic [2*W-1:0]       rd [2];

    for (genvar s = 0; s < 2; s++) begin : g_slot
        wire bw = bin_wr && wsel == 1'(s);
        wire pw = p4_v && p4_slot == 1'(s);
        wire pr = p_issue && psel == 1'(s);
        wire rr = r_issue && rsel == 1'(s);
        ram_1w1r #(.W(2 * W), .DEPTH(64)) u_slot (
            .clk,
            .we(bw || pw),
            .waddr(bw ? bin_idx : p4_addr),
            .wdata(bw ? {bin_re, bin_im} : {p4_re, p4_im}),
            .re(pr || rr),
            .raddr(pr ? p_addr : r_raddr),
            .rdata(rd[s])
        );
`ifndef SYNTHESIS
        always_ff @(posedge clk) begin
            if (rst_n && !drop && bw && pw) begin
                $error("rx_symbol: slot %0d written by a bin and a product", s);
                $stop;
            end
            if (rst_n && !drop && pr && rr) begin
                $error("rx_symbol: slot %0d read by both engines", s);
                $stop;
            end
        end
`endif
    end

    assign hs_re    = p_issue;
    assign hs_raddr = p_addr;
    assign g_re     = r_soft;
    assign g_raddr  = r_addr;

    // ---- the products pipeline ----------------------------------------------------------
    logic                 p1_v, p1_slot;
    logic [5:0]           p1_k, p1_addr;
    logic                 p2_v, p2_slot;
    logic [5:0]           p2_k, p2_addr;
    logic signed [ZH-2:0] p2_rr, p2_ii, p2_ir, p2_ri;
    logic                 p3_v, p3_slot;
    logic [5:0]           p3_k, p3_addr;
    logic signed [ZH-1:0] p3_re, p3_im;
    logic [5:0]           p4_k;

    wire signed [W-1:0] z_re = rd[p1_slot][2*W-1:W];
    wire signed [W-1:0] z_im = rd[p1_slot][W-1:0];

    // v / 2^s half to even (s ≤ 19), saturated to U bits.
    function automatic logic signed [U-1:0] narrow_u(input logic signed [ZH-1:0] v,
                                                     input logic [4:0] s);
        logic signed [ZH-1:0] fl;
        logic [ZH-1:0]        mask;
        logic                 half, sticky, up;
        logic signed [ZH:0]   r;
        fl     = v >>> s;
        mask   = (ZH'(1) << s) - ZH'(1);
        half   = s != 0 && v[6'(s) - 6'd1];
        sticky = s > 1 && ((v & (mask >> 1)) != '0);
        up     = half && (sticky || fl[0]);
        r      = (ZH+1)'(fl) + (ZH+1)'(up);
        if (r > (ZH+1)'((1 << (U - 1)) - 1))   narrow_u = U'((1 << (U - 1)) - 1);
        else if (r < -(ZH+1)'(1 << (U - 1)))   narrow_u = U'(-(1 << (U - 1)));
        else                                   narrow_u = r[U-1:0];
    endfunction

    always_ff @(posedge clk) begin
        p2_rr <= (ZH-1)'(z_re) * (ZH-1)'(hs_rdata_re);
        p2_ii <= (ZH-1)'(z_im) * (ZH-1)'(hs_rdata_im);
        p2_ir <= (ZH-1)'(z_im) * (ZH-1)'(hs_rdata_re);
        p2_ri <= (ZH-1)'(z_re) * (ZH-1)'(hs_rdata_im);
        p3_re <= ZH'(p2_rr) + ZH'(p2_ii);
        p3_im <= ZH'(p2_ir) - ZH'(p2_ri);
        p4_re <= narrow_u(p3_re, shift_r);
        p4_im <= narrow_u(p3_im, shift_r);
        p1_k <= p_cnt;        p1_addr <= p_addr;  p1_slot <= psel;
        p2_k <= p1_k;         p2_addr <= p1_addr; p2_slot <= p1_slot;
        p3_k <= p2_k;         p3_addr <= p2_addr; p3_slot <= p2_slot;
        p4_k <= p3_k;         p4_addr <= p3_addr; p4_slot <= p3_slot;
        p4_last <= p3_k == 6'd51;
    end

    // ---- the rotation pipeline ----------------------------------------------------------
    // issue → r1 (the store's word, g, the phase) → r2 (a, the table) → r3 (products) →
    // r4 (v, thresholds; the pilots summed) → r5 (demapper) → r6 (|m|·q) → r7 (out).
    logic                 r1_v, r1_pil, r1_neg, r1_slot;
    logic [5:0]           r1_j;
    logic [L-1:0]         r1_tab;
    logic                 r2_v, r2_pil;
    logic [5:0]           r2_j;
    logic signed [PB-1:0] r2_are, r2_aim;
    logic signed [Q-1:0]  r2_cr, r2_ci;
    logic [GW-1:0]        r2_g;
    logic                 r3_v, r3_pil;
    logic [5:0]           r3_j;
    logic signed [RW-1:0] r3_rr, r3_ii, r3_ri, r3_ir;
    logic [TW-1:0]        r3_thi, r3_tlo;
    logic                 r4_v, r4_pil;
    logic [5:0]           r4_j;
    logic signed [U-1:0]  r4_re, r4_im;
    logic signed [TH-1:0] r4_thi, r4_tlo;
    logic                 r5_v;
    logic [5:0]           r5_j;
    logic signed [MW-1:0] r5_m [6];
    logic                 r6_v;
    logic [5:0]           r6_j;
    logic [MG-1:0]        r6_mag [6];
    logic [5:0]           r6_neg;

    // The issue's phase: −c·θ for a pilot, −(φ + c·θ) for a data subcarrier, + half a step.
    wire signed [5:0] r_c   = r_pil ? pilot_c(r_cnt[1:0]) : data_c(r_cnt);
    wire [P-1:0]      r_ph  = HALF - (r_pil ? '0 : phi) - c_theta(r_c, th_cur);
    // the pilot's sign: the polarity times the base sign
    wire              r_neg = r_pil && (cur_neg ^ PILOT_BASE_NEG[r_cnt[1:0]]);

    wire signed [U-1:0] u_re = rd[r1_slot][2*W-1:W];
    wire signed [U-1:0] u_im = rd[r1_slot][W-1:0];
    wire [L-1:0] tab = r1_tab;
    wire signed [Q-1:0] tc = nco_cos(int'(tab[L-3:0]));
    wire signed [Q-1:0] ts = nco_sin(int'(tab[L-3:0]));

    logic signed [RW-1:0] ar, ai, cr2, ci2;
    assign ar  = RW'(r2_are);
    assign ai  = RW'(r2_aim);
    assign cr2 = RW'(r2_cr);
    assign ci2 = RW'(r2_ci);
    logic signed [RW-1:0] rot_re, rot_im;
    assign rot_re = r3_rr - r3_ii;
    assign rot_im = r3_ri + r3_ir;
    logic signed [U-1:0] v_re, v_im;
    round_sat #(.IN_W(RW), .SHIFT(Q - 1), .OUT_W(U)) u_rv_re (.in(rot_re), .out(v_re));
    round_sat #(.IN_W(RW), .SHIFT(Q - 1), .OUT_W(U)) u_rv_im (.in(rot_im), .out(v_im));
    logic signed [TH-1:0] thi, tlo;
    round_sat #(.IN_W(TW + 1), .SHIFT(KAPPA_FRAC), .OUT_W(TH)) u_thi (.in({1'b0, r3_thi}), .out(thi));
    round_sat #(.IN_W(TW + 1), .SHIFT(KAPPA_FRAC), .OUT_W(TH)) u_tlo (.in({1'b0, r3_tlo}), .out(tlo));

    // The demapper of one axis: −v, |v| − t_hi, ||v| − t_hi| − t_lo.
    logic signed [MW-1:0] m0_re, m1_re, m2_re, m0_im, m1_im, m2_im;
    always_comb begin
        logic signed [MW-1:0] ar_, ai_;
        ar_   = r4_re[U-1] ? -MW'(r4_re) : MW'(r4_re);
        ai_   = r4_im[U-1] ? -MW'(r4_im) : MW'(r4_im);
        m0_re = -MW'(r4_re);
        m0_im = -MW'(r4_im);
        m1_re = ar_ - MW'(r4_thi);
        m1_im = ai_ - MW'(r4_thi);
        m2_re = (m1_re[MW-1] ? -m1_re : m1_re) - MW'(r4_tlo);
        m2_im = (m1_im[MW-1] ? -m1_im : m1_im) - MW'(r4_tlo);
    end

    // The scale and the clip: (|m|·q + 2^(down−1)) >> down, at most SOFT_MAX.
    function automatic logic [SOFT_BITS-2:0] clip(input logic [MG-1:0] mag,
                                                 input logic [4:0] down);
        logic [MG:0] r;
        r = ((MG+1)'(mag) + ((MG+1)'(1) << (down - 5'd1))) >> down;
        clip = r > (MG+1)'(SOFT_MAX) ? (SOFT_BITS-1)'(SOFT_MAX) : r[SOFT_BITS-2:0];
    endfunction

    always_ff @(posedge clk) begin
        // r1
        r1_tab <= r_ph[P-1 -: L];
        r1_j   <= r_cnt;
        r1_pil <= r_pil;
        r1_neg <= r_neg;
        r1_slot <= rsel;
        // r2
        r2_are <= r1_pil && r1_neg ? -PB'(u_re) : PB'(u_re);
        r2_aim <= r1_pil && r1_neg ? -PB'(u_im) : PB'(u_im);
        unique case (tab[L-1 -: 2])
            2'd0: begin r2_cr <= tc;  r2_ci <= ts;  end
            2'd1: begin r2_cr <= -ts; r2_ci <= tc;  end
            2'd2: begin r2_cr <= -tc; r2_ci <= -ts; end
            default: begin r2_cr <= ts; r2_ci <= -tc; end
        endcase
        r2_g   <= g_rdata;
        r2_j   <= r1_j;
        r2_pil <= r1_pil;
        // r3
        r3_rr  <= ar * cr2;
        r3_ii  <= ai * ci2;
        r3_ri  <= ar * ci2;
        r3_ir  <= ai * cr2;
        r3_thi <= TW'(r2_g) * TW'(k_hi(cur_mod));
        r3_tlo <= TW'(r2_g) * TW'(k_lo(cur_mod));
        r3_j   <= r2_j;
        r3_pil <= r2_pil;
        // r4
        r4_re  <= v_re;
        r4_im  <= v_im;
        r4_thi <= thi;
        r4_tlo <= tlo;
        r4_j   <= r3_j;
        r4_pil <= r3_pil;
        // r5: the lanes in on-air order
        for (int i = 0; i < 6; i++) r5_m[i] <= '0;
        unique case (cur_mod)
            2'd0: r5_m[0] <= m0_re;
            2'd1: begin r5_m[0] <= m0_re; r5_m[1] <= m0_im; end
            2'd2: begin
                r5_m[0] <= m0_re; r5_m[1] <= m1_re; r5_m[2] <= m0_im; r5_m[3] <= m1_im;
            end
            default: begin
                r5_m[0] <= m0_re; r5_m[1] <= m1_re; r5_m[2] <= m2_re;
                r5_m[3] <= m0_im; r5_m[4] <= m1_im; r5_m[5] <= m2_im;
            end
        endcase
        r5_j <= r4_j;
        // r6
        for (int i = 0; i < 6; i++) begin
            logic signed [MW-1:0] mm;
            mm = r5_m[i][MW-1] ? -r5_m[i] : r5_m[i];     // |m| ≤ 2^(U−1): no overflow (G0)
            r6_mag[i] <= MG'(unsigned'(mm)) * MG'(cur_q);
            r6_neg[i] <= r5_m[i][MW-1];
        end
        r6_j <= r5_j;
        // r7
        for (int i = 0; i < 6; i++) begin
            logic [SOFT_BITS-2:0] c;
            c = clip(r6_mag[i], cur_down);
            soft_v[i] <= r6_neg[i] ? -SOFT_BITS'(c) : SOFT_BITS'(c);
        end
        soft_j    <= r6_j;
        soft_last <= r6_j == 6'd47;
        soft_mod  <= cur_mod;
    end

    // ---- control --------------------------------------------------------------------------
    wire th_take = r_go;
    always_ff @(posedge clk) begin
        if (!rst_n || drop) begin
            chan_ok   <= 1'b0;
            q_ok      <= '0;
            mod_ok    <= 1'b0;
            th_full   <= 1'b0;
            st[0]     <= S_EMPTY;
            st[1]     <= S_EMPTY;
            wsel      <= 1'b0;
            psel      <= 1'b0;
            rsel      <= 1'b0;
            fill_cnt  <= '0;
            wpol      <= '0;
            wfirst    <= 1'b1;
            p_run     <= 1'b0;
            p_wait    <= 1'b0;
            rs        <= R_IDLE;
            p1_v      <= 1'b0;
            p2_v      <= 1'b0;
            p3_v      <= 1'b0;
            p4_v      <= 1'b0;
            r1_v      <= 1'b0;
            r2_v      <= 1'b0;
            r3_v      <= 1'b0;
            r4_v      <= 1'b0;
            r5_v      <= 1'b0;
            r6_v      <= 1'b0;
            soft_valid <= 1'b0;
            pil_valid <= 1'b0;
            phi_valid <= 1'b0;
            cr_valid  <= 1'b0;
        end else begin
            // the packet's inputs
            if (chan_valid) begin
                chan_ok <= 1'b1;
                shift_r <= chan_shift;
                n_r     <= chan_n;
            end
            if (scale_valid) begin
                q_ok[scale_mod]  <= 1'b1;
                q_val[scale_mod] <= scale_q;
            end
            if (mod_valid) begin
                mod_ok <= 1'b1;
                dmod   <= data_mod;
            end
            if (theta_valid) th_buf <= P'(theta);
            th_full <= theta_valid ? !th_take || th_full : th_full && !th_take;

            // bins
            if (bin_wr) begin
                if (fill_cnt == '0) begin
                    sl_neg[wsel] <= POLARITY_NEG[wpol];
                    sl_sig[wsel] <= wfirst;
                end
                st[wsel] <= bin_end ? S_FULL : S_FILL;
                fill_cnt <= bin_end ? '0 : fill_cnt + 6'd1;
                if (bin_end) begin
                    wsel   <= !wsel;
                    wpol   <= wpol == 7'd126 ? '0 : wpol + 7'd1;
                    wfirst <= 1'b0;
                end
            end

            // the products engine
            if (p_start) begin
                p_run <= 1'b1;
                p_cnt <= '0;
            end else if (p_run) begin
                p_cnt <= p_cnt + 6'd1;
                if (p_cnt == 6'd51) begin
                    p_run  <= 1'b0;
                    p_wait <= 1'b1;
                end
            end
            p1_v <= p_issue;
            p2_v <= p1_v;
            p3_v <= p2_v;
            p4_v <= p3_v;
            pil_valid <= p4_v && p4_k == 6'd3;
            if (p4_v && p4_k < 6'd4) begin
                pil_re[p4_k[1:0]] <= sl_neg[p4_slot] ^ PILOT_BASE_NEG[p4_k[1:0]]
                                     ? -PB'(p4_re) : PB'(p4_re);
                pil_im[p4_k[1:0]] <= sl_neg[p4_slot] ^ PILOT_BASE_NEG[p4_k[1:0]]
                                     ? -PB'(p4_im) : PB'(p4_im);
            end
            if (p4_v && p4_last) begin
                st[p4_slot] <= S_PROD;
                p_wait      <= 1'b0;
                psel        <= !psel;
            end

            // the rotation engine
            r1_v <= r_issue;
            r2_v <= r1_v;
            r3_v <= r2_v;
            r4_v <= r3_v;
            r5_v <= r4_v && !r4_pil;
            r6_v <= r5_v;
            soft_valid <= r6_v;
            phi_valid  <= 1'b0;
            if (r4_v && r4_pil) begin
                sum_re   <= sum_re + SW'(r4_re);
                sum_im   <= sum_im + SW'(r4_im);
                pil_seen <= pil_seen + 3'd1;
            end
            unique case (rs)
                R_IDLE: if (r_go) begin
                    th_cur   <= th_full ? th_buf : P'(theta);
                    cur_neg  <= sl_neg[rsel];
                    cur_sig  <= sl_sig[rsel];
                    r_cnt    <= '0;
                    sum_re   <= '0;
                    sum_im   <= '0;
                    pil_seen <= '0;
                    rs       <= R_PIL;
                end
                R_PIL: begin
                    r_cnt <= r_cnt + 6'd1;
                    if (r_cnt == 6'd3) rs <= R_SUM;
                end
                R_SUM: if (pil_seen == 3'd4) begin
                    cr_valid <= 1'b1;
                    cr_re    <= CORDIC_IN_BITS'(sum_re);
                    cr_im    <= CORDIC_IN_BITS'(sum_im);
                    rs       <= R_CORD;
                end
                R_CORD: if (cr_take) begin
                    cr_valid <= 1'b0;
                    rs       <= R_ANG;
                end
                R_ANG: if (ca_valid) begin
                    phi       <= P'(ca_angle) << (P - A);
                    phi_valid <= 1'b1;
                    rs        <= R_SCALE;
                end
                R_SCALE: if (scale_in) begin
                    cur_mod  <= sym_mod;
                    cur_q    <= q_val[sym_mod];
                    cur_down <= n_r + 5'(S_FRAC);
                    r_cnt    <= '0;
                    rs       <= R_SOFT;
                end
                R_SOFT: begin
                    r_cnt <= r_cnt + 6'd1;
                    if (r_cnt == 6'd47) begin
                        st[rsel] <= S_EMPTY;
                        rsel     <= !rsel;
                        rs       <= R_IDLE;
                    end
                end
                default: rs <= R_IDLE;
            endcase
        end
    end

    // θ's sign bit and the phase below the table's address do not matter (modulo 2^P; the
    // table takes the top L bits).
    logic unused;
    assign unused = ^{theta[P], r_ph[P-L-1:0]};

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (rst_n && !drop) begin
            if (bin_valid && L_USED[bin_idx] && !bins_ok) begin
                $error("rx_symbol: a bin for a slot that is not empty");
                $stop;
            end
            if (theta_valid && th_full && !th_take) begin
                $error("rx_symbol: theta while one waits");
                $stop;
            end
            if (chan_valid && chan_ok) begin
                $error("rx_symbol: a second chan_valid in a packet");
                $stop;
            end
        end
    end
`endif
endmodule

`default_nettype wire
