// The timing tracker (ADR 0010, decisions 1 and 6.2; R0-09's data path): per packet the prior
// rate from the front's CFO word, SIGNAL's slope θ and its window move; per symbol the two pilot
// pairs' angles, the measurement, the least-squares fit's sums, its division, the drift, θ and
// the window's move.
// Golden model: radio_phy::rx_ofdm_fixed::FixedTracking::{from_cfo_word, unknown, step,
// move_window}. Gate G3: make -C hw rx_track. Architecture and widths: docs/gates/rx_track.md
// (G0), tests/g0_rx_track.rs.
//
// One shared multiplier (the founder's choice, 2026-11-24): MA × MB bits signed, operands
// registered on the edge that issues; the two halves of MA times MB on the next edge, their sum
// on the one after (pipelined, the founder's choice of 2026-11-25: the single-cycle product
// was the data path's worst path through the open flow, docs/gates/silicon_probe.md). A
// sequencer issues at most one product a clock and reads it three edges later. Every product
// of the block goes through it.
//
// Packet: start with the CFO word and the fs/fc register (ratio; 0: none written, the prior
// of an unknown offset). rate0 = word·ratio rounded half to even by RATIO_SHIFT (it always fits
// RATE0_BITS); SIGNAL's θ = rate0·7 rounded by R (δ is 0), out on theta_valid with its move.
// The channel block's quotients are taken from its q_valid as they pass (job 1 the outer
// coefficient, 2 the inner, 3 the variance; the others are ignored).
//
// Symbol: pilots on pil_valid into one register, free once the step has made its pair products
// (8 edges after it starts). One set suffices: SIGNAL's register is free at most 16 edges after
// its pilots (the start's 7 edges, then the step's 9) and the symbol block's pilots are at least
// 48 clocks apart; every later symbol's come only after the last θ, as its window is placed by
// that θ. A
// step: the pair products p3·conj(p0) and p2·conj(p1) (eight products), each pair's CORDIC
// request held until taken (cr_valid/cr_take, the outer one first); the predicted drift, the
// back-correction δ·τ and τ²; then, once the three quotients are in, λ = weight·variance
// rounded by V, the sums (a packet whose variance is 0 or above 3·2^V measures nothing), the
// determinant; then with each angle its coefficient's product, the measurement e, z, Στz, the
// numerator. A data symbol whose packet measures asks the divider for |num|·2^R / det
// (dv_req held until dv_gnt, FIT_QBITS quotient bits, below 2^FIT_QBITS on every frame: G0);
// then the drift with the new δ, θ = drift + the window's moves, out with the move. SIGNAL's
// step only fills the sums (its quotient is 0); a data symbol of a packet that measures
// nothing gives θ from the predicted drift.
//
// Latencies, in edges after the edge that takes (or brings) the trigger; the bench checks every
// one. θ0: 7 after start (4 without a ratio). A step starts on edge T = max(its pilots' edge,
// the last step's end) + 1; its outer request is up on T + 7 (the inner on T + 11). λ is asked on
// C = max(T + 15, the last quotient's edge + 1) and the determinant follows; the angles' products
// go on G0 = max(C + 9, the outer angle's edge + 1) and G1 = max(G0 + 1, the inner's + 1). Then
// without a division: SIGNAL's step ends on G1 + 3 (its packet measures nothing) or G1 + 13; a
// data symbol of a packet that measures nothing has θ on G1 + 4. A data symbol that measures:
// dv_req on G1 + 13, held until dv_gnt; θ on the 6th edge after the one that brings dv_done
// (div_serial: FIT_QBITS + 1 edges after its grant). The end of a data step is its θ's edge.
//
// Outputs: theta_valid for one cycle with theta (P + 1 bits) and move_early (the window a
// sample earlier: θ > 2^(P−7)) or move_late (later: θ < −2^(P−7)). drop (and rst_n): idle,
// nothing out; the CORDIC and the divider take the same drop. A start is legal only while the
// block is idle with no pilots held (the top level drops at every packet's end), never with a
// drop, a quotient or pilots in its cycle (the quotients come after the channel, long after).

`default_nettype none

module rx_track
    import rx_front_rom_pkg::*;
    import rx_channel_pkg::*;
    import rx_symbol_pkg::*;
    import rx_track_pkg::*;
#(
    parameter int Q_W = 28
) (
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              drop,
    // the packet's start
    input  wire                              start,
    input  wire signed [NCO_PHASE_BITS-1:0]  cfo_word,
    input  wire        [15:0]                ratio,
    // the channel block's quotients, as they pass
    input  wire                              q_valid,
    input  wire        [2:0]                 q_job,
    input  wire        [Q_W-1:0]             q_value,
    // the symbol block's pilots
    input  wire                              pil_valid,
    input  wire signed [PILOT_BITS-1:0]      pil_re [4],
    input  wire signed [PILOT_BITS-1:0]      pil_im [4],
    // θ and the window's move
    output logic                             theta_valid,
    output logic signed [NCO_PHASE_BITS:0]   theta,
    output logic                             move_early,
    output logic                             move_late,
    // δ after the last step (FIT_QBITS + 1 bits): status, and the bench's check of the fit
    output logic signed [FIT_QBITS:0]        fit_delta,
    // the shared CORDIC
    output logic                             cr_valid,
    output logic signed [CORDIC_IN_BITS-1:0] cr_re,
    output logic signed [CORDIC_IN_BITS-1:0] cr_im,
    input  wire                              cr_take,
    input  wire                              ca_valid,
    input  wire signed [ANGLE_BITS-1:0]      ca_angle,
    // the shared divider
    output logic                             dv_req,
    output logic       [FIT_NUM_BITS-1:0]          dv_num,
    output logic       [FIT_DEN_BITS-1:0]          dv_den,
    input  wire                              dv_gnt,
    input  wire                              dv_done,
    input  wire        [Q_W-1:0]             dv_quot
);
    localparam int P    = NCO_PHASE_BITS;
    localparam int A    = ANGLE_BITS;
    localparam int R    = RATE_FRAC;
    localparam int PB   = PILOT_BITS;
    localparam int CI   = CORDIC_IN_BITS;
    localparam int MA   = 24;              // the multiplier's operands
    localparam int MB   = 44;
    localparam int MP   = MA + MB;
    localparam int RB0  = RATE0_BITS;      // rate0
    localparam int DB   = FIT_QBITS + 1;   // δ
    localparam int RB   = DB + 1;          // rate0 + δ (RB0 < DB)
    localparam int TH   = P + 1;           // θ, the drift
    localparam int ZB   = P - 2;           // z
    localparam int BK   = 30;              // δ·τ rounded by R
    localparam int SHB  = 8;               // the window's shift
    localparam int TAUB = 13;
    localparam int NB   = 11;
    localparam int S1B  = 23;
    localparam int S2B  = 35;
    localparam int Y0B  = 31;
    localparam int Y1B  = 43;
    localparam int LAMB = 27;
    localparam int DETB = FIT_DEN_BITS;
    localparam int NUMB = 55;

    // ---- rounding helpers ----------------------------------------------------------------
    // v / 2^s, half to even (s ≥ 1), as a MP + 1 bit value.
    function automatic logic signed [MP:0] rsh_even(input logic signed [MP-1:0] v,
                                                    input int s);
        logic signed [MP-1:0] fl;
        logic                 half, sticky;
        logic [MP-1:0]        mask;
        fl     = v >>> s;
        mask   = (MP'(1) << (s - 1)) - MP'(1);
        half   = v[s - 1];
        sticky = (v & mask) != '0;
        rsh_even = (MP+1)'(fl) + (MP+1)'(half && (sticky || fl[0]));
    endfunction
    function automatic logic signed [TH-1:0] sat_th(input logic signed [MP:0] v);
        if (v > (MP+1)'((1 << (TH - 1)) - 1))  sat_th = TH'((1 << (TH - 1)) - 1);
        else if (v < -(MP+1)'(1 << (TH - 1)))  sat_th = TH'(-(1 << (TH - 1)));
        else                                   sat_th = TH'(v);
    endfunction

    // ---- the multiplier ------------------------------------------------------------------
    logic signed [MA-1:0] ma;
    logic signed [MB-1:0] mb;
    logic signed [MP-1:0] mp;
    // ma = ah·2^MAL + al, mb = bh·2^MBL + bl, the low halves unsigned: the four products of
    // the halves (12 × 22 bits) on the first edge, their sum on the second. (Halves of ma
    // alone left a 12 × 44 product per edge, still the data path's worst path at 160 MHz:
    // docs/gates/silicon_probe.md, 2026-11-25.)
    localparam int MAL = MA / 2;
    localparam int MBL = MB / 2;
    wire signed [MA-MAL-1:0] ah = $signed(ma[MA-1:MAL]);
    wire signed [MAL:0]    al = $signed({1'b0, ma[MAL-1:0]});
    wire signed [MB-MBL-1:0] bh = $signed(mb[MB-1:MBL]);
    wire signed [MBL:0]    bl = $signed({1'b0, mb[MBL-1:0]});
    logic signed [MP-1:0] mhh, mhl, mlh, mll;
    always_ff @(posedge clk) begin
        mhh <= MP'(ah) * MP'(bh);
        mhl <= MP'(ah) * MP'(bl);
        mlh <= MP'(al) * MP'(bh);
        mll <= MP'(al) * MP'(bl);
        mp  <= (mhh <<< (MAL + MBL)) + (mhl <<< MAL) + (mlh <<< MBL) + mll;
    end

    // ---- packet state --------------------------------------------------------------------
    logic signed [RB0-1:0]  rate0;
    logic                   known;          // the ratio register was written
    logic signed [DB-1:0]   delta;
    logic signed [SHB-1:0]  shift;
    logic [NB-1:0]          n;
    logic [S1B-1:0]         s1;
    logic [S2B-1:0]         s2;
    logic signed [Y0B-1:0]  y0;
    logic signed [Y1B-1:0]  y1;
    logic [COEF_Q-1:0]      co, ci;
    logic [VAR_Q-1:0]       var_q;
    logic                   co_ok, ci_ok, v_ok;
    logic [NB-1:0]          sym;            // the step's symbol index i
    logic [TAUB-1:0]        tau;            // 7 + 5i

    wire usable = var_q != '0 && var_q <= VAR_Q'(3 << V_FRAC);

    // ---- the pilots held -----------------------------------------------------------------
    logic signed [PB-1:0] pq_re [4];
    logic signed [PB-1:0] pq_im [4];
    logic                 pq_full;

    wire signed [PB-1:0] p0r = pq_re[0], p0i = pq_im[0];
    wire signed [PB-1:0] p1r = pq_re[1], p1i = pq_im[1];
    wire signed [PB-1:0] p2r = pq_re[2], p2i = pq_im[2];
    wire signed [PB-1:0] p3r = pq_re[3], p3i = pq_im[3];

    // ---- the step's registers ------------------------------------------------------------
    logic signed [MP-1:0]  ta;              // a product held for its partner
    logic signed [CI-1:0]  o_re, o_im, i_re, i_im;
    logic                  req_o, req_i;
    logic                  got_o, got_i;
    logic signed [A-1:0]   a_o, a_i;
    logic signed [TH-1:0]  drift;           // the predicted drift, then the fitted one
    logic [P-1:0]          pm;              // the prediction modulo 2^P
    logic signed [BK-1:0]  back;
    logic [2*TAUB-1:0]     tt;              // τ²
    logic [LAMB-1:0]       lam;
    logic [DETB-1:0]       det;
    logic signed [P-1:0]   err;
    logic signed [NUMB-1:0] num;

    // moved: the window's moves before this symbol (SIGNAL's prediction has none in it).
    wire signed [TH+SHB:0] moved = sym == '0 ? '0 : (TH+SHB+1)'(shift) <<< (P - 6);

    // The pair angles against the prediction's turn, modulo 2^P (42 = 32 + 8 + 2, 14 = 8 + 4 + 2).
    wire [P-1:0] turn_o = (pm << 5) + (pm << 3) + (pm << 1);
    wire [P-1:0] turn_i = (pm << 3) + (pm << 2) + (pm << 1);
    wire signed [P-1:0] ang_o = {a_o, (P-A)'(0)} - turn_o;
    wire signed [P-1:0] ang_i = {a_i, (P-A)'(0)} - turn_i;

    // z = e + δ·τ (rounded) saturated to P − 2 bits.
    wire signed [BK:0] eb = (BK+1)'(err) + (BK+1)'(back);
    wire signed [ZB-1:0] z = eb > (BK+1)'((1 << (ZB - 1)) - 1) ? ZB'((1 << (ZB - 1)) - 1)
                           : eb < -(BK+1)'(1 << (ZB - 1))      ? ZB'(-(1 << (ZB - 1)))
                           : ZB'(eb);

    wire signed [MP:0] p_rsh_r = rsh_even(mp, R);
    wire [NUMB-1:0]    num_mag = num[NUMB-1] ? NUMB'(-num) : NUMB'(num);

    typedef enum logic [5:0] {
        IDLE,
        K1, K1W, K2, K3, K4, K4W, K5,                   // the packet's start
        A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13,
        C0, C1, C2, C3, C4, C5, C6, C7, C8,             // λ, the sums, the determinant
        G0, G1, G2, G3, G4,                             // the angles' products, e
        F0, F1, F2, F3, F4, F5, F6, F7, F8, F9,         // z, Στz, the numerator
        D0, D1, D2, D3, D4, D5,                         // the division, the drift
        TH_OUT
    } st_t;
    st_t st;

    wire quiet = st == IDLE && !pq_full;

    assign fit_delta = delta;

    always_comb begin
        cr_valid = req_o || req_i;
        cr_re    = req_o ? o_re : i_re;
        cr_im    = req_o ? o_im : i_im;
    end

    always_ff @(posedge clk) begin
        theta_valid <= 1'b0;
        if (!rst_n || drop) begin
            st     <= IDLE;
            pq_full <= 1'b0;
            req_o  <= 1'b0;
            req_i  <= 1'b0;
            got_o  <= 1'b0;
            got_i  <= 1'b0;
            dv_req <= 1'b0;
            co_ok  <= 1'b0;
            ci_ok  <= 1'b0;
            v_ok   <= 1'b0;
            move_early <= 1'b0;
            move_late  <= 1'b0;
        end else begin
            // ---- the quotients as they pass
            if (q_valid) begin
                if (q_job == 3'd1) begin co    <= COEF_Q'(q_value); co_ok <= 1'b1; end
                if (q_job == 3'd2) begin ci    <= COEF_Q'(q_value); ci_ok <= 1'b1; end
                if (q_job == 3'd3) begin var_q <= VAR_Q'(q_value);  v_ok  <= 1'b1; end
            end
            // ---- the pilots held
            if (pil_valid) begin
                pq_re <= pil_re;
                pq_im <= pil_im;
            end
            if (pil_valid)      pq_full <= 1'b1;
            else if (st == A8)  pq_full <= 1'b0;
            // ---- the CORDIC's answers: the outer pair's, then the inner's
            if (cr_take) begin
                if (req_o) req_o <= 1'b0;
                else       req_i <= 1'b0;
            end
            if (ca_valid) begin
                if (!got_o) begin a_o <= ca_angle; got_o <= 1'b1; end
                else        begin a_i <= ca_angle; got_i <= 1'b1; end
            end
            if (dv_gnt) dv_req <= 1'b0;

            unique case (st)
                IDLE: if (pq_full) st <= A0;

                // ---- the packet: rate0 (word·ratio issued with start), SIGNAL's θ
                K1: st <= K1W;
                K1W: st <= K2;
                K2: begin
                    // rate0 = word·ratio >> RATIO_SHIFT, half to even: a P-bit word times a
                    // 16-bit ratio stays below 2^(RATE0_BITS - 1) (the model's saturation never
                    // acts, FixedTracking::from_cfo_word).
                    rate0 <= RB0'(rsh_even(mp, RATIO_SHIFT));
                    st    <= K3;
                end
                K3: begin
                    ma <= MA'(7);
                    mb <= MB'(rate0);
                    st <= K4;
                end
                K4: st <= K4W;
                K4W: st <= K5;
                K5: begin
                    drift       <= sat_th(p_rsh_r);
                    theta       <= sat_th(p_rsh_r);
                    theta_valid <= 1'b1;
                    move_early  <= sat_th(p_rsh_r) > TH'(1 << (P - 7));
                    move_late   <= sat_th(p_rsh_r) < -TH'(1 << (P - 7));
                    if (sat_th(p_rsh_r) > TH'(1 << (P - 7)))       shift <= shift - SHB'(1);
                    else if (sat_th(p_rsh_r) < -TH'(1 << (P - 7))) shift <= shift + SHB'(1);
                    st <= IDLE;
                end

                // ---- a step: the pair products
                // (A product issued in state S is read in S + 3.)
                A0:  begin ma <= MA'(p3r); mb <= MB'(p0r); got_o <= 1'b0; got_i <= 1'b0; st <= A1; end
                A1:  begin ma <= MA'(p3i); mb <= MB'(p0i); st <= A2; end
                A2:  begin ma <= MA'(p3i); mb <= MB'(p0r); st <= A3; end
                A3:  begin ma <= MA'(p3r); mb <= MB'(p0i); ta <= mp; st <= A4; end
                A4:  begin ma <= MA'(p2r); mb <= MB'(p1r); o_re <= CI'(ta + mp); st <= A5; end
                A5:  begin ma <= MA'(p2i); mb <= MB'(p1i); ta <= mp; st <= A6; end
                A6:  begin ma <= MA'(p2i); mb <= MB'(p1r); o_im <= CI'(ta - mp); req_o <= 1'b1;
                           st <= A7; end
                A7:  begin ma <= MA'(p2r); mb <= MB'(p1i); ta <= mp; st <= A8; end
                // the predicted drift, δ·τ, τ²
                A8:  begin ma <= MA'(tau); mb <= MB'(RB'(rate0) + RB'(delta)); i_re <= CI'(ta + mp);
                           st <= A9; end
                A9:  begin ma <= MA'(tau); mb <= MB'(delta); ta <= mp; st <= A10; end
                A10: begin ma <= MA'(tau); mb <= MB'(tau); i_im <= CI'(ta - mp); req_i <= 1'b1;
                           st <= A11; end
                A11: begin
                    drift <= sat_th(p_rsh_r);
                    pm    <= P'(sat_th(p_rsh_r)) + P'(moved);
                    st    <= A12;
                end
                A12: begin back <= BK'(p_rsh_r); st <= A13; end
                A13: begin tt <= (2*TAUB)'(mp); st <= C0; end

                // ---- λ, the sums, the determinant
                C0: if (co_ok && ci_ok && v_ok) begin
                    ma <= MA'(var_q);
                    mb <= MB'(known ? WEIGHT_KNOWN : WEIGHT_UNKNOWN);
                    st <= C1;
                end
                C1: begin
                    if (usable) begin
                        n  <= n + NB'(1);
                        s1 <= s1 + S1B'(tau);
                        s2 <= s2 + S2B'(tt);
                    end
                    st <= C2;
                end
                C2: st <= C3;
                C3: begin lam <= LAMB'(rsh_even(mp, V_FRAC)); st <= C4; end
                C4: begin ma <= MA'(n); mb <= MB'(s2 + S2B'(lam)); st <= C5; end
                C5: begin ma <= MA'(s1); mb <= MB'(s1); st <= C6; end
                C6: st <= C7;
                C7: begin ta <= mp; st <= C8; end
                C8: begin det <= DETB'(ta - mp); st <= G0; end

                // ---- the angles' products, e
                G0: if (got_o) begin ma <= MA'(co); mb <= MB'(ang_o); st <= G1; end
                G1: if (got_i) begin ma <= MA'(ci); mb <= MB'(ang_i); st <= G2; end
                G2: st <= G3;
                G3: begin ta <= mp; st <= G4; end
                G4: begin
                    err <= P'(rsh_even(ta + mp, K_FRAC));
                    if (usable)            st <= F0;
                    else if (sym == '0)    st <= IDLE;
                    else                   st <= TH_OUT;
                end

                // ---- z, Στz, the numerator
                F0: begin ma <= MA'(tau); mb <= MB'(z); y0 <= y0 + Y0B'(z); st <= F1; end
                F1: st <= F2;
                F2: st <= F3;
                F3: begin y1 <= y1 + Y1B'(mp); st <= F4; end
                F4: begin ma <= MA'(n); mb <= MB'(y1); st <= F5; end
                F5: begin ma <= MA'(s1); mb <= MB'(y0); st <= F6; end
                F6: st <= F7;
                F7: begin ta <= mp; st <= F8; end
                F8: begin num <= NUMB'(ta - mp); st <= F9; end
                F9: begin
                    if (sym == '0) st <= IDLE;              // SIGNAL: its quotient is 0
                    else begin                              // det > 0: λ > 0 (G0)
                        dv_req <= 1'b1;
                        dv_num <= FIT_NUM_BITS'(num_mag) << R;
                        dv_den <= det;
                        st     <= D0;
                    end
                end

                // ---- the division, the drift with the new δ
                D0: if (dv_gnt) st <= D1;
                D1: if (dv_done) begin
                    delta <= num[NUMB-1] ? -DB'(dv_quot[FIT_QBITS-1:0])
                                         :  DB'(dv_quot[FIT_QBITS-1:0]);
                    st    <= D2;
                end
                D2: begin ma <= MA'(tau); mb <= MB'(RB'(rate0) + RB'(delta)); st <= D3; end
                D3: st <= D4;
                D4: st <= D5;
                D5: begin drift <= sat_th(p_rsh_r); st <= TH_OUT; end

                // ---- θ and the move
                TH_OUT: begin
                    theta       <= sat_th((MP+1)'(drift) + (MP+1)'(moved));
                    theta_valid <= 1'b1;
                    move_early  <= sat_th((MP+1)'(drift) + (MP+1)'(moved)) > TH'(1 << (P - 7));
                    move_late   <= sat_th((MP+1)'(drift) + (MP+1)'(moved)) < -TH'(1 << (P - 7));
                    if (sat_th((MP+1)'(drift) + (MP+1)'(moved)) > TH'(1 << (P - 7)))
                        shift <= shift - SHB'(1);
                    else if (sat_th((MP+1)'(drift) + (MP+1)'(moved)) < -TH'(1 << (P - 7)))
                        shift <= shift + SHB'(1);
                    st <= IDLE;
                end
                default: st <= IDLE;
            endcase

            // A step ends: the next symbol.
            if ((st == G4 && !usable && sym == '0) || (st == F9 && sym == '0) || st == TH_OUT) begin
                sym <= sym + NB'(1);
                tau <= tau + TAUB'(5);
            end

            // ---- the packet's start (legal only while quiet)
            if (start) begin
                ma     <= MA'({1'b0, ratio});
                mb     <= MB'(cfo_word);
                st     <= ratio != '0 ? K1 : K3;
                known  <= ratio != '0;
                rate0  <= '0;
                delta  <= '0;
                shift  <= '0;
                n      <= '0;
                s1     <= '0;
                s2     <= '0;
                y0     <= '0;
                y1     <= '0;
                sym    <= '0;
                tau    <= TAUB'(7);
                co_ok  <= 1'b0;
                ci_ok  <= 1'b0;
                v_ok   <= 1'b0;
            end
        end
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (rst_n && start && drop) $error("rx_track: start with drop");
        if (rst_n && !drop) begin
            if (start && !quiet) $error("rx_track: start while busy");
            if (start && q_valid) $error("rx_track: a quotient with start");
            if (pil_valid && pq_full && st != A8) $error("rx_track: pilots while a set is held");
            if (st == F9 && sym != '0 && usable && det == '0) $error("rx_track: det 0");
            if (pil_valid && start) $error("rx_track: pilots with start");
            if (st == A0 && int'(sym) > LAST_SYMBOL) $error("rx_track: symbol %0d", sym);
            if (ca_valid && got_i) $error("rx_track: a third angle");
            if (q_valid && q_job == 3'd1 && q_value >= Q_W'(1 << COEF_Q))
                $error("rx_track: outer coefficient %0d", q_value);
            if (q_valid && q_job == 3'd2 && q_value >= Q_W'(1 << COEF_Q))
                $error("rx_track: inner coefficient %0d", q_value);
            if (q_valid && q_job == 3'd3 && q_value >= Q_W'(1 << VAR_Q))
                $error("rx_track: variance %0d", q_value);
            if (st == D1 && dv_done && dv_quot >= Q_W'(1 << FIT_QBITS))
                $error("rx_track: quotient %0d", dv_quot);
        end
    end
`endif
endmodule

`default_nettype wire
