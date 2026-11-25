// The data path's top level behind rx_handoff (R0-09, ADR 0010 decisions 1, 2, 5 and 6.2): the
// reader of the hand-off's stream, the FFT, the channel and quotients, the symbol block, the
// tracker with the shared CORDIC and divider, the deinterleaver, the Viterbi decoder, SIGNAL's
// verdict and the octets, and the MAC's events. G0: docs/gates/rx_data.md. Golden models:
// radio_phy::rx_data_fixed::RxDataFixed for what comes out, radio_phy::rx_data::RxDataSchedule
// for the cycle of every beat taken and every event. Gate G3: make -C hw rx_data, at these pins
// a cycle at a time.
//
// In: the hand-off's stream (in_valid/in_ready; in_end with in_kind: a mark; in_first with
// in_cfo and in_tag on a stream's first sample); ratio, the MAC's fs/fc register (0: none).
// Out: the verdict for the hand-off (sig_valid with sig_tag, sig_fail, sig_symbols,
// sig_patient: rx_handoff takes them on the next edge); for the MAC rx_start (rate, length),
// the octets (rx_octets' outputs) and rx_end with a status. An event "in cycle c" is a register
// output seen in cycle c (rx_start and SIGNAL's failure are rx_signal's sig_valid; the octets
// and their ends rx_octets'; the ends by a mark this block's own register, or, for a frame cut
// before any data window and after SIGNAL's, rx_signal's sig_valid itself: the end in
// rx_start's cycle).
//
// The reader (radio_phy::rx_data's "The reader"; in_ready is a register): in_ready for cycle c
// is computed on edge c − 1 from the state after that edge. Windows: T1 at beats 0 … 63, T2 at
// 64 … 127, SIGNAL at 144, data symbol k at 144 + 80·k + σ, σ the tracker's moves so far
// (move_early −1, move_late +1, with each θ, θ0's included). A window's samples go to fft64 through one register;
// T1 and T2 with in_next on their 64th. SIGNAL's window is at 144 (θ0's move acts from data
// symbol 1 on, as in the model). Data symbol k's window: beats before its old place less one
// (σ without θ of symbol k − 1) are taken freely; from there on, only from θ of symbol k − 1
// seen (+ 1: the registered ready), for k ≥ 2 also the verdict seen; its first sample also
// waits for the FFT's flush (65 cycles after the last flushed window's 64th sample) and for
// rx_symbol's bins_ok. From the frame's last window on, or after a failed SIGNAL, with the
// verdict seen: the tail is taken and dropped. After a mark: nothing until the packet's end is
// out.
//
// rx_end status: 0 decoded, 1 seed zero (rx_octets), 2 SIGNAL failed (rx_end_info: rx_signal's
// sig_why), 3 truncated (a mark of kind 0 or 1 before the frame's end), 4 aborted (rx_end_info
// the mark's kind, 2 or 3). An end by a mark comes in the cycle after the mark, or once the data
// windows complete have been decoded (their last trellis step + 7 cycles), and never before the
// verdict when SIGNAL's window was complete. The drop to every block is the end itself.
//
// The age (the founder, 2026-11-25; docs/gates/rx_data.md): rx_age = adc_count − end_stamp
// modulo 2^16, both from rx_handoff (the samples written before this cycle, and the frame's
// end in that count, taken when the verdict was applied), with rx_age_valid on the decoded and
// the seed-zero ends only (status 0 and 1). rx_age means nothing without rx_age_valid. Golden
// model: radio_phy::rx_data::rx_end_age.

`default_nettype none

module rx_data
    import rx_front_rom_pkg::*;
    import rx_channel_pkg::*;
    import rx_symbol_pkg::*;
    import rx_track_pkg::*;
    import fft64_rom_pkg::*;
(
    input  wire                              clk,
    input  wire                              rst_n,       // synchronous, active low
    input  wire        [15:0]                ratio,       // fs/fc (rx_track), the MAC's register
    // the hand-off's stream
    input  wire                              in_valid,
    output logic                             in_ready,
    input  wire                              in_end,
    input  wire        [1:0]                 in_kind,
    input  wire                              in_first,
    input  wire signed [Y_BITS-1:0]          in_re,
    input  wire signed [Y_BITS-1:0]          in_im,
    input  wire signed [NCO_PHASE_BITS-1:0]  in_cfo,
    input  wire        [7:0]                 in_tag,
    // the age's count and the frame's end in it, from rx_handoff
    input  wire        [15:0]                adc_count,
    input  wire        [15:0]                end_stamp,
    // the verdict, to rx_handoff
    output logic                             sig_valid,
    output logic       [7:0]                 sig_tag,
    output logic                             sig_fail,
    output logic       [10:0]                sig_symbols,
    output logic                             sig_patient,
    // the MAC
    output logic                             rx_start,
    output logic       [2:0]                 rx_rate,
    output logic       [11:0]                rx_length,
    output logic                             oct_valid,
    output logic       [7:0]                 oct_data,
    output logic                             oct_first,
    output logic                             oct_last,
    output logic       [6:0]                 oct_seed,
    output logic       [15:0]                oct_service,
    output logic                             rx_end,
    output logic       [2:0]                 rx_end_status,
    output logic       [1:0]                 rx_end_info,
    output logic                             rx_age_valid,
    output logic       [15:0]                rx_age
);
    localparam int N_W   = 18;   // beats of a packet: 144 + 80·1366 + 64 < 2^17
    localparam int K_W   = 11;   // windows, symbols (≤ 1366)
    localparam int ST_W  = 16;   // trellis steps (≤ 16 + 8·4095 + 6)

    // ------------------------------------------------------------------ the blocks' wires
    logic drop;

    logic                         fft_in_valid, fft_in_next, fft_in_ready;
    logic signed [Y_BITS-1:0]     fft_in_re, fft_in_im;
    logic                         fft_out_valid;
    logic [5:0]                   fft_out_bin;
    logic signed [FFT_W_BITS-1:0] fft_out_re, fft_out_im;

    logic                         chan_valid;
    logic [4:0]                   chan_shift, chan_n;
    logic [GSUM_BITS-1:0]         chan_gsum;
    logic                         q_valid;
    logic [2:0]                   q_job;
    logic [27:0]                  q_value;
    logic                         ch_div_start, ch_div_done;
    logic [47:0]                  ch_div_num;
    logic [23:0]                  ch_div_den;
    logic [4:0]                   ch_div_qbits;
    logic [27:0]                  quot;
    logic                         hs_re, g_re;
    logic [5:0]                   hs_raddr, g_raddr;
    logic signed [HS_BITS-1:0]    hs_rdata_re, hs_rdata_im;
    logic [U_BITS-2:0]            g_rdata;

    logic                              bins_ok;
    logic                              pil_valid;
    logic signed [PILOT_BITS-1:0]      pil_re [4];
    logic signed [PILOT_BITS-1:0]      pil_im [4];
    logic                              theta_valid, move_early, move_late;
    logic signed [NCO_PHASE_BITS:0]    theta;
    logic signed [FIT_QBITS:0]         fit_delta;
    logic                              sy_cr_valid, sy_cr_take, sy_ca_valid;
    logic signed [CORDIC_IN_BITS-1:0]  sy_cr_re, sy_cr_im;
    logic                              phi_valid;
    logic signed [NCO_PHASE_BITS-1:0]  phi;
    logic                              soft_valid, soft_last;
    logic [5:0]                        soft_j;
    logic [1:0]                        soft_mod;
    logic signed [SOFT_BITS-1:0]       soft_v [6];

    logic                              tr_cr_valid, tr_cr_take, tr_ca_valid;
    logic signed [CORDIC_IN_BITS-1:0]  tr_cr_re, tr_cr_im, c_re, c_im;
    logic                              c_valid, c_busy, c_out;
    logic signed [ANGLE_BITS-1:0]      angle;
    logic                              tr_dv_req, tr_dv_gnt, tr_dv_done;
    logic [FIT_NUM_BITS-1:0]           tr_dv_num, d_num;
    logic [FIT_DEN_BITS-1:0]           tr_dv_den, d_den;
    logic [4:0]                        d_qbits;
    logic                              d_start, d_busy, d_done;

    logic                              slot_ok, v_valid, v_first, v_last;
    logic signed [SOFT_BITS-1:0]       v_a, v_b;
    logic                              dec_valid, dec_bit, dec_last;

    logic                              s_valid, s_fail, s_patient;
    logic [1:0]                        s_why, s_code, s_mod;
    logic [5:0]                        s_agree, s_disagree;
    logic [2:0]                        s_rate;
    logic [11:0]                       s_length;
    logic [10:0]                       s_symbols;
    logic [15:0]                       s_steps;

    logic                              o_end_valid, o_end_seed;

    // ------------------------------------------------------------------ packet state
    logic            open_q;        // a first beat taken, no mark yet
    logic            mark_q;        // a mark taken, the packet's end not yet out
    logic [1:0]      kind_q;        // the mark's kind
    logic            ended_q;       // the packet's end has been out
    logic [7:0]      tag_q;
    logic signed [NCO_PHASE_BITS-1:0] cfo_q;
    logic            start_q;       // rx_track's start and SIGNAL's block, from the first beat
    logic [N_W-1:0]  n_q;           // beats of the packet taken
    logic [K_W:0]    idx_q;         // the window being filled: 0 T1, 1 T2, 2 SIGNAL, 2 + k data k
    logic [N_W-1:0]  base_q;        // its nominal first beat (σ not added)
    logic [N_W-1:0]  free_q;        // data windows: beats below are taken freely
    logic signed [11:0] sigma_q;    // the tracker's moves: at most 1 + 1366 (θ0 and a data symbol's)
    logic [K_W:0]    thetas_q;      // θs seen (θ0 the first)
    logic [6:0]      fft_cnt_q;     // the FFT's flush: 0 when a flushed window may start
    logic            drain_q;       // the tail is taken and dropped
    logic            vseen_q, vok_q;
    logic [K_W-1:0]  symbols_q;     // the frame's data symbols
    logic [7:0]      ndbps_q;
    logic [K_W-1:0]  wd_q;          // data windows complete
    logic [ST_W-1:0] exp_q;         // trellis steps of the data windows complete (real ones)
    logic [ST_W-1:0] got_q;         // data trellis steps seen
    logic            sigblk_q;      // SIGNAL's 24 steps are past
    logic [2:0]      since_q;       // edges since the last data step, saturating
    logic            mend_q;        // the end by a mark, registered
    logic            ready_q;
    logic            fft_v_q, fft_next_q;
    logic signed [Y_BITS-1:0] fft_re_q, fft_im_q;
    logic [K_W:0]    ow_q;          // FFT output windows (0 T1, 1 T2, then symbols)
    logic [5:0]      ob_q;          // bins of the current output window

    // ------------------------------------------------------------------ the take
    wire take   = in_valid && ready_q;
    wire t_mark = take && in_end;
    wire t_samp = take && !in_end;
    wire t_new  = t_samp && in_first;

    function automatic logic [7:0] ndbps_of(input logic [2:0] r);
        case (r)
            3'd0: return 8'd24;
            3'd1: return 8'd36;
            3'd2: return 8'd48;
            3'd3: return 8'd72;
            3'd4: return 8'd96;
            3'd5: return 8'd144;
            3'd6: return 8'd192;
            default: return 8'd216;
        endcase
    endfunction

    // ------------------------------------------------------------------ the ends
    wire sig_ok    = s_valid && !s_fail;
    wire sig_done  = idx_q > (K_W+1)'(2);                 // SIGNAL's window complete
    // A mark taken before this cycle, SIGNAL complete, no data window: the end with rx_start.
    wire end_at_v  = sig_ok && mark_q && !ended_q && sig_done && wd_q == '0;
    assign rx_end  = s_valid && s_fail || o_end_valid || mend_q || end_at_v;
    assign drop    = rx_end;
    always_comb begin
        rx_end_status = 3'd0;
        rx_end_info   = 2'd0;
        if (s_valid && s_fail) begin
            rx_end_status = 3'd2;
            rx_end_info   = s_why;
        end else if (o_end_valid) begin
            rx_end_status = o_end_seed ? 3'd1 : 3'd0;
        end else if (mend_q || end_at_v) begin
            rx_end_status = kind_q[1] ? 3'd4 : 3'd3;
            rx_end_info   = kind_q[1] ? kind_q : 2'd0;
        end
    end
    assign rx_age_valid = rx_end && rx_end_status <= 3'd1;
    assign rx_age       = adc_count - end_stamp;

    // ------------------------------------------------------------------ next state
    logic            open_d, mark_d, ended_d, drain_d, vseen_d, vok_d, sigblk_d, mend_d, ready_d;
    logic [1:0]      kind_d;
    logic [N_W-1:0]  n_d, base_d, free_d;
    logic [K_W:0]    idx_d, thetas_d;
    logic signed [11:0] sigma_d;
    logic [6:0]      fft_cnt_d;
    logic [K_W-1:0]  symbols_d, wd_d;
    logic [7:0]      ndbps_d;
    logic [ST_W-1:0] exp_d, got_d;
    logic [2:0]      since_d;
    logic            win_last, fft_v_d, fft_next_d, dstep;
    logic [N_W-1:0]  start_cur, start_d;

    always_comb begin
        open_d    = open_q;
        mark_d    = mark_q;
        kind_d    = kind_q;
        ended_d   = ended_q || rx_end;
        drain_d   = drain_q;
        vseen_d   = vseen_q;
        vok_d     = vok_q;
        symbols_d = symbols_q;
        ndbps_d   = ndbps_q;
        n_d       = n_q;
        idx_d     = idx_q;
        base_d    = base_q;
        free_d    = free_q;
        sigma_d   = sigma_q;
        thetas_d  = thetas_q;
        wd_d      = wd_q;
        exp_d     = exp_q;
        got_d     = got_q;
        sigblk_d  = sigblk_q;
        fft_cnt_d = fft_cnt_q == '0 ? '0 : fft_cnt_q - 7'd1;
        fft_v_d   = 1'b0;
        fft_next_d = 1'b0;
        win_last  = 1'b0;

        // The tracker's θ and its move.
        if (open_q && theta_valid) begin
            thetas_d = thetas_q + 1'b1;
            if (move_early) sigma_d = sigma_q - 12'sd1;
            else if (move_late) sigma_d = sigma_q + 12'sd1;
        end

        // The window being filled starts at its base, + σ for the data symbols: SIGNAL's θ0
        // moves the windows from data symbol 1 on (SIGNAL's own is at 144).
        start_cur = idx_q < (K_W+1)'(3) ? base_q : base_q + N_W'(signed'(sigma_q));

        // A sample of the open packet.
        if (t_samp && open_q && !in_first) begin
            n_d = n_q + 1'b1;
            if (!drain_q && n_q >= start_cur && n_q < start_cur + N_W'(64)) begin
                fft_v_d = 1'b1;
                if (n_q == start_cur + N_W'(63)) begin
                    win_last   = 1'b1;
                    fft_next_d = idx_q < (K_W+1)'(2);
                end
            end
        end

        // The verdict (also after a mark: the end by the mark waits for it).
        if ((open_q || mark_q) && s_valid) begin
            vseen_d   = 1'b1;
            vok_d     = !s_fail;
            symbols_d = s_symbols;
            ndbps_d   = ndbps_of(s_rate);
            // An end by a mark needs fewer complete windows than symbols: each is full.
            if (!s_fail && wd_q != '0) exp_d = ST_W'(ndbps_of(s_rate));
        end

        // A window complete.
        if (win_last) begin
            idx_d = idx_q + 1'b1;
            if (idx_q >= (K_W+1)'(2)) begin
                fft_cnt_d = 7'd64;
                base_d    = base_q + N_W'(80);
            end else begin
                base_d    = idx_q == '0 ? N_W'(64) : N_W'(144);
            end
            // The next data window's free beats: its nominal place with σ before the last θ's
            // move, less one (data symbol 1: θ0's move not counted; later, σ as it is now: θ of
            // the window just complete is still to come).
            free_d = idx_q == (K_W+1)'(2) ? N_W'(144 + 80 - 1)
                   : base_q + N_W'(80) + N_W'(signed'(sigma_q)) - N_W'(1);
            if (idx_q >= (K_W+1)'(3)) begin
                wd_d = wd_q + 1'b1;
                // After the verdict's own update (a window may complete in its cycle).
                if (vseen_d && vok_d) exp_d = exp_d + ST_W'(ndbps_d);
            end
        end

        // The trellis steps of the data block.
        dstep = v_valid && sigblk_q;
        if (dstep) got_d = got_q + 1'b1;
        if (v_valid && v_last && !sigblk_q) sigblk_d = 1'b1;
        since_d = dstep ? 3'd0 : (since_q == 3'd7 ? 3'd7 : since_q + 3'd1);

        // The tail: the frame's last window, or a failed SIGNAL (0 symbols), once the verdict
        // is seen.
        if (open_q && vseen_d && wd_d >= symbols_d) drain_d = 1'b1;

        // A mark.
        if (t_mark) begin
            open_d = 1'b0;
            if (open_q) begin
                mark_d = 1'b1;
                kind_d = in_kind;
            end
        end

        // A first beat: a new packet.
        if (t_new) begin
            open_d    = 1'b1;
            mark_d    = 1'b0;
            ended_d   = 1'b0;
            drain_d   = 1'b0;
            vseen_d   = 1'b0;
            vok_d     = 1'b0;
            n_d       = N_W'(1);
            idx_d     = '0;
            base_d    = '0;
            free_d    = '0;
            sigma_d   = '0;
            thetas_d  = '0;
            wd_d      = '0;
            exp_d     = '0;
            got_d     = '0;
            sigblk_d  = 1'b0;
            fft_cnt_d = '0;
            fft_v_d   = 1'b1;        // T1's first sample
        end

        // The end by a mark, registered: before SIGNAL's window is complete, at once; after it,
        // with the verdict seen (not in its own cycle when the mark came earlier: end_at_v),
        // when the frame is not over and the complete windows' steps are decoded.
        mend_d = 1'b0;
        if (mark_d && !ended_d && !rx_end) begin
            if (!sig_done)
                mend_d = 1'b1;
            else if (vseen_d && vok_d && wd_q < symbols_d) begin
                if (exp_d == '0)
                    mend_d = !(s_valid && mark_q);
                else
                    mend_d = got_d == exp_d && since_d >= 3'd5;
            end
        end

        // The registered ready.
        start_d = base_d + N_W'(signed'(sigma_d));     // read only for data windows
        if (mark_d)
            ready_d = ended_d;
        else if (!open_d || drain_d)
            ready_d = 1'b1;
        else if (idx_d < (K_W+1)'(3))
            ready_d = 1'b1;
        else if (n_d < free_d)
            ready_d = 1'b1;
        else if (thetas_d < idx_d - (K_W+1)'(2) || (idx_d >= (K_W+1)'(4) && !vseen_d))
            ready_d = 1'b0;
        else
            ready_d = n_d != start_d || (fft_cnt_d == '0 && bins_ok);
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            open_q   <= 1'b0;
            mark_q   <= 1'b0;
            ended_q  <= 1'b1;
            ready_q  <= 1'b1;   // idle: no packet, a beat is taken at once
            mend_q   <= 1'b0;
            start_q  <= 1'b0;
            fft_v_q  <= 1'b0;
            fft_next_q <= 1'b0;
            drain_q  <= 1'b0;
            vseen_q  <= 1'b0;
            vok_q    <= 1'b0;
            ow_q     <= '0;
            ob_q     <= '0;
        end else begin
            open_q    <= open_d;
            mark_q    <= mark_d && !(ended_d || mend_d);
            kind_q    <= kind_d;
            ended_q   <= ended_d || mend_d;
            drain_q   <= drain_d;
            vseen_q   <= vseen_d;
            vok_q     <= vok_d;
            symbols_q <= symbols_d;
            ndbps_q   <= ndbps_d;
            n_q       <= n_d;
            idx_q     <= idx_d;
            base_q    <= base_d;
            free_q    <= free_d;
            sigma_q   <= sigma_d;
            thetas_q  <= thetas_d;
            wd_q      <= wd_d;
            exp_q     <= exp_d;
            got_q     <= got_d;
            sigblk_q  <= sigblk_d;
            since_q   <= since_d;
            fft_cnt_q <= fft_cnt_d;
            mend_q    <= mend_d;
            ready_q   <= ready_d;
            start_q   <= t_new;
            if (t_new) begin
                tag_q <= in_tag;
                cfo_q <= in_cfo;
            end
            fft_v_q    <= fft_v_d && !drop;
            fft_next_q <= fft_next_d;
            fft_re_q   <= in_re;
            fft_im_q   <= in_im;
            if (drop) begin
                ow_q <= '0;
                ob_q <= '0;
            end else if (fft_out_valid) begin
                ob_q <= ob_q + 6'd1;
                if (ob_q == 6'd63) ow_q <= ow_q + 1'b1;
            end
        end
    end

    assign in_ready = ready_q;

    // ------------------------------------------------------------------ the blocks
    assign fft_in_valid = fft_v_q;
    assign fft_in_next  = fft_next_q;
    assign fft_in_re    = fft_re_q;
    assign fft_in_im    = fft_im_q;

    fft64 u_fft (
        .clk, .rst_n, .drop,
        .in_valid(fft_in_valid), .in_next(fft_in_next), .in_re(fft_in_re), .in_im(fft_in_im),
        .in_ready(fft_in_ready),
        .out_valid(fft_out_valid), .out_bin(fft_out_bin), .out_re(fft_out_re), .out_im(fft_out_im)
    );

    wire to_chan = fft_out_valid && ow_q < (K_W+1)'(2);
    wire to_sym  = fft_out_valid && ow_q >= (K_W+1)'(2);

    rx_channel u_chan (
        .clk, .rst_n, .drop,
        .bin_valid(to_chan), .bin_t2(ow_q == (K_W+1)'(1)), .bin_idx(fft_out_bin),
        .bin_re(fft_out_re), .bin_im(fft_out_im),
        .chan_valid, .shift(chan_shift), .gsum(chan_gsum), .gsum_n(chan_n),
        .q_valid, .q_job, .q_value,
        .scale_req(sig_ok && s_mod != 2'd0), .scale_mod(s_mod),
        .div_start(ch_div_start), .div_num(ch_div_num), .div_den(ch_div_den),
        .div_qbits(ch_div_qbits), .div_done(ch_div_done), .div_quot(quot),
        .hs_re, .hs_raddr, .hs_rdata_re, .hs_rdata_im,
        .g_re, .g_raddr, .g_rdata
    );

    wire scale_valid = q_valid && (q_job == 3'd0 || q_job[2]);

    rx_symbol u_sym (
        .clk, .rst_n, .drop,
        .bin_valid(to_sym), .bin_idx(fft_out_bin), .bin_re(fft_out_re), .bin_im(fft_out_im),
        .bins_ok,
        .chan_valid, .chan_shift, .chan_n,
        .hs_re, .hs_raddr, .hs_rdata_re, .hs_rdata_im,
        .g_re, .g_raddr, .g_rdata,
        .scale_valid, .scale_mod(q_job[2] ? q_job[1:0] : 2'd0), .scale_q(q_value[SQ_BITS-1:0]),
        .mod_valid(sig_ok), .data_mod(s_mod),
        .pil_valid, .pil_re, .pil_im, .theta_valid, .theta,
        .cr_valid(sy_cr_valid), .cr_re(sy_cr_re), .cr_im(sy_cr_im), .cr_take(sy_cr_take),
        .ca_valid(sy_ca_valid), .ca_angle(angle),
        .phi_valid, .phi, .soft_valid, .soft_j, .soft_mod, .soft_last, .soft_v
    );

    rx_track u_track (
        .clk, .rst_n, .drop, .start(start_q), .cfo_word(cfo_q), .ratio,
        .q_valid, .q_job, .q_value, .pil_valid, .pil_re, .pil_im,
        .theta_valid, .theta, .move_early, .move_late, .fit_delta,
        .cr_valid(tr_cr_valid), .cr_re(tr_cr_re), .cr_im(tr_cr_im), .cr_take(tr_cr_take),
        .ca_valid(tr_ca_valid), .ca_angle(angle),
        .dv_req(tr_dv_req), .dv_num(tr_dv_num), .dv_den(tr_dv_den), .dv_gnt(tr_dv_gnt),
        .dv_done(tr_dv_done), .dv_quot(quot)
    );

    cordic_share u_cshare (
        .clk, .rst_n, .drop,
        .a_valid(sy_cr_valid), .a_re(sy_cr_re), .a_im(sy_cr_im), .a_take(sy_cr_take),
        .out_a(sy_ca_valid),
        .b_valid(tr_cr_valid), .b_re(tr_cr_re), .b_im(tr_cr_im), .b_take(tr_cr_take),
        .out_b(tr_ca_valid),
        .c_valid, .c_re, .c_im, .c_busy, .c_out_valid(c_out)
    );

    cordic_iter u_cordic (
        .clk, .rst_n, .drop,
        .in_valid(c_valid), .in_re(c_re), .in_im(c_im),
        .busy(c_busy), .out_valid(c_out), .out_angle(angle)
    );

    div_share #(
        .A_NUM_W(48), .A_DEN_W(24), .B_NUM_W(FIT_NUM_BITS), .B_DEN_W(FIT_DEN_BITS),
        .NUM_W(FIT_NUM_BITS), .DEN_W(FIT_DEN_BITS), .QB_W(5)
    ) u_dshare (
        .clk, .rst_n, .drop,
        .a_start(ch_div_start), .a_num(ch_div_num), .a_den(ch_div_den), .a_qbits(ch_div_qbits),
        .a_done(ch_div_done),
        .b_req(tr_dv_req), .b_num(tr_dv_num), .b_den(tr_dv_den), .b_qbits(5'(FIT_QBITS)),
        .b_gnt(tr_dv_gnt), .b_done(tr_dv_done),
        .d_start, .d_num, .d_den, .d_qbits, .d_busy, .d_done
    );

    div_serial #(.NUM_W(FIT_NUM_BITS), .DEN_W(FIT_DEN_BITS), .Q_W(28)) u_div (
        .clk, .rst_n, .drop,
        .start(d_start), .num(d_num), .den(d_den), .qbits(d_qbits),
        .busy(d_busy), .done(d_done), .quot
    );

    rx_deint u_deint (
        .clk, .rst_n, .drop,
        .blk_valid(start_q || sig_ok),
        .blk_rate(start_q ? 2'd0 : s_code),
        .blk_steps(start_q ? 16'd24 : s_steps),
        .soft_valid, .soft_j, .soft_mod, .soft_last, .soft_v,
        .slot_ok, .v_valid, .v_first, .v_last, .v_a, .v_b
    );

    viterbi u_vit (
        .clk, .rst_n, .in_valid(v_valid), .in_first(v_first), .in_last(v_last),
        .in_a(v_a), .in_b(v_b), .out_valid(dec_valid), .out_bit(dec_bit), .out_last(dec_last)
    );

    rx_signal u_sig (
        .clk, .rst_n, .drop, .v_valid, .v_first, .v_last, .v_a, .v_b,
        .dec_valid, .dec_bit, .dec_last,
        .sig_valid(s_valid), .sig_fail(s_fail), .sig_why(s_why), .sig_agree(s_agree),
        .sig_disagree(s_disagree), .sig_rate(s_rate), .sig_length(s_length),
        .sig_symbols(s_symbols), .sig_patient(s_patient), .sig_steps(s_steps),
        .sig_code(s_code), .sig_mod(s_mod)
    );

    rx_octets u_oct (
        .clk, .rst_n, .start(sig_ok), .length(s_length), .drop,
        .in_valid(dec_valid), .in_bit(dec_bit),
        .oct_valid, .oct_data, .oct_first, .oct_last, .oct_seed, .oct_service,
        .end_valid(o_end_valid), .end_seed(o_end_seed)
    );

    // ------------------------------------------------------------------ the outputs
    assign sig_valid   = s_valid;
    assign sig_tag     = tag_q;
    assign sig_fail    = s_fail;
    assign sig_symbols = s_symbols;
    assign sig_patient = s_patient;
    assign rx_start    = sig_ok;
    assign rx_rate     = s_rate;
    assign rx_length   = s_length;

    // Read for the checks below and for lint: what no output carries.
    wire unused = &{1'b0, chan_gsum, fit_delta, phi_valid, phi, dec_last, s_agree, s_disagree,
                    fft_in_ready, slot_ok};

`ifndef SYNTHESIS
    // The contracts the schedule assumes and this level can see (radio_phy::rx_data's
    // violations): a stream's first beat only with no packet open; a flushed window's sample
    // never into the FFT's skid. (A soft pass into a full slot stops rx_deint itself.)
    always_ff @(posedge clk) begin
        if (rst_n) begin
            if (t_new && open_q) begin
                $display("rx_data: a first beat with no mark before it");
                $stop;
            end
            if (fft_v_q && !fft_in_ready) begin
                $display("rx_data: a sample into the FFT's skid");
                $stop;
            end
        end
    end
`endif
endmodule

`default_nettype wire
