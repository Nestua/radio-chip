// The hand-off of the receiver front (ADR 0008, decisions 1 to 3): the second history, the
// DC offset of a packet from its two long training symbols, the replay of the ADC's samples
// from T1's FFT window on (offset out, CFO word turned out, 13 bits), the stream's marks and
// the hold on the gain control. Golden model: radio_phy::rx_front_fixed::RxHandoffFixed for
// every value, radio_phy::rx_handoff::run_handoff for every cycle (its module text is this
// block's schedule). Gate G3: make -C hw handoff (hw/tb/handoff_chain.sv: behind rx_front).
//
// In: the front's own stream (in_valid, 12-bit I and Q; a sample in R clocks, R at least 8)
// and reset, so that this block's history holds sample n at address n mod HISTORY as
// rx_front's acq_fft assumes; the front's acq_valid with acq_fft and acq_cfo, which comes
// four cycles after the cycle of the sample it is applied at.
//
// The tick. Everything the model does at sample n is done in one cycle, six after the
// sample's (memory 1, rotator 4, buffer 1: the last cycle before sample n itself can be at
// the stream's pins), in the model's order: the verdict kept since the last tick; the hold
// down at the frame's last sample; a declaration (the open stream closed with its mark, what
// was read ahead dropped, the new packet opened, the hold up); the rail flags and the
// give_up (2 of the last 4 ADC samples on a rail, 24 of 32 once the verdict said patient);
// the overrun (the reader HISTORY samples behind). A beat taken in the tick's own cycle is
// counted before the tick compares the reader's position. hold is a register: it shows the
// tick's result from the next cycle.
//
// The verdict: sig_valid for a cycle with the tag of the stream's first beat, sig_fail or
// the number of data symbols (above MAX_SYMBOLS is MAX_SYMBOLS), and the patient bit. The
// first strobe since the last tick that carries the open packet's tag is kept and applied
// at the next tick, if that packet is still open then and has no verdict; a strobe in a
// tick's own cycle waits for the next one (and is compared with the tag as that tick
// leaves it).
//
// The offset, in cycles after the tick t of a declaration (rx_handoff.rs has the table):
// the 128 samples of the two long symbols addressed in t + 1 … t + 128, their two sums
// registered in t + 130, and dc_estimate.sv's 17 cycles from there: the offset registered
// in t + 147. The replay's first address goes out in that cycle.
//
// The replay: one address a cycle while the words on their way and the beats in the buffer
// are fewer than BUF (a registered count), each word less the offset through a rotator of
// nco.sv's stages whose phase is zero at the window's first sample and steps back by the
// CFO word, narrowed to 13 bits (half to even, saturating).
//
// Out: a stream with valid and ready. By precedence: the mark in its register (out_end with
// out_kind: 1 stopped, 2 acquisition, 3 overrun; 0 a packet read to its end, which is not
// in the register but follows the last sample taken); the buffer's oldest sample, from
// START_DELAY ticks after the declaration's on. out_first marks the first sample of T1's
// window; out_cfo and out_tag belong to it and mean nothing on other beats.
//
// After a reset nothing of a packet is left: no beat, no hold, the tag 0.

`default_nettype none

module rx_handoff
    import rx_front_rom_pkg::*;
#(
    parameter int W            = SEARCH_W,      // GI − B, 1 … GI_LEN: the back-off is GI_LEN − W
    localparam int POS_W       = $clog2(HISTORY),
    localparam int P           = NCO_PHASE_BITS,
    localparam int L           = NCO_LUT_BITS,
    localparam int Q           = NCO_AMP_BITS,
    localparam int X_W         = Y_BITS - 1     // the ADC's 12 bits
) (
    input  wire logic                     clk,
    input  wire logic                     rst_n,     // synchronous, active low
    input  wire logic                     in_valid,
    input  wire logic signed [X_W-1:0]    in_re,
    input  wire logic signed [X_W-1:0]    in_im,
    input  wire logic                     acq_valid,
    input  wire logic [POS_W-1:0]         acq_fft,
    input  wire logic signed [P-1:0]      acq_cfo,
    input  wire logic                     sig_valid,
    input  wire logic [7:0]               sig_tag,
    input  wire logic                     sig_fail,
    input  wire logic [10:0]              sig_symbols,
    input  wire logic                     sig_patient,
    input  wire logic                     out_ready,
    output logic                          out_valid,
    output logic                          out_end,
    output logic [1:0]                    out_kind,
    output logic                          out_first,
    output logic signed [Y_BITS-1:0]      out_re,
    output logic signed [Y_BITS-1:0]      out_im,
    output logic signed [P-1:0]           out_cfo,
    output logic [7:0]                    out_tag,
    output logic                          hold
);
    // The model's constants (rx_front_fixed.rs: HandoffConfig::default, HANDOFF_TAIL,
    // MAX_DATA_SYMBOLS; IEEE 802.11-2020 §17.3.5.4 for the longest PPDU).
    localparam int START_DELAY = 20;
    localparam int TAIL        = 8;
    localparam int MAX_SYMBOLS = 1366;
    localparam int SIGNAL_END  = 2 * LTF_SYMBOL_LEN + GI_LEN + LTF_SYMBOL_LEN;   // 208
    localparam int BACKOFF     = GI_LEN - W;
    localparam int CLIP_COUNT  = 2;       // of the last 4
    localparam int PAT_COUNT   = 24;      // of the last 32
    localparam int BUF         = 7;       // beats in the buffer and on their way
    localparam int EST_LEN     = 147;     // cycles from a declaration's tick to the offset
    localparam int C_W         = 17;      // sample counts from T1's window (see wr_off)
    localparam int WATCHDOG    = SIGNAL_END + 80 * MAX_SYMBOLS + BACKOFF;

    localparam logic [1:0] K_PACKET = 2'd0, K_STOPPED = 2'd1, K_ACQ = 2'd2, K_OVERRUN = 2'd3;

    // ---------------------------------------------------------------- the second history
    logic [POS_W-1:0]   wr_cnt;        // samples written, modulo HISTORY
    logic               rd_en;
    logic [POS_W-1:0]   rd_ptr;
    logic [2*X_W-1:0]   rd_word;
    always_ff @(posedge clk) begin
        if (!rst_n)        wr_cnt <= '0;
        else if (in_valid) wr_cnt <= wr_cnt + 1'b1;
    end
    ram_1w1r #(.W(2 * X_W), .DEPTH(HISTORY)) u_hist (
        .clk, .we(in_valid), .waddr(wr_cnt), .wdata({in_re, in_im}),
        .re(rd_en), .raddr(rd_ptr), .rdata(rd_word)
    );

    // ---------------------------------------------------------------- the tick's inputs
    logic [5:0] tk_sr;
    logic       tick;
    logic       rail_q;                 // the newest sample sits on a rail
    logic                 acq_p;        // an acquisition waits for its sample's tick
    logic [POS_W-1:0]     acq_fft_q;
    logic signed [P-1:0]  acq_cfo_q;
    assign tick = tk_sr[5];
    function automatic logic on_rail(input logic signed [X_W-1:0] v);
        return v == {1'b1, {(X_W-1){1'b0}}} || v == {1'b0, {(X_W-1){1'b1}}};
    endfunction
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            tk_sr <= '0;
            acq_p <= 1'b0;
        end else begin
            tk_sr <= {tk_sr[4:0], in_valid};
            if (acq_valid) acq_p <= 1'b1;
            else if (tick) acq_p <= 1'b0;
        end
        if (in_valid) rail_q <= on_rail(in_re) || on_rail(in_im);
        if (acq_valid) begin
            acq_fft_q <= acq_fft;
            acq_cfo_q <= acq_cfo;
        end
    end

    // ---------------------------------------------------------------- the packet's state
    logic                 open, vd, hold_q, patient, mark_v, kept, slot_v, slot_fail, slot_pat;
    logic [1:0]           mark_k;
    logic [7:0]           tag;
    logic [31:0]          rails;
    logic [5:0]           c32;          // rail flags set among the last 32
    logic [C_W-1:0]       slot_hend;
    logic [POS_W-1:0]     fft_q;
    logic signed [P-1:0]  cfo_q;
    // The newest sample written, counted from the window. It matters while the reader is
    // short of the stream's end, and then it is at most HISTORY past the reader (the
    // overrun): below WATCHDOG + TAIL + HISTORY = 110 024 + BACKOFF, inside C_W bits. Past
    // that it may wrap: nothing compares it once the last sample is taken.
    logic [C_W-1:0]       wr_off;
    logic [C_W-1:0]       tkn;          // samples the reader has taken
    logic [C_W-1:0]       ft;           // samples read from the history
    logic [C_W-1:0]       hend;         // the hold's end: the frame's last sample and one
    logic [C_W-1:0]       lastr;        // the stream's last sample and one
    logic [4:0]           start_cnt;    // ticks until the stream may hand on
    logic                 dc_v;
    logic signed [X_W-1:0] dc_re, dc_im;

    // ---------------------------------------------------------------- the pins
    logic [3:0]  fcnt;
    logic [2:0]  wp, rp;
    logic [2*Y_BITS:0] fifo [0:7];
    logic [2*Y_BITS:0] head;
    logic        gate_open, pkt_end, s_avail, xfer, x_mark, x_end, x_smp;
    assign head      = fifo[rp];
    assign gate_open = open && start_cnt == '0;
    assign pkt_end   = gate_open && tkn >= lastr;
    assign s_avail   = gate_open && !pkt_end && fcnt != '0;
    assign out_valid = mark_v || pkt_end || s_avail;
    assign out_end   = mark_v || pkt_end;
    assign out_kind  = mark_v ? mark_k : K_PACKET;
    assign out_first = !out_end && head[2*Y_BITS];
    assign out_re    = signed'(head[2*Y_BITS-1:Y_BITS]);
    assign out_im    = signed'(head[Y_BITS-1:0]);
    assign out_cfo   = cfo_q;
    assign out_tag   = tag;
    assign hold      = hold_q;
    assign xfer      = out_valid && out_ready;
    assign x_mark    = xfer && mark_v;
    assign x_end     = xfer && !mark_v && pkt_end;
    assign x_smp     = xfer && !mark_v && !pkt_end;

    // ---------------------------------------------------------------- the tick
    logic                open_e, mark_e, ap, stop, len, open1, pat1, hold1, hold2, mark1_v;
    logic [1:0]          mark1_k;
    logic [C_W-1:0]      tkn_n, hend1, lastr1, age0;
    logic [POS_W-1:0]    age0_w;
    logic [31:0]         rails_n;
    logic [5:0]          c32_n;
    logic [2:0]          c4_n;
    logic                pat_now, give_up, overrun, declare, flush;
    assign open_e   = open && !x_end;
    assign mark_e   = mark_v && !x_mark;
    assign tkn_n    = tkn + C_W'(x_smp);
    assign ap       = slot_v && open_e && !vd;
    assign stop     = ap && slot_fail;
    assign len      = ap && !slot_fail;
    assign open1    = open_e && !stop;
    assign hend1    = len ? slot_hend : hend;
    assign lastr1   = hend1 + C_W'(TAIL);
    assign pat1     = len ? slot_pat : patient;
    assign hold1    = hold_q && !stop;
    assign mark1_v  = mark_e || stop;
    assign mark1_k  = mark_e ? mark_k : K_STOPPED;
    assign hold2    = hold1 && !(open1 && wr_off >= hend1);
    assign declare  = tick && acq_p;
    assign pat_now  = acq_p ? (pat1 && hold2) : pat1;
    assign rails_n  = {rails[30:0], rail_q};
    assign c32_n    = c32 + 6'(rail_q) - 6'(rails[31]);
    assign c4_n     = 3'(rails_n[0]) + 3'(rails_n[1]) + 3'(rails_n[2]) + 3'(rails_n[3]);
    assign give_up  = pat_now ? c32_n >= 6'(PAT_COUNT) : c4_n >= 3'(CLIP_COUNT);
    // (At a declaration the new packet's registers win whatever this says.)
    assign overrun  = open1 && tkn_n < lastr1 && tkn_n + C_W'(HISTORY) <= wr_off;
    // The window's first sample is at most 285 samples old (rx_front.sv): the newest
    // sample written is wr_cnt − 1.
    assign age0_w   = wr_cnt - POS_W'(1) - acq_fft_q;
    assign age0     = C_W'(age0_w);
    // What was read ahead for a stream that is closed stays where it is, unseen (the pins
    // show the buffer of an open stream only), until the next declaration drops it.
    assign flush    = declare;

    // ---------------------------------------------------------------- the verdict's strobe
    logic            accept;
    logic [7:0]      tag_cmp;
    logic [10:0]     sym_c;
    logic [C_W-1:0]  sig_hend;
    assign tag_cmp  = declare ? tag + 8'd1 : tag;
    assign accept   = sig_valid && (tick || !kept) && sig_tag == tag_cmp;
    assign sym_c    = sig_symbols > 11'(MAX_SYMBOLS) ? 11'(MAX_SYMBOLS) : sig_symbols;
    // 80 data-symbol samples: 64 + 16.
    assign sig_hend = C_W'(SIGNAL_END + BACKOFF) + {sym_c, 6'b0} + C_W'({sym_c, 4'b0});

    // ---------------------------------------------------------------- the read port
    logic [7:0]  ec;                    // cycles since the declaration's tick, to EST_LEN
    logic        est_rd, fetch;
    logic [3:0]  occ;                   // words on their way and beats in the buffer
    logic [P-1:0] phase;
    assign est_rd = open && !dc_v && ec >= 8'd1 && ec <= 8'(2 * LTF_SYMBOL_LEN);
    assign fetch  = open && dc_v && occ < 4'(BUF) && ft < lastr && ft <= wr_off;
    assign rd_en  = est_rd || fetch;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // What a packet's state means hangs on open, and a declaration loads the rest
            // (the verdict's flag and slot, the patience, the counts, the offset's valid, the
            // credits): none of that needs a reset.
            open      <= 1'b0;
            hold_q    <= 1'b0;
            mark_v    <= 1'b0;
            tag       <= '0;
            rails     <= '0;
            c32       <= '0;
        end else begin
            // Between ticks only the taking of beats moves the packet's state.
            if (x_mark) mark_v <= 1'b0;
            if (x_end)  open   <= 1'b0;
            tkn <= tkn_n;
            if (in_valid) wr_off <= wr_off + 1'b1;
            if (fetch) begin
                ft     <= ft + 1'b1;
                rd_ptr <= rd_ptr + 1'b1;
                phase  <= phase - P'(cfo_q);
            end
            if (est_rd) rd_ptr <= rd_ptr + 1'b1;
            occ <= occ + 4'(fetch) - 4'(x_smp);
            // The estimate's clock: the offset is registered in EST_LEN, the replay reads
            // from the window's first sample again.
            if (open && !dc_v) begin
                ec <= ec + 1'b1;
                if (ec == 8'(EST_LEN - 1)) begin
                    dc_v   <= 1'b1;
                    rd_ptr <= fft_q;
                end
            end
            if (accept) begin
                slot_v    <= 1'b1;
                slot_fail <= sig_fail;
                slot_pat  <= sig_patient;
                slot_hend <= sig_hend;
                kept      <= 1'b1;
            end
            if (tick) begin
                slot_v  <= accept;
                kept    <= accept;
                rails   <= rails_n;
                c32     <= c32_n;
                open    <= open1 && !overrun;
                hend    <= hend1;
                lastr   <= lastr1;
                patient <= pat_now;
                vd      <= vd || len;
                hold_q  <= hold2 && !give_up && !overrun;
                mark_v  <= mark1_v || overrun;
                mark_k  <= mark1_v ? mark1_k : K_OVERRUN;
                if (start_cnt != '0) start_cnt <= start_cnt - 1'b1;
                if (acq_p) begin
                    // The declaration: the open stream's mark (a packet's if its last
                    // sample was taken), then the new packet.
                    mark_v    <= mark1_v || open1;
                    mark_k    <= mark1_v ? mark1_k : tkn_n >= lastr1 ? K_PACKET : K_ACQ;
                    open      <= 1'b1;
                    vd        <= 1'b0;
                    hold_q    <= !give_up;
                    tag       <= tag + 8'd1;
                    fft_q     <= acq_fft_q;
                    cfo_q     <= acq_cfo_q;
                    rd_ptr    <= acq_fft_q;
                    wr_off    <= age0;
                    tkn       <= '0;
                    ft        <= '0;
                    phase     <= '0;
                    // (The last sample follows the hold's end at the next tick, long
                    // before the stream may hand on.)
                    hend      <= C_W'(WATCHDOG);
                    start_cnt <= 5'(START_DELAY);
                    dc_v      <= 1'b0;
                    ec        <= 8'd1;
                end
            end
            if (flush) occ <= '0;
        end
    end

    // ---------------------------------------------------------------- the offset
    // The two sums: the word of the address of cycle t + k is here in t + k + 1.
    localparam int S_W = X_W + 6;                     // 18
    logic                    w_v, w_second;
    logic signed [X_W-1:0]   w_re, w_im;
    logic signed [S_W-1:0]   s1_re, s1_im, s2_re, s2_im;
    assign w_re = signed'(rd_word[2*X_W-1:X_W]);
    assign w_im = signed'(rd_word[X_W-1:0]);
    always_ff @(posedge clk) begin
        w_v      <= est_rd;
        w_second <= ec > 8'(LTF_SYMBOL_LEN);
        if (declare) begin
            s1_re <= '0;
            s1_im <= '0;
            s2_re <= '0;
            s2_im <= '0;
        end else if (w_v && !w_second) begin
            s1_re <= s1_re + S_W'(w_re);
            s1_im <= s1_im + S_W'(w_im);
        end else if (w_v) begin
            s2_re <= s2_re + S_W'(w_re);
            s2_im <= s2_im + S_W'(w_im);
        end
    end

    // The offset from the sums (dc_estimate.sv): started when both are registered, in
    // t + 130; registered 17 cycles later, in EST_LEN.
    logic est_start, est_done;
    assign est_start = open && !dc_v && ec == 8'(2 * LTF_SYMBOL_LEN + 2);
    dc_estimate u_est (
        .clk, .rst_n, .start(est_start), .cfo(cfo_q), .s1_re, .s1_im, .s2_re, .s2_im,
        .done(est_done), .dc_re, .dc_im
    );

    // ---------------------------------------------------------------- the replay's rotator
    // nco.sv's four stages with the phase given per sample and 13 bits out.
    localparam int PW   = Y_BITS + Q;                 // product sum width
    localparam int DROP = Q - 1;                      // Y + Q − 1 down to Y bits
    logic                     v0, v1, v2, v3, v4, f0, f1, f2, f3, f4;
    logic [P-1:0]             ph0;
    logic signed [Y_BITS-1:0] y0_re, y0_im, y1_re, y1_im, y2_re, y2_im, z_re, z_im, n_re, n_im;
    logic [1:0]               q1;
    logic [L-3:0]             r1;
    logic signed [Q-1:0]      c2, s2, tc, ts;
    logic signed [PW-1:0]     yr, yi, cc, ss, p3_re, p3_im;
    // The word less the offset: two 12-bit values, no saturation.
    assign y0_re = Y_BITS'(w_re) - Y_BITS'(dc_re);
    assign y0_im = Y_BITS'(w_im) - Y_BITS'(dc_im);
    assign tc = nco_cos(int'(r1));
    assign ts = nco_sin(int'(r1));
    assign yr = PW'(y2_re);
    assign yi = PW'(y2_im);
    assign cc = PW'(c2);
    assign ss = PW'(s2);
    round_sat #(.IN_W(PW), .SHIFT(DROP), .OUT_W(Y_BITS)) u_rs_re (.in(p3_re), .out(n_re));
    round_sat #(.IN_W(PW), .SHIFT(DROP), .OUT_W(Y_BITS)) u_rs_im (.in(p3_im), .out(n_im));
    always_ff @(posedge clk) begin
        if (!rst_n || flush) begin
            {v0, v1, v2, v3, v4} <= '0;
        end else begin
            {v0, v1, v2, v3, v4} <= {fetch, v0, v1, v2, v3};
        end
        f0    <= ft == '0;
        ph0   <= phase;
        // stage 1: the sample and the table address of its phase
        f1    <= f0;
        y1_re <= y0_re;
        y1_im <= y0_im;
        q1    <= ph0[P-1 -: 2];
        r1    <= ph0[P-3 -: L-2];
        // stage 2: the quarter-wave table, folded by quadrant
        f2    <= f1;
        y2_re <= y1_re;
        y2_im <= y1_im;
        unique case (q1)
            2'd0: begin c2 <= tc;  s2 <= ts;  end
            2'd1: begin c2 <= -ts; s2 <= tc;  end
            2'd2: begin c2 <= -tc; s2 <= -ts; end
            default: begin c2 <= ts; s2 <= -tc; end
        endcase
        // stage 3: the product; stage 4: narrowed
        f3    <= f2;
        p3_re <= yr * cc - yi * ss;
        p3_im <= yr * ss + yi * cc;
        f4    <= f3;
        z_re  <= n_re;
        z_im  <= n_im;
    end

    // ---------------------------------------------------------------- the buffer
    always_ff @(posedge clk) begin
        if (!rst_n || flush) begin
            wp   <= '0;
            rp   <= '0;
            fcnt <= '0;
        end else begin
            if (v4) wp <= wp + 1'b1;
            if (x_smp) rp <= rp + 1'b1;
            fcnt <= fcnt + 4'(v4) - 4'(x_smp);
        end
        if (v4) fifo[wp] <= {f4, z_re, z_im};
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (rst_n && tick && in_valid)
            $error("rx_handoff: a sample in the cycle of the tick before it (R below 7)");
        if (rst_n && v4 && fcnt == 4'd8)
            $error("rx_handoff: a word for a full buffer");
    end
`endif

    // Bits that only feed a comparison or a table address elsewhere.
    logic unused;
    assign unused = &{1'b0, ph0[P-L-1:0], est_done};
endmodule

`default_nettype wire
