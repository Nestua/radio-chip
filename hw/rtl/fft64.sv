// The receiver's 64-point forward FFT (IEEE 802.11-2020 §17.3.5.10, forward; R0-09's data path,
// ADR 0010 decision 1): radix 2, decimation in time, as a single-path delay-feedback pipeline
// fed in natural order (R2SDF; the founder's choice of 2026-11-23, docs/gates/fft64.md G0).
// Golden model: radio_phy::rx_ofdm_fixed::fft64_window_fixed (fft64_fixed on the input
// shifted into the word). Gate G3: make -C hw fft64.
//
// Why this computes the model bit for bit (G0, tests/g0_fft64_sdf.rs): label each position of
// the model's in-place array by its bit reverse. The input x[n] sits at label n, stage s pairs
// labels l and l + D (D = 32 >> s), and bin k leaves at label rev6(k). A stream in natural
// order meets each butterfly's two inputs D samples apart: stage s keeps D words of delay, the
// twiddle multiplies the later input, every butterfly is the model's (t = w·b exact,
// a·2^(T−2) ± t shifted right by T − 2, + 1 on the stages of HALVE, half to even, saturated to
// W bits). Labels travel with the data (each stage's output is D tokens late: label − D).
//
// Input: a window's 64 samples in time order, IN_BITS wide, with in_valid, gaps allowed
// anywhere. in_next with a window's 64th sample: the next window follows without a flush (T1
// then T2). Otherwise the block flushes the window out with a frame of 64 zero tokens, one a
// clock, in_ready low meanwhile. in_ready is for a reader whose valid is registered: one
// sample presented while in_ready is low is held (a skid of one) and enters after the flush;
// a second is a protocol error (simulation stops).
//
// Output: bins in bit-reversed order, each with its index, out_valid for one clock each.
// Never stalls. A window's last bin leaves 75 edges after the edge that took its 64th sample
// when the flush follows at once (63 tokens of delay, 12 pipeline registers behind the input
// register); with in_next, as the next window's samples push it out.
//
// drop (an abort; and rst_n): every token in flight dropped, the labels back to a frame's start, the
// delay lines left as they are (their contents reach no bin of a later window: G0's test).
//
// Pipeline: an input register, then per stage a twiddle register (the product) and a
// butterfly register. Stages 0 and 1 multiply by 1 and −j only (shifts); stages 2 and 3 by
// constants (products by the ROM's few magnitudes, selected); stages 4 and 5 by the ROM in a
// general product.

`default_nettype none

module fft64
    import fft64_rom_pkg::*;
(
    input  wire                       clk,
    input  wire                       rst_n,     // synchronous, active low
    input  wire                       drop,
    input  wire                       in_valid,  // not AXI: a beat offered while in_ready is low is consumed (the skid)
    input  wire                       in_next,
    input  wire signed [IN_BITS-1:0]  in_re,
    input  wire signed [IN_BITS-1:0]  in_im,
    output logic                      in_ready,  // for a registered valid: drop valid the cycle after it falls
    output logic                      out_valid,
    output logic [5:0]                out_bin,
    output logic signed [W_BITS-1:0]  out_re,
    output logic signed [W_BITS-1:0]  out_im
);
    localparam int FT = T_BITS - 2;         // the twiddle's fractional bits
    localparam int PW = W_BITS + T_BITS + 1; // w·b, exact
    localparam int SW = PW + 1;             // a·2^FT ± w·b

    localparam logic signed [T_BITS-1:0] ONE = T_BITS'(1 << FT);
    localparam logic signed [SW-1:0]     TOP = SW'((1 << (W_BITS - 1)) - 1);
    localparam logic signed [SW-1:0]     BOT = -TOP - SW'(1);

    // One token: the data and its tags. `cur`: the token's input frame is a window; `prev`:
    // the input frame before it was. A bin at label 0 left with its own window's 64th sample;
    // the others with the next frame's first 63 tokens.
    typedef struct packed {
        logic                     v;
        logic [5:0]               lab;
        logic                     cur;
        logic                     prev;
        logic signed [W_BITS-1:0] re;
        logic signed [W_BITS-1:0] im;
    } tok_t;

    function automatic logic [5:0] rev6(input logic [5:0] x);
        for (int b = 0; b < 6; b++) rev6[b] = x[5-b];
    endfunction

    // The model's j·step: j = rev6(label) mod 2^s, step = 32 >> s.
    function automatic int tw_index(input logic [5:0] lab, input int s);
        int j;
        j = 0;
        for (int b = 0; b < s; b++) j = j | (int'(lab[5-b]) << b);
        return j * (32 >> s);
    endfunction

    // v / 2^sh, half to even, saturated to W bits.
    function automatic logic signed [W_BITS-1:0] narrow(input logic signed [SW-1:0] v,
                                                       input int sh);
        logic signed [SW-1:0] fl, r;
        logic [SW-1:0] rem, half;
        logic up;
        fl   = v >>> sh;
        rem  = SW'(v - (fl <<< sh));
        half = SW'(1) << (sh - 1);
        up   = (rem > half) || (rem == half && fl[0]);
        r    = fl + SW'(up);
        if (r > TOP) return TOP[W_BITS-1:0];
        if (r < BOT) return BOT[W_BITS-1:0];
        return r[W_BITS-1:0];
    endfunction

    // x·w for the twiddles of stages 2 and 3: w is 0, ±ONE or ± one of the ROM's magnitudes
    // cos π/4, cos π/8, sin π/8; each branch a product by a constant.
    localparam logic signed [T_BITS-1:0] C8  = tw_re(8);
    localparam logic signed [T_BITS-1:0] C4R = tw_re(4);
    localparam logic signed [T_BITS-1:0] C4I = -tw_im(4);
    function automatic logic signed [PW-1:0] mulc(input logic signed [W_BITS-1:0] x,
                                                 input logic signed [T_BITS-1:0] w);
        logic signed [PW-1:0] xe;
        xe = PW'(x);
        case (w)
            ONE:   return xe <<< FT;
            -ONE:  return -(xe <<< FT);
            C8:    return xe * PW'(C8);
            -C8:   return -(xe * PW'(C8));
            C4R:   return xe * PW'(C4R);
            -C4R:  return -(xe * PW'(C4R));
            C4I:   return xe * PW'(C4I);
            -C4I:  return -(xe * PW'(C4I));
            default: return '0;  // 0, and no other value occurs in stages 2 and 3
        endcase
    endfunction

    // ---- Input: tokens from the window, the skid, or the flush ----------------------------

    logic [5:0] cnt;          // the next token's label
    logic       flushing;
    logic       prev_real;    // the last complete input frame was a window
    logic       skid_v;
    logic signed [IN_BITS-1:0] skid_re, skid_im;

    assign in_ready = !flushing && !skid_v;

    logic take_in, take_skid, tok_v, tok_next;
    logic signed [IN_BITS-1:0] tok_re, tok_im;
    always_comb begin
        take_skid = skid_v && !flushing;
        take_in   = in_valid && in_ready;
        tok_v     = flushing || take_skid || take_in;
        // The skid fills only during a flush, so its sample is a frame's label 0: in_next is
        // read at label 63 alone, never the skid's.
        tok_next  = in_next;
        tok_re    = flushing ? '0 : (take_skid ? skid_re : in_re);
        tok_im    = flushing ? '0 : (take_skid ? skid_im : in_im);
    end

    tok_t t_in;
    always_ff @(posedge clk) begin
        if (!rst_n || drop) begin
            cnt       <= '0;
            flushing  <= 1'b0;
            prev_real <= 1'b0;
            skid_v    <= 1'b0;
            t_in.v    <= 1'b0;
        end else begin
            if (in_valid && !in_ready && !skid_v) begin
                skid_v    <= 1'b1;
                skid_re   <= in_re;
                skid_im   <= in_im;
            end else if (take_skid) begin
                skid_v <= 1'b0;
            end
            t_in.v <= tok_v;
            if (tok_v) begin
                cnt <= cnt + 6'd1;
                if (cnt == 6'd63) begin
                    prev_real <= !flushing;
                    flushing  <= !flushing && !tok_next;
                end
            end
        end
        t_in.lab  <= cnt;
        t_in.cur  <= !flushing;
        t_in.prev <= prev_real;
        t_in.re   <= W_BITS'(tok_re) <<< IN_SHIFT;
        t_in.im   <= W_BITS'(tok_im) <<< IN_SHIFT;
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk)
        if (rst_n && !drop && in_valid && !in_ready && skid_v)
            $error("fft64: a second sample presented while in_ready is low");
`endif

    // ---- Six stages ------------------------------------------------------------------------

    tok_t st[7];  // st[s]: stage s's input tokens; st[6] the bins
    assign st[0] = t_in;

    for (genvar s = 0; s < 6; s++) begin : g_stage
        localparam int D  = 32 >> s;
        localparam int SH = FT + int'(HALVE[s]);

        // Twiddle register: the later input's product w·b, the earlier input as it is.
        logic                  pv, plater, pcur, pprev;
        logic [5:0]            plab;
        logic signed [PW-1:0]  pre, pim;

        logic                  later;
        logic signed [T_BITS-1:0] wr, wi;
        logic signed [PW-1:0]  mr, mi;
        logic signed [W_BITS-1:0] br, bi;
        always_comb begin
            later = st[s].lab[5-s];
            br    = st[s].re;
            bi    = st[s].im;
            wr    = tw_re(tw_index(st[s].lab, s));
            wi    = tw_im(tw_index(st[s].lab, s));
            if (s < 2) begin
                // w is 1 or −j (exact in Q(T, T − 2)): w·b by shifts.
                if (wi == '0) begin
                    mr = PW'(br) <<< FT;
                    mi = PW'(bi) <<< FT;
                end else begin
                    mr = PW'(bi) <<< FT;
                    mi = -(PW'(br) <<< FT);
                end
            end else if (s < 4) begin
                mr = mulc(br, wr) - mulc(bi, wi);
                mi = mulc(bi, wr) + mulc(br, wi);
            end else begin
                mr = PW'(wr) * PW'(br) - PW'(wi) * PW'(bi);
                mi = PW'(wr) * PW'(bi) + PW'(wi) * PW'(br);
            end
            if (!later) begin
                mr = PW'(br);
                mi = PW'(bi);
            end
        end

`ifndef SYNTHESIS
        // mulc knows only the ROM's magnitudes of stages 2 and 3; a changed ROM must not
        // multiply by 0 silently.
        if (s == 2 || s == 3) begin : g_check
            always_ff @(posedge clk)
                if (st[s].v && later &&
                    (mulc(16'sd1, wr) == '0 && wr != '0 || mulc(16'sd1, wi) == '0 && wi != '0))
                    $error("fft64: stage %0d twiddle (%0d, %0d) not in mulc", s, wr, wi);
        end
`endif

        always_ff @(posedge clk) begin
            if (!rst_n || drop) pv <= 1'b0;
            else                 pv <= st[s].v;
            plab   <= st[s].lab;
            plater <= later;
            pcur   <= st[s].cur;
            pprev  <= st[s].prev;
            pre    <= mr;
            pim    <= mi;
        end

        // Butterfly register and the delay line (D words, advancing on a token).
        logic signed [W_BITS-1:0] fr[D], fi[D];
        logic signed [W_BITS-1:0] push_re, push_im, o_re, o_im;
        logic signed [SW-1:0]     ar, ai;
        always_comb begin
            ar = SW'(fr[0]) <<< FT;
            ai = SW'(fi[0]) <<< FT;
            if (plater) begin
                o_re    = narrow(ar + SW'(pre), SH);
                o_im    = narrow(ai + SW'(pim), SH);
                push_re = narrow(ar - SW'(pre), SH);
                push_im = narrow(ai - SW'(pim), SH);
            end else begin
                // The first half of a group: the last group's difference out, this input in.
                o_re    = fr[0];
                o_im    = fi[0];
                push_re = pre[W_BITS-1:0];
                push_im = pim[W_BITS-1:0];
            end
        end

        always_ff @(posedge clk) begin
            if (pv) begin
                for (int i = 0; i < D - 1; i++) begin
                    fr[i] <= fr[i+1];
                    fi[i] <= fi[i+1];
                end
                fr[D-1] <= push_re;
                fi[D-1] <= push_im;
            end
        end

        tok_t q;
        always_ff @(posedge clk) begin
            if (!rst_n || drop) q.v <= 1'b0;
            else                 q.v <= pv;
            q.lab  <= plab - 6'(D);
            q.cur  <= pcur;
            q.prev <= pprev;
            q.re   <= o_re;
            q.im   <= o_im;
        end
        assign st[s+1] = q;
    end

    // ---- Output -----------------------------------------------------------------------------

    always_comb begin
        out_valid = st[6].v && (st[6].lab == 6'd0 ? st[6].cur : st[6].prev);
        out_bin   = rev6(st[6].lab);
        out_re    = st[6].re;
        out_im    = st[6].im;
    end
endmodule

`default_nettype wire
