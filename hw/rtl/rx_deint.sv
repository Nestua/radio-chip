// The deinterleaver and depuncturer (ADR 0010, decisions 1 and 6.2; R0-09's data path): a
// symbol's soft values in on-air order in, the Viterbi decoder's depunctured pairs out, a
// trellis step a clock. Golden model: radio_phy::rx_ofdm_fixed::deinterleave_soft per symbol,
// then radio_phy::viterbi::depuncture, the block's first `steps` pairs. Gate G3:
// make -C hw rx_deint. Architecture: docs/gates/rx_deint.md (G0), tests/g0_rx_deint.rs.
//
// Two banks of one write and one read port (the founder's choice, 2026-11-24). A subcarrier is
// a 48-bit word (lane i in bits 8i+7 .. 8i). Coded bit k of a symbol (IEEE 802.11-2020
// §17.3.5.7) lies on data subcarrier 3·(k mod 16) + ⌊⌊k/16⌋/N_BPSC⌋: its group ⌊j/3⌋ is
// k mod 16. Bank b holds the groups of parity b, 24 words a slot (word 3·⌊g/2⌋ + j mod 3), two
// slots; so two consecutive coded bits are always in different banks, and a step reads any two
// of them in one clock.
//
// The reader walks k with counters c = k mod 16, r = ⌊k/16⌋ as ⌊r/N_BPSC⌋ (rq) and r mod N_BPSC
// (rr); the lane is s·⌊rr/s⌋ + (rr − c) mod s, s = max(N_BPSC/2, 1). The rate's pattern
// (§17.3.5.6) says which of A and B a step keeps: 1/2 both; 2/3 both, A; 3/4 both, A, B. A kept
// value takes the next k, a stolen one is an erasure (0). A symbol is whole periods, so the
// phase is 0 at every symbol's start (G0); the reader does not force it.
//
// In: blk_valid with the block's code rate (0 1/2, 1 2/3, 2 3/4) and steps (SIGNAL 24; the data
// 16 + 8·LENGTH + 6, at most 32782: 16 bits), at most one block at a time: the next taken no
// earlier than the edge of the previous block's last v_valid (simulation stops otherwise). The block's symbols are the next ones
// written. The soft pass from rx_symbol (a subcarrier a beat, soft_j 0 .. 47 in order, its
// N_BPSC values in lanes 0 .. N_BPSC − 1, the others ignored; soft_mod; soft_last on the 48th;
// soft_last and soft_mod count only with soft_valid),
// written only while slot_ok is high at the beat that starts a symbol (simulation stops on a
// write to a full slot): slot_ok is high while a slot is free. The modulation of a symbol is
// its soft_mod at soft_last.
// Out, no ready (the decoder never stalls): v_valid with (v_a, v_b), v_first on the block's
// first step, v_last on its last. After the last step the rest of the symbol is not read and
// its slot is free. drop (and rst_n): no block, both slots empty, nothing in flight; the
// memories' contents left.
//
// Latencies (tb_rx_deint checks every one):
//   a symbol's first step: v_valid 2 edges after the latest of the edge that took its
//                          soft_last, the edge that took blk_valid (the block's first symbol),
//                          and the edge before the previous step's v_valid; then a step every
//                          edge to the symbol's end (or the block's);
//   slot_ok                high again from the edge before the v_valid of a symbol's last step
//                          (the step that frees its slot), when the other slot is not full.
// The decoder's contract (viterbi.sv: a block's first step n + 2 edges after the previous
// block's last) is the top level's: SIGNAL's verdict comes between the two blocks.

`default_nettype none

module rx_deint
    import rx_symbol_pkg::*;
(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         drop,
    // the block
    input  wire                         blk_valid,
    input  wire        [1:0]            blk_rate,
    input  wire        [15:0]           blk_steps,
    // the soft pass (rx_symbol)
    input  wire                         soft_valid,
    input  wire        [5:0]            soft_j,
    input  wire        [1:0]            soft_mod,
    input  wire                         soft_last,
    input  wire signed [SOFT_BITS-1:0]  soft_v [6],
    output logic                        slot_ok,
    // to the Viterbi decoder
    output logic                        v_valid,
    output logic                        v_first,
    output logic                        v_last,
    output logic signed [SOFT_BITS-1:0] v_a,
    output logic signed [SOFT_BITS-1:0] v_b
);
    localparam int LW = SOFT_BITS;      // a lane
    localparam int WW = 6 * LW;         // a word: a subcarrier
    localparam int SW = 24;             // words of a slot in a bank

    // ---- the slots -------------------------------------------------------------------------
    logic [1:0] full;                   // a symbol written and not yet read out
    logic [1:0] smod [2];               // its modulation
    logic       wr, rd;                 // the slot written next, the slot read next

    assign slot_ok = !full[wr];

    // ---- write: subcarrier j to bank (⌊j/3⌋ mod 2), word slot·24 + 3·⌊j/6⌋ + j mod 3 -------
    // ⌊j/3⌋ = ⌊43·j / 128⌋ for j ≤ 47 (the error 43/128 − 1/3 < 1/384 per unit; 47/384 < 1/3).
    logic [3:0]  wg;
    logic [1:0]  wm3;
    logic [5:0]  waddr;
    logic [WW-1:0] wdata;
    always_comb begin
        wg    = 4'((12'(soft_j) * 12'd43) >> 7);
        wm3   = 2'(soft_j - 6'(3 * 6'(wg)));
        waddr = 6'(wr ? SW : 0) + 6'(3 * 6'(wg[3:1])) + 6'(wm3);
        for (int i = 0; i < 6; i++) wdata[i*LW +: LW] = soft_v[i];
    end
    wire we0 = soft_valid && !wg[0];
    wire we1 = soft_valid &&  wg[0];

    // ---- the block and the walk -------------------------------------------------------------
    logic        active;                // a block whose last step has not been issued
    logic [1:0]  rate;
    logic [15:0] last_n, step_n;        // the block's last step; the next step's index
    logic [1:0]  phase;                 // the pattern's
    logic [3:0]  c;                     // k mod 16
    logic [2:0]  rr;                    // ⌊k/16⌋ mod N_BPSC
    logic [1:0]  rq;                    // ⌊⌊k/16⌋ / N_BPSC⌋; 3 past the symbol's end

    // N_BPSC − 1 of a modulation code.
    function automatic logic [2:0] nb_max(input logic [1:0] m);
        case (m)
            2'd0:    nb_max = 3'd0;
            2'd1:    nb_max = 3'd1;
            2'd2:    nb_max = 3'd3;
            default: nb_max = 3'd5;
        endcase
    endfunction

    // The next coded bit's counters.
    function automatic logic [8:0] inc(input logic [8:0] w, input logic [2:0] nm);
        logic [3:0] wc;
        logic [2:0] wr_;
        logic [1:0] wq;
        {wq, wr_, wc} = w;
        if (wc != 4'd15)    inc = {wq, wr_, wc + 4'd1};
        else if (wr_ != nm) inc = {wq, wr_ + 3'd1, 4'd0};
        else                inc = {wq + 2'd1, 3'd0, 4'd0};
    endfunction

    // A value's lane: s·⌊rr/s⌋ + (rr − c) mod s.
    function automatic logic [2:0] lane_of(input logic [3:0] wc, input logic [2:0] wr_,
                                           input logic [1:0] m);
        logic [1:0] r3, c3, d;
        case (m)
            2'd0, 2'd1: lane_of = wr_;                              // s = 1
            2'd2:       lane_of = {wr_[2:1], wr_[0] ^ wc[0]};       // s = 2
            default: begin                                          // s = 3
                r3 = wr_ >= 3'd3 ? 2'(wr_ - 3'd3) : 2'(wr_);
                c3 = 2'(wc % 4'd3);
                d  = r3 >= c3 ? r3 - c3 : 2'(3'(r3) + 3'd3 - 3'(c3));
                lane_of = (wr_ >= 3'd3 ? 3'd3 : 3'd0) + 3'(d);
            end
        endcase
    endfunction

    // A value's word in its bank (slot rd): 24·rd + 3·⌊c/2⌋ + rq.
    function automatic logic [5:0] word_of(input logic [2:0] c_hi, input logic [1:0] q,
                                           input logic slot);
        word_of = 6'(slot ? SW : 0) + 6'(3 * 6'(c_hi)) + 6'(q);
    endfunction

    // ---- issue: a step a clock while a block is open and its symbol's slot is full ---------
    logic [1:0] m_rd;
    logic [2:0] nm;
    logic       ka, kb;                 // which of A, B this step keeps
    logic [8:0] w0, w1, w2, wa, wb, wn;
    logic       iss, sym_end, blk_end;
    always_comb begin
        m_rd = smod[rd];
        nm   = nb_max(m_rd);
        // §17.3.5.6, Figure 17-9: 1/2 (1,1); 2/3 (1,1)(1,0); 3/4 (1,1)(1,0)(0,1).
        ka = !(rate == 2'd2 && phase == 2'd2);
        kb = phase == 2'd0 || (rate == 2'd2 && phase == 2'd2);
        w0 = {rq, rr, c};
        w1 = inc(w0, nm);
        w2 = inc(w1, nm);
        wa = w0;
        wb = ka ? w1 : w0;
        wn = (ka && kb) ? w2 : w1;
        iss     = active && full[rd];
        sym_end = wn[8:7] == 2'd3;
        blk_end = step_n == last_n;
    end

    // Bank b reads whichever kept value has c's parity b (never both: G0).
    wire        a_bank = wa[0];
    wire        b_bank = wb[0];
    wire        re0 = iss && ((ka && !a_bank) || (kb && !b_bank));
    wire        re1 = iss && ((ka &&  a_bank) || (kb &&  b_bank));
    wire [5:0]  raddr0 = (ka && !a_bank) ? word_of(wa[3:1], wa[8:7], rd) : word_of(wb[3:1], wb[8:7], rd);
    wire [5:0]  raddr1 = (ka &&  a_bank) ? word_of(wa[3:1], wa[8:7], rd) : word_of(wb[3:1], wb[8:7], rd);

    logic [WW-1:0] rdata0, rdata1;
    ram_1w1r #(.W(WW), .DEPTH(2 * SW)) u_bank0 (
        .clk, .we(we0), .waddr, .wdata, .re(re0), .raddr(raddr0), .rdata(rdata0)
    );
    ram_1w1r #(.W(WW), .DEPTH(2 * SW)) u_bank1 (
        .clk, .we(we1), .waddr, .wdata, .re(re1), .raddr(raddr1), .rdata(rdata1)
    );

    // ---- the step in flight: registered with the read, out an edge later --------------------
    logic       p_valid, p_first, p_last, p_ka, p_kb, p_abank, p_bbank;
    logic [2:0] p_alane, p_blane;

    function automatic logic signed [LW-1:0] pick(input logic [WW-1:0] wd, input logic [2:0] l);
        pick = wd[l*LW +: LW];
    endfunction

    always_ff @(posedge clk) begin
        if (!rst_n || drop) begin
            full    <= 2'b00;
            wr      <= 1'b0;
            rd      <= 1'b0;
            active  <= 1'b0;
            p_valid <= 1'b0;
            v_valid <= 1'b0;
            phase   <= 2'd0;
            {rq, rr, c} <= 9'd0;
        end else begin
            // write
            if (soft_valid && soft_last) begin
                full[wr] <= 1'b1;
                smod[wr] <= soft_mod;
                wr       <= !wr;
            end
            // the block
            if (blk_valid) begin
                active <= 1'b1;
                rate   <= blk_rate;
                last_n <= blk_steps - 16'd1;
                step_n <= 16'd0;
                phase  <= 2'd0;
                {rq, rr, c} <= 9'd0;
            end
            // issue
            p_valid <= iss;
            if (iss) begin
                p_first <= step_n == 16'd0;
                p_last  <= blk_end;
                p_ka    <= ka;
                p_kb    <= kb;
                p_abank <= a_bank;
                p_bbank <= b_bank;
                p_alane <= lane_of(wa[3:0], wa[6:4], m_rd);
                p_blane <= lane_of(wb[3:0], wb[6:4], m_rd);
                step_n  <= step_n + 16'd1;
                phase   <= (rate == 2'd0 || (rate == 2'd1 && phase == 2'd1) || phase == 2'd2)
                           ? 2'd0 : phase + 2'd1;
                if (sym_end || blk_end) begin
                    full[rd]    <= 1'b0;
                    rd          <= !rd;
                    {rq, rr, c} <= 9'd0;
                end else begin
                    {rq, rr, c} <= wn;
                end
                if (blk_end) active <= 1'b0;
            end
            // out
            v_valid <= p_valid;
        end
        if (p_valid) begin
            v_first <= p_first;
            v_last  <= p_last;
            v_a     <= p_ka ? pick(p_abank ? rdata1 : rdata0, p_alane) : '0;
            v_b     <= p_kb ? pick(p_bbank ? rdata1 : rdata0, p_blane) : '0;
        end
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (rst_n && !drop) begin
            if (soft_valid && full[wr]) begin
                $error("rx_deint: soft values written to a full slot");
                $stop;
            end
            if (soft_valid && soft_j != 6'(3 * 6'(wg) + 6'(wm3))) begin
                $error("rx_deint: soft_j %0d split as %0d·3 + %0d", soft_j, wg, wm3);
                $stop;
            end
            if (blk_valid && (active || blk_steps == 16'd0 || blk_rate == 2'd3)) begin
                $error("rx_deint: a block while one is open, or of no steps, or rate 3");
                $stop;
            end
            if (iss && ka && kb && a_bank == b_bank) begin
                $error("rx_deint: two reads of one bank in a step");
                $stop;
            end
        end
    end
`endif
endmodule

`default_nettype wire
