// Soft-decision Viterbi decoder for the 802.11 K = 7, (133, 171)₈ code (IEEE 802.11-2024
// §17.3.5.6), one trellis step a clock, survivors by register exchange (ADR 0010, decision 4).
// Golden model: radio_phy::viterbi::decode_windowed with window TRACEBACK_DEPTH = 96 and
// Termination::ZeroAfter(steps). Gate G3: make -C hw viterbi. G0: docs/gates/viterbi.md
// (2026-11-23, the RTL's G0).
//
// Input: one depunctured pair (A, B) a step, 8-bit soft values in ±127 (0 an erasure, −128
// never given), positive = "0 more likely". in_first on a block's first step, in_last on its
// last (the end of the tail: the data path does not feed the pad). Never stalls; idle cycles
// between steps are allowed anywhere.
//
// Output, a bit an edge at most, in order:
// - the bit of step t − 95, decided by the best state after step t (ties: the lowest state),
//   LAT = 4 edges after the edge that took step t, for 95 ≤ t < steps − 1;
// - at the last step, the min(96, steps) bits not yet decided, from state 0's path (the
//   encoder's state after the tail), one an edge from LAT edges after the edge that took the
//   last step; out_last on the final one.
// Contract: a block's first step comes no earlier than n + 2 edges after the edge that took
// the previous block's last step, n = min(96, steps of that block): the flush register is
// loaded two edges after a last step, and a block of one step must not load it before the
// previous flush is out. (The data path's blocks are hundreds of clocks apart.)
//
// Arithmetic (G0):
// - path metrics are 13-bit and compared modulo 2^13: the spread of the 64 metrics is at most
//   24·127 = 3048 after six steps and two candidates of one state differ by at most
//   3048 + 2·254 = 3556 < 4096, so the sign of the 13-bit difference is the sign of the true one;
// - no valid flags and no 14th bit: during a block's first six steps every reachable state's
//   predecessor with dropped bit 1 is unreachable (the encoder starts in state 0), so the
//   survivor bit is forced to 0 there, as the model's choice always is. Nothing is cleared at a
//   block's start: through six forced steps every state's metric descends from state 0's, so
//   whatever state 0 held before is an offset common to all 64, which no comparison modulo
//   2^13 sees (a mutation run confirmed that clearing them changes nothing). Unreachable
//   states' registers hold garbage that no output reads;
// - branch metric: Σ s·(1 − 2c) over (A, B). The two branches into a state come from
//   predecessors that differ in the oldest bit, a tap of both generators, so their outputs are
//   complements and their metrics opposite;
// - survivors: state s's register holds bits t − 6 down to t − 95 of its path (the newest six
//   are s itself); a step shifts in the survivor bit, the dropped bit of the chosen predecessor;
// - the best state: a tree of 63 compare-selects over the registered metrics, the lower index
//   kept on a tie, carrying each candidate's oldest register bit instead of its index, in three
//   registered stages of two levels (64 → 16 → 4 → 1).

`default_nettype none

module viterbi (
    input  wire              clk,
    input  wire              rst_n,     // synchronous, active low
    input  wire              in_valid,
    input  wire              in_first,
    input  wire              in_last,
    input  wire signed [7:0] in_a,
    input  wire signed [7:0] in_b,
    output logic             out_valid,
    output logic             out_bit,
    output logic             out_last
);
    localparam int S = 64;          // states
    localparam int D = 96;          // traceback window
    localparam int R = D - 6;       // register bits a state
    localparam int M = 13;          // metric bits

    // Outputs (A, B) of the branch from p0 = {s[4:0], 0} into state s; the input bit is s[5].
    function automatic logic [1:0] code_of(input logic [5:0] s);
        logic [6:0] w;
        w = {s, 1'b0};
        return {^(w & 7'o133), ^(w & 7'o171)};
    endfunction

    // ---- input stage: the two branch-metric magnitudes, the step's index (saturating at 95) ----
    logic                s1_valid, s1_last;
    logic signed [8:0]   bm_pp, bm_pm;     // A + B, A − B
    logic [6:0]          s1_idx, idx_next;

    always_comb begin
        if (in_first) idx_next = 7'd0;
        else if (s1_idx == 7'(D - 1)) idx_next = s1_idx;
        else idx_next = s1_idx + 7'd1;
    end

    always_ff @(posedge clk) begin
        // The index needs no reset: a block's first step carries in_first, which ignores it.
        if (!rst_n) begin
            s1_valid <= 1'b0;
        end else begin
            s1_valid <= in_valid;
            if (in_valid) begin
                s1_last  <= in_last;
                s1_idx   <= idx_next;
                bm_pp    <= 9'(in_a) + 9'(in_b);
                bm_pm    <= 9'(in_a) - 9'(in_b);
            end
        end
    end

    // ---- add-compare-select and register exchange ----
    logic [M-1:0] metric [S];
    logic [R-1:0] path   [S];
    wire          forced = s1_idx < 7'd6;

    for (genvar s = 0; s < S; s++) begin : g_acs
        localparam logic [1:0] C = code_of(6'(s));
        localparam int P0 = (s << 1) & (S - 1);
        localparam int P1 = P0 | 1;
        logic signed [8:0] bm;
        logic [M-1:0]      m0, m1, diff;
        logic              d;
        always_comb begin
            case (C)
                2'b00:   bm = bm_pp;
                2'b01:   bm = bm_pm;
                2'b10:   bm = -bm_pm;
                default: bm = -bm_pp;
            endcase
            m0   = metric[P0] + M'(bm);
            m1   = metric[P1] - M'(bm);
            diff = m1 - m0;
            // Ties keep the predecessor whose dropped bit is 0.
            d    = !forced && !diff[M-1] && diff != '0;
        end
        // State 0's metric is reset only so that a four-valued simulation after power-up is not
        // X for ever (every metric descends from it); the logic does not need it (G0 item 1).
        always_ff @(posedge clk) begin
            if (s == 0 && !rst_n) begin
                metric[s] <= '0;
            end else if (s1_valid) begin
                metric[s] <= d ? m1 : m0;
                path[s]   <= d ? {path[P1][R-2:0], 1'b1} : {path[P0][R-2:0], 1'b0};
            end
        end
    end

    // A step was taken at the last edge: its decision is due when its index has reached 95; a
    // last step starts the flush with n = index + 1 bits, whose first bit takes the output in
    // the cycle that step's own decision would have had (the flush has the output's priority).
    logic       acs_tag, acs_last;
    logic [6:0] acs_n;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            acs_tag  <= 1'b0;
            acs_last <= 1'b0;
        end else begin
            acs_tag  <= s1_valid && s1_idx == 7'(D - 1);
            acs_last <= s1_valid && s1_last;
            acs_n    <= s1_idx + 7'd1;
        end
    end

    // ---- best state: three stages of two compare-select levels ----
    // A candidate is (metric, oldest register bit); the right (higher) one wins only when
    // strictly better, modulo 2^M.
    function automatic logic right_wins(input logic [M-1:0] l, input logic [M-1:0] r);
        logic [M-1:0] dd;
        dd = r - l;
        return !dd[M-1] && dd != '0;
    endfunction

    logic [M-1:0] ma [16];
    logic         ba [16];
    logic [M-1:0] mb [4];
    logic         bb [4];
    logic         va, vb;

    always_ff @(posedge clk) begin
        for (int k = 0; k < 16; k++) begin
            logic [M-1:0] lm, rm;
            logic         lb, rb;
            lm = right_wins(metric[4*k], metric[4*k+1]) ? metric[4*k+1] : metric[4*k];
            lb = right_wins(metric[4*k], metric[4*k+1]) ? path[4*k+1][R-1] : path[4*k][R-1];
            rm = right_wins(metric[4*k+2], metric[4*k+3]) ? metric[4*k+3] : metric[4*k+2];
            rb = right_wins(metric[4*k+2], metric[4*k+3]) ? path[4*k+3][R-1] : path[4*k+2][R-1];
            ma[k] <= right_wins(lm, rm) ? rm : lm;
            ba[k] <= right_wins(lm, rm) ? rb : lb;
        end
        for (int k = 0; k < 4; k++) begin
            logic [M-1:0] lm, rm;
            logic         lb, rb;
            lm = right_wins(ma[4*k], ma[4*k+1]) ? ma[4*k+1] : ma[4*k];
            lb = right_wins(ma[4*k], ma[4*k+1]) ? ba[4*k+1] : ba[4*k];
            rm = right_wins(ma[4*k+2], ma[4*k+3]) ? ma[4*k+3] : ma[4*k+2];
            rb = right_wins(ma[4*k+2], ma[4*k+3]) ? ba[4*k+3] : ba[4*k+2];
            mb[k] <= right_wins(lm, rm) ? rm : lm;
            bb[k] <= right_wins(lm, rm) ? rb : lb;
        end
    end

    logic best_bit;
    always_comb begin
        logic [M-1:0] lm, rm;
        logic         lb, rb;
        lm = right_wins(mb[0], mb[1]) ? mb[1] : mb[0];
        lb = right_wins(mb[0], mb[1]) ? bb[1] : bb[0];
        rm = right_wins(mb[2], mb[3]) ? mb[3] : mb[2];
        rb = right_wins(mb[2], mb[3]) ? bb[3] : bb[2];
        best_bit = right_wins(lm, rm) ? rb : lb;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            va <= 1'b0;
            vb <= 1'b0;
        end else begin
            va <= acs_tag;
            vb <= va;
        end
    end

    // ---- flush: state 0's path after the last step, a bit an edge ----
    // The window's 96 bits are state 0's register above the six bits of state 0 itself; flushed
    // bit k of n is bit fcnt = n − 1 − k. Captured the edge after the last step's update,
    // before a next block's first step can change the register.
    logic [D-1:0] fvec;
    logic [6:0]   fcnt;
    logic         fstart, factive;
    wire          fbit = fvec[fcnt];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            fstart  <= 1'b0;
            factive <= 1'b0;
        end else begin
            fstart <= acs_last;
            if (acs_last) begin
                fvec <= {path[0], 6'b0};
                fcnt <= acs_n - 7'd1;
            end
            if (fstart) factive <= 1'b1;
            else if (factive && fcnt == 7'd0) factive <= 1'b0;
            if (factive) fcnt <= fcnt - 7'd1;
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_last  <= 1'b0;
        end else if (factive) begin
            out_valid <= 1'b1;
            out_bit   <= fbit;
            out_last  <= fcnt == 7'd0;
        end else begin
            out_valid <= vb;
            out_bit   <= best_bit;
            out_last  <= 1'b0;
        end
    end
endmodule

`default_nettype wire
