// LTF search of the receiver front (ADR 0006; ADR 0008, decision 4): the energy of the search
// correlator's outputs, its running maximum, and the candidate handed to the decision engine.
// Golden model: radio_phy::rx_front_fixed::RxFrontFixed::search_step, with the search control
// of push (step 1: an event opens the search or moves its deadline) and of apply_pending (an
// acquisition closes it). Gate G3: make -C hw search (the chain of hw/tb/search_chain.sv).
//
// Arithmetic. e(j) = Σ |c(j + k)|², k < W = GI − B, c the correlator's output of the window
// that starts at position j + k: a running sum (plus the new square, minus the one W back),
// exact, so equal to the model's fresh sum. |c|² needs 2·CORR_BITS bits (both components at
// −2^(K−1)), e four more for W ≤ 16.
//
// Timeline. The search lives on the NCO path's samples. s_valid marks a sample leaving
// rx_latch, s_event its detection event (rx_latch.out_event); both go down PATH_DELAY clocks
// (nco 4 and ltf_corr 15, counted from cycle to cycle) to the cycle C of that sample's
// correlator output, which exists from the 128th sample on. From there (the schedule of
// radio_phy::rx_decision allows these three):
//   C + 1  the squares' sum, registered
//   C + 2  e, registered; at the energy ring's write port in this cycle (address j mod RING)
//   C + 3  the state after the sample: out_valid, searching, and acc_valid with its fields
//
// State, per sample and in the model's order — the event, the position, the end of a decision:
//   wait   WAIT0 = 126 + W + SEARCH_OFFSET at the opening event, down to 0: the position
//          j = n − 126 − W has reached the search's first (the event's sample + SEARCH_OFFSET).
//   left   LEFT0 = WAIT0 + SEARCH_LEN − 1 at every event, down to 0: j + 1 has reached the
//          deadline, the search closes after this sample. (The model's deadline; it is not
//          handed to the decision, which has no use for it.)
//   best   the running maximum (the first of equal energies) and its age in positions; it is
//          accepted when 63 + W old, and the maximum restarts at the next position.
//   busy   a decision in flight: L samples from the acceptance, counted here. `acquired`
//          (a strobe from the decision engine, any time before the search's own sample n + L;
//          in the detector's timeline the result is due D samples and 20 clocks earlier)
//          makes the end of the decision close the search — also one opened at that very
//          sample, as in the model. No strobe: not acquired.
// The model accepts only with no decision in flight. With L ≤ 63 + W that never decides
// anything (after an acceptance at sample n the next maximum cannot be 63 + W old before
// n + 64 + W), so there is no such gate here: an elaboration check on L, and a simulation
// check. It also makes every accepted maximum exactly 63 + W old, which is what keeps
// e(imax) … e(imax + W) in the ring's 81 words while the decision reads them (oldest first:
// rx_decision).
//
// Handed to the decision with acc_valid: acc_n, the sample of the acceptance, acc_imax, the
// maximum's position, and acc_opened, the opening event's sample, all modulo 2^POS_W (the
// history's addresses; acc_opened is a tag: a search under a tone stays open without bound);
// acc_ring, the ring address of e(imax); acc_emax. ring_re/ring_addr/ring_data: the ring's
// read port (ram_1w1r: never the address being written, the data for one cycle).
//
// Stream: never stalls; one sample in 8 clocks at most (ltf_corr). CHECK_TIMING = 1 stops
// simulation when a correlator output comes without its sample's tick or a tick from the
// 128th sample on without a correlator output (PATH_DELAY wrong, or samples too close), on an
// acceptance with a decision in flight, and on `acquired` without one.

`default_nettype none

module rx_search
    import rx_front_rom_pkg::*;
#(
    parameter int W            = SEARCH_W,           // GI − B energy positions, 1 … GI_LEN
    parameter int L            = DECISION_LATENCY,   // samples from an acceptance to its result
    parameter int PATH_DELAY   = 19,                 // s_valid to c_valid, in cycles
    parameter bit CHECK_TIMING = 1'b1,
    localparam int E_W         = 2 * CORR_BITS + 4,
    localparam int POS_W       = $clog2(HISTORY),
    localparam int RING        = LTF_SYMBOL_LEN + GI_LEN + 1,
    localparam int RA_W        = $clog2(RING)
) (
    input  wire logic                         clk,
    input  wire logic                         rst_n,     // synchronous, active low
    input  wire logic                         s_valid,
    input  wire logic                         s_event,
    input  wire logic                         c_valid,
    input  wire logic signed [CORR_BITS-1:0]  c_re,
    input  wire logic signed [CORR_BITS-1:0]  c_im,
    input  wire logic                         acquired,
    output logic                              e_valid,
    output logic [E_W-1:0]                    e,
    output logic                              out_valid,
    output logic                              searching,
    output logic                              acc_valid,
    output logic [POS_W-1:0]                  acc_n,
    output logic [POS_W-1:0]                  acc_imax,
    output logic [RA_W-1:0]                   acc_ring,
    output logic [E_W-1:0]                    acc_emax,
    output logic [POS_W-1:0]                  acc_opened,
    input  wire logic                         ring_re,
    input  wire logic [RA_W-1:0]              ring_addr,
    output logic [E_W-1:0]                    ring_data
);
    localparam int P_W     = 2 * CORR_BITS;                               // |c|²
    localparam int WAIT0   = 2 * LTF_SYMBOL_LEN - 2 + W + SEARCH_OFFSET;
    localparam int LEFT0   = WAIT0 + SEARCH_LEN - 1;
    localparam int ACCEPT  = LTF_SYMBOL_LEN + W - 1;                      // the maximum's age
    localparam int CW      = $clog2(LEFT0 + 1);
    localparam int AGE_W   = $clog2(ACCEPT + 1);
    localparam int LW      = $clog2(L + 1);
    localparam int NP_W    = $clog2(W + 1);

    if (W < 1 || W > GI_LEN) begin : g_bad_w
        $error("rx_search: W must be 1 … GI_LEN");
    end
    if (L < 1 || L > ACCEPT) begin : g_bad_l
        $error("rx_search: L must be 1 … 63 + W (a decision ends before the next acceptance)");
    end
    if (PATH_DELAY < 1) begin : g_bad_delay
        $error("rx_search: PATH_DELAY must be at least 1");
    end
    if ((1 << POS_W) < RING + L) begin : g_bad_pos
        $error("rx_search: positions modulo 2^POS_W must tell a decision's samples apart");
    end

    // ---- the sample's tick and event, down to the correlator's output and two cycles on
    logic [PATH_DELAY+1:0] tk, ev;       // bit k: s_valid, s_event k + 1 cycles ago
    logic                  t_c, t_s, ev_s;
    always_ff @(posedge clk) begin
        if (!rst_n) tk <= '0;
        else        tk <= {tk[PATH_DELAY:0], s_valid};
        ev <= {ev[PATH_DELAY:0], s_event};
    end
    assign t_c  = tk[PATH_DELAY-1];      // cycle C
    assign t_s  = tk[PATH_DELAY+1];      // cycle C + 2: the state takes the sample
    assign ev_s = ev[PATH_DELAY+1];

    // ---- C → C + 1: |c|²
    logic                     p_valid;
    logic [P_W-1:0]           pair;
    logic signed [P_W-1:0]    sq_re, sq_im;
    assign sq_re = P_W'(c_re) * P_W'(c_re);
    assign sq_im = P_W'(c_im) * P_W'(c_im);
    always_ff @(posedge clk) begin
        if (!rst_n) p_valid <= 1'b0;
        else        p_valid <= c_valid;
        if (c_valid) pair <= unsigned'(sq_re) + unsigned'(sq_im);
    end

    // ---- C + 1 → C + 2: the running sum of W squares
    logic [P_W-1:0]  sr [W];             // the last W squares, sr[0] the newest
    logic [NP_W-1:0] np;                 // squares in the sum, saturating at W
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            np      <= '0;
            e       <= '0;
            e_valid <= 1'b0;
        end else begin
            e_valid <= p_valid && int'(np) >= W - 1;
            if (p_valid) begin
                e <= e + E_W'(pair) - (int'(np) == W ? E_W'(sr[W-1]) : '0);
                if (int'(np) < W) np <= np + 1'b1;
            end
        end
        if (p_valid) begin
            sr[0] <= pair;
            for (int i = 1; i < W; i++) sr[i] <= sr[i-1];
        end
    end

    // ---- the energy ring: e(j) at j mod RING, written in cycle C + 2
    logic [RA_W-1:0] wa;
    always_ff @(posedge clk) begin
        if (!rst_n)       wa <= '0;
        else if (e_valid) wa <= (int'(wa) == RING - 1) ? '0 : wa + 1'b1;
    end
    ram_1w1r #(.W(E_W), .DEPTH(RING)) u_ring (
        .clk,
        .we(e_valid), .waddr(wa), .wdata(e),
        .re(ring_re), .raddr(ring_addr), .rdata(ring_data)
    );

    // ---- C + 2 → C + 3: the search
    logic [POS_W-1:0] npos, jpos;        // this sample's number, this energy's position
    logic             open, busy, acq_seen, best_v;
    logic [CW-1:0]    wait_q, left_q, wait_a, left_a;
    logic [E_W-1:0]   best_e;
    logic [POS_W-1:0] best_pos, open_pos;
    logic [RA_W-1:0]  best_ring;
    logic [AGE_W-1:0] age, age_a;
    logic [LW-1:0]    cnt;
    logic             opening, open_a, started, take, accept, dead, dec_end;

    always_comb begin
        // The event.
        opening = ev_s && !open;
        open_a  = open || ev_s;
        wait_a  = opening ? CW'(WAIT0) : (wait_q == '0 ? '0 : wait_q - 1'b1);
        left_a  = ev_s    ? CW'(LEFT0) : (left_q == '0 ? '0 : left_q - 1'b1);
        // The position.
        started = open_a && e_valid && wait_a == '0;
        take    = started && (!best_v || e > best_e);
        age_a   = take ? '0 : age + 1'b1;   // read only while a maximum is held: ≤ ACCEPT
        accept  = started && !take && int'(age_a) >= ACCEPT;
        dead    = open_a && left_a == '0;
        // The end of a decision in flight.
        dec_end = busy && int'(cnt) == 1;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            npos      <= '0;
            jpos      <= '0;
            open      <= 1'b0;
            busy      <= 1'b0;
            acq_seen  <= 1'b0;
            best_v    <= 1'b0;
            out_valid <= 1'b0;
            acc_valid <= 1'b0;
        end else begin
            out_valid <= t_s;
            acc_valid <= t_s && accept;
            if (acquired && busy) acq_seen <= 1'b1;
            if (e_valid) jpos <= jpos + 1'b1;
            if (t_s) begin
                npos <= npos + 1'b1;
                open <= open_a && !dead && !(dec_end && acq_seen);
                if (take)                   best_v <= 1'b1;
                else if (accept || opening) best_v <= 1'b0;
                if (accept) begin
                    busy <= 1'b1;
                end else if (dec_end) begin
                    busy     <= 1'b0;
                    acq_seen <= 1'b0;
                end
            end
        end
        if (t_s) begin
            wait_q <= wait_a;
            left_q <= left_a;
            age    <= age_a;
            if (opening) open_pos <= npos;
            if (take) begin
                best_e    <= e;
                best_pos  <= jpos;
                best_ring <= wa;
            end
            if (accept)    cnt <= LW'(L);
            else if (busy) cnt <= cnt - 1'b1;
            if (accept) begin
                acc_n      <= npos;
                acc_imax   <= best_pos;
                acc_ring   <= best_ring;
                acc_emax   <= best_e;
                acc_opened <= open_pos;
            end
        end
    end
    assign searching = open;

    // ---- simulation checks (procedural $error: not for synthesis, see ram_1w1r)
`ifndef SYNTHESIS
    if (CHECK_TIMING) begin : g_check
        logic [7:0] seen;                // ticks at the correlator's output, saturating
        always_ff @(posedge clk) begin
            if (!rst_n)                     seen <= '0;
            else if (t_c && seen != 8'hff)  seen <= seen + 1'b1;
            if (rst_n) begin
                if (c_valid && !t_c)
                    $error("rx_search: a correlator output without its sample's tick (PATH_DELAY)");
                if (t_c && !c_valid && int'(seen) >= 2 * LTF_SYMBOL_LEN - 1)
                    $error("rx_search: a sample without its correlator output (samples closer than ltf_corr takes them, or PATH_DELAY)");
                if (t_s && accept && busy)
                    $error("rx_search: an acceptance with a decision in flight");
                if (acquired && !busy)
                    $error("rx_search: acquired without a decision in flight");
            end
        end
    end
`endif
endmodule

`default_nettype wire
