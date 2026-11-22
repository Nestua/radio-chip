// Provisional CFO latch and the delayed NCO path of the receiver front (ADR 0006, decisions 2
// and 3). Golden model: radio_phy::rx_front_fixed::RxFrontFixed::push, step 1 (the latch at a
// detection event) and step 2 (the step in effect at each sample, SampleTrace.step). Gate G3:
// make -C hw latch (with dc_block, det_a, cordic_arb and nco, hw/tb/latch_chain.sv).
//
// The latch. At an event at sample n ≥ GATE − 1 = LATCH_LAG + STF_PERIOD + COARSE_LEN − 1 = 135,
// the model takes the angle of Σ x(k)·conj(x(k+16)) over the COARSE_LEN products whose later
// sample is n − 119 … n − 24. det_a's C(m) covers the later samples m − 47 … m, so that sum is
// exactly C(n − 24) + C(n − 72): a delay line of C (ram_1w2r, DEPTH = 73, written with C(n) and
// read at n − 24 and n − 72 in the same cycle, so never the written address) and one adder. Both
// C are full windows for n ≥ 135 (n − 72 ≥ 63); the model skips earlier events (checked_sub),
// and so does the gate here. CORDIC → angle (2π/2^A) → step per sample: angle·2^(P−A−S),
// S = log2 STF_PERIOD (the model's per_sample, whose rounding is exact since P − A ≥ S).
//
// The NCO path runs D_SAMPLES behind: sample k is emitted (out_*) when sample k + D_SAMPLES
// enters, with the step in effect at k. Latch results are tagged with their event's sample and
// kept in a small FIFO until the emission of that sample, so a result that arrives early never
// changes an earlier sample's step: the order of the model is kept for any input timing that
// meets the one contract below.
//
// The CORDIC is not here: the latch asks the front's one CORDIC through its arbiter
// (cordic_arb, requester a: rq_valid with rq_re/rq_im, a pulse that is always taken) and gets
// a_valid with a_angle, in the order of its requests.
//
// Contract: a latch result must be back before its sample is emitted. From the sample's x edge
// the result is usable CORDIC_ITERS + 8 = 24 clocks later (x to det_a's input: 1; det_a's
// event, in whose cycle the request is made: 4; the arbiter's register: 1; the CORDIC: 17;
// the FIFO write: 1), and the sample is emitted D_SAMPLES samples after it: the D_SAMPLES
// sample periods in between must add up to 24 clocks, so with R clocks per sample at least,
// D_SAMPLES·R ≥ 24: 3 at R = 8 and 4 at R = 6 (both with nothing to spare), 5 at R = 5; the
// decision's schedule runs D_SAMPLES = 4 at R = 8.
// CHECK_TIMING = 1 stops simulation when the contract is broken; hardware has no
// such guard: a late result is applied one sample late, and one that arrives after its
// sample was emitted blocks later latches until the 8-bit tag comes round. Pick D_SAMPLES
// from the formula, not by trial.
//
// out_event is the detection event of the emitted sample (for the LTF search, which lives on
// the NCO path's samples: rx_search): det_a's flag is kept by the sample's tag from stage 4
// (4 clocks after the sample's edge) to the emission, inside the contract above.
//
// Inputs, all the same sample stream, each in its own pipeline stage of det_a:
//   y_*: det_a's input (dc_block output); c_*: det_a stage 2 (C of that sample); d_valid/event_i:
//   det_a stage 4. Never stalls.

`default_nettype none

module rx_latch
    import rx_front_rom_pkg::*;
#(
    parameter int D_SAMPLES    = 3,
    parameter bit CHECK_TIMING = 1'b1
) (
    input  wire logic                             clk,
    input  wire logic                             rst_n,     // synchronous, active low
    input  wire logic                             y_valid,
    input  wire logic signed [Y_BITS-1:0]         y_re,
    input  wire logic signed [Y_BITS-1:0]         y_im,
    input  wire logic                             c_valid,
    input  wire logic signed [2*DET_BITS+5:0]     c_re,
    input  wire logic signed [2*DET_BITS+5:0]     c_im,
    input  wire logic                             d_valid,
    input  wire logic                             event_i,
    output logic                                  rq_valid,
    output logic signed [CORDIC_IN_BITS-1:0]      rq_re,
    output logic signed [CORDIC_IN_BITS-1:0]      rq_im,
    input  wire logic                             a_valid,
    input  wire logic signed [ANGLE_BITS-1:0]     a_angle,
    output logic                                  out_valid,
    output logic signed [Y_BITS-1:0]              out_re,
    output logic signed [Y_BITS-1:0]              out_im,
    output logic signed [NCO_PHASE_BITS-1:0]      out_step,
    output logic                                  out_event
);
    localparam int C_W   = 2 * DET_BITS + 6;
    localparam int L_W   = C_W + 1;                     // C(n − 24) + C(n − 72)
    localparam int GATE  = LATCH_LAG + STF_PERIOD + COARSE_LEN;
    localparam int DEPTH = LATCH_LAG + AUTOCORR_LEN + 1;
    localparam int AW    = $clog2(DEPTH);
    localparam int P     = NCO_PHASE_BITS;
    localparam int A     = ANGLE_BITS;
    localparam int S     = $clog2(STF_PERIOD);
    localparam int UP    = P - A;
    localparam int TW    = 8;                           // sample tags, compared modulo 2^TW
    localparam int FD    = 1 << $clog2(D_SAMPLES + 2);  // FIFO depth (power of two)
    localparam int FAW   = $clog2(FD);

    // The structure relies on these; a different model needs a different latch.
    if (COARSE_LEN != 2 * AUTOCORR_LEN) begin : g_bad_coarse
        $error("rx_latch: COARSE_LEN must be two detector windows");
    end
    if ((1 << S) != STF_PERIOD || UP < S) begin : g_bad_step
        $error("rx_latch: per_sample needs STF_PERIOD = 2^S and P − A ≥ S");
    end
    if (L_W > CORDIC_IN_BITS) begin : g_bad_cordic
        $error("rx_latch: the latch sum does not fit the CORDIC input");
    end
    if (D_SAMPLES < 1 || D_SAMPLES + 2 > (1 << (TW - 1))) begin : g_bad_d
        $error("rx_latch: D_SAMPLES out of range");
    end

    // ---- C delay line: write C(n), read C(n − 24) and C(n − 72)
    logic [AW-1:0]        wa, ra_lag, ra_old;
    logic [2*C_W-1:0]     rd_lag, rd_old;
    always_ff @(posedge clk) begin
        if (!rst_n)       wa <= '0;
        else if (c_valid) wa <= (int'(wa) == DEPTH - 1) ? '0 : wa + 1'b1;
    end
    // wa − LATCH_LAG and wa − (DEPTH − 1) = wa + 1, modulo DEPTH
    assign ra_lag = (int'(wa) >= LATCH_LAG) ? AW'(int'(wa) - LATCH_LAG)
                                            : AW'(int'(wa) + DEPTH - LATCH_LAG);
    assign ra_old = (int'(wa) == DEPTH - 1) ? '0 : wa + 1'b1;

    ram_1w2r #(.W(2 * C_W), .DEPTH(DEPTH)) u_cline (
        .clk,
        .we(c_valid), .waddr(wa), .wdata({c_re, c_im}),
        .re_a(c_valid), .raddr_a(ra_lag), .rdata_a(rd_lag),
        .re_b(c_valid), .raddr_b(ra_old), .rdata_b(rd_old)
    );

    // ---- latch sum, registered the clock after the read: aligned with det_a stage 4
    logic                  rv;
    logic signed [L_W-1:0] l_re, l_im;
    always_ff @(posedge clk) begin
        if (!rst_n) rv <= 1'b0;
        else        rv <= c_valid;
        if (rv) begin
            l_re <= L_W'($signed(rd_lag[2*C_W-1:C_W])) + L_W'($signed(rd_old[2*C_W-1:C_W]));
            l_im <= L_W'($signed(rd_lag[C_W-1:0]))     + L_W'($signed(rd_old[C_W-1:0]));
        end
    end

    // ---- stage-4 sample counter (saturating at GATE − 1) and tag
    logic [$clog2(GATE)-1:0] nd;
    logic [TW-1:0]           td;
    logic                    issue;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            nd <= '0;
            td <= '0;
        end else if (d_valid) begin
            if (int'(nd) < GATE - 1) nd <= nd + 1'b1;
            td <= td + 1'b1;
        end
    end
    assign issue = d_valid && event_i && int'(nd) >= GATE - 1;

    // ---- the request to the CORDIC (the angle of the latch sum)
    assign rq_valid = issue;
    assign rq_re    = CORDIC_IN_BITS'(l_re);
    assign rq_im    = CORDIC_IN_BITS'(l_im);

    // ---- per_sample: angle (A bits) → step (P bits). The model rounds (angle·2^UP + 2^(S−1))
    // >> S; with UP ≥ S (checked above) the low S bits are zero and it is the exact shift.
    logic signed [P-1:0] a_step;
    assign a_step = P'(a_angle) <<< (UP - S);

    // ---- result FIFO: {tag, step, done}; issue pushes, the CORDIC fills in order, emission pops
    logic [TW-1:0] f_tag  [0:FD-1];
    logic [P-1:0]  f_step [0:FD-1];
    logic [FAW:0]  wp, dp, rp;                     // one extra bit: full vs empty
    logic          head_due, head_done;

    // ---- NCO-path delay: emit sample k when k + D_SAMPLES enters
    logic signed [Y_BITS-1:0] dl_re [0:D_SAMPLES-1];
    logic signed [Y_BITS-1:0] dl_im [0:D_SAMPLES-1];
    logic [$clog2(D_SAMPLES+1)-1:0] ny;           // saturates at D_SAMPLES
    logic [TW-1:0]                  te;           // tag of the sample to emit next
    logic                           emit;
    logic signed [P-1:0]            step;         // step in effect, as of the last emission
    assign emit      = y_valid && int'(ny) == D_SAMPLES;
    assign head_due  = rp != wp && f_tag[rp[FAW-1:0]] == te;
    assign head_done = rp != dp;

    always_ff @(posedge clk) begin
        if (issue) f_tag[wp[FAW-1:0]] <= td;
        if (a_valid) f_step[dp[FAW-1:0]] <= a_step;
        if (!rst_n) begin
            wp <= '0;
            dp <= '0;
            rp <= '0;
        end else begin
            if (issue) wp <= wp + 1'b1;
            if (a_valid) dp <= dp + 1'b1;
            if (emit && head_due) rp <= rp + 1'b1;
        end
    end

    // ---- the event flag of each sample in flight, by its tag (at most D_SAMPLES + 1 < FD)
    logic ev_mem [0:FD-1];
    always_ff @(posedge clk) begin
        if (d_valid) ev_mem[td[FAW-1:0]] <= event_i;
        if (emit) out_event <= ev_mem[te[FAW-1:0]];
    end

    always_ff @(posedge clk) begin
        if (y_valid) begin
            dl_re[0] <= y_re;
            dl_im[0] <= y_im;
            for (int i = 1; i < D_SAMPLES; i++) begin
                dl_re[i] <= dl_re[i-1];
                dl_im[i] <= dl_im[i-1];
            end
        end
        if (emit) begin
            out_re   <= dl_re[D_SAMPLES-1];
            out_im   <= dl_im[D_SAMPLES-1];
            out_step <= head_due ? f_step[rp[FAW-1:0]] : step;
        end
        if (!rst_n) begin
            ny        <= '0;
            te        <= '0;
            step      <= '0;
            out_valid <= 1'b0;
        end else begin
            if (y_valid && int'(ny) < D_SAMPLES) ny <= ny + 1'b1;
            if (emit) te <= te + 1'b1;
            if (emit && head_due) step <= f_step[rp[FAW-1:0]];
            out_valid <= emit;
        end
    end

    // ---- simulation checks of the contract (procedural $error: not for synthesis, see
    // ram_1w1r)
`ifndef SYNTHESIS
    if (CHECK_TIMING) begin : g_check
        always_ff @(posedge clk) if (rst_n) begin
            if (emit && head_due && !head_done)
                $error("rx_latch: the latch of sample tag %0d is not back before its emission (D_SAMPLES too small for this sample rate)", te);
            if (emit && rp != wp && $signed(f_tag[rp[FAW-1:0]] - te) < 0)
                $error("rx_latch: a latch result was skipped");
            if (emit && $signed(td - te) <= 0)
                $error("rx_latch: the event flag of sample tag %0d is not back before its emission", te);
            if (issue && wp - rp == (FAW+1)'(FD))
                $error("rx_latch: result FIFO overflow");
        end
    end
`endif
endmodule

`default_nettype wire
