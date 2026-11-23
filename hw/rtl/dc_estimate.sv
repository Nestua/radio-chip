// The DC offset of a packet from the sums of its two long training symbols (ADR 0008,
// decision 1). Golden model: radio_phy::rx_handoff::dc_offset_of_sums, which is
// radio_phy::rx_front_fixed::ltf_dc_offset_fixed in the form computed here. Gate G3:
// make -C hw dc_estimate.
//
// With e the rotator's table at 64 CFO words and half an address, A its amplitude and
// d = A − e: S = A·Σ₂ − e·Σ₁, the offset is S·d* / (64·|d|²) rounded half to even and
// saturated to the ADC's 12 bits; zero when 256·|d|² < A² (the CFO turns the field too
// little in 64 samples to tell the offset from the signal).
//
// In: start for a cycle with the two sums (18 bits a component), which stay until done; the
// CFO word, valid from the cycle before start at the latest and until done. Out: done for a
// cycle, 17 cycles after start's, with the offset, which then stays. In cycles after start:
//   + 1, + 2   A·Σ₂ and e·Σ₁, then S (30 bits)
//   + 3, + 4   the four products of S·d*, then the numerator (41 bits; 45 are kept)
//   + 5        the two magnitudes in two restoring dividers side by side: the upper bits
//              against the divisor (a quotient of 2^11 or more: a rail of the ADC's range
//              whatever the rest, and what keeps the remainder below the divisor), the
//              remainder register of the divisor's 28 bits
//   + 16       11 quotient bits each, a bit a cycle
//   + 17       the offset: half to even from the remainder, the sign, the range
// A start before done abandons the estimate in flight.

`default_nettype none

module dc_estimate
    import rx_front_rom_pkg::*;
#(
    localparam int P   = NCO_PHASE_BITS,
    localparam int L   = NCO_LUT_BITS,
    localparam int Q   = NCO_AMP_BITS,
    localparam int X_W = Y_BITS - 1,                  // the ADC's 12 bits
    localparam int S_W = X_W + 6                      // a sum of 64 samples
) (
    input  wire logic                  clk,
    input  wire logic                  rst_n,         // synchronous, active low
    input  wire logic                  start,
    input  wire logic signed [P-1:0]   cfo,
    input  wire logic signed [S_W-1:0] s1_re,
    input  wire logic signed [S_W-1:0] s1_im,
    input  wire logic signed [S_W-1:0] s2_re,
    input  wire logic signed [S_W-1:0] s2_im,
    output logic                       done,
    output logic signed [X_W-1:0]      dc_re,
    output logic signed [X_W-1:0]      dc_im
);
    localparam int STEPS = X_W - 1;                   // the dividers' quotient bits: 11
    localparam int AMP   = (1 << (Q - 1)) - 1;        // A: 1023
    localparam int PA_W  = S_W + Q + 1;               // 30
    localparam int SS_W  = PA_W + 1;                  // 31
    localparam int N_W   = SS_W + Q + 3;              // 45
    localparam int DEN_W = 2 * Q + 6;                 // 28

    // The estimate's clock: 1 in the cycle after start, 0 when idle.
    logic [4:0] cnt;
    logic       load, step, finish;
    assign load   = cnt == 5'd4;
    assign step   = cnt >= 5'd5 && cnt < 5'(5 + STEPS);
    assign finish = cnt == 5'(5 + STEPS);
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cnt  <= '0;
            done <= 1'b0;
        end else begin
            done <= finish && !start;
            if (start)        cnt <= 5'd1;
            else if (finish)  cnt <= '0;
            else if (cnt != '0) cnt <= cnt + 1'b1;
        end
    end

    // e, d = A − e, 64·|d|² and the skipped division's test: from the CFO word alone.
    logic [P-1:0]        e_ph;
    logic signed [Q-1:0] e_tc, e_ts, e_re, e_im;
    logic signed [Q+1:0] d_re, d_im;                  // |d| < 2^Q
    logic [2*Q+1:0]      d2;
    logic                skip;
    assign e_ph = {cfo[P-7:0], 6'b0} + (P'(1) << (P - L - 1));
    assign e_tc = nco_cos(int'(e_ph[P-3 -: L-2]));
    assign e_ts = nco_sin(int'(e_ph[P-3 -: L-2]));
    always_ff @(posedge clk) begin
        unique case (e_ph[P-1 -: 2])
            2'd0: begin e_re <= e_tc;  e_im <= e_ts;  end
            2'd1: begin e_re <= -e_ts; e_im <= e_tc;  end
            2'd2: begin e_re <= -e_tc; e_im <= -e_ts; end
            default: begin e_re <= e_ts; e_im <= -e_tc; end
        endcase
        d_re <= (Q+2)'(AMP) - (Q+2)'(e_re);
        d_im <= -(Q+2)'(e_im);
        d2   <= (2*Q+2)'(d_re * d_re) + (2*Q+2)'(d_im * d_im);
        skip <= {d2, 8'b0} < (2*Q+10)'(AMP * AMP);
    end

    // S = A·Σ₂ − e·Σ₁ and the numerator S·d*: registers that follow their inputs; the
    // dividers take the numerator when it has settled.
    logic signed [PA_W-1:0] pa_re, pa_im, pe_re, pe_im;
    logic signed [SS_W-1:0] ss_re, ss_im;
    logic signed [N_W-1:0]  m_rr, m_ii, m_ir, m_ri, num_re, num_im;
    always_ff @(posedge clk) begin
        pa_re  <= PA_W'(s2_re) * PA_W'(AMP);
        pa_im  <= PA_W'(s2_im) * PA_W'(AMP);
        pe_re  <= PA_W'(e_re) * PA_W'(s1_re) - PA_W'(e_im) * PA_W'(s1_im);
        pe_im  <= PA_W'(e_re) * PA_W'(s1_im) + PA_W'(e_im) * PA_W'(s1_re);
        ss_re  <= SS_W'(pa_re) - SS_W'(pe_re);
        ss_im  <= SS_W'(pa_im) - SS_W'(pe_im);
        m_rr   <= N_W'(ss_re) * N_W'(d_re);
        m_ii   <= N_W'(ss_im) * N_W'(d_im);
        m_ir   <= N_W'(ss_im) * N_W'(d_re);
        m_ri   <= N_W'(ss_re) * N_W'(d_im);
        num_re <= m_rr + m_ii;
        num_im <= m_ir - m_ri;
    end

    logic [DEN_W-1:0] den;
    assign den = {d2[2*Q-1:0], 6'b0};

    logic signed [X_W-1:0] q_re, q_im;
    for (genvar g = 0; g < 2; g++) begin : g_div
        logic signed [N_W-1:0] num;
        logic [N_W-1:0]        mag;
        logic                  neg, sat, ge, up;
        logic [DEN_W-1:0]      rem;
        logic [DEN_W:0]        r2;
        logic [STEPS-1:0]      lo, quo;
        logic [STEPS:0]        qr;
        logic signed [X_W-1:0] val;
        assign num = g == 0 ? num_re : num_im;
        assign mag = num[N_W-1] ? N_W'(-num) : N_W'(num);
        assign r2  = {rem, lo[STEPS-1]};
        assign ge  = r2 >= {1'b0, den};
        always_ff @(posedge clk) begin
            if (load) begin
                neg <= num[N_W-1];
                sat <= mag[N_W-1:STEPS] >= (N_W-STEPS)'(den);
                rem <= mag[STEPS +: DEN_W];
                lo  <= mag[STEPS-1:0];
                quo <= '0;
            end else if (step) begin
                rem <= ge ? DEN_W'(r2 - {1'b0, den}) : r2[DEN_W-1:0];
                lo  <= {lo[STEPS-2:0], 1'b0};
                quo <= {quo[STEPS-2:0], ge};
            end
        end
        // Half to even from the remainder; the sign; the ADC's range.
        assign up  = {rem, 1'b0} > {1'b0, den} || ({rem, 1'b0} == {1'b0, den} && quo[0]);
        assign qr  = {1'b0, quo} + (STEPS+1)'(up);
        always_comb begin
            if (sat || qr[STEPS]) val = neg ? {1'b1, {(X_W-1){1'b0}}} : {1'b0, {(X_W-1){1'b1}}};
            else                  val = neg ? -X_W'(qr) : X_W'(qr);
        end
        if (g == 0) begin : g_re
            assign q_re = val;
        end else begin : g_im
            assign q_im = val;
        end
    end
    always_ff @(posedge clk) begin
        if (finish) begin
            dc_re <= skip ? '0 : q_re;
            dc_im <= skip ? '0 : q_im;
        end
    end

    // Bits that only feed a table address or a comparison elsewhere.
    logic unused;
    assign unused = &{1'b0, e_ph[P-L-1:0], d2[2*Q+1:2*Q], cfo[P-1:P-6]};
endmodule

`default_nettype wire
