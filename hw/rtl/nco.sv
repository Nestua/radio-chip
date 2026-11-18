// NCO of the receiver front: z = y·e^(jφ), φ a NCO_PHASE_BITS-bit phase (units 2π/2^P) that
// advances by the sample's step after each valid sample, the product narrowed to Z_BITS (half
// to even, saturating). Golden model: radio_phy::rx_front_fixed::nco_mix, with the phase
// accumulated as RxFrontFixed::push does (step 2). Gate G3: make -C hw nco.
//
// e^(jφ): the phase's top NCO_LUT_BITS bits address a quarter-wave table (rx_front_rom_pkg
// nco_cos/nco_sin, r < 2^(L−2)), folded by the top two bits: (c, s), (−s, c), (−c, −s), (s, −c).
// The products y·e^(jφ) are |·| < 2 · 2^(Y−1) · (2^(Q−1) − 1) < 2^(Y+Q−1), Y + Q bits signed; the
// model narrows them from Y + Q − 1 to Z bits, i.e. drops Y + Q − 1 − Z low bits (round_sat).
//
// Stream: one sample per in_valid, never stalls. in_step is the step in effect at this sample
// (after the receiver's latch, if any); the phase register starts at 0 after reset and moves
// only on in_valid. out_valid 4 clocks after the edge that takes the sample.

`default_nettype none

module nco
    import rx_front_rom_pkg::*;
(
    input  wire logic                             clk,
    input  wire logic                             rst_n,     // synchronous, active low
    input  wire logic                             in_valid,
    input  wire logic signed [Y_BITS-1:0]         in_re,
    input  wire logic signed [Y_BITS-1:0]         in_im,
    input  wire logic signed [NCO_PHASE_BITS-1:0] in_step,
    output logic                                  out_valid,
    output logic signed [Z_BITS-1:0]              out_re,
    output logic signed [Z_BITS-1:0]              out_im
);
    localparam int P   = NCO_PHASE_BITS;
    localparam int L   = NCO_LUT_BITS;
    localparam int Q   = NCO_AMP_BITS;
    localparam int PW  = Y_BITS + Q;              // product sum width
    localparam int DROP = Y_BITS + Q - 1 - Z_BITS;

    // ---- phase accumulator
    logic [P-1:0] phase;
    always_ff @(posedge clk) begin
        if (!rst_n)        phase <= '0;
        else if (in_valid) phase <= phase + P'(in_step);
    end

    // ---- stage 1: sample and table address of this sample's phase
    logic signed [Y_BITS-1:0] y1_re, y1_im;
    logic [1:0]               q1;
    logic [L-3:0]             r1;
    always_ff @(posedge clk) begin
        y1_re <= in_re;
        y1_im <= in_im;
        q1    <= phase[P-1 -: 2];
        r1    <= phase[P-3 -: L-2];
    end

    // ---- stage 2: quarter-wave table, folded by quadrant
    logic signed [Q-1:0]      c2, s2;
    logic signed [Y_BITS-1:0] y2_re, y2_im;
    logic signed [Q-1:0]      tc, ts;
    assign tc = nco_cos(int'(r1));
    assign ts = nco_sin(int'(r1));
    always_ff @(posedge clk) begin
        unique case (q1)
            2'd0: begin c2 <= tc;  s2 <= ts;  end
            2'd1: begin c2 <= -ts; s2 <= tc;  end
            2'd2: begin c2 <= -tc; s2 <= -ts; end
            default: begin c2 <= ts; s2 <= -tc; end
        endcase
        y2_re <= y1_re;
        y2_im <= y1_im;
    end

    // ---- stage 3: complex product (exact)
    logic signed [PW-1:0] p3_re, p3_im;
    always_ff @(posedge clk) begin
        p3_re <= PW'(y2_re * c2) - PW'(y2_im * s2);
        p3_im <= PW'(y2_re * s2) + PW'(y2_im * c2);
    end

    // ---- stage 4: narrow to Z bits
    logic signed [Z_BITS-1:0] n_re, n_im;
    round_sat #(.IN_W(PW), .SHIFT(DROP), .OUT_W(Z_BITS)) u_rs_re (.in(p3_re), .out(n_re));
    round_sat #(.IN_W(PW), .SHIFT(DROP), .OUT_W(Z_BITS)) u_rs_im (.in(p3_im), .out(n_im));
    always_ff @(posedge clk) begin
        out_re <= n_re;
        out_im <= n_im;
    end

    logic [3:0] v;
    always_ff @(posedge clk) begin
        if (!rst_n) v <= '0;
        else        v <= {v[2:0], in_valid};
    end
    assign out_valid = v[3];

    // The phase bits below the table address only accumulate.
    logic unused;
    assign unused = ^phase[P-L-1:0];
endmodule

`default_nettype wire
