// CORDIC vectoring: the angle of (re + j·im) in units of 2π/2^ANGLE_BITS, in
// [−2^(A−1), 2^(A−1)), 0 for a zero input. Golden model: radio_phy::rx_front_fixed::cordic_angle
// (the provisional latch and the final coarse and fine CFO of RxFrontFixed). Gate G3:
// make -C hw cordic.
//
// Datapath, exactly the model's:
//   1. block normalisation: len = bit length of max(|re|, |im|); both components shifted
//      (arithmetic right by len − T, or left by T − len) to T = CORDIC_BITS − 3 magnitude bits;
//   2. half turn: if x < 0, negate both and start the angle at 2^(A−1);
//   3. CORDIC_ITERS micro-rotations: if y > 0, x += y >>> i, y −= x >>> i, z += atan(i);
//      else the opposite. x, y are CORDIC_BITS-bit registers, z wraps at A bits (the model's
//      final wrap of z into [−2^(A−1), 2^(A−1)) is that wrap).
// T magnitude bits grow by at most √2 · 1.65 < 4 in the iterations, so x and y never wrap.
//
// Stream: one input per in_valid, never stalls; fully pipelined, one stage per step:
// out_valid CORDIC_ITERS + 2 clocks after the edge that takes the input. Iterative (one
// micro-rotation per clock) would be smaller; the receiver needs an angle only at detection
// events and decisions, so part d may fold it.

`default_nettype none

module cordic
    import rx_front_rom_pkg::*;
(
    input  wire logic                             clk,
    input  wire logic                             rst_n,     // synchronous, active low
    input  wire logic                             in_valid,
    input  wire logic signed [CORDIC_IN_BITS-1:0] in_re,
    input  wire logic signed [CORDIC_IN_BITS-1:0] in_im,
    output logic                                  out_valid,
    output logic signed [ANGLE_BITS-1:0]          out_angle
);
    localparam int IN = CORDIC_IN_BITS;
    localparam int CW = CORDIC_BITS;
    localparam int A  = ANGLE_BITS;
    localparam int N  = CORDIC_ITERS;
    localparam int T  = CORDIC_BITS - 3;
    localparam int LW = $clog2(IN + 1);

    // ---- 1. block normalisation (combinational, registered into stage 0)
    logic [IN-1:0]  mag_re, mag_im, mag;
    logic [LW-1:0]  len;
    logic signed [IN-1:0] sh_re, sh_im;
    assign mag_re = in_re[IN-1] ? IN'(-in_re) : IN'(in_re);   // |−2^(IN−1)| fits unsigned
    assign mag_im = in_im[IN-1] ? IN'(-in_im) : IN'(in_im);
    assign mag    = mag_re | mag_im;                          // same highest bit as the max

    always_comb begin
        len = '0;
        for (int b = 0; b < IN; b++)
            if (mag[b]) len = LW'(b + 1);
    end

    always_comb begin
        if (int'(len) > T) begin
            sh_re = in_re >>> (int'(len) - T);
            sh_im = in_im >>> (int'(len) - T);
        end else begin
            sh_re = in_re <<< (T - int'(len));
            sh_im = in_im <<< (T - int'(len));
        end
    end

    // ---- pipeline registers: stage 0 normalised, stage 1 half turn, stages 2..N+1 iterations
    logic signed [CW-1:0] x [0:N+1];
    logic signed [CW-1:0] y [0:N+1];
    logic signed [A-1:0]  z [0:N+1];
    logic                 zero [0:N+1];
    logic [N+1:0]         v;

    always_ff @(posedge clk) begin
        x[0]    <= CW'(sh_re);   // |sh| < 2^T: the low CW bits are the value
        y[0]    <= CW'(sh_im);
        z[0]    <= '0;
        zero[0] <= (mag == '0);
    end

    always_ff @(posedge clk) begin
        if (x[0][CW-1]) begin
            x[1] <= -x[0];
            y[1] <= -y[0];
            z[1] <= {1'b1, {(A-1){1'b0}}};   // 2^(A−1), the half turn
        end else begin
            x[1] <= x[0];
            y[1] <= y[0];
            z[1] <= z[0];
        end
        zero[1] <= zero[0];
    end

    for (genvar i = 0; i < N; i++) begin : g_iter
        localparam logic signed [A-1:0] ATAN = cordic_atan(i);
        logic y_pos;
        assign y_pos = !y[i+1][CW-1] && (y[i+1] != '0);
        always_ff @(posedge clk) begin
            if (y_pos) begin
                x[i+2] <= x[i+1] + (y[i+1] >>> i);
                y[i+2] <= y[i+1] - (x[i+1] >>> i);
                z[i+2] <= z[i+1] + ATAN;
            end else begin
                x[i+2] <= x[i+1] - (y[i+1] >>> i);
                y[i+2] <= y[i+1] + (x[i+1] >>> i);
                z[i+2] <= z[i+1] - ATAN;
            end
            zero[i+2] <= zero[i+1];
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) v <= '0;
        else        v <= {v[N:0], in_valid};
    end

    assign out_valid = v[N+1];
    assign out_angle = zero[N+1] ? '0 : z[N+1];

    // x of the last stage is not needed (only its sign and y drive the iterations), and the
    // shifted components' top bits are their sign extension (|sh| < 2^T).
    logic unused;
    assign unused = ^{x[N+1], sh_re[IN-1:CW], sh_im[IN-1:CW]};
endmodule

`default_nettype wire
