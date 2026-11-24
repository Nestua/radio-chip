// CORDIC vectoring, iterative: the same arithmetic as hw/rtl/cordic.sv (the angle of re + j·im
// in units of 2π/2^ANGLE_BITS, in [−2^(A−1), 2^(A−1)), 0 for a zero input), one micro-rotation
// a clock on one set of registers. The data path's own CORDIC (ADR 0010, decision 1: three
// angles a symbol, the tracker's two and the common phase). Golden model:
// radio_phy::rx_front_fixed::cordic_angle. Gate G3: make -C hw cordic_iter (the CORDIC's
// vectors, hw/vectors/cordic.txt).
//
// Datapath, exactly the pipelined block's: the block normalisation into the edge that takes the
// input, the half turn on the next edge, then CORDIC_ITERS edges of micro-rotations (shift i by
// a counter, atan(i) from the package's table).
//
// Protocol: in_valid only while busy is low (simulation stops otherwise); out_valid for one
// cycle CORDIC_ITERS + 1 clock edges after the edge that took the input (17, as the pipelined
// block's). busy is already low in out_valid's cycle: the next input can be taken on that
// cycle's edge, an angle every 18 clocks back to back. drop (and rst_n): back to idle, nothing
// comes out.

`default_nettype none

module cordic_iter
    import rx_front_rom_pkg::*;
(
    input  wire logic                             clk,
    input  wire logic                             rst_n,     // synchronous, active low
    input  wire logic                             drop,
    input  wire logic                             in_valid,
    input  wire logic signed [CORDIC_IN_BITS-1:0] in_re,
    input  wire logic signed [CORDIC_IN_BITS-1:0] in_im,
    output logic                                  busy,
    output logic                                  out_valid,
    output logic signed [ANGLE_BITS-1:0]          out_angle
);
    localparam int IN = CORDIC_IN_BITS;
    localparam int CW = CORDIC_BITS;
    localparam int A  = ANGLE_BITS;
    localparam int N  = CORDIC_ITERS;
    localparam int T  = CORDIC_BITS - 3;
    localparam int LW = $clog2(IN + 1);
    localparam int IW = $clog2(N + 1);

    // ---- block normalisation (combinational, into the taking edge)
    logic [IN-1:0]  mag_re, mag_im, mag;
    logic [LW-1:0]  len;
    logic signed [IN-1:0] sh_re, sh_im;
    assign mag_re = in_re[IN-1] ? IN'(-in_re) : IN'(in_re);
    assign mag_im = in_im[IN-1] ? IN'(-in_im) : IN'(in_im);
    assign mag    = mag_re | mag_im;

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

    // ---- the registers
    typedef enum logic [1:0] {IDLE, HALF, ITER} state_t;
    state_t state;
    logic signed [CW-1:0] x, y;
    logic signed [A-1:0]  z;
    logic                 zero;
    logic [IW-1:0]        i;

    logic signed [A-1:0] atan_i;
    always_comb begin
        atan_i = '0;
        for (int k = 0; k < N; k++)
            if (int'(i) == k) atan_i = cordic_atan(k);
    end

    wire y_pos = !y[CW-1] && (y != '0);
    wire signed [CW-1:0] ys = y >>> i;
    wire signed [CW-1:0] xs = x >>> i;

    assign busy = state != IDLE;

    always_ff @(posedge clk) begin
        if (!rst_n || drop) begin
            state     <= IDLE;
            out_valid <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            case (state)
                IDLE: if (in_valid) begin
                    x     <= CW'(sh_re);       // |sh| ≤ 2^T: the low CW bits are the value
                    y     <= CW'(sh_im);
                    z     <= '0;
                    zero  <= (mag == '0);
                    state <= HALF;
                end
                HALF: begin
                    if (x[CW-1]) begin
                        x <= -x;
                        y <= -y;
                        z <= {1'b1, {(A-1){1'b0}}};   // 2^(A−1), the half turn
                    end
                    i     <= '0;
                    state <= ITER;
                end
                ITER: begin
                    if (y_pos) begin
                        x <= x + ys;
                        y <= y - xs;
                        z <= z + atan_i;
                    end else begin
                        x <= x - ys;
                        y <= y + xs;
                        z <= z - atan_i;
                    end
                    i <= i + 1'b1;
                    if (int'(i) == N - 1) begin
                        state     <= IDLE;
                        out_valid <= 1'b1;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end

    assign out_angle = zero ? '0 : z;

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (rst_n && !drop && in_valid && busy) begin
            $error("cordic_iter: an input while busy");
            $stop;
        end
    end
`endif

    // The shifted components' top bits are their sign extension (|sh| ≤ 2^T).
    logic unused;
    assign unused = ^{sh_re[IN-1:CW], sh_im[IN-1:CW]};
endmodule

`default_nettype wire
