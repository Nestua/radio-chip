// Test wrapper for gate G3 `latch` (make -C hw latch): the receiver's steps 1 and 2 as the top
// level will wire them — dc_block → det_a → rx_latch (provisional latch, NCO path D_SAMPLES
// behind) → nco. Not synthesised on its own.
//
// in_clear marks the sample at which the model applies an acquisition (until the decision
// engine exists, part d step 3, the vectors supply it). It is delayed to the cycle det_a's
// stage 4 holds that sample (det_a's `clear` contract): the x edge, then det_a stages 1–3.

`default_nettype none

module latch_chain
    import rx_front_rom_pkg::*;
#(
    parameter int D_SAMPLES = 3
) (
    input  wire logic                         clk,
    input  wire logic                         rst_n,
    input  wire logic                         in_valid,
    input  wire logic signed [Y_BITS-2:0]     in_re,      // 12-bit ADC
    input  wire logic signed [Y_BITS-2:0]     in_im,
    input  wire logic                         in_clear,
    output logic                              d_valid,
    output logic                              event_o,
    output logic                              s_valid,
    output logic signed [NCO_PHASE_BITS-1:0]  s_step,
    output logic                              z_valid,
    output logic signed [Z_BITS-1:0]          z_re,
    output logic signed [Z_BITS-1:0]          z_im
);
    logic                         y_valid, c_valid, fire;
    logic signed [Y_BITS-1:0]     y_re, y_im, n_re, n_im;
    logic signed [2*DET_BITS+5:0] c_re, c_im;
    logic [3:0]                   clr;

    always_ff @(posedge clk) begin
        if (!rst_n) clr <= '0;
        else        clr <= {clr[2:0], in_valid && in_clear};
    end

    dc_block u_dc (
        .clk, .rst_n, .in_valid, .in_re, .in_im,
        .out_valid(y_valid), .out_re(y_re), .out_im(y_im)
    );
    det_a u_det (
        .clk, .rst_n, .in_valid(y_valid), .in_re(y_re), .in_im(y_im), .clear(clr[3]),
        .c_valid, .c_re, .c_im, .out_valid(d_valid), .fire, .event_o
    );
    rx_latch #(.D_SAMPLES(D_SAMPLES)) u_latch (
        .clk, .rst_n, .y_valid, .y_re, .y_im, .c_valid, .c_re, .c_im,
        .d_valid, .event_i(event_o),
        .out_valid(s_valid), .out_re(n_re), .out_im(n_im), .out_step(s_step)
    );
    nco u_nco (
        .clk, .rst_n, .in_valid(s_valid), .in_re(n_re), .in_im(n_im), .in_step(s_step),
        .out_valid(z_valid), .out_re(z_re), .out_im(z_im)
    );

    logic unused;
    assign unused = fire;
endmodule

`default_nettype wire
