// Test wrapper for gate G3 `det` (make -C hw det): the receiver's step 1, DC blocker into
// detector A, as the top level will wire them. Not synthesised on its own.

`default_nettype none

module det_chain
    import rx_front_rom_pkg::*;
(
    input  wire logic                     clk,
    input  wire logic                     rst_n,
    input  wire logic                     in_valid,
    input  wire logic signed [Y_BITS-2:0] in_re,      // 12-bit ADC
    input  wire logic signed [Y_BITS-2:0] in_im,
    output logic                          y_valid,
    output logic signed [Y_BITS-1:0]      y_re,
    output logic signed [Y_BITS-1:0]      y_im,
    output logic                          d_valid,
    output logic                          fire,
    output logic                          event_o
);
    dc_block u_dc (
        .clk, .rst_n, .in_valid, .in_re, .in_im,
        .out_valid(y_valid), .out_re(y_re), .out_im(y_im)
    );
    det_a u_det (
        .clk, .rst_n, .in_valid(y_valid), .in_re(y_re), .in_im(y_im), .clear(1'b0),
        .out_valid(d_valid), .fire, .event_o
    );
endmodule

`default_nettype wire
