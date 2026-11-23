// Testbench wrapper for the gate `handoff`: the front's top level and the hand-off behind
// it, as a receiver will wire them. The same stream and reset to both (the hand-off's
// history counts the front's samples), the acquisition's strobe, window address and CFO
// word from the one to the other, and a probe of the offset. Not synthesised on its own: the receiver's top level
// arrives with the decoder.

`default_nettype none

module handoff_chain
    import rx_front_rom_pkg::*;
#(
    parameter int D_SAMPLES = 4,
    parameter int W         = SEARCH_W,
    localparam int POS_W    = $clog2(HISTORY),
    localparam int P        = NCO_PHASE_BITS
) (
    input  wire logic                     clk,
    input  wire logic                     rst_n,
    input  wire logic                     in_valid,
    input  wire logic signed [Y_BITS-2:0] in_re,
    input  wire logic signed [Y_BITS-2:0] in_im,
    input  wire logic                     sig_valid,
    input  wire logic [7:0]               sig_tag,
    input  wire logic                     sig_fail,
    input  wire logic [10:0]              sig_symbols,
    input  wire logic                     sig_patient,
    input  wire logic                     out_ready,
    output logic                          out_valid,
    output logic                          out_end,
    output logic [1:0]                    out_kind,
    output logic                          out_first,
    output logic signed [Y_BITS-1:0]      out_re,
    output logic signed [Y_BITS-1:0]      out_im,
    output logic signed [P-1:0]           out_cfo,
    output logic [7:0]                    out_tag,
    output logic                          hold,
    // probe: the offset and whether it is registered
    output logic                          p_dc_v,
    output logic signed [Y_BITS-2:0]      p_dc_re,
    output logic signed [Y_BITS-2:0]      p_dc_im
);
    logic                acq_valid;
    logic [POS_W-1:0]    acq_fft, acq_t1;
    logic signed [P-1:0] acq_cfo, acq_coarse, acq_fine;

    rx_front #(.D_SAMPLES(D_SAMPLES), .W(W)) u_front (
        .clk, .rst_n, .in_valid, .in_re, .in_im,
        .acq_valid, .acq_fft, .acq_t1, .acq_cfo, .acq_coarse, .acq_fine
    );
    rx_handoff #(.W(W)) u_handoff (
        .clk, .rst_n, .in_valid, .in_re, .in_im,
        .acq_valid, .acq_fft, .acq_cfo,
        .sig_valid, .sig_tag, .sig_fail, .sig_symbols, .sig_patient,
        .out_ready, .out_valid, .out_end, .out_kind, .out_first, .out_re, .out_im,
        .out_cfo, .out_tag, .hold
    );

    assign p_dc_v  = u_handoff.dc_v;
    assign p_dc_re = u_handoff.dc_re;
    assign p_dc_im = u_handoff.dc_im;

    logic unused;
    assign unused = &{1'b0, acq_t1, acq_coarse, acq_fine};
endmodule

`default_nettype wire
