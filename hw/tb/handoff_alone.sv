// Testbench wrapper for the gate `handoff_forced`: the hand-off alone, its acquisitions
// given by the testbench, with a probe of the offset and of the buffer's fill.

`default_nettype none

module handoff_alone
    import rx_front_rom_pkg::*;
#(
    localparam int POS_W = $clog2(HISTORY),
    localparam int P     = NCO_PHASE_BITS
) (
    input  wire logic                     clk,
    input  wire logic                     rst_n,
    input  wire logic                     in_valid,
    input  wire logic signed [Y_BITS-2:0] in_re,
    input  wire logic signed [Y_BITS-2:0] in_im,
    input  wire logic                     acq_valid,
    input  wire logic [POS_W-1:0]         acq_fft,
    input  wire logic signed [P-1:0]      acq_cfo,
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
    output logic                          p_dc_v,
    output logic signed [Y_BITS-2:0]      p_dc_re,
    output logic signed [Y_BITS-2:0]      p_dc_im,
    output logic [3:0]                    p_fcnt
);
    rx_handoff u_handoff (
        .clk, .rst_n, .in_valid, .in_re, .in_im, .acq_valid, .acq_fft, .acq_cfo,
        .sig_valid(1'b0), .sig_tag(8'd0), .sig_fail(1'b0), .sig_symbols(11'd0), .sig_patient(1'b0),
        .out_ready, .out_valid, .out_end, .out_kind, .out_first, .out_re, .out_im,
        .out_cfo, .out_tag, .hold
    );
    assign p_dc_v  = u_handoff.dc_v;
    assign p_dc_re = u_handoff.dc_re;
    assign p_dc_im = u_handoff.dc_im;
    assign p_fcnt  = u_handoff.fcnt;
endmodule

`default_nettype wire
