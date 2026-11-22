// Test wrapper for gate G3 `decide` (make -C hw decide): the receiver front's top level
// (hw/rtl/rx_front.sv: dc_block → det_a → rx_latch → nco → ltf_corr → rx_search → rx_decide,
// the CORDIC shared through cordic_arb, the loop closed) with the gate's probes brought out
// of it. Nothing comes from the vectors but the ADC samples. Not synthesised.

`default_nettype none

module decide_chain
    import rx_front_rom_pkg::*;
#(
    parameter int D_SAMPLES = 4,
    parameter int W         = SEARCH_W,
    localparam int POS_W    = $clog2(HISTORY),
    localparam int S_W      = CORDIC_IN_BITS,
    localparam int M_W      = 2 * (Z_BITS + TAP_BITS + 7) + 8,
    localparam int N_W      = 2 * Z_BITS + 7
) (
    input  wire logic                         clk,
    input  wire logic                         rst_n,
    input  wire logic                         in_valid,
    input  wire logic signed [Y_BITS-2:0]     in_re,      // 12-bit ADC
    input  wire logic signed [Y_BITS-2:0]     in_im,
    output logic                              d_valid,      // det_a's stage 4
    output logic                              event_d,
    output logic                              acc_valid,
    output logic [POS_W-1:0]                  acc_n,
    output logic                              res_valid,
    output logic                              res_sums,
    output logic signed [S_W-1:0]             res_coarse_re,
    output logic signed [S_W-1:0]             res_coarse_im,
    output logic [M_W-1:0]                    res_confirmation,
    output logic [M_W-1:0]                    res_coherence,
    output logic [N_W-1:0]                    res_energy,
    output logic signed [S_W-1:0]             res_fine_re,
    output logic signed [S_W-1:0]             res_fine_im,
    output logic                              rot_start,
    output logic                              cq_granted,   // the decision's request taken
    output logic                              dec_valid,
    output logic                              acquired,
    output logic [POS_W-1:0]                  acq_t1,
    output logic signed [NCO_PHASE_BITS-1:0]  acq_cfo,
    output logic signed [NCO_PHASE_BITS-1:0]  acq_coarse,
    output logic signed [NCO_PHASE_BITS-1:0]  acq_fine
);
    logic [POS_W-1:0] acq_fft;
    logic             acq_valid;

    rx_front #(.D_SAMPLES(D_SAMPLES), .W(W)) u_front (
        .clk, .rst_n, .in_valid, .in_re, .in_im,
        .acq_valid, .acq_fft, .acq_t1, .acq_cfo, .acq_coarse, .acq_fine
    );

    // The gate's probes inside the front.
    assign d_valid          = u_front.d_valid;
    assign event_d          = u_front.event_d;
    assign acc_valid        = u_front.acc_valid;
    assign acc_n            = u_front.acc_n;
    assign res_valid        = u_front.res_valid;
    assign res_sums         = u_front.res_sums;
    assign res_coarse_re    = u_front.res_coarse_re;
    assign res_coarse_im    = u_front.res_coarse_im;
    assign res_confirmation = u_front.res_confirmation;
    assign res_coherence    = u_front.res_coherence;
    assign res_energy       = u_front.res_energy;
    assign res_fine_re      = u_front.res_fine_re;
    assign res_fine_im      = u_front.res_fine_im;
    assign rot_start        = u_front.rot_start;
    assign cq_granted       = u_front.cq_req && u_front.cq_grant;
    assign dec_valid        = u_front.dec_valid;
    assign acquired         = acq_valid;

    logic unused;
    assign unused = &{1'b0, acq_fft};
endmodule

`default_nettype wire
