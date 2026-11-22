// Test wrapper for gate G3 `decide` (make -C hw decide): the receiver front with its loop
// closed, as the top level will wire it — dc_block → det_a → rx_latch → nco → ltf_corr →
// rx_search → rx_decide, the CORDIC shared through cordic_arb. The decision's `acquired`
// clears det_a's run counter and closes the search: nothing comes from the vectors but the
// ADC samples. Not synthesised on its own.

`default_nettype none

module decide_chain
    import rx_front_rom_pkg::*;
#(
    parameter int D_SAMPLES = 4,
    parameter int W         = SEARCH_W,
    localparam int E_W      = 2 * CORR_BITS + 4,
    localparam int POS_W    = $clog2(HISTORY),
    localparam int RA_W     = $clog2(LTF_SYMBOL_LEN + GI_LEN + 1),
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
    logic                              y_valid, cd_valid, fire;
    logic                              l_valid, l_event, z_valid, c_valid;
    logic signed [Y_BITS-1:0]          y_re, y_im, n_re, n_im;
    logic signed [2*DET_BITS+5:0]      cd_re, cd_im;
    logic signed [NCO_PHASE_BITS-1:0]  l_step;
    logic signed [Z_BITS-1:0]          z_re, z_im;
    logic signed [CORR_BITS-1:0]       c_re, c_im;
    logic                              rq_valid, a_valid, cq_req, cq_grant, cq_valid;
    assign cq_granted = cq_req && cq_grant;
    logic signed [CORDIC_IN_BITS-1:0]  cq_re, cq_im;
    logic                              ring_re, acc_lapped, e_valid, s_valid, searching;
    logic [RA_W-1:0]                   ring_addr, acc_ring;
    logic [E_W-1:0]                    ring_data, e, acc_emax;
    logic [POS_W-1:0]                  acc_imax, acc_opened;
    logic signed [CORDIC_IN_BITS-1:0]  rq_re, rq_im;
    logic signed [ANGLE_BITS-1:0]      a_angle;

    dc_block u_dc (
        .clk, .rst_n, .in_valid, .in_re, .in_im,
        .out_valid(y_valid), .out_re(y_re), .out_im(y_im)
    );
    det_a u_det (
        .clk, .rst_n, .in_valid(y_valid), .in_re(y_re), .in_im(y_im), .clear(acquired),
        .c_valid(cd_valid), .c_re(cd_re), .c_im(cd_im), .out_valid(d_valid), .fire,
        .event_o(event_d)
    );
    rx_latch #(.D_SAMPLES(D_SAMPLES)) u_latch (
        .clk, .rst_n, .y_valid, .y_re, .y_im, .c_valid(cd_valid), .c_re(cd_re), .c_im(cd_im),
        .d_valid, .event_i(event_d),
        .rq_valid, .rq_re, .rq_im, .a_valid, .a_angle,
        .out_valid(l_valid), .out_re(n_re), .out_im(n_im), .out_step(l_step),
        .out_event(l_event)
    );
    // The front's CORDIC: the latch first, then the decision.
    cordic_arb u_cordic (
        .clk, .rst_n, .a_req(rq_valid), .a_re(rq_re), .a_im(rq_im),
        .b_req(cq_req), .b_re(cq_re), .b_im(cq_im), .b_grant(cq_grant),
        .a_valid, .b_valid(cq_valid), .angle(a_angle)
    );
    nco u_nco (
        .clk, .rst_n, .in_valid(l_valid), .in_re(n_re), .in_im(n_im), .in_step(l_step),
        .out_valid(z_valid), .out_re(z_re), .out_im(z_im)
    );
    ltf_corr u_corr (
        .clk, .rst_n, .in_valid(z_valid), .in_re(z_re), .in_im(z_im),
        .out_valid(c_valid), .out_re(c_re), .out_im(c_im)
    );
    rx_search #(.W(W)) u_search (
        .clk, .rst_n, .s_valid(l_valid), .s_event(l_event), .c_valid, .c_re, .c_im,
        .acquired,
        .e_valid, .e, .out_valid(s_valid), .searching,
        .acc_valid, .acc_n, .acc_imax, .acc_lapped, .acc_ring, .acc_emax, .acc_opened,
        .ring_re, .ring_addr, .ring_data
    );

    rx_decide #(.W(W)) u_decide (
        .clk, .rst_n, .y_valid, .y_re, .y_im,
        .acc_valid, .acc_n, .acc_imax, .acc_lapped, .acc_ring, .acc_emax,
        .ring_re, .ring_addr, .ring_data,
        .cq_req, .cq_re, .cq_im, .cq_grant, .cq_valid, .cq_angle(a_angle),
        .dec_valid, .acquired, .acq_t1, .acq_cfo, .acq_coarse, .acq_fine,
        .res_valid, .res_sums, .res_coarse_re, .res_coarse_im, .res_confirmation,
        .res_coherence, .res_energy, .res_fine_re, .res_fine_im, .rot_start
    );

    logic unused;
    assign unused = &{1'b0, fire, e_valid, e, s_valid, searching, acc_opened};
endmodule

`default_nettype wire
