// Test wrapper for gate G3 `decide_t1` (make -C hw decide_t1): the decision engine alone,
// rx_decide behind the front's arbiter and CORDIC (cordic_arb, the latch never asking), its
// candidate and energy ring driven by the testbench with energies the chain cannot produce.
// Brought out: the history's write (the sample count that times the result), the candidate,
// the ring's read port, and T1 with the result's probes. The sums and phase words are not
// compared here (gate `decide`). Not synthesised on its own.

`default_nettype none

module decide_alone
    import rx_front_rom_pkg::*;
#(
    parameter int W         = SEARCH_W,
    parameter int L         = DECISION_LATENCY,
    localparam int E_W      = 2 * CORR_BITS + 4,
    localparam int POS_W    = $clog2(HISTORY),
    localparam int RA_W     = $clog2(LTF_SYMBOL_LEN + GI_LEN + 1)
) (
    input  wire logic                     clk,
    input  wire logic                     rst_n,
    input  wire logic                     y_valid,
    input  wire logic signed [Y_BITS-1:0] y_re,
    input  wire logic signed [Y_BITS-1:0] y_im,
    input  wire logic                     acc_valid,
    input  wire logic [POS_W-1:0]         acc_n,
    input  wire logic [POS_W-1:0]         acc_imax,
    input  wire logic                     acc_lapped,
    input  wire logic [RA_W-1:0]          acc_ring,
    input  wire logic [E_W-1:0]           acc_emax,
    output logic                          ring_re,
    output logic [RA_W-1:0]               ring_addr,
    input  wire logic [E_W-1:0]           ring_data,
    output logic                          res_valid,
    output logic                          res_sums,
    output logic [POS_W-1:0]              acq_t1,
    output logic                          dec_valid
);
    localparam int S_W = CORDIC_IN_BITS;
    localparam int P   = NCO_PHASE_BITS;
    localparam int C_W = Z_BITS + TAP_BITS + 7;
    localparam int M_W = 2 * C_W + 8;
    localparam int N_W = 2 * Z_BITS + 7;

    logic                         cq_req, cq_grant, cq_valid, a_valid_unused, acquired;
    logic signed [S_W-1:0]        cq_re, cq_im;
    logic signed [ANGLE_BITS-1:0] cq_angle;
    logic signed [P-1:0]          acq_cfo, acq_coarse, acq_fine;
    logic signed [S_W-1:0]        r_cre, r_cim, r_fre, r_fim;
    logic [M_W-1:0]               r_conf, r_coh;
    logic [N_W-1:0]               r_en;
    logic                         rot_start;

    cordic_arb u_cordic (
        .clk, .rst_n, .a_req(1'b0), .a_re('0), .a_im('0),
        .b_req(cq_req), .b_re(cq_re), .b_im(cq_im), .b_grant(cq_grant),
        .a_valid(a_valid_unused), .b_valid(cq_valid), .angle(cq_angle)
    );
    rx_decide #(.W(W), .L(L)) u_dec (
        .clk, .rst_n, .y_valid, .y_re, .y_im,
        .acc_valid, .acc_n, .acc_imax, .acc_lapped, .acc_ring, .acc_emax,
        .ring_re, .ring_addr, .ring_data,
        .cq_req, .cq_re, .cq_im, .cq_grant, .cq_valid, .cq_angle,
        .dec_valid, .acquired, .acq_t1, .acq_cfo, .acq_coarse, .acq_fine,
        .res_valid, .res_sums, .res_coarse_re(r_cre), .res_coarse_im(r_cim),
        .res_confirmation(r_conf), .res_coherence(r_coh), .res_energy(r_en),
        .res_fine_re(r_fre), .res_fine_im(r_fim), .rot_start
    );

    logic unused;
    assign unused = &{1'b0, a_valid_unused, acquired, acq_cfo, acq_coarse, acq_fine, r_cre, r_cim,
                      r_fre, r_fim, r_conf, r_coh, r_en, rot_start};
endmodule

`default_nettype wire
