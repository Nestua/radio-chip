// The receiver front's top level, up to the acquisition (ADR 0006; ADR 0008, decisions 4 and
// 6): dc_block → det_a → rx_latch → nco → ltf_corr → rx_search → rx_decide, the one CORDIC
// shared through cordic_arb (the latch first). The decision's `acquired` clears det_a's run
// counter and closes the search. Golden model: radio_phy::rx_front_fixed::RxFrontFixed::push
// (the FixedAcquisition it returns). Gate G3: make -C hw front at the pins; make -C hw decide
// (hw/tb/decide_chain.sv, which wraps this module and looks inside) for everything between.
//
// In: one chain's stream, 12-bit I and Q with a valid, a sample in R clocks, R at least 8
// (ltf_corr) and D_SAMPLES·R at least 24 (rx_latch). The front never stalls: there is no
// ready.
//
// Out: acq_valid for one cycle when an acquisition is applied: the cycle after the third
// clock edge from the edge that takes the sample it is applied at (the model's decided_at;
// dc_block's and det_a's latency), so a register clocked by this clock takes it at the
// fourth. Valid in that cycle only, to be latched there by whoever uses them (they are
// rx_decide's working registers: the next candidate's decision moves them, and they have no
// reset value):
//   acq_t1      the first sample of the first long training symbol, modulo HISTORY;
//   acq_fft     the first sample of that symbol's FFT window, GI_LEN − W before T1, modulo
//               HISTORY (the model's fft_start);
//   acq_cfo     the CFO word handed on, 2π/2^P per sample (the downstream NCO's step is its
//               negative); acq_coarse and acq_fine the two estimates it is made of.
// Samples are counted from reset, the first is sample 0: a memory written with the input
// stream at the sample's number modulo HISTORY (the hand-off's history, ADR 0008 decision
// 1) holds the window's first sample at address acq_fft. With L = 64 an acquisition is
// applied 189 + W + L to 189 + 2·W + L samples after T1 (rvec holds every acquisition of the
// gate's vectors to it), so the window's first sample is at most 285 samples old, the
// number of samples since it is (the memory's write count − acq_fft) modulo HISTORY, and
// it is overwritten no sooner than 227 samples later.
//
// The hand-off (the second history, the DC estimate, the replay, the hold) is not here.

`default_nettype none

module rx_front
    import rx_front_rom_pkg::*;
#(
    parameter int D_SAMPLES = 4,                     // the NCO path's delay (rx_latch)
    parameter int W         = SEARCH_W,              // GI − B, 1 … GI_LEN
    localparam int POS_W    = $clog2(HISTORY),
    localparam int P        = NCO_PHASE_BITS
) (
    input  wire logic                     clk,
    input  wire logic                     rst_n,     // synchronous, active low
    input  wire logic                     in_valid,
    input  wire logic signed [Y_BITS-2:0] in_re,     // 12-bit ADC
    input  wire logic signed [Y_BITS-2:0] in_im,
    output logic                          acq_valid,
    output logic [POS_W-1:0]              acq_fft,
    output logic [POS_W-1:0]              acq_t1,
    output logic signed [P-1:0]           acq_cfo,
    output logic signed [P-1:0]           acq_coarse,
    output logic signed [P-1:0]           acq_fine
);
    localparam int E_W  = 2 * CORR_BITS + 4;
    localparam int RA_W = $clog2(LTF_SYMBOL_LEN + GI_LEN + 1);
    localparam int S_W  = CORDIC_IN_BITS;
    localparam int M_W  = 2 * (Z_BITS + TAP_BITS + 7) + 8;
    localparam int N_W  = 2 * Z_BITS + 7;

    logic                              y_valid, cd_valid, d_valid, fire, event_d;
    logic                              l_valid, l_event, z_valid, c_valid;
    logic signed [Y_BITS-1:0]          y_re, y_im, n_re, n_im;
    logic signed [2*DET_BITS+5:0]      cd_re, cd_im;
    logic signed [P-1:0]               l_step;
    logic signed [Z_BITS-1:0]          z_re, z_im;
    logic signed [CORR_BITS-1:0]       c_re, c_im;
    logic                              rq_valid, a_valid, cq_req, cq_grant, cq_valid;
    logic signed [S_W-1:0]             rq_re, rq_im, cq_re, cq_im;
    logic signed [ANGLE_BITS-1:0]      a_angle;
    logic                              ring_re, e_valid, s_valid, searching;
    logic                              acc_valid, acc_lapped;
    logic [RA_W-1:0]                   ring_addr, acc_ring;
    logic [E_W-1:0]                    ring_data, e, acc_emax;
    logic [POS_W-1:0]                  acc_n, acc_imax, acc_opened;
    logic                              dec_valid, acquired;
    // the decision's probes (the gate `decide` reads them through its wrapper)
    logic                              res_valid, res_sums, rot_start;
    logic signed [S_W-1:0]             res_coarse_re, res_coarse_im, res_fine_re, res_fine_im;
    logic [M_W-1:0]                    res_confirmation, res_coherence;
    logic [N_W-1:0]                    res_energy;

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

    assign acq_valid = acquired;
    assign acq_fft   = acq_t1 - POS_W'(GI_LEN - W);          // modulo HISTORY

    logic unused;
    assign unused = &{1'b0, fire, e_valid, e, s_valid, searching, acc_opened, dec_valid,
                      res_valid, res_sums, res_coarse_re, res_coarse_im, res_confirmation,
                      res_coherence, res_energy, res_fine_re, res_fine_im, rot_start};
endmodule

`default_nettype wire
