// Test wrapper for gate G3 `search` (make -C hw search): the receiver front up to the LTF
// search as the top level will wire it — dc_block → det_a → rx_latch (NCO path D_SAMPLES
// behind, with each sample's detection event) → nco → ltf_corr → rx_search. Not synthesised
// on its own.
//
// in_clear marks the sample at which the model applies an acquisition (until the decision
// engine exists, part d step 3, the vectors supply it): delayed to det_a's stage 4 it clears
// the run counter (as in latch_chain) and is the search's `acquired`. ring_* is the energy
// ring's read port, the decision's in step 3 and the testbench's here.

`default_nettype none

module search_chain
    import rx_front_rom_pkg::*;
#(
    parameter int D_SAMPLES = 4,
    parameter int W         = SEARCH_W,
    localparam int E_W      = 2 * CORR_BITS + 4,
    localparam int POS_W    = $clog2(HISTORY),
    localparam int RA_W     = $clog2(LTF_SYMBOL_LEN + GI_LEN + 1)
) (
    input  wire logic                         clk,
    input  wire logic                         rst_n,
    input  wire logic                         in_valid,
    input  wire logic signed [Y_BITS-2:0]     in_re,      // 12-bit ADC
    input  wire logic signed [Y_BITS-2:0]     in_im,
    input  wire logic                         in_clear,
    input  wire logic                         ring_re,
    input  wire logic [RA_W-1:0]              ring_addr,
    output logic [E_W-1:0]                    ring_data,
    output logic                              e_valid,
    output logic [E_W-1:0]                    e,
    output logic                              s_valid,
    output logic                              searching,
    output logic                              acc_valid,
    output logic [POS_W-1:0]                  acc_n,
    output logic [POS_W-1:0]                  acc_imax,
    output logic [RA_W-1:0]                   acc_ring,
    output logic [E_W-1:0]                    acc_emax,
    output logic [POS_W-1:0]                  acc_opened
);
    logic                              y_valid, cd_valid, d_valid, fire, event_d;
    logic                              l_valid, l_event, z_valid, c_valid;
    logic signed [Y_BITS-1:0]          y_re, y_im, n_re, n_im;
    logic signed [2*DET_BITS+5:0]      cd_re, cd_im;
    logic signed [NCO_PHASE_BITS-1:0]  l_step;
    logic signed [Z_BITS-1:0]          z_re, z_im;
    logic signed [CORR_BITS-1:0]       c_re, c_im;
    logic [3:0]                        clr;

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
        .c_valid(cd_valid), .c_re(cd_re), .c_im(cd_im), .out_valid(d_valid), .fire,
        .event_o(event_d)
    );
    rx_latch #(.D_SAMPLES(D_SAMPLES)) u_latch (
        .clk, .rst_n, .y_valid, .y_re, .y_im, .c_valid(cd_valid), .c_re(cd_re), .c_im(cd_im),
        .d_valid, .event_i(event_d),
        .out_valid(l_valid), .out_re(n_re), .out_im(n_im), .out_step(l_step),
        .out_event(l_event)
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
        .acquired(clr[3]),
        .e_valid, .e, .out_valid(s_valid), .searching,
        .acc_valid, .acc_n, .acc_imax, .acc_ring, .acc_emax, .acc_opened,
        .ring_re, .ring_addr, .ring_data
    );

    logic unused;
    assign unused = fire;
endmodule

`default_nettype wire
