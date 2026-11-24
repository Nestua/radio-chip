// The symbol block with its CORDIC for gate G3 (make -C hw rx_symbol): rx_symbol's requests go
// to cordic_iter, one drop for both; cordic_hold stands for the arbiter giving the CORDIC to the
// tracker (the request waits). The channel's stores, the tracker and the quotients are the
// testbench's.

`default_nettype none

module symbol_chain
    import rx_channel_pkg::*;
    import rx_front_rom_pkg::*;
    import rx_symbol_pkg::*;
(
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              drop,
    input  wire                              cordic_hold,
    input  wire                              bin_valid,
    input  wire        [5:0]                 bin_idx,
    input  wire signed [W_BITS-1:0]          bin_re,
    input  wire signed [W_BITS-1:0]          bin_im,
    output logic                             bins_ok,
    input  wire                              chan_valid,
    input  wire        [4:0]                 chan_shift,
    input  wire        [4:0]                 chan_n,
    output logic                             hs_re,
    output logic       [5:0]                 hs_raddr,
    input  wire signed [HS_BITS-1:0]         hs_rdata_re,
    input  wire signed [HS_BITS-1:0]         hs_rdata_im,
    output logic                             g_re,
    output logic       [5:0]                 g_raddr,
    input  wire        [U_BITS-2:0]          g_rdata,
    input  wire                              scale_valid,
    input  wire        [1:0]                 scale_mod,
    input  wire        [SQ_BITS-1:0]         scale_q,
    input  wire                              mod_valid,
    input  wire        [1:0]                 data_mod,
    output logic                             pil_valid,
    output logic signed [PILOT_BITS-1:0]     pil_re [4],
    output logic signed [PILOT_BITS-1:0]     pil_im [4],
    input  wire                              theta_valid,
    input  wire signed [NCO_PHASE_BITS:0]    theta,
    output logic                             cr_valid,
    output logic                             cr_take,
    output logic                             cordic_busy,
    output logic                             phi_valid,
    output logic signed [NCO_PHASE_BITS-1:0] phi,
    output logic                             soft_valid,
    output logic       [5:0]                 soft_j,
    output logic       [1:0]                 soft_mod,
    output logic                             soft_last,
    output logic signed [SOFT_BITS-1:0]      soft_v [6]
);
    logic signed [CORDIC_IN_BITS-1:0] cr_re, cr_im;
    logic                             ca_valid;
    logic signed [ANGLE_BITS-1:0]     ca_angle;

    assign cr_take = cr_valid && !cordic_busy && !cordic_hold;

    rx_symbol u_sym (
        .clk, .rst_n, .drop,
        .bin_valid, .bin_idx, .bin_re, .bin_im, .bins_ok,
        .chan_valid, .chan_shift, .chan_n,
        .hs_re, .hs_raddr, .hs_rdata_re, .hs_rdata_im,
        .g_re, .g_raddr, .g_rdata,
        .scale_valid, .scale_mod, .scale_q, .mod_valid, .data_mod,
        .pil_valid, .pil_re, .pil_im, .theta_valid, .theta,
        .cr_valid, .cr_re, .cr_im, .cr_take, .ca_valid, .ca_angle,
        .phi_valid, .phi, .soft_valid, .soft_j, .soft_mod, .soft_last, .soft_v
    );

    cordic_iter u_cordic (
        .clk, .rst_n, .drop,
        .in_valid(cr_take), .in_re(cr_re), .in_im(cr_im),
        .busy(cordic_busy), .out_valid(ca_valid), .out_angle(ca_angle)
    );
endmodule

`default_nettype wire
