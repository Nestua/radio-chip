// The tracker with the data path's shared units for gate G3 (make -C hw rx_track): rx_track is
// requester b of cordic_share (before cordic_iter) and of div_share (before div_serial); the
// testbench is requester a of both (the symbol block's common phase, the channel block's
// quotients), as the top level will connect them. One drop for all.

`default_nettype none

module track_chain
    import rx_front_rom_pkg::*;
    import rx_symbol_pkg::*;
    import rx_track_pkg::*;
(
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              drop,
    input  wire                              start,
    input  wire signed [NCO_PHASE_BITS-1:0]  cfo_word,
    input  wire        [15:0]                ratio,
    input  wire                              q_valid,
    input  wire        [2:0]                 q_job,
    input  wire        [27:0]                q_value,
    input  wire                              pil_valid,
    input  wire signed [PILOT_BITS-1:0]      pil_re [4],
    input  wire signed [PILOT_BITS-1:0]      pil_im [4],
    output logic                             theta_valid,
    output logic signed [NCO_PHASE_BITS:0]   theta,
    output logic                             move_early,
    output logic                             move_late,
    output logic signed [FIT_QBITS:0]        fit_delta,
    // the other CORDIC requester
    input  wire                              ca_req,
    input  wire signed [CORDIC_IN_BITS-1:0]  ca_re,
    input  wire signed [CORDIC_IN_BITS-1:0]  ca_im,
    output logic                             ca_take,
    output logic                             ca_out,
    output logic signed [ANGLE_BITS-1:0]     angle,
    // the other divider requester
    input  wire                              da_start,
    input  wire        [47:0]                da_num,
    input  wire        [23:0]                da_den,
    input  wire        [4:0]                 da_qbits,
    output logic                             da_done,
    output logic       [27:0]                quot,
    // the tracker's side, observed
    output logic                             tr_cr_valid,
    output logic                             tr_cr_take,
    output logic                             tr_ca_valid,
    output logic                             tr_dv_req,
    output logic                             tr_dv_gnt,
    output logic                             tr_dv_done
);
    logic signed [CORDIC_IN_BITS-1:0] tr_re, tr_im, c_re, c_im;
    logic                             c_valid, c_busy, c_out;
    logic [FIT_NUM_BITS-1:0]          tr_num, d_num;
    logic [FIT_DEN_BITS-1:0]          tr_den, d_den;
    logic [4:0]                       d_qbits;
    logic                             d_start, d_busy, d_done;

    rx_track u_track (
        .clk, .rst_n, .drop, .start, .cfo_word, .ratio,
        .q_valid, .q_job, .q_value, .pil_valid, .pil_re, .pil_im,
        .theta_valid, .theta, .move_early, .move_late, .fit_delta,
        .cr_valid(tr_cr_valid), .cr_re(tr_re), .cr_im(tr_im), .cr_take(tr_cr_take),
        .ca_valid(tr_ca_valid), .ca_angle(angle),
        .dv_req(tr_dv_req), .dv_num(tr_num), .dv_den(tr_den), .dv_gnt(tr_dv_gnt),
        .dv_done(tr_dv_done), .dv_quot(quot)
    );

    cordic_share u_cshare (
        .clk, .rst_n, .drop,
        .a_valid(ca_req), .a_re(ca_re), .a_im(ca_im), .a_take(ca_take), .out_a(ca_out),
        .b_valid(tr_cr_valid), .b_re(tr_re), .b_im(tr_im), .b_take(tr_cr_take), .out_b(tr_ca_valid),
        .c_valid, .c_re, .c_im, .c_busy, .c_out_valid(c_out)
    );

    cordic_iter u_cordic (
        .clk, .rst_n, .drop,
        .in_valid(c_valid), .in_re(c_re), .in_im(c_im),
        .busy(c_busy), .out_valid(c_out), .out_angle(angle)
    );

    div_share #(
        .A_NUM_W(48), .A_DEN_W(24), .B_NUM_W(FIT_NUM_BITS), .B_DEN_W(FIT_DEN_BITS),
        .NUM_W(FIT_NUM_BITS), .DEN_W(FIT_DEN_BITS), .QB_W(5)
    ) u_dshare (
        .clk, .rst_n, .drop,
        .a_start(da_start), .a_num(da_num), .a_den(da_den), .a_qbits(da_qbits), .a_done(da_done),
        .b_req(tr_dv_req), .b_num(tr_num), .b_den(tr_den), .b_qbits(5'(FIT_QBITS)),
        .b_gnt(tr_dv_gnt), .b_done(tr_dv_done),
        .d_start, .d_num, .d_den, .d_qbits, .d_busy, .d_done
    );

    div_serial #(.NUM_W(FIT_NUM_BITS), .DEN_W(FIT_DEN_BITS), .Q_W(28)) u_div (
        .clk, .rst_n, .drop,
        .start(d_start), .num(d_num), .den(d_den), .qbits(d_qbits),
        .busy(d_busy), .done(d_done), .quot
    );
endmodule

`default_nettype wire
