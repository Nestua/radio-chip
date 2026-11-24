// Test harness for the rx_channel G3 gate (not synthesised): the channel block and the serial
// divider it shares in the data path, on one drop (rx_channel.sv's contract).

`default_nettype none

module channel_chain
    import rx_channel_pkg::*;
(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         drop,
    input  wire                         bin_valid,
    input  wire                         bin_t2,
    input  wire        [5:0]            bin_idx,
    input  wire signed [W_BITS-1:0]     bin_re,
    input  wire signed [W_BITS-1:0]     bin_im,
    output logic                        chan_valid,
    output logic       [4:0]            shift,
    output logic       [GSUM_BITS-1:0]  gsum,
    output logic       [4:0]            gsum_n,
    output logic                        q_valid,
    output logic       [2:0]            q_job,
    output logic       [27:0]           q_value,
    input  wire                         scale_req,
    input  wire        [1:0]            scale_mod,
    output logic                        div_busy,
    input  wire                         hs_re,
    input  wire        [5:0]            hs_raddr,
    output logic signed [HS_BITS-1:0]   hs_rdata_re,
    output logic signed [HS_BITS-1:0]   hs_rdata_im,
    input  wire                         g_re,
    input  wire        [5:0]            g_raddr,
    output logic       [U_BITS-2:0]     g_rdata
);
    localparam int NUM_W = 48, DEN_W = 24, Q_W = 28, QB_W = $clog2(Q_W + 1);

    logic             div_start, div_done;
    logic [NUM_W-1:0] div_num;
    logic [DEN_W-1:0] div_den;
    logic [QB_W-1:0]  div_qbits;
    logic [Q_W-1:0]   div_quot;

    rx_channel #(.NUM_W(NUM_W), .DEN_W(DEN_W), .Q_W(Q_W)) u_ch (
        .clk, .rst_n, .drop, .bin_valid, .bin_t2, .bin_idx, .bin_re, .bin_im,
        .chan_valid, .shift, .gsum, .gsum_n, .q_valid, .q_job, .q_value, .scale_req, .scale_mod,
        .div_start, .div_num, .div_den, .div_qbits, .div_done, .div_quot,
        .hs_re, .hs_raddr, .hs_rdata_re, .hs_rdata_im, .g_re, .g_raddr, .g_rdata
    );

    div_serial #(.NUM_W(NUM_W), .DEN_W(DEN_W), .Q_W(Q_W)) u_div (
        .clk, .rst_n, .drop, .start(div_start), .num(div_num), .den(div_den), .qbits(div_qbits),
        .busy(div_busy), .done(div_done), .quot(div_quot)
    );
endmodule

`default_nettype wire
