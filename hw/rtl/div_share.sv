// The data path's serial divider shared by two requesters (ADR 0010, decision 1): the channel
// block's quotients (a) and the tracker's fit (b). a pulses its start with its operands and
// waits for its done (rx_channel as built); when the divider is busy the request waits here in a
// register and goes on the first free edge. b holds its request until b_gnt. a has priority
// on the same edge. done goes to whoever's division it is (the quotient is broadcast).
// Golden model: none of its own; gate G3 through make -C hw rx_track (the bench's channel is
// a) and make -C hw rx_channel (a alone, unchanged).
//
// div_serial takes a start while its busy is low, which it already is in its done cycle. An
// a request with no wait passes in the same cycle: the channel's latencies are as before when
// the tracker is not dividing. One a request at a time (the channel waits for its done).

`default_nettype none

module div_share #(
    parameter int A_NUM_W = 48,
    parameter int A_DEN_W = 24,
    parameter int B_NUM_W = 64,
    parameter int B_DEN_W = 47,
    parameter int NUM_W   = 64,
    parameter int DEN_W   = 47,
    parameter int QB_W    = 5
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   drop,
    input  wire                   a_start,
    input  wire     [A_NUM_W-1:0] a_num,
    input  wire     [A_DEN_W-1:0] a_den,
    input  wire     [QB_W-1:0]    a_qbits,
    output logic                  a_done,
    input  wire                   b_req,
    input  wire     [B_NUM_W-1:0] b_num,
    input  wire     [B_DEN_W-1:0] b_den,
    input  wire     [QB_W-1:0]    b_qbits,
    output logic                  b_gnt,
    output logic                  b_done,
    // the divider
    output logic                  d_start,
    output logic    [NUM_W-1:0]   d_num,
    output logic    [DEN_W-1:0]   d_den,
    output logic    [QB_W-1:0]    d_qbits,
    input  wire                   d_busy,
    input  wire                   d_done
);
    logic               pend;
    logic [A_NUM_W-1:0] p_num;
    logic [A_DEN_W-1:0] p_den;
    logic [QB_W-1:0]    p_qbits;
    logic               owner_b;
    logic               a_out;      // a's division asked, its done not yet out

    wire want_a = a_start || pend;
    wire gnt_a  = want_a && !d_busy;

    always_comb begin
        b_gnt   = b_req && !d_busy && !want_a;
        d_start = gnt_a || b_gnt;
        if (gnt_a) begin
            d_num   = pend ? NUM_W'(p_num) : NUM_W'(a_num);
            d_den   = pend ? DEN_W'(p_den) : DEN_W'(a_den);
            d_qbits = pend ? p_qbits : a_qbits;
        end else begin
            d_num   = NUM_W'(b_num);
            d_den   = DEN_W'(b_den);
            d_qbits = b_qbits;
        end
        a_done = d_done && !owner_b;
        b_done = d_done && owner_b;
    end

    always_ff @(posedge clk)
        if (!rst_n || drop) begin
            pend    <= 1'b0;
            owner_b <= 1'b0;
            a_out   <= 1'b0;
        end else begin
            if (a_start && !gnt_a) begin
                pend    <= 1'b1;
                p_num   <= a_num;
                p_den   <= a_den;
                p_qbits <= a_qbits;
            end else if (gnt_a) begin
                pend <= 1'b0;
            end
            if (d_start) owner_b <= b_gnt;
            if (a_start)     a_out <= 1'b1;
            else if (a_done) a_out <= 1'b0;
        end

`ifndef SYNTHESIS
    always_ff @(posedge clk)
        if (rst_n && !drop && a_start && a_out && !a_done)
            $error("div_share: a second request from a before its done");
`endif
endmodule

`default_nettype wire
