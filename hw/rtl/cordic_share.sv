// The data path's iterative CORDIC shared by two requesters (ADR 0010, decision 1): the symbol
// block's common phase (a) and the tracker's pair angles (b). Each holds its request (valid and
// its operands) until take; a has priority on a tie. In the schedule (radio_phy::rx_data) a
// symbol's two pair angles are asked before its phase (SIGNAL's too: its phase waits for the
// products pass), so the priority never decides an order there. The answer
// goes to whoever's request the CORDIC took (the angle is broadcast; out_a / out_b say whose).
// Golden model: none of its own (it moves operands); gate G3 through make -C hw rx_track, where
// the bench's other requester is a.
//
// cordic_iter takes an input while its busy is low, which it already is in its out_valid cycle:
// a take in that cycle changes the owner on the edge after the answer was routed.

`default_nettype none

module cordic_share
    import rx_front_rom_pkg::*;
(
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              drop,
    input  wire                              a_valid,
    input  wire signed [CORDIC_IN_BITS-1:0]  a_re,
    input  wire signed [CORDIC_IN_BITS-1:0]  a_im,
    output logic                             a_take,
    output logic                             out_a,
    input  wire                              b_valid,
    input  wire signed [CORDIC_IN_BITS-1:0]  b_re,
    input  wire signed [CORDIC_IN_BITS-1:0]  b_im,
    output logic                             b_take,
    output logic                             out_b,
    // the CORDIC
    output logic                             c_valid,
    output logic signed [CORDIC_IN_BITS-1:0] c_re,
    output logic signed [CORDIC_IN_BITS-1:0] c_im,
    input  wire                              c_busy,
    input  wire                              c_out_valid
);
    logic owner_b;

    always_comb begin
        a_take  = a_valid && !c_busy;
        b_take  = b_valid && !c_busy && !a_valid;
        c_valid = a_take || b_take;
        c_re    = a_take ? a_re : b_re;
        c_im    = a_take ? a_im : b_im;
        out_a   = c_out_valid && !owner_b;
        out_b   = c_out_valid && owner_b;
    end

    always_ff @(posedge clk)
        if (!rst_n || drop) owner_b <= 1'b0;
        else if (c_valid)   owner_b <= b_take;
endmodule

`default_nettype wire
