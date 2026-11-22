// The receiver front's one CORDIC with its arbiter (ADR 0006: the provisional latch before
// the decision engine; the schedule of radio_phy::rx_decision). Golden model of the angles:
// radio_phy::rx_front_fixed::cordic_angle (hw/rtl/cordic.sv). Gate G3: make -C hw cordic_arb.
//
// Two requesters:
//   a  the latch (rx_latch): a_req is a one-cycle pulse with its input and is always taken.
//   b  the decision engine: holds b_req and its input until the cycle b_grant is 1, which is
//      every cycle a does not ask (b_grant = b_req and not a_req, combinational). A decision
//      that meets a latch asks again in the next cycle: one cycle lost. A latch in every
//      cycle would keep b waiting without end; the stream's latches are DET_RUN samples
//      apart at least.
// The chosen input is registered (the arbiter's stage) and goes through the CORDIC; the
// owner travels beside it. a_valid or b_valid marks `angle` CORDIC_ITERS + 2 = 18 clock edges
// after the edge that takes the request (cycle to cycle, g + 19: rx_decision's ARB + CORDIC).
// Never stalls; requests may come back to back. Both requesters share this reset: a request
// in a reset cycle is dropped, whatever b_grant shows in it.

`default_nettype none

module cordic_arb
    import rx_front_rom_pkg::*;
(
    input  wire logic                             clk,
    input  wire logic                             rst_n,     // synchronous, active low
    input  wire logic                             a_req,
    input  wire logic signed [CORDIC_IN_BITS-1:0] a_re,
    input  wire logic signed [CORDIC_IN_BITS-1:0] a_im,
    input  wire logic                             b_req,
    input  wire logic signed [CORDIC_IN_BITS-1:0] b_re,
    input  wire logic signed [CORDIC_IN_BITS-1:0] b_im,
    output logic                                  b_grant,
    output logic                                  a_valid,
    output logic                                  b_valid,
    output logic signed [ANGLE_BITS-1:0]          angle
);
    localparam int LAT = CORDIC_ITERS + 1;       // the CORDIC's edges after its taking edge

    assign b_grant = b_req && !a_req;

    // ---- the arbiter's stage
    logic                             in_valid, in_b;
    logic signed [CORDIC_IN_BITS-1:0] in_re, in_im;
    always_ff @(posedge clk) begin
        if (!rst_n) in_valid <= 1'b0;
        else        in_valid <= a_req || b_req;
        in_b  <= !a_req;
        in_re <= a_req ? a_re : b_re;
        in_im <= a_req ? a_im : b_im;
    end

    logic out_valid;
    cordic u_cordic (
        .clk, .rst_n, .in_valid, .in_re, .in_im, .out_valid, .out_angle(angle)
    );

    // ---- the owner, beside the CORDIC's stages (read only with out_valid: no reset)
    // The edge at which the CORDIC takes the input puts its owner into own[0]; the angle is
    // seen LAT edges later, beside own[LAT].
    logic [LAT:0] own;
    always_ff @(posedge clk) own <= {own[LAT-1:0], in_b};

    assign a_valid = out_valid && !own[LAT];
    assign b_valid = out_valid && own[LAT];
endmodule

`default_nettype wire
