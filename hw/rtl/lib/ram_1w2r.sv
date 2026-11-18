// Memory wrapper: one write port, two independent read ports, synchronous read — each read
// port with the semantics of ram_1w1r (a same-cycle read of the written address is illegal). Built from two ram_1w1r copies written
// together (on an FPGA two block RAMs; an ASIC flow may substitute a 1W2R macro of the same
// timing). Gate: make -C hw ram.

`default_nettype none

module ram_1w2r #(
    parameter int W     = 16,
    parameter int DEPTH = 64,
    parameter int AW    = $clog2(DEPTH),
    parameter bit CHECK_COLLISION = 1'b1
) (
    input  wire logic          clk,
    input  wire logic          we,
    input  wire logic [AW-1:0] waddr,
    input  wire logic [W-1:0]  wdata,
    input  wire logic          re_a,
    input  wire logic [AW-1:0] raddr_a,
    output logic [W-1:0]       rdata_a,
    input  wire logic          re_b,
    input  wire logic [AW-1:0] raddr_b,
    output logic [W-1:0]       rdata_b
);
    ram_1w1r #(.W(W), .DEPTH(DEPTH), .AW(AW), .CHECK_COLLISION(CHECK_COLLISION)) u_a (
        .clk, .we, .waddr, .wdata, .re(re_a), .raddr(raddr_a), .rdata(rdata_a)
    );
    ram_1w1r #(.W(W), .DEPTH(DEPTH), .AW(AW), .CHECK_COLLISION(CHECK_COLLISION)) u_b (
        .clk, .we, .waddr, .wdata, .re(re_b), .raddr(raddr_b), .rdata(rdata_b)
    );
endmodule

`default_nettype wire
