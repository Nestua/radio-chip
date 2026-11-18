// Test wrapper for gate `ram` (make -C hw ram): the two memory shapes part d uses (ADR 0006).
//   h: ram_1w2r, 26 bits × 512 — the receiver's history y (2 × 13 bits);
//   c: ram_1w1r, 64 bits × 72 — the delay line of detector A's C (2 × 32 bits), a depth that is
//      not a power of two.
// The collision check is off: the random test makes collisions on purpose and does not compare
// those reads (it checks that they disturb nothing else).

`default_nettype none

module ram_bank (
    input  wire logic        clk,
    input  wire logic        h_we,
    input  wire logic [8:0]  h_waddr,
    input  wire logic [25:0] h_wdata,
    input  wire logic        h_re_a,
    input  wire logic [8:0]  h_raddr_a,
    output logic [25:0]      h_rdata_a,
    input  wire logic        h_re_b,
    input  wire logic [8:0]  h_raddr_b,
    output logic [25:0]      h_rdata_b,
    input  wire logic        c_we,
    input  wire logic [6:0]  c_waddr,
    input  wire logic [63:0] c_wdata,
    input  wire logic        c_re,
    input  wire logic [6:0]  c_raddr,
    output logic [63:0]      c_rdata
);
    ram_1w2r #(.W(26), .DEPTH(512), .CHECK_COLLISION(1'b0)) u_h (
        .clk, .we(h_we), .waddr(h_waddr), .wdata(h_wdata),
        .re_a(h_re_a), .raddr_a(h_raddr_a), .rdata_a(h_rdata_a),
        .re_b(h_re_b), .raddr_b(h_raddr_b), .rdata_b(h_rdata_b)
    );
    ram_1w1r #(.W(64), .DEPTH(72), .CHECK_COLLISION(1'b0)) u_c (
        .clk, .we(c_we), .waddr(c_waddr), .wdata(c_wdata),
        .re(c_re), .raddr(c_raddr), .rdata(c_rdata)
    );
endmodule

`default_nettype wire
