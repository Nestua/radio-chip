// Memory wrapper: one write port, one read port, one clock, synchronous read. The only way RTL
// in hw/rtl/ may hold a memory (CLAUDE.md, RTL rules): an FPGA infers block RAM from this
// behavioural body; an ASIC flow replaces the body with a macro of the same ports and timing.
// Gate: make -C hw ram (hw/tb/tb_ram.cpp against a reference model of these semantics).
//
// Semantics, all on the rising edge of clk:
//   we:  mem[waddr] <= wdata.
//   re:  rdata <= mem[raddr] as it was before this edge's write (read-first: a read of the
//        address written in the same cycle returns the old word). Without re, rdata holds.
// No reset: the contents are undefined until written (callers track validity); rdata is
// undefined until the first read. DEPTH need not be a power of two; addresses ≥ DEPTH are not
// allowed (undefined).

`default_nettype none

module ram_1w1r #(
    parameter int W     = 16,
    parameter int DEPTH = 64,
    parameter int AW    = $clog2(DEPTH)
) (
    input  wire logic          clk,
    input  wire logic          we,
    input  wire logic [AW-1:0] waddr,
    input  wire logic [W-1:0]  wdata,
    input  wire logic          re,
    input  wire logic [AW-1:0] raddr,
    output logic [W-1:0]       rdata
);
    logic [W-1:0] mem [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (re) rdata <= mem[raddr];
        if (we) mem[waddr] <= wdata;
    end
endmodule

`default_nettype wire
