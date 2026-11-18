// Memory wrapper: one write port, one read port, one clock, synchronous read. The only way RTL
// in hw/rtl/ may hold a memory (CLAUDE.md, RTL rules): an FPGA infers block RAM from this
// behavioural body; an ASIC flow replaces the body with a macro of the same ports and timing.
// Gate: make -C hw ram (hw/tb/tb_ram.cpp against a reference model of these semantics).
//
// Semantics, all on the rising edge of clk — the subset every FPGA block RAM and ASIC SRAM
// macro provides:
//   we:  mem[waddr] <= wdata.
//   re:  rdata <= mem[raddr] (available after the edge). A read of the address written in the
//        same cycle is ILLEGAL (rdata undefined); with CHECK_COLLISION = 1 simulation stops on
//        one. After a cycle without re, rdata is undefined (callers register what they keep).
// No reset: the contents are undefined until written (callers track validity). DEPTH need not
// be a power of two; addresses ≥ DEPTH are illegal.

`default_nettype none

module ram_1w1r #(
    parameter int W     = 16,
    parameter int DEPTH = 64,
    parameter int AW    = $clog2(DEPTH),
    parameter bit CHECK_COLLISION = 1'b1   // the gate's random test turns it off
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

    // Simulation check of the contract (no hardware: $error is ignored by synthesis).
    if (CHECK_COLLISION) begin : g_check
        always_ff @(posedge clk)
            if (we && re && waddr == raddr)
                $error("ram_1w1r: read and write of address %0d in the same cycle", waddr);
    end
endmodule

`default_nettype wire
