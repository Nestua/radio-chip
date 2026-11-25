// ram_1w1r for the open ASIC flow on IHP SG13G2: the body hw/rtl/lib/ram_1w1r.sv leaves to "an
// ASIC flow replaces the body with a macro of the same ports and timing". Used only by the
// silicon probe (tools/syn/orfs_block.sh with the config tools/syn/ihp_sg13g2/rx_data_sram.mk,
// which lists this file in place of hw/rtl/lib/ram_1w1r.sv; docs/gates/silicon_probe.md); not
// RTL of the chip, and not simulated by the gates. A_DLY/B_DLY are tied to 1, not checked
// against the macros' datasheet.
//
// The macros are the PDK's two-port SRAMs (IHP-Open-PDK, Apache-2.0,
// ihp-sg13g2/libs.ref/sg13g2_sram: RM_IHPSG13_2P_64x32_c2 and RM_IHPSG13_2P_64x22_c2_bm_bist),
// used as black boxes from their LEF and Liberty views: port A writes, port B reads, one clock
// on both. The wrapper's contract (one write, one read, synchronous read of latency 1, a read of
// the address written in the same cycle illegal, rdata undefined after a cycle without re)
// is the subset these macros provide (docs/gates/memory.md, 2026-11-19).
//
// Only DEPTH ≤ 64 is built from macros (every store of the data path, rx_data); a deeper
// memory keeps the behavioural body (flip-flops), as before. The width is covered by macros of
// 32 and 22 bits, the cheapest mix: ≤ 22 one 22; ≤ 32 one 32; ≤ 44 two 22; ≤ 54 32 + 22;
// ≤ 64 two 32 (the unused bits tied off).

`default_nettype none

module ram_1w1r #(
    parameter int W     = 16,
    parameter int DEPTH = 64,
    parameter int AW    = $clog2(DEPTH),
    parameter bit CHECK_COLLISION = 1'b1
) (
    input  wire logic          clk,
    input  wire logic          we,
    input  wire logic [AW-1:0] waddr,
    input  wire logic [W-1:0]  wdata,
    input  wire logic          re,
    input  wire logic [AW-1:0] raddr,
    output logic [W-1:0]       rdata
);
    if (DEPTH > 64 || W > 64) begin : g_flops
        logic [W-1:0] mem [0:DEPTH-1];
        always_ff @(posedge clk) begin
            if (re) rdata <= mem[raddr];
            if (we) mem[waddr] <= wdata;
        end
    end else begin : g_macros
        // n32 macros of 32 bits, then n22 of 22.
        localparam int N32 = W <= 22 ? 0 : W <= 32 ? 1 : W <= 44 ? 0 : W <= 54 ? 1 : 2;
        localparam int N22 = W <= 22 ? 1 : W <= 32 ? 0 : W <= 44 ? 2 : W <= 54 ? 1 : 0;
        localparam int WP  = 32 * N32 + 22 * N22;
        logic [WP-1:0] din, dout;
        logic [5:0]    wa, ra;
        assign din = WP'(wdata);
        assign wa  = 6'(waddr);
        assign ra  = 6'(raddr);
        for (genvar i = 0; i < N32; i++) begin : g32
            RM_IHPSG13_2P_64x32_c2 u_m (
                .A_CLK(clk), .A_MEN(we), .A_WEN(we), .A_REN(1'b0), .A_ADDR(wa),
                .A_DIN(din[32*i +: 32]), .A_DLY(1'b1), .A_DOUT(),
                .B_CLK(clk), .B_MEN(re), .B_WEN(1'b0), .B_REN(re), .B_ADDR(ra),
                .B_DIN(32'b0), .B_DLY(1'b1), .B_DOUT(dout[32*i +: 32])
            );
        end
        for (genvar i = 0; i < N22; i++) begin : g22
            RM_IHPSG13_2P_64x22_c2_bm_bist u_m (
                .A_CLK(clk), .A_MEN(we), .A_WEN(we), .A_REN(1'b0), .A_ADDR(wa),
                .A_DIN(din[32*N32 + 22*i +: 22]), .A_DLY(1'b1), .A_DOUT(), .A_BM({22{1'b1}}),
                .A_BIST_CLK(1'b0), .A_BIST_EN(1'b0), .A_BIST_MEN(1'b0), .A_BIST_WEN(1'b0),
                .A_BIST_REN(1'b0), .A_BIST_ADDR(6'b0), .A_BIST_DIN(22'b0), .A_BIST_BM(22'b0),
                .B_CLK(clk), .B_MEN(re), .B_WEN(1'b0), .B_REN(re), .B_ADDR(ra),
                .B_DIN(22'b0), .B_DLY(1'b1), .B_DOUT(dout[32*N32 + 22*i +: 22]), .B_BM({22{1'b1}}),
                .B_BIST_CLK(1'b0), .B_BIST_EN(1'b0), .B_BIST_MEN(1'b0), .B_BIST_WEN(1'b0),
                .B_BIST_REN(1'b0), .B_BIST_ADDR(6'b0), .B_BIST_DIN(22'b0), .B_BIST_BM(22'b0)
            );
        end
        assign rdata = dout[W-1:0];
    end
endmodule

`default_nettype wire
