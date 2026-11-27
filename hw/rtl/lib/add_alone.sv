// An add that the synthesis tool maps by itself: a + b in a module of its own, kept as a
// module through synthesis (the attribute below). For an add behind a late input, such as a
// memory macro's read. Measured through the open flow (yosys with ABC, IHP SG13G2;
// docs/gates/silicon_probe.md, 2026-11-27): Hs's 18-bit add in rx_channel left synthesis as a
// ripple-carry chain inside the flattened design (5.0 ns behind the macro's read) and as 13
// cells (1.2 ns) alone in a module. The likely reason, not shown beyond those two results:
// the mapper recovers area on every path shorter than the module's longest and takes a
// macro's output as an input at time 0.
// No function of its own: y = a + b, modulo 2^W. A tool that does not know the attribute
// ignores it.
// Golden model: none of its own (the callers' models). Gate: the callers' gates
// (make -C hw rx_channel).

`default_nettype none

(* keep_hierarchy *)
module add_alone #(
    parameter int W = 16
) (
    input  wire logic signed [W-1:0] a,
    input  wire logic signed [W-1:0] b,
    output wire logic signed [W-1:0] y
);
    assign y = a + b;
endmodule

`default_nettype wire
