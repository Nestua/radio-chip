// An unsigned product that the synthesis tool maps by itself: a · b in a module of its own,
// kept as a module through synthesis (the attribute below), as lib/add_alone.sv is for an add.
// Measured through the open flow (yosys with ABC, IHP SG13G2; docs/gates/silicon_probe.md,
// 2026-11-27): rx_channel's 37 × 8 products left synthesis 8.1 ns deep inside the flattened
// design and 3.0 ns deep alone in a module. The likely reason is add_alone's: the mapper
// recovers area on every path shorter than the module's longest.
// No function of its own: y = a · b, exact, AW + BW bits. A tool that does not know the
// attribute ignores it.
// Golden model: none of its own (the callers' models). Gate: the callers' gates
// (make -C hw rx_channel).

`default_nettype none

(* keep_hierarchy *)
module mul_alone #(
    parameter int AW = 37,
    parameter int BW = 8
) (
    input  wire logic [AW-1:0]    a,
    input  wire logic [BW-1:0]    b,
    output wire logic [AW+BW-1:0] y
);
    assign y = (AW+BW)'(a) * (AW+BW)'(b);
endmodule

`default_nettype wire
