// |m| · q that the synthesis tool maps by itself: the magnitude of a signed value times an
// unsigned one, in a module of its own, kept as a module through synthesis (the attribute
// below), as lib/add_alone.sv is for an add. Measured through the open flow (yosys with ABC,
// IHP SG13G2; docs/gates/silicon_probe.md, 2026-11-27): one endpoint of rx_symbol's six
// |m|·q products was the last short of 160 MHz inside the flattened design (−0.12 ns); with
// all six in modules nothing fails (+0.33 ns, the whole of rx_data).
// y = |m| · q in YW bits; MW − 1 + QW bits hold it for every m, −2^(MW−1) included (its
// magnitude is read unsigned). A tool that does not know the attribute ignores it.
// Golden model: none of its own (the callers' models). Gate: the callers' gates
// (make -C hw rx_symbol).

`default_nettype none

(* keep_hierarchy *)
module absmul_alone #(
    parameter int MW = 17,
    parameter int QW = 20,
    parameter int YW = 36
) (
    input  wire logic signed [MW-1:0] m,
    input  wire logic        [QW-1:0] q,
    output wire logic        [YW-1:0] y
);
    wire logic signed [MW-1:0] mm = m[MW-1] ? -m : m;
    assign y = YW'(unsigned'(mm)) * YW'(q);
endmodule

`default_nettype wire
