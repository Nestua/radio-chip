// An add that the synthesis tool maps by itself: a + b in a module of its own, kept as a
// module through synthesis (the attribute below). For an add behind a late input, such as a
// memory macro's read: the open flow's mapper (ABC under yosys) gives all of a module's logic
// one required time, the depth of its longest path, and knows nothing of a macro's read
// delay, so an add in a large module is recovered for area into a ripple-carry chain. Alone,
// its longest path is the add's own and it is mapped for speed (docs/gates/silicon_probe.md,
// 2026-11-27: Hs's 22-bit add in rx_channel, about 30 gates in series against about 10).
// No function of its own: y = a + b, modulo 2^W. Other tools ignore the attribute.
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
