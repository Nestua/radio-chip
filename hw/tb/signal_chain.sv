// Bench wrapper for make -C hw rx_signal: the pairs rx_deint would hand on go to the Viterbi
// decoder (hw/rtl/viterbi.sv) and to rx_signal together, and rx_signal takes the decoder's own
// bits, as the data path's top level will wire them. Not synthesised.

`default_nettype none

module signal_chain (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              drop,
    input  wire              v_valid,
    input  wire              v_first,
    input  wire              v_last,
    input  wire signed [7:0] v_a,
    input  wire signed [7:0] v_b,
    output logic             dec_valid,
    output logic             dec_bit,
    output logic             dec_last,
    output logic             sig_valid,
    output logic             sig_fail,
    output logic [1:0]       sig_why,
    output logic [5:0]       sig_agree,
    output logic [5:0]       sig_disagree,
    output logic [2:0]       sig_rate,
    output logic [11:0]      sig_length,
    output logic [10:0]      sig_symbols,
    output logic             sig_patient,
    output logic [15:0]      sig_steps,
    output logic [1:0]       sig_code,
    output logic [1:0]       sig_mod
);
    viterbi u_vit (
        .clk, .rst_n, .in_valid(v_valid), .in_first(v_first), .in_last(v_last),
        .in_a(v_a), .in_b(v_b), .out_valid(dec_valid), .out_bit(dec_bit), .out_last(dec_last)
    );
    rx_signal u_sig (
        .clk, .rst_n, .drop, .v_valid, .v_first, .v_last, .v_a, .v_b,
        .dec_valid, .dec_bit, .dec_last,
        .sig_valid, .sig_fail, .sig_why, .sig_agree, .sig_disagree, .sig_rate, .sig_length,
        .sig_symbols, .sig_patient, .sig_steps, .sig_code, .sig_mod
    );
endmodule

`default_nettype wire
