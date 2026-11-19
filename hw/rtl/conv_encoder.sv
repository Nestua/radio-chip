// 802.11 convolutional encoder: K = 7, g0 = 133₈, g1 = 171₈, rate 1/2 (IEEE 802.11-2020
// §17.3.5.6). Golden model: radio_phy::convolutional::ConvolutionalEncoder::encode_bit.
// Gate G3: make -C hw conv_encoder.
//
// The six delay cells hold the previous inputs: d[5] the last one (D1), d[0] the oldest (D6).
// With the current input u in front, the window {u, d} is read by the generators MSB first:
//   A = u ⊕ D2 ⊕ D3 ⊕ D5 ⊕ D6     (133₈ = 1011011)
//   B = u ⊕ D1 ⊕ D2 ⊕ D3 ⊕ D6     (171₈ = 1111001)
// Puncturing to rates 2/3 and 3/4 is the consumer's (it steals output bits; the encoder runs
// the same at every rate).
//
// Stream: one bit per in_valid, never stalls; the pair (A, B) is registered one clock edge
// after its input. `clear` returns the cells to zero for the next frame (SERVICE starts from
// state 0) and takes precedence over an input in the same cycle, which is dropped.

`default_nettype none

module conv_encoder (
    input  wire  clk,
    input  wire  rst_n,     // synchronous, active low
    input  wire  clear,
    input  wire  in_valid,
    input  wire  in_bit,
    output logic out_valid,
    output logic out_a,
    output logic out_b
);
    logic [5:0] d;

    always_ff @(posedge clk) begin
        if (!rst_n || clear) begin
            d         <= '0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                out_a <= in_bit ^ d[4] ^ d[3] ^ d[1] ^ d[0];
                out_b <= in_bit ^ d[5] ^ d[4] ^ d[3] ^ d[0];
                d     <= {in_bit, d[5:1]};
            end
        end
    end
endmodule

`default_nettype wire
