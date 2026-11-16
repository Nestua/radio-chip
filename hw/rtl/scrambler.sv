// 802.11 OFDM data scrambler, S(x) = x^7 + x^4 + 1 (IEEE 802.11-2020 §17.3.5.5).
// Bit-exact with radio_phy::scrambler::Scrambler (gate G3: make -C hw scrambler).
//
// One bit per cycle on a valid/ready-free stream: the scrambler never stalls, so the
// consumer must accept out_bit whenever out_valid is high. Latency: 1 cycle.
// `load` takes `seed` into the state; the next in_valid bit uses that seed.

`default_nettype none

module scrambler (
    input  wire       clk,
    input  wire       rst_n,     // synchronous, active low
    input  wire       load,
    input  wire [6:0] seed,      // state bit i = register x(i+1); must be non-zero
    input  wire       in_valid,
    input  wire       in_bit,
    output logic      out_valid,
    output logic      out_bit
);
    logic [6:0] state;
    logic       seq;

    assign seq = state[6] ^ state[3];  // x7 XOR x4

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state     <= 7'h7f;
            out_valid <= 1'b0;
            out_bit   <= 1'b0;
        end else if (load) begin
            state     <= seed;
            out_valid <= 1'b0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                out_bit <= in_bit ^ seq;
                state   <= {state[5:0], seq};
            end
        end
    end
endmodule

`default_nettype wire
