// The descrambler and the octets out: one frame's decoded DATA bits, from the first SERVICE
// bit, to the PSDU's octets for the MAC (IEEE 802.11-2020 §17.3.5.2, §17.3.5.5).
// Golden model: radio_phy::rx_octets::RxOctets (the bits-to-octets part of RxDataFixed).
// Gate G3: make -C hw rx_octets. G0 and G1: docs/gates/rx_octets.md.
//
// Input: the Viterbi decoder's bits of the data block (viterbi.sv's out_valid/out_bit), a bit
// a clock at most, gaps anywhere; never stalls. `start` with `length` (SIGNAL's LENGTH,
// 1…4095: a 0 would end after 4096 octets, the model never; taken only with the start) arms a
// frame; bits while no frame is armed are ignored
// (SIGNAL's, the tail's). `drop` disarms (the packet ended by a mark). Priority: drop, start,
// then the bit of the same cycle (a bit with a start or a drop is not taken).
//
// Output, each registered on the edge that takes the bit that completes it (latency 1 edge
// after the decoder's out_bit is registered; the schedule's DataTiming::octet = 1):
// - oct_valid for one cycle with oct_data, the PSDU's next octet descrambled, the first
//   received bit in bit 0; oct_first on the first, with oct_seed (the transmitter's seed,
//   bit i = x(i+1)) and oct_service (SERVICE descrambled, the first bit in bit 0; bits 0…6
//   are zero by construction), both steady from the first octet to the frame's end (they
//   change with the next frame's bits: read them with oct_first); oct_last on the last. The
//   data outputs are not reset: they mean something only with their valid;
// - end_valid for one cycle: with the last octet (end_seed 0, decoded), or, when the first
//   seven bits were all zero (no seed: no octet is given), on the bit the last octet would have
//   come on (end_seed 1). The frame is then disarmed: the tail's bits are not read.
//
// Arithmetic (G0, tests/g0_rx_octets.rs):
// - the first seven SERVICE bits are zero before scrambling, so the first seven received bits
//   are the scrambler's outputs and, shifted into the state as they come, are the state after
//   them; from there each bit is descrambled with x7 ⊕ x4, which is shifted in. The head is
//   thus shifted in as received and descrambles to zero, the same path as every later bit;
// - the seed, the state seven steps earlier, by running y(n − 7) = y(n) ⊕ y(n − 4) backwards:
//   seven XORs of the head (the model searches the 127 seeds; equal on all 128 heads);
// - counts: 4 bits through SERVICE, 3 within an octet, 12 for the octets left (LENGTH ≤ 4095).

`default_nettype none

module rx_octets (
    input  wire         clk,
    input  wire         rst_n,       // synchronous, active low
    input  wire         start,
    input  wire  [11:0] length,
    input  wire         drop,
    input  wire         in_valid,
    input  wire         in_bit,
    output logic        oct_valid,
    output logic  [7:0] oct_data,
    output logic        oct_first,
    output logic        oct_last,
    output logic  [6:0] oct_seed,
    output logic [15:0] oct_service,
    output logic        end_valid,
    output logic        end_seed
);
    logic        armed;      // a frame is being read
    logic        psdu;       // past SERVICE
    logic        zero;       // the head was all zero: count only (set on every head)
    logic        first;      // the next octet is the frame's first
    logic  [3:0] cnt;        // bit of SERVICE (0…15), then of the octet (bits 2:0, wrapping)
    logic [11:0] left;       // octets left, the current one included
    logic  [6:0] state;      // the scrambler, bit i = x(i+1)
    logic  [6:0] part;       // the current octet's bits so far, the newest in bit 6
    logic  [8:0] service;    // SERVICE bits 7…15, descrambled

    wire       take = armed && in_valid;            // after drop and start in the chain below
    wire       head = !psdu && cnt < 4'd7;
    wire       seq  = state[6] ^ state[3];          // x7 XOR x4
    wire       c    = head ? 1'b0 : in_bit ^ seq;   // the descrambled bit
    wire [6:0] next = {state[5:0], in_bit ^ c};     // the head: as received; then the sequence
    wire       done = psdu && cnt[2:0] == 3'd7;     // an octet's last bit
    wire       last = left == 12'd1;

    // The seed from the head h(0…6) = next[6…0] (the state after it, x1 the newest).
    function automatic logic [6:0] seed_of(input logic [6:0] s);
        logic m1, m2, m3, m4, m5, m6, m7;
        m1 = s[0] ^ s[4];       // h6 ^ h2
        m2 = s[1] ^ s[5];       // h5 ^ h1
        m3 = s[2] ^ s[6];       // h4 ^ h0
        m4 = s[3] ^ m1;         // h3 ^ y(-1)
        m5 = s[4] ^ m2;         // h2 ^ y(-2)
        m6 = s[5] ^ m3;         // h1 ^ y(-3)
        m7 = s[6] ^ m4;         // h0 ^ y(-4)
        return {m7, m6, m5, m4, m3, m2, m1};
    endfunction

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            armed     <= 1'b0;
            oct_valid <= 1'b0;
            end_valid <= 1'b0;
        end else begin
            oct_valid <= 1'b0;
            end_valid <= 1'b0;
            if (drop) begin
                armed <= 1'b0;
            end else if (start) begin
                armed <= 1'b1;
                psdu  <= 1'b0;
                first <= 1'b1;
                cnt   <= 4'd0;
                left  <= length;
            end else if (take) begin
                state <= next;
                cnt   <= cnt + 4'd1;
                if (!psdu) begin
                    if (cnt == 4'd6) begin
                        zero     <= next == 7'd0;
                        oct_seed <= seed_of(next);
                    end
                    service <= {c, service[8:1]};   // the last nine: bits 7…15
                    if (cnt == 4'd15) psdu <= 1'b1;     // cnt wraps to 0
                end else begin
                    part <= {c, part[6:1]};
                    if (done) begin
                        left <= left - 12'd1;
                        if (!zero) begin
                            oct_valid <= 1'b1;
                            oct_data  <= {c, part};
                            oct_first <= first;
                            oct_last  <= last;
                            first     <= 1'b0;
                        end
                        if (last) begin
                            end_valid <= 1'b1;
                            end_seed  <= zero;
                            armed     <= 1'b0;
                        end
                    end
                end
            end
        end
    end

    assign oct_service = {service, 7'd0};
endmodule

`default_nettype wire
