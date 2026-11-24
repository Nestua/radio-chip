// The data path's serial divider (ADR 0010, decision 1: one divider for the per-packet
// quotients and the tracker's division a symbol): unsigned, restoring, one quotient bit a clock.
// Golden model: the integer divisions of radio_phy::rx_ofdm_fixed (tracker_constants,
// soft_scale; later the tracker's fit), held through hw/rtl/rx_channel.sv.
// Gate G3: make -C hw rx_channel (tests/g0_rx_channel.rs restates the algorithm).
//
// quot = floor(num / den) in qbits bits, legal when num < den·2^qbits (the caller's bound,
// docs/gates/rx_channel.md G0; simulation stops on a violation). The remainder starts as
// floor(num / 2^qbits), which is below den, and the numerator's last qbits bits are shifted in
// one a clock: the remainder never needs more than DEN_W + 1 bits. den = 0 gives all ones
// (the caller replaces the quotient; the run's length does not depend on the operands).
//
// Timing: start taken on an edge e while idle; the quotient's bits on e + 1 … e + qbits; done
// high and quot valid for the one cycle after edge e + qbits; busy from e + 1 to that edge.
// A start while busy is a protocol error. drop (and rst_n) abandon a division.

`default_nettype none

module div_serial #(
    parameter int NUM_W = 48,
    parameter int DEN_W = 24,
    parameter int Q_W   = 28,
    parameter int QB_W  = $clog2(Q_W + 1)
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    drop,
    input  wire                    start,
    input  wire        [NUM_W-1:0] num,
    input  wire        [DEN_W-1:0] den,
    input  wire        [QB_W-1:0]  qbits,   // 1 … Q_W
    output logic                   busy,
    output logic                   done,
    output logic       [Q_W-1:0]   quot
);
    logic [DEN_W-1:0] rem;
    logic [NUM_W-1:0] rest;     // the numerator's bits not yet shifted in, MSB first
    logic [DEN_W-1:0] dv;
    logic [QB_W-1:0]  left;

    wire [DEN_W:0] trial = {rem, rest[NUM_W-1]};
    wire           take  = trial >= {1'b0, dv};
    // When taken, trial − dv < dv < 2^DEN_W: the low DEN_W bits are the difference.
    wire [DEN_W-1:0] diff = trial[DEN_W-1:0] - dv;

    // floor(num / 2^qbits): below den by the caller's bound, so DEN_W bits hold it.
    wire [NUM_W-1:0] head  = num >> qbits;
    wire [7:0]       up    = 8'(NUM_W) - 8'(qbits);
    wire [NUM_W-1:0] align = num << up;

    always_ff @(posedge clk) begin
        done <= 1'b0;
        if (!rst_n || drop) begin
            busy <= 1'b0;
            left <= '0;
        end else if (start && !busy) begin
            busy <= 1'b1;
            left <= qbits;
            rem  <= head[DEN_W-1:0];
            rest <= align;
            dv   <= den;
            quot <= '0;
        end else if (busy) begin
            rem  <= take ? diff : trial[DEN_W-1:0];
            rest <= rest << 1;
            quot <= {quot[Q_W-2:0], take};
            left <= left - QB_W'(1);
            if (left == QB_W'(1)) begin
                busy <= 1'b0;
                done <= 1'b1;
            end
        end
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk)
        if (rst_n && !drop && start) begin
            if (busy) $error("div_serial: start while busy");
            if (qbits == '0 || int'(qbits) > Q_W) $error("div_serial: qbits %0d", qbits);
            if (den != '0 && head >= NUM_W'(den))
                $error("div_serial: num %0d >= den %0d * 2^%0d", num, den, qbits);
        end
`endif
endmodule

`default_nettype wire
