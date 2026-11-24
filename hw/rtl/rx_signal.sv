// SIGNAL and the verdict (ADR 0010, decisions 1, 5 and 6.2; R0-09's data path): SIGNAL's field
// from the Viterbi decoder's bits, its checks, the check of its coded bits against the field
// encoded again, the verdict for the front and the data block for the deinterleaver. Golden
// model: radio_phy::rx_ofdm::signal_checked, radio_phy::rx_front_fixed::SignalVerdict::of
// (HandoffConfig::default: patient at 40 agreeing), radio_phy::ppdu::data_symbols. Gate G3:
// make -C hw rx_signal. Architecture: docs/gates/rx_signal.md (G0), tests/g0_rx_signal.rs.
//
// The stream from the decoder's flush (the founder's choice, 2026-11-24): SIGNAL's 24 pairs are
// kept as they pass from rx_deint to the decoder (per value its sign and whether it is 0); the
// decoder's 24 bits come out of its flush, are encoded again as they come (conv_encoder.sv) and
// each (A, B) compared with its pair: a nonzero value agrees when its sign is the bit's
// (negative for 1). Bits 0 … 17 are the field and its parity (IEEE 802.11-2020 §17.3.4); the
// tail is not looked at, nor the reserved bit (§17.3.4.3). Checks in the model's order: parity,
// rate (Table 17-6), length ≠ 0. The data block: steps = 16 + 8·LENGTH + 6; symbols
// ⌈steps / N_DBPS⌉ = ((steps + N_DBPS − 1) >> a)·M >> s, N_DBPS = 2^a·t, (M, s) = (2731, 13),
// (3641, 15), (4855, 17) for t = 3, 9, 27 (exact at every rate and length: G0).
//
// In: after drop or reset, the first 24 pairs on v_* are SIGNAL's (v_first on the first,
// v_last on the 24th: simulation stops otherwise), later ones are ignored; the decoder's bits
// count only once those 24 pairs are in, the first 24 of them (dec_last on the 24th) — so the
// flush of a block cut by a drop is ignored: by the decoder's contract (the next block's first
// step n + 2 edges after the last) it ends before the next SIGNAL's 24th pair. A pair in a
// drop's own cycle is not kept. The top level drops at the end of every packet, a failed SIGNAL
// and an abort included: until a drop the block takes nothing more.
// Out, for one cycle, 1 edge after the edge that took the 24th bit (with viterbi.sv: 29 edges
// after the edge that took SIGNAL's last step: its 24th bit registered 4 + 23 edges after, taken
// an edge later): sig_valid; sig_why 0 decoded, 1 parity, 2 rate,
// 3 length, sig_fail = sig_why ≠ 0; sig_agree and sig_disagree (also when it failed); when it
// decoded: sig_rate (0 … 7 for 6 … 54 Mb/s), sig_length, sig_symbols, sig_patient, and the data
// block: sig_steps, sig_code (0 1/2, 1 2/3, 2 3/4), sig_mod (0 BPSK … 3 64-QAM); all 0 when it
// failed.

`default_nettype none

module rx_signal #(
    parameter int PATIENT_AGREE = 40
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               drop,
    // SIGNAL's depunctured pairs, as rx_deint hands them to the decoder
    input  wire               v_valid,
    input  wire               v_first,
    input  wire               v_last,
    input  wire signed [7:0]  v_a,
    input  wire signed [7:0]  v_b,
    // the decoder's bits
    input  wire               dec_valid,
    input  wire               dec_bit,
    input  wire               dec_last,
    // the verdict, the field, the data block
    output logic              sig_valid,
    output logic              sig_fail,
    output logic [1:0]        sig_why,
    output logic [5:0]        sig_agree,
    output logic [5:0]        sig_disagree,
    output logic [2:0]        sig_rate,
    output logic [11:0]       sig_length,
    output logic [10:0]       sig_symbols,
    output logic              sig_patient,
    output logic [15:0]       sig_steps,
    output logic [1:0]        sig_code,
    output logic [1:0]        sig_mod
);
    // ---- SIGNAL's pairs: {A negative, A nonzero, B negative, B nonzero}, oldest at 0 -------
    logic [3:0] pr [24];
    logic [4:0] pn;                     // pairs kept since the drop
    logic [4:0] bn;                     // bits taken
    wire        take_pair = v_valid && pn != 5'd24;
    wire        take_bit  = dec_valid && pn == 5'd24 && bn != 5'd24;

    // ---- the field: bits 0 … 17, bit 0 at fld[0] once all are in -----------------------------
    logic [17:0] fld;
    logic        parity;

    // ---- the encoder and the compare ----------------------------------------------------------
    logic enc_valid, enc_a, enc_b;
    conv_encoder u_enc (
        .clk, .rst_n, .clear(drop), .in_valid(take_bit), .in_bit(dec_bit),
        .out_valid(enc_valid), .out_a(enc_a), .out_b(enc_b)
    );
    logic [3:0] hp;                     // the pair of the bit in the encoder
    logic       hp_last;                // it is the 24th
    logic [5:0] agree, disagree;
    logic [1:0] inc_ag, inc_dis;
    always_comb begin
        inc_ag  = 2'(hp[2] && hp[3] == enc_a) + 2'(hp[0] && hp[1] == enc_b);
        inc_dis = 2'(hp[2] && hp[3] != enc_a) + 2'(hp[0] && hp[1] != enc_b);
    end
    wire [5:0] agree_n    = agree + 6'(inc_ag);
    wire [5:0] disagree_n = disagree + 6'(inc_dis);

    // ---- the rate (Table 17-6): R1 … R4 = fld[0 … 3] ----------------------------------------
    logic       rate_ok;
    logic [2:0] ridx;
    logic [2:0] ra;                     // N_DBPS = 2^a·t
    logic [1:0] rt;                     // t: 0 → 3, 1 → 9, 2 → 27
    logic [7:0] ndbps;
    logic [1:0] rcode, rmod;
    always_comb begin
        rate_ok = 1'b1;
        {ridx, ra, rt, ndbps, rcode, rmod} = '0;
        case ({fld[0], fld[1], fld[2], fld[3]})
            4'b1101: {ridx, ra, rt, ndbps, rcode, rmod} = {3'd0, 3'd3, 2'd0, 8'd24,  2'd0, 2'd0};
            4'b1111: {ridx, ra, rt, ndbps, rcode, rmod} = {3'd1, 3'd2, 2'd1, 8'd36,  2'd2, 2'd0};
            4'b0101: {ridx, ra, rt, ndbps, rcode, rmod} = {3'd2, 3'd4, 2'd0, 8'd48,  2'd0, 2'd1};
            4'b0111: {ridx, ra, rt, ndbps, rcode, rmod} = {3'd3, 3'd3, 2'd1, 8'd72,  2'd2, 2'd1};
            4'b1001: {ridx, ra, rt, ndbps, rcode, rmod} = {3'd4, 3'd5, 2'd0, 8'd96,  2'd0, 2'd2};
            4'b1011: {ridx, ra, rt, ndbps, rcode, rmod} = {3'd5, 3'd4, 2'd1, 8'd144, 2'd2, 2'd2};
            4'b0001: {ridx, ra, rt, ndbps, rcode, rmod} = {3'd6, 3'd6, 2'd0, 8'd192, 2'd1, 2'd3};
            4'b0011: {ridx, ra, rt, ndbps, rcode, rmod} = {3'd7, 3'd3, 2'd2, 8'd216, 2'd2, 2'd3};
            default: rate_ok = 1'b0;
        endcase
    end
    wire [11:0] length = fld[16:5];
    wire [15:0] steps  = {1'b0, length, 3'd0} + 16'd22;
    // (steps + N_DBPS − 1) >> a, registered while the field waits for the flush's end.
    wire [15:0] ceil_in = steps + 16'(ndbps) - 16'd1;
    logic [13:0] z;                     // at most 8204 (9 Mb/s, 4095 octets): 14 bits (G0)
    // ⌊z·M / 2^s⌋ for t's constant.
    logic [25:0] zm;
    logic [10:0] symbols;
    always_comb begin
        case (rt)
            2'd0:    begin zm = 26'(z) * 26'd2731; symbols = 11'(zm >> 13); end
            2'd1:    begin zm = 26'(z) * 26'd3641; symbols = 11'(zm >> 15); end
            default: begin zm = 26'(z) * 26'd4855; symbols = 11'(zm >> 17); end
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n || drop) begin
            pn        <= 5'd0;
            bn        <= 5'd0;
            parity    <= 1'b0;
            agree     <= 6'd0;
            disagree  <= 6'd0;
            hp_last   <= 1'b0;
            sig_valid <= 1'b0;
        end else begin
            if (take_pair) begin
                for (int i = 0; i < 23; i++) pr[i] <= pr[i+1];
                pr[23] <= {v_a < 0, v_a != 0, v_b < 0, v_b != 0};
                pn     <= pn + 5'd1;
            end
            if (take_bit) begin
                for (int i = 0; i < 23; i++) pr[i] <= pr[i+1];
                hp      <= pr[0];
                hp_last <= bn == 5'd23;
                bn      <= bn + 5'd1;
                if (bn < 5'd18) begin
                    fld    <= {dec_bit, fld[17:1]};
                    parity <= parity ^ dec_bit;
                end
            end
            if (enc_valid) begin
                agree    <= agree_n;
                disagree <= disagree_n;
            end
            sig_valid <= enc_valid && hp_last;
        end
        z <= 14'(ceil_in >> ra);
        if (enc_valid && hp_last) begin
            sig_agree    <= agree_n;
            sig_disagree <= disagree_n;
            if (parity) begin
                sig_why <= 2'd1;
            end else if (!rate_ok) begin
                sig_why <= 2'd2;
            end else if (length == 12'd0) begin
                sig_why <= 2'd3;
            end else begin
                sig_why <= 2'd0;
            end
            if (!parity && rate_ok && length != 12'd0) begin
                sig_fail    <= 1'b0;
                sig_rate    <= ridx;
                sig_length  <= length;
                sig_symbols <= symbols;
                sig_patient <= ridx <= 3'd2 && agree_n >= 6'(PATIENT_AGREE);
                sig_steps   <= steps;
                sig_code    <= rcode;
                sig_mod     <= rmod;
            end else begin
                sig_fail    <= 1'b1;
                sig_rate    <= 3'd0;
                sig_length  <= 12'd0;
                sig_symbols <= 11'd0;
                sig_patient <= 1'b0;
                sig_steps   <= 16'd0;
                sig_code    <= 2'd0;
                sig_mod     <= 2'd0;
            end
        end
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin
        if (rst_n && !drop) begin
            if (take_pair && (v_first != (pn == 5'd0) || v_last != (pn == 5'd23))) begin
                $error("rx_signal: pair %0d with first %0d, last %0d", pn, v_first, v_last);
                $stop;
            end
            if (take_bit && dec_last != (bn == 5'd23)) begin
                $error("rx_signal: bit %0d with last %0d", bn, dec_last);
                $stop;
            end
        end
    end
`endif
endmodule

`default_nettype wire
