// rvec rx-channel-pkg — generated from radio_phy::rx_ofdm_fixed, do not edit.
// Regenerate: make -C hw vectors

`default_nettype none

package rx_channel_pkg;
    // Widths of FixedOfdmConfig::default() (docs/gates/rx_channel.md, G0).
    localparam int W_BITS    = 16;  // bins
    localparam int U_BITS    = 16;  // gains, signed word (U - 1 magnitude bits)
    localparam int M_BITS    = 16;  // the tracker's denominator after its shift
    localparam int K_FRAC    = 16;  // the coefficients' fractional bits
    localparam int V_FRAC    = 10;  // the variance's fractional bits
    localparam int HS_BITS   = 18;  // Hs's parts: W + 2
    localparam int FULL_BITS = 34;  // |Hs|^2, unsigned
    localparam int GSUM_BITS = 21;  // sum of 52 gains
    localparam int COEF_Q    = 14;  // the coefficients' quotient bits: K - 2
    localparam int VAR_Q     = 12;  // the variance's, saturating: V + 2 (variance_max)
    // IEEE 802.11-2020 §17.3.3: the 52 used bins and those where L(k) = -1 (bit b: bin b).
    localparam logic [63:0] L_USED = 64'hffffffc007fffffe;
    localparam logic [63:0] L_NEG  = 64'h0a60530000567d4c;
    // The pilots' bins, subcarriers -21, -7, 7, 21 (§17.3.5.9).
    localparam int PILOT_BIN [4] = '{43, 57, 7, 21};
    // The soft scale's constants A = scale_constant(m) (BPSK, QPSK, 16-QAM, 64-QAM; soft 127 / clip 3),
    // and the quotient bits of A·2^n / Σg: bit length of A + 1.
    localparam int A_BITS = 19;
    localparam logic [A_BITS-1:0] SCALE_A [4] = '{19'd70443, 19'd99621, 19'd222759, 19'd456521};
    localparam int SCALE_Q [4] = '{18, 18, 19, 20};
endpackage

`default_nettype wire
