// rvec rx-symbol-pkg — generated from radio_phy, do not edit.
// Regenerate: make -C hw vectors

`default_nettype none

package rx_symbol_pkg;
    // Widths of FixedOfdmConfig::default() (docs/gates/rx_symbol.md, G0).
    localparam int PILOT_BITS = 17;  // a pilot with its sign removed: U + 1
    localparam int KAPPA_BITS = 9;  // the demapper's constants, C = 10 fractional bits
    localparam int KAPPA_FRAC = 10;
    localparam int S_FRAC     = 4;  // the soft scale's fractional bits F
    localparam int SOFT_MAX   = 127;  // soft values clip at ±SOFT_MAX
    localparam int SOFT_BITS  = 8;  // ±SOFT_MAX, signed
    localparam int SQ_BITS    = 20;  // the soft scale q ≤ 2·A (the largest A's, + 1 bit)
    // The 48 data subcarriers from -26 to 26 (IEEE 802.11-2020 §17.3.5.10): bin, number.
    localparam int DATA_BIN [48] = '{38, 39, 40, 41, 42, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 58, 59, 60, 61, 62, 63, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26};
    localparam int DATA_C   [48] = '{-26, -25, -24, -23, -22, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26};
    // The pilots -21, -7, 7, 21 and their base signs (bit i: -1), §17.3.5.9.
    localparam int PILOT_C [4] = '{-21, -7, 7, 21};
    localparam logic [3:0] PILOT_BASE_NEG = 4'b1000;
    // The polarity sequence p_n, n = 0 .. 126 (bit n: -1), §17.3.5.10.
    localparam logic [126:0] POLARITY_NEG = 127'h7f1d8a5f542de72b306d746440934f70;
    // The thresholds' constants per modulation (BPSK, QPSK, 16-QAM, 64-QAM): kappa of
    // the first fold (16-QAM c = 2, 64-QAM c = 4) and of the second (64-QAM c = 2).
    localparam int K_HI [4] = '{0, 0, 324, 316};
    localparam int K_LO [4] = '{0, 0, 0, 158};
endpackage

`default_nettype wire
