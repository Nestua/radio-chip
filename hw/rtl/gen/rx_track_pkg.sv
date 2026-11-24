// rvec rx-track-pkg — generated from radio_phy, do not edit.
// Regenerate: make -C hw vectors

`default_nettype none

package rx_track_pkg;
    // Widths of FixedOfdmConfig::default() (docs/gates/rx_track.md, G0).
    localparam int RATE_FRAC   = 10;  // R: the rates' fractional bits
    localparam int RATE0_BITS  = 26;  // rate0: P - 6 + R
    localparam int FIT_QBITS   = 26;  // the fit's quotient bits (|delta| < 2^FIT_QBITS)
    localparam int FIT_NUM_BITS = 64;  // the divider's operands: |num| * 2^R
    localparam int FIT_DEN_BITS = 47;  // and det
    localparam int RATIO_SHIFT = 12;  // rate0 = word * ratio >> (20 + 2 - R)
    localparam int LAST_SYMBOL = 1366;  // SIGNAL 0, DATA k at k + 1
    // The prior's weight: a clock offset known from the CFO word (TIMING_HINT_PPM) and an
    // unknown one (TIMING_PRIOR_PPM), FixedTracking::{from_cfo_word, unknown}.
    localparam int WEIGHT_BITS = 25;
    localparam logic [WEIGHT_BITS-1:0] WEIGHT_KNOWN   = 25'd25938223;
    localparam logic [WEIGHT_BITS-1:0] WEIGHT_UNKNOWN = 25'd64846;
endpackage

`default_nettype wire
