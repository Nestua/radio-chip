// rvec fft64-rom — generated from radio_phy::rx_ofdm_fixed, do not edit.
// Regenerate: make -C hw vectors

`default_nettype none

package fft64_rom_pkg;
    // Widths of FixedOfdmConfig::default().
    localparam int IN_BITS  = 13;
    localparam int IN_SHIFT = 1;
    localparam int W_BITS   = 16;
    localparam int T_BITS   = 12;
    // Bit s set: stage s (0 first) halves its outputs.
    localparam logic [5:0] HALVE = 6'b101010;

    // Twiddle e^(-j2pi k/64), k < 32, real part, Q(T, T - 2): fft64_twiddles().
    function automatic logic signed [11:0] tw_re(input int i);
        case (i)
              0: tw_re = 12'sd1024;
              1: tw_re = 12'sd1019;
              2: tw_re = 12'sd1004;
              3: tw_re = 12'sd980;
              4: tw_re = 12'sd946;
              5: tw_re = 12'sd903;
              6: tw_re = 12'sd851;
              7: tw_re = 12'sd792;
              8: tw_re = 12'sd724;
              9: tw_re = 12'sd650;
             10: tw_re = 12'sd569;
             11: tw_re = 12'sd483;
             12: tw_re = 12'sd392;
             13: tw_re = 12'sd297;
             14: tw_re = 12'sd200;
             15: tw_re = 12'sd100;
             16: tw_re = 12'sd0;
             17: tw_re = -12'sd100;
             18: tw_re = -12'sd200;
             19: tw_re = -12'sd297;
             20: tw_re = -12'sd392;
             21: tw_re = -12'sd483;
             22: tw_re = -12'sd569;
             23: tw_re = -12'sd650;
             24: tw_re = -12'sd724;
             25: tw_re = -12'sd792;
             26: tw_re = -12'sd851;
             27: tw_re = -12'sd903;
             28: tw_re = -12'sd946;
             29: tw_re = -12'sd980;
             30: tw_re = -12'sd1004;
             31: tw_re = -12'sd1019;
            default: tw_re = '0;
        endcase
    endfunction

    // Twiddle e^(-j2pi k/64), k < 32, imaginary part, Q(T, T - 2): fft64_twiddles().
    function automatic logic signed [11:0] tw_im(input int i);
        case (i)
              0: tw_im = 12'sd0;
              1: tw_im = -12'sd100;
              2: tw_im = -12'sd200;
              3: tw_im = -12'sd297;
              4: tw_im = -12'sd392;
              5: tw_im = -12'sd483;
              6: tw_im = -12'sd569;
              7: tw_im = -12'sd650;
              8: tw_im = -12'sd724;
              9: tw_im = -12'sd792;
             10: tw_im = -12'sd851;
             11: tw_im = -12'sd903;
             12: tw_im = -12'sd946;
             13: tw_im = -12'sd980;
             14: tw_im = -12'sd1004;
             15: tw_im = -12'sd1019;
             16: tw_im = -12'sd1024;
             17: tw_im = -12'sd1019;
             18: tw_im = -12'sd1004;
             19: tw_im = -12'sd980;
             20: tw_im = -12'sd946;
             21: tw_im = -12'sd903;
             22: tw_im = -12'sd851;
             23: tw_im = -12'sd792;
             24: tw_im = -12'sd724;
             25: tw_im = -12'sd650;
             26: tw_im = -12'sd569;
             27: tw_im = -12'sd483;
             28: tw_im = -12'sd392;
             29: tw_im = -12'sd297;
             30: tw_im = -12'sd200;
             31: tw_im = -12'sd100;
            default: tw_im = '0;
        endcase
    endfunction
endpackage

`default_nettype wire
