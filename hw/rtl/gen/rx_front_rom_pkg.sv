// rvec rx-front-rom — generated from radio_phy::rx_front_fixed, do not edit.
// Regenerate: make -C hw vectors

`default_nettype none

package rx_front_rom_pkg;
    // Formats of FixedRxConfig::default().
    localparam int Z_BITS    = 12;
    localparam int TAP_BITS  = 8;
    localparam int CORR_BITS = 20;

    // LTF taps t(k), k = 0..32 (t(64 - k) = conj(t(k)); t(0), t(32) real): ltf_taps().
    function automatic logic signed [7:0] ltf_tap_re(input int k);
        case (k)
             0: ltf_tap_re = 8'sd124;
             1: ltf_tap_re = -8'sd4;
             2: ltf_tap_re = 8'sd31;
             3: ltf_tap_re = 8'sd77;
             4: ltf_tap_re = 8'sd17;
             5: ltf_tap_re = 8'sd47;
             6: ltf_tap_re = -8'sd91;
             7: ltf_tap_re = -8'sd30;
             8: ltf_tap_re = 8'sd77;
             9: ltf_tap_re = 8'sd42;
            10: ltf_tap_re = 8'sd1;
            11: ltf_tap_re = -8'sd108;
            12: ltf_tap_re = 8'sd19;
            13: ltf_tap_re = 8'sd46;
            14: ltf_tap_re = -8'sd18;
            15: ltf_tap_re = 8'sd94;
            16: ltf_tap_re = 8'sd49;
            17: ltf_tap_re = 8'sd29;
            18: ltf_tap_re = -8'sd45;
            19: ltf_tap_re = -8'sd104;
            20: ltf_tap_re = 8'sd65;
            21: ltf_tap_re = 8'sd55;
            22: ltf_tap_re = -8'sd48;
            23: ltf_tap_re = -8'sd45;
            24: ltf_tap_re = -8'sd28;
            25: ltf_tap_re = -8'sd96;
            26: ltf_tap_re = -8'sd101;
            27: ltf_tap_re = 8'sd59;
            28: ltf_tap_re = -8'sd2;
            29: ltf_tap_re = -8'sd73;
            30: ltf_tap_re = 8'sd73;
            31: ltf_tap_re = 8'sd10;
            32: ltf_tap_re = -8'sd124;
            default: ltf_tap_re = '0;
        endcase
    endfunction
    function automatic logic signed [7:0] ltf_tap_im(input int k);
        case (k)
             0: ltf_tap_im = 8'sd0;
             1: ltf_tap_im = -8'sd95;
             2: ltf_tap_im = -8'sd88;
             3: ltf_tap_im = 8'sd65;
             4: ltf_tap_im = 8'sd22;
             5: ltf_tap_im = -8'sd69;
             6: ltf_tap_im = -8'sd44;
             7: ltf_tap_im = -8'sd84;
             8: ltf_tap_im = -8'sd20;
             9: ltf_tap_im = 8'sd3;
            10: ltf_tap_im = -8'sd91;
            11: ltf_tap_im = -8'sd37;
            12: ltf_tap_im = -8'sd46;
            13: ltf_tap_im = -8'sd12;
            14: ltf_tap_im = 8'sd127;
            15: ltf_tap_im = -8'sd3;
            16: ltf_tap_im = -8'sd49;
            17: ltf_tap_im = 8'sd78;
            18: ltf_tap_im = 8'sd31;
            19: ltf_tap_im = 8'sd52;
            20: ltf_tap_im = 8'sd73;
            21: ltf_tap_im = 8'sd11;
            22: ltf_tap_im = 8'sd64;
            23: ltf_tap_im = -8'sd17;
            24: ltf_tap_im = -8'sd119;
            25: ltf_tap_im = -8'sd13;
            26: ltf_tap_im = -8'sd16;
            27: ltf_tap_im = -8'sd59;
            28: ltf_tap_im = 8'sd43;
            29: ltf_tap_im = 8'sd91;
            30: ltf_tap_im = 8'sd84;
            31: ltf_tap_im = 8'sd77;
            32: ltf_tap_im = 8'sd0;
            default: ltf_tap_im = '0;
        endcase
    endfunction
endpackage

`default_nettype wire
