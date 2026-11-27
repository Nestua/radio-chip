// The channel and the per-packet quotients (ADR 0010, decisions 1 and 6.2; R0-09's data path):
// Hs(k) = L(k)·(T1(k) + T2(k)) kept for the packet, the gains g(k) = |Hs(k)|² after the packet's
// shift, Σg, then on one serial divider the soft scale for BPSK, the timing tracker's two
// coefficients and its variance, and later, on request, the data's soft scale.
// Golden model: radio_phy::rx_ofdm_fixed::{channel_from_bins, tracker_constants, soft_scale}.
// Gate G3: make -C hw rx_channel. Architecture and widths: docs/gates/rx_channel.md (G0),
// tests/g0_rx_channel.rs.
//
// The bin store accumulates (the founder's choice, 2026-11-24): T1's bins are written as
// L(k)·T1(k); as T2's bin k comes, word k is read and Hs(k) = word + L(k)·T2(k) written back,
// and its |Hs|² goes into the running largest (the first pass, at the bins' pace). Then the
// shift; a second pass reads the 52 words again, squares, rounds half to even, saturates to
// 2^15 − 1, writes g(k), sums Σg and keeps the pilots' four gains. One multiplier, six times,
// makes the tracker's products (pipelined, the founder's choice of 2026-11-25: operands, then the
// two halves' products, then their sum, read three edges after the issue; the single-cycle
// product was a worst path through the open flow, docs/gates/silicon_probe.md); the divisions
// in a fixed order: BPSK's scale, outer, inner, variance (12 bits, saturating), then a data
// scale per request.
//
// Bins: bin_valid with bin_t2 (0: T1's window, 1: T2's), the index, the value, at most one a
// clock, in any order within a window; the unused bins are ignored. All of T1's bins come
// before T2's; a packet's first T1 bin starts it (in IDLE or READY), its 52nd used T2 bin ends
// the first pass. A bin at any other time is a protocol error (simulation stops).
//
// Out: chan_valid for one cycle with shift, gsum, gsum_n (held until the next packet), then
// q_valid for one cycle per quotient: q_job 0 BPSK's scale, 1 outer, 2 inner, 3 variance,
// 4 + m the scale of modulation m (0 BPSK … 3 64-QAM) after a scale_req (taken from
// chan_valid on, one outstanding until its q_valid; served after the variance; simulation
// stops on a second one, one at another time, or one on the edge of the next packet's first
// bin, which it would carry into that packet). Latencies (G0): chan_valid 63 edges after the
// edge that took the last used T2 bin; the quotients 21, 37, 53, 67 edges after chan_valid; a
// request 3 + q edges after it when the divider is free.
//
// No ready on the bins: a packet's 128 bins are taken at any pace up to one a clock, and the
// next packet's come only after this one has ended (ADR 0010, Context 3 rule 5), by when the
// block is in READY.
//
// The stores: hs_* and g_* read ports for the products and the demapper, on the 52 used bins
// (the other words are never written: undefined, where the model has 0). Hs answers from
// chan_valid until the edge that takes the next packet's first bin (the block reads it itself
// before); g from chan_valid until the next packet's second pass.
// drop (and rst_n): back to IDLE, the stores left as they are. The divider is outside (the
// data path shares it, ADR 0010); it must take the same drop.

`default_nettype none

module rx_channel
    import rx_channel_pkg::*;
#(
    parameter int NUM_W = 48,
    parameter int DEN_W = 24,
    parameter int Q_W   = 28,
    parameter int QB_W  = $clog2(Q_W + 1)
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         drop,
    // bins from fft64, tagged by the data path's top level
    input  wire                         bin_valid,
    input  wire                         bin_t2,
    input  wire        [5:0]            bin_idx,
    input  wire signed [W_BITS-1:0]     bin_re,
    input  wire signed [W_BITS-1:0]     bin_im,
    // the channel
    output logic                        chan_valid,
    output logic       [4:0]            shift,
    output logic       [GSUM_BITS-1:0]  gsum,
    output logic       [4:0]            gsum_n,
    // the quotients
    output logic                        q_valid,
    output logic       [2:0]            q_job,
    output logic       [Q_W-1:0]        q_value,
    input  wire                         scale_req,
    input  wire        [1:0]            scale_mod,
    // the shared divider
    output logic                        div_start,
    output logic       [NUM_W-1:0]      div_num,
    output logic       [DEN_W-1:0]      div_den,
    output logic       [QB_W-1:0]       div_qbits,
    input  wire                         div_done,
    input  wire        [Q_W-1:0]        div_quot,
    // the stores
    input  wire                         hs_re,
    input  wire        [5:0]            hs_raddr,
    output logic signed [HS_BITS-1:0]   hs_rdata_re,
    output logic signed [HS_BITS-1:0]   hs_rdata_im,
    input  wire                         g_re,
    input  wire        [5:0]            g_raddr,
    output logic       [U_BITS-2:0]     g_rdata
);
    localparam int GW = U_BITS - 1;                 // a gain's bits (non-negative)
    localparam logic [GW-1:0] GMAX = '1;            // 2^15 − 1

    typedef enum logic [2:0] {IDLE, LOAD, DRAIN, PASS2, P2DRAIN, QUOT, READY} state_t;
    state_t state;

    function automatic logic [5:0] bitlen64(input logic [63:0] v);
        bitlen64 = '0;
        for (int i = 0; i < 64; i++) if (v[i]) bitlen64 = 6'(i + 1);
    endfunction

    // ---- stage 0: bins in, the store's ports -----------------------------------------------
    wire used   = L_USED[bin_idx];
    wire neg    = L_NEG[bin_idx];
    wire open_t1 = state == IDLE || state == LOAD || state == READY;
    wire t1_wr  = bin_valid && used && !bin_t2 && open_t1;
    wire t2_in  = bin_valid && used && bin_t2 && state == LOAD;

    wire signed [HS_BITS-1:0] x_re = HS_BITS'(bin_re);
    wire signed [HS_BITS-1:0] x_im = HS_BITS'(bin_im);
    wire signed [HS_BITS-1:0] lx_re = neg ? -x_re : x_re;  // L(k)·bin, exact in W + 2 bits
    wire signed [HS_BITS-1:0] lx_im = neg ? -x_im : x_im;

    logic [5:0] ra;                                  // the second pass's address
    logic [5:0] t2cnt;                               // T2's used bins taken
    wire p2_issue = state == PASS2;
    // The store answers outside reads from chan_valid until the edge that takes the next
    // packet's first T1 bin (which writes it).
    wire ext_ok   = (state == QUOT || state == READY) && !t1_wr;

    // stage 1 registers
    logic                      s1_v, s1_p1;
    logic [5:0]                s1_k;
    logic signed [HS_BITS-1:0] s1_add_re, s1_add_im;

    logic [2*HS_BITS-1:0] st_rdata;
    wire signed [HS_BITS-1:0] st_re = st_rdata[2*HS_BITS-1:HS_BITS];
    wire signed [HS_BITS-1:0] st_im = st_rdata[HS_BITS-1:0];
    // Hs(k) in the first pass (the word + L·T2, no overflow: G0), the word in the second.
    // The adds sit behind the store's read (a macro's read is about half the clock at 160 MHz
    // on 130 nm), so each is a module the synthesis tool maps by itself (lib/add_alone.sv).
    wire signed [HS_BITS-1:0] hs1_re, hs1_im;
    add_alone #(.W(HS_BITS)) u_add_re (.a(st_re), .b(s1_add_re), .y(hs1_re));
    add_alone #(.W(HS_BITS)) u_add_im (.a(st_im), .b(s1_add_im), .y(hs1_im));

    wire                  st_we    = t1_wr || (s1_v && s1_p1);
    wire [5:0]            st_waddr = t1_wr ? bin_idx : s1_k;
    wire [2*HS_BITS-1:0]  st_wdata = t1_wr ? {lx_re, lx_im} : {hs1_re, hs1_im};
    wire                  st_rd    = t2_in || p2_issue || (ext_ok && hs_re);
    wire [5:0]            st_raddr = t2_in ? bin_idx : p2_issue ? ra : hs_raddr;

    ram_1w1r #(.W(2 * HS_BITS), .DEPTH(64)) u_hs (
        .clk, .we(st_we), .waddr(st_waddr), .wdata(st_wdata),
        .re(st_rd), .raddr(st_raddr), .rdata(st_rdata)
    );
    assign hs_rdata_re = st_re;
    assign hs_rdata_im = st_im;

    // ---- stages 2 to 5 ---------------------------------------------------------------------
    logic                      s2_v, s2_p1;
    logic [5:0]                s2_k;
    logic signed [HS_BITS-1:0] s2_re, s2_im;
    logic                      s3_v, s3_p1;
    logic [5:0]                s3_k;
    logic [FULL_BITS-1:0]      s3_pr, s3_pi;
    logic                      s4_v, s4_p1;
    logic [5:0]                s4_k;
    logic [FULL_BITS-1:0]      s4_full;
    logic                      s5_v;
    logic [5:0]                s5_k;
    logic [GW-1:0]             s5_floor;
    logic                      s5_up;

    logic [FULL_BITS-1:0] largest;

    // Half to even at the shift: floor, the guard bit (the first one dropped), the sticky bits
    // below it. full < 2^(15 + shift) (shift from the largest), so floor fits 15 bits.
    wire [GW-1:0]        fl    = GW'(s4_full >> shift);
    wire [FULL_BITS-1:0] below = s4_full & ((FULL_BITS'(1) << shift) - FULL_BITS'(1));
    wire [FULL_BITS-1:0] halfw = (FULL_BITS'(1) << shift) >> 1;
    wire                 up    = shift != 0 && (below > halfw || (below == halfw && fl[0]));

    wire [GW:0]   g_sum  = {1'b0, s5_floor} + (GW + 1)'(s5_up);
    wire [GW-1:0] g_new  = g_sum[GW] ? GMAX : g_sum[GW-1:0];

    ram_1w1r #(.W(GW), .DEPTH(64)) u_g (
        .clk, .we(s5_v), .waddr(s5_k), .wdata(g_new),
        .re(g_re), .raddr(g_raddr), .rdata(g_rdata)
    );

    // ---- the quotients' operands -----------------------------------------------------------
    logic [GW-1:0]  pg [4];                          // g at −21, −7, 7, 21
    wire  [GW:0]    a_sum = {1'b0, pg[0]} + {1'b0, pg[3]};
    wire  [GW:0]    b_sum = {1'b0, pg[1]} + {1'b0, pg[2]};
    wire  [GW:0]    a_op  = a_sum == '0 ? (GW + 1)'(1) : a_sum;  // a pair with no pilot: 1
    wire  [GW:0]    b_op  = b_sum == '0 ? (GW + 1)'(1) : b_sum;

    logic [3:0]  mstep;
    logic        cr;                                 // the tracker's operands ready
    logic [29:0] po, pi;
    logic [45:0] no, ni;
    logic [36:0] sa;
    logic [52:0] spread;
    logic [49:0] d;
    logic [5:0]  s;
    logic [15:0] dn, nos, nis;
    logic [52:0] sps;
    logic        dz, ovf;
    logic [NUM_W-1:0] num_o, num_i, num_v;
    logic [DEN_W-1:0] den28, den52;

    // One multiplier, 37 × 16 bits, in three stages: the operands chosen by the step are
    // registered, then the products of mb's two bytes, then their sum (mp: the product of the
    // step three before). The order puts each product three steps ahead of its use.
    logic [36:0] ma, mqa;
    logic [15:0] mb, mqb;
    logic [44:0] mph, mpl;
    logic [52:0] mp;
    always_comb begin
        unique case (mstep)
            4'd0:    begin ma = 37'(pg[0]); mb = 16'(pg[3]); end
            4'd1:    begin ma = 37'(pg[1]); mb = 16'(pg[2]); end
            4'd2:    begin ma = 37'(gsum);  mb = 16'(a_op);  end
            4'd4:    begin ma = 37'(po);    mb = 16'(b_op);  end
            4'd5:    begin ma = 37'(pi);    mb = 16'(a_op);  end
            4'd6:    begin ma = sa;         mb = 16'(b_op);  end
            default: begin ma = '0;         mb = '0;         end
        endcase
    end
    always_ff @(posedge clk) begin
        mqa <= ma;
        mqb <= mb;
        mph <= 45'(mqa) * 45'(mqb[15:8]);
        mpl <= 45'(mqa) * 45'(mqb[7:0]);
        mp  <= {mph, 8'b0} + 53'(mpl);
    end

    wire [55:0] x5   = {sps, 2'b00} + 56'(sps);      // 5·(spread >> s)
    wire [23:0] y208 = 24'(dn) * 24'd208;            // 52·dn·2^(12 − 10)

    // ---- the divider's jobs ----------------------------------------------------------------
    logic       waiting;
    logic [2:0] cur_job;
    logic [2:0] next_job;                            // 0 … 3, then 4: the requests
    logic       pend;
    logic [1:0] pmod;

    // The next job is asked on the edge that brings the last one's quotient (the divider is
    // free then): a division every qbits + 2 clocks.
    wire       freed = waiting && div_done;
    wire [2:0] nj    = freed && !cur_job[2] ? cur_job + 3'd1 : next_job;
    wire job_ok = (!waiting || freed) &&
                  ((state == QUOT && !nj[2] && (nj == 3'd0 || cr)) || (state == READY && pend));

    always_ff @(posedge clk) begin
        chan_valid <= 1'b0;
        q_valid    <= 1'b0;
        div_start  <= 1'b0;
        if (!rst_n || drop) begin
            state   <= IDLE;
            s1_v    <= 1'b0;
            s2_v    <= 1'b0;
            s3_v    <= 1'b0;
            s4_v    <= 1'b0;
            s5_v    <= 1'b0;
            waiting <= 1'b0;
            pend    <= 1'b0;
            cr      <= 1'b0;
        end else begin
            // stage 0 → 1
            s1_v      <= t2_in || p2_issue;
            s1_p1     <= t2_in;
            s1_k      <= t2_in ? bin_idx : ra;
            s1_add_re <= t2_in ? lx_re : '0;
            s1_add_im <= t2_in ? lx_im : '0;
            // 1 → 2 → 3 → 4 → 5
            s2_v  <= s1_v;
            s2_p1 <= s1_p1;
            s2_k  <= s1_k;
            s2_re <= hs1_re;
            s2_im <= hs1_im;
            s3_v  <= s2_v;
            s3_p1 <= s2_p1;
            s3_k  <= s2_k;
            s3_pr <= FULL_BITS'(s2_re * s2_re);   // ≤ 2^32 (G0)
            s3_pi <= FULL_BITS'(s2_im * s2_im);
            s4_v    <= s3_v;
            s4_p1   <= s3_p1;
            s4_k    <= s3_k;
            s4_full <= s3_pr + s3_pi;
            s5_v     <= s4_v && !s4_p1;
            s5_k     <= s4_k;
            s5_floor <= fl;
            s5_up    <= up;
            if (s4_v && s4_p1 && s4_full > largest) largest <= s4_full;
            if (s5_v) begin
                gsum <= gsum + GSUM_BITS'(g_new);
                for (int p = 0; p < 4; p++) if (int'(s5_k) == PILOT_BIN[p]) pg[p] <= g_new;
            end

            unique case (state)
                IDLE, READY: if (t1_wr) begin
                    state   <= LOAD;
                    largest <= '0;
                    t2cnt   <= '0;
                    pend    <= 1'b0;
                    cr      <= 1'b0;
                end
                LOAD: if (t2_in) begin
                    // The first pass ends on T2's 52nd used bin.
                    t2cnt <= t2cnt + 6'd1;
                    if (t2cnt == 6'd51) state <= DRAIN;
                end
                DRAIN: if (!s1_v && !s2_v && !s3_v && !s4_v) begin
                    shift <= bitlen64(64'(largest)) > 6'(U_BITS - 1)
                             ? 5'(bitlen64(64'(largest)) - 6'(U_BITS - 1)) : 5'd0;
                    state <= PASS2;
                    ra    <= 6'd1;
                    gsum  <= '0;
                end
                PASS2: begin
                    ra <= ra == 6'd26 ? 6'd38 : ra + 6'd1;
                    if (ra == 6'd63) state <= P2DRAIN;
                end
                P2DRAIN: if (!s1_v && !s2_v && !s3_v && !s4_v && !s5_v) begin
                    gsum_n     <= 5'(bitlen64(64'(gsum)));
                    chan_valid <= 1'b1;
                    state      <= QUOT;
                    mstep      <= 4'd0;
                    next_job   <= 3'd0;
                end
                default: ;
            endcase

            // The tracker's products and shifts, one step a clock from QUOT's first.
            if (state == QUOT && !cr) begin
                mstep <= mstep + 4'd1;
                unique case (mstep)
                    4'd3: po <= mp[29:0];
                    4'd4: pi <= mp[29:0];
                    4'd5: sa <= mp[36:0];
                    4'd7: no <= mp[45:0];
                    4'd8: ni <= mp[45:0];
                    4'd9: spread <= mp;
                    4'd10: d <= {no, 3'b000} + 50'(no) + 50'(ni);
                    4'd11: s <= bitlen64(64'(d)) > 6'(M_BITS) ? bitlen64(64'(d)) - 6'(M_BITS) : 6'd0;
                    4'd12: begin
                        dn  <= 16'(d >> s);
                        nos <= 16'(no >> s);
                        nis <= 16'(ni >> s);
                        sps <= spread >> s;
                        dz  <= d == '0;
                    end
                    4'd13: begin
                        // (3·no' << (K + 1)) + 14·dn over 28·dn: the coefficient rounded half up.
                        num_o <= (NUM_W'(nos) * NUM_W'(3) << (K_FRAC + 1)) + NUM_W'(dn) * NUM_W'(14);
                        num_i <= (NUM_W'(nis) << (K_FRAC + 1)) + NUM_W'(dn) * NUM_W'(14);
                        den28 <= DEN_W'(dn) * DEN_W'(28);
                        den52 <= DEN_W'(dn) * DEN_W'(52);
                        ovf   <= x5 >= 56'(y208);
                        num_v <= NUM_W'(x5) << V_FRAC;
                        cr    <= 1'b1;
                    end
                    default: ;
                endcase
            end

            // Requests for a data scale.
            if (scale_req && (state == QUOT || state == READY)) begin
                pend <= 1'b1;
                pmod <= scale_mod;
            end

            // A division done: the quotient, with the cases the model replaces.
            if (div_done && waiting) begin
                waiting <= 1'b0;
                q_valid <= 1'b1;
                q_job   <= cur_job;
                if (cur_job == 3'd0 || cur_job[2]) q_value <= gsum == '0 ? '0 : div_quot;
                else if (dz) q_value <= '0;
                else if (cur_job == 3'd3) q_value <= ovf ? Q_W'((1 << VAR_Q) - 1) : div_quot;
                else q_value <= div_quot;
                if (!cur_job[2]) begin
                    next_job <= cur_job + 3'd1;
                    if (cur_job == 3'd3) state <= READY;
                end
            end

            // A division asked.
            if (job_ok) begin
                waiting   <= 1'b1;
                div_start <= 1'b1;
                if (state == READY) begin
                    cur_job   <= 3'd4 + 3'(pmod);
                    div_num   <= NUM_W'(SCALE_A[pmod]) << gsum_n;
                    div_den   <= DEN_W'(gsum);
                    div_qbits <= QB_W'(SCALE_Q[pmod]);
                    pend      <= 1'b0;
                end else begin
                    cur_job <= nj;
                    unique case (nj)
                        3'd0: begin
                            div_num   <= NUM_W'(SCALE_A[0]) << gsum_n;
                            div_den   <= DEN_W'(gsum);
                            div_qbits <= QB_W'(SCALE_Q[0]);
                        end
                        3'd1: begin
                            div_num   <= num_o;
                            div_den   <= den28;
                            div_qbits <= QB_W'(COEF_Q);
                        end
                        3'd2: begin
                            div_num   <= num_i;
                            div_den   <= den28;
                            div_qbits <= QB_W'(COEF_Q);
                        end
                        default: begin
                            div_num   <= ovf ? '0 : num_v;
                            div_den   <= den52;
                            div_qbits <= QB_W'(VAR_Q);
                        end
                    endcase
                end
            end
        end
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk)
        if (rst_n && !drop && bin_valid && used) begin
            if (!bin_t2 && !open_t1) $error("rx_channel: T1 bin %0d in state %0d", bin_idx, state);
            if (bin_t2 && state != LOAD) $error("rx_channel: T2 bin %0d in state %0d", bin_idx, state);
            if (t1_wr && s1_v && s1_p1) $error("rx_channel: T1 bin while T2's write is pending");
        end
    // One data-scale request outstanding, and only while the packet's channel exists.
    always_ff @(posedge clk)
        if (rst_n && !drop && scale_req) begin
            if (state != QUOT && state != READY) $error("rx_channel: scale_req in state %0d", state);
            if (t1_wr) $error("rx_channel: scale_req on the next packet's first bin");
            if (pend) $error("rx_channel: scale_req while one is pending");
            if (waiting && cur_job[2]) $error("rx_channel: scale_req while a data scale is divided");
        end
`endif
endmodule

`default_nettype wire
