// sgb_sfx_play.v -- SGB built-in SFX bank playback engine (PSRAM-streamed BRR)
// ---------------------------------------------------------------------------
// Plays pre-rendered BRR sound effects (the SGB BIOS built-in SFX bank, see
// sgb_sfx/build_bank.py) streamed from PSRAM, triggered by an effect index.
//
//   xClk -- PSRAM arbiter side. Streams BRR bytes from PSRAM into an async
//           FIFO (gray-coded pointers, BSRAM storage).
//   hClk -- audio side (16.777216 MHz). Reads BRR bytes, decodes BRR at
//           ~16 kHz, emits hPcm/hPcmValid/hPlaying for the audio mixer.
//
// Bank = contiguous BRR at BANK_BASE in PSRAM; per effect:
//   offset / byte length / loop byte offset (0xFFFF = one-shot).
//
// BYTE ALIGNMENT (the odd-offset fix)
//   PSRAM delivers 16-bit words, so every fetch is even-aligned, but BRR
//   blocks are 9 bytes: both stream lengths and loop offsets can be ODD
//   (e.g. B01 loop@837, B07 loop@1017 in the 7-effect bank). Two rules keep
//   the byte stream coherent:
//     * writer: every segment fetch starts at (base & ~1) and covers
//       ceil((bytes + base[0]) / 2) words -- at most one garbage byte leads
//       (odd base) and one garbage byte trails (odd total) per segment;
//     * reader: consumes bytes in stream order and discards the known
//       garbage at each boundary:
//         skip_start = offset[0]                      at playback start
//         skip_wrap  = (bytes[0] ^ offset[0])         at every loop wrap
//                    + (offset[0] ^ loop[0])
//       (tail pad of the ending segment + leading pad of the next; the tail
//       term is the same for the first pass and for loop passes). Both are
//       pure functions of the static effect table.
//
// BRR DECODE follows the ares SNES DSP algorithm exactly (the same one the
// bank tooling was validated against, sgb_sfx/snesdsp.py _brr_decode):
//   half-scale sample = (nib << shift) >> 1, filter from the two previous
//   *full-scale* outputs (p2 pre-halved), clamp16, then <<1 with 16-bit
//   wrap. Nibbles are HIGH-first within each BRR data byte. All four
//   filters; arithmetic widened so f2/f3 cannot wrap before the clamp.
//   Predictor history is zeroed at playback start and preserved across
//   loop wraps (like the real DSP voice).
//
// CONTROL
//   hStart pulse -> (re)trigger hEffectIndex; interrupts a playing effect.
//   hStop  pulse -> stop (SGB SOUND bit-7 semantics), applied immediately.
//   Commands cross to xClk on a gray sequence + {play,idx} payload; the
//   writer echoes the processed sequence back. The reader starts a new
//   effect only on that echo, flushing first exactly the FIFO residue
//   snapshot taken at that moment: the writer has already stopped the old
//   stream (>=16 xClk gap before the echo guarantees the write-gray
//   synchronizers caught the last old word), and new-stream words only
//   arrive after the snapshot, so they are never flushed.
//
// RESOURCES (GW5A-25 target is >90% used; same discipline as sgb_snd.v):
//   FIFO is one BSRAM (512 x 16, simple-dual-port inference, registered
//   read), sample tick is an 11-bit down-counter (~15993 Hz, -0.04%),
//   no DSP blocks, all filter math is shifted-add in LUTs.
// ---------------------------------------------------------------------------

module sgb_sfx_play #(
    parameter [22:0] BANK_BASE = 23'h100000,
    // sample-tick divider (hClk cycles per tick, minus one). 1048 gives
    // 16.777216 MHz / 1049 ~ 15993.5 Hz; overridable to speed up sims.
    parameter [11:0] TICK_PERIOD = 12'd1048
)(
    // PSRAM / arbiter side (xClk)
    input             xClk,
    input             xReset,
    input             xRamReady,       // BIST finished; no PSRAM traffic before
    output reg        xRamReq,         // one-cycle pulse per burst
    output            xRamRnW,
    output reg [22:0] xRamAddr,
    output     [15:0] xRamDin,
    output reg [10:0] xRamBurstLen,    // bytes, always even and >= 2
    input             xRamDone,
    input             xRamDoutValid,
    input      [15:0] xRamDout,

    // control (hClk)
    input             hClk,
    input             hReset,
    input             hStart,          // pulse: begin playing hEffectIndex
    input             hStop,           // pulse: stop playback
    input      [2:0]  hEffectIndex,    // 0..6

    // PCM output (hClk)
    output reg        hPcmValid = 1'b0, // one-hClk pulse per output sample
    output reg signed [15:0] hPcm = 16'sd0, // holds last sample; 0 when idle
    output reg        hPlaying = 1'b0
);

    localparam FW = 9;                 // FIFO: 512 words x 16 bit (1 BSRAM)

    assign xRamDin = 16'd0;
    assign xRamRnW = 1'b1;

    // ---- effect table: {offset[22:0], bytes[15:0], loop[15:0]} ----
    // The 8 effects Kirby Dream Land 2 can request (see sgb_sfx/bank7).
    function [54:0] fx_tab;
        input [2:0] i;
        begin
            case (i)
                3'd0:    fx_tab = {23'd0,     16'd6381,  16'hFFFF};  // A1F SwordSwing
                3'd1:    fx_tab = {23'd6384,  16'd7641,  16'hFFFF};  // A26 PictureFloats
                3'd2:    fx_tab = {23'd14032, 16'd4941,  16'hFFFF};  // A30 SmallLaser
                3'd3:    fx_tab = {23'd18976, 16'd5175,  16'd837};   // B01 ApplauseSmall
                3'd4:    fx_tab = {23'd24160, 16'd10530, 16'd4302};  // B04 Wind
                3'd5:    fx_tab = {23'd34704, 16'd4059,  16'd1017};  // B07 StormThunder
                3'd6:    fx_tab = {23'd38768, 16'd23796, 16'd22482}; // B08 LightningB
                3'd7:    fx_tab = {23'd62576, 16'd25101, 16'hFFFF};  // B0B Wave
                default: fx_tab = 55'd0;
            endcase
        end
    endfunction

    // reader-side parity {bytes[0], offset[0], loop[0]} per effect
    function [2:0] fx_par;
        input [2:0] i;
        begin
            case (i)
                3'd0:    fx_par = 3'b100;
                3'd1:    fx_par = 3'b100;
                3'd2:    fx_par = 3'b100;
                3'd3:    fx_par = 3'b101;
                3'd4:    fx_par = 3'b000;
                3'd5:    fx_par = 3'b101;
                3'd6:    fx_par = 3'b000;
                3'd7:    fx_par = 3'b100;
                default: fx_par = 3'b000;
            endcase
        end
    endfunction

    // =====================================================================
    // declarations
    // =====================================================================

    // hClk command encoding
    reg [1:0] cmd_seq = 2'd0;
    reg [3:0] cmd_pl  = 4'd0;
    reg hStart_q = 1'b0, hStop_q = 1'b0;

    // xClk command sync
    reg [1:0] seq_x1 = 0, seq_x2 = 0, seq_last = 0;
    reg [3:0] pl_x1 = 0, pl_x2 = 0;
    reg [2:0] settle_x = 3'd0;

    // writer echo of processed commands
    reg [1:0] ack_seq = 2'd0;
    reg [3:0] ack_pl  = 4'd0;

    // FIFO
    reg  [15:0] fmem [0:(1<<FW)-1];
    reg  [FW:0] wbin = 0, wgray = 0;
    reg  [FW:0] rgray_x1 = 0, rgray_x = 0;
    reg  [FW:0] wgray_h1 = 0, wgray_h = 0;
    reg  [FW:0] rbin = 0, rgray = 0;
    reg         byte_sel = 0;          // 0 = low byte of word is next
    reg  [15:0] fword = 0;
    reg         rd_pend = 0;
    reg  [15:0] wbuf = 0;
    reg         wbuf_v = 0;
    reg  [1:0]  skip_cnt = 0;
    reg         byte_valid = 0;
    reg  [7:0]  bdata = 0;

    // writer streaming FSM
    localparam ST_IDLE=3'd0, ST_GAP=3'd1, ST_SEG=3'd2, ST_REQ=3'd3,
               ST_RX=3'd4, ST_ABORT=3'd5;
    reg [2:0]  st = ST_IDLE;
    reg [4:0]  gap = 5'd0;
    reg        cmd_pend = 0, pend_play = 0;
    reg [2:0]  pend_idx = 0, fxIdx = 0;
    reg [1:0]  pend_seq = 0;
    reg        seg_loop = 0;           // 0 = first pass, 1 = loop region
    reg        req_out = 0;            // burst request outstanding in arbiter
    reg [13:0] rem_w = 0;              // words left to fetch in this segment
    reg [22:0] curAddr = 0;
    reg [FW:0] burst_w = 0;

    // hClk ack sync + flush/run control
    reg [1:0] ack_h1 = 0, ack_h = 0;
    reg [3:0] ackp_h1 = 0, ackp_h = 0;
    reg [2:0] settle_h = 3'd0;
    reg [1:0] ack_last = 0;
    reg        flush_active = 0;
    reg        flush_start = 0;        // 1-cycle: clear stale pump state
    reg        run_start = 0;          // 1-cycle: decoder priming
    reg        run_state = 0;
    reg [10:0] flush_rem = 0;          // bytes left to discard
    reg [1:0]  skip_start = 0;
    reg [1:0]  skip_wrap = 0;

    // BRR decoder
    reg        bst = 1'b0;             // 0 = header, 1 = data
    reg [3:0]  nib_i = 0;
    reg [3:0]  bshift = 0;
    reg [1:0]  bfilter = 0;
    reg        bend = 0, bloop = 0;
    reg signed [15:0] bprev1 = 0, bprev2 = 0;
    reg [7:0]  cur_byte = 0;
    reg        have_cur = 0;
    reg        end_pulse_q = 0;        // decoder natural end (1 cycle)
    reg        wrap_pulse = 0;         // decoder: load skip_wrap (1 cycle)
    reg        breq = 0;               // decoder wants a byte
    reg [11:0] tick_cnt = 12'd0;

    integer sn;
    reg signed [15:0] n16, half, fout, out16;
    reg signed [17:0] acc;
    reg signed [19:0] p1x, p2x, t13;

    // =====================================================================
    // hClk command encoding: gray sequence + {play,idx} payload
    // =====================================================================
    wire [1:0] gray_inc = {cmd_seq[0], ~cmd_seq[0]};  // +1 on a 2-bit gray
    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            cmd_seq <= 2'd0; cmd_pl <= 4'd0; hStart_q <= 0; hStop_q <= 0;
        end else begin
            hStart_q <= hStart;
            hStop_q  <= hStop;
            if (hStart && !hStart_q) begin
                cmd_seq <= cmd_seq ^ gray_inc;
                cmd_pl  <= {1'b1, hEffectIndex};
            end else if (hStop && !hStop_q) begin
                cmd_seq <= cmd_seq ^ gray_inc;
                cmd_pl  <= 4'd0;
            end
        end
    end

    // =====================================================================
    // xClk: command sync
    // =====================================================================
    always @(posedge xClk or posedge xReset) begin
        if (xReset) begin
            seq_x1 <= 0; seq_x2 <= 0; pl_x1 <= 0; pl_x2 <= 0; settle_x <= 0;
        end else begin
            seq_x1 <= cmd_seq; seq_x2 <= seq_x1;
            pl_x1  <= cmd_pl;  pl_x2  <= pl_x1;
            if (settle_x != 3'd7) settle_x <= settle_x + 3'd1;
        end
    end
    wire cmd_new     = (settle_x == 3'd7) && (seq_x2 != seq_last);
    wire cmd_play    = pl_x2[3];
    wire [2:0] cmd_idx = pl_x2[2:0];

    // =====================================================================
    // xClk: FIFO write side (BSRAM simple dual port)
    // =====================================================================
    wire        xWrEn = (st == ST_RX) && xRamDoutValid;
    wire [FW:0] wbin_next  = wbin + (xWrEn ? {{FW{1'b0}},1'b1} : {(FW+1){1'b0}});
    wire [FW:0] wgray_next = (wbin_next >> 1) ^ wbin_next;
    always @(posedge xClk) begin
        if (xWrEn) fmem[wbin[FW-1:0]] <= xRamDout;
        wbin  <= wbin_next;
        wgray <= wgray_next;
    end

    // sync rgray into xClk -> binary reader pointer (stale = conservative)
    always @(posedge xClk) begin rgray_x1 <= rgray; rgray_x <= rgray_x1; end
    wire [FW:0] rbin_x = rgray_x ^ (rgray_x >> 1) ^ (rgray_x >> 2)
                               ^ (rgray_x >> 3) ^ (rgray_x >> 4)
                               ^ (rgray_x >> 5) ^ (rgray_x >> 6)
                               ^ (rgray_x >> 7) ^ (rgray_x >> 8) ^ (rgray_x >> 9);
    // free words. v = (rbin_x - wbin) mod 2^(FW+1): with occupancy occ in
    // [0, 2^FW], v is 0 when occ=0, else 2^(FW+1) - occ. So free = 2^FW when
    // v=0, else v - 2^FW (values 1..2^FW-1 cannot occur without overflow).
    // The stale synced reader pointer only ever understates free space.
    wire [FW:0] free_w = rbin_x - wbin;
    wire [FW:0] space  = (free_w == {(FW+1){1'b0}}) ? {1'b1,{FW{1'b0}}} :
                         (free_w >  {1'b1,{FW{1'b0}}}) ? (free_w - {1'b1,{FW{1'b0}}}) :
                         {(FW+1){1'b0}};

    // segment parameters (combinational from the table)
    wire [54:0] fxt      = fx_tab(fxIdx);
    wire [22:0] seg_base = BANK_BASE + fxt[54:32]
                           + (seg_loop ? {7'd0, fxt[15:0]} : 23'd0);
    wire [15:0] seg_len  = seg_loop ? (fxt[31:16] - fxt[15:0]) : fxt[31:16];
    wire [13:0] seg_remw = ({1'b0, seg_len} + {13'd0, seg_base[0]} + 14'd1) >> 1;
    wire        has_loop = (fxt[15:0] != 16'hFFFF);

    // =====================================================================
    // xClk: streaming FSM
    // =====================================================================
    always @(posedge xClk or posedge xReset) begin
        if (xReset) begin
            st <= ST_IDLE; xRamReq <= 0; req_out <= 0; cmd_pend <= 0;
            gap <= 0; rem_w <= 0; curAddr <= 0; seg_loop <= 0; fxIdx <= 0;
            ack_seq <= 0; ack_pl <= 0; pend_play <= 0; pend_idx <= 0;
            pend_seq <= 0; burst_w <= 0;
        end else begin
            xRamReq <= 1'b0;

            case (st)
            ST_IDLE: begin
                if (cmd_pend && xRamReady) begin
                    cmd_pend <= 0;
                    st  <= ST_GAP;
                    gap <= 5'd16;
                end
            end
            ST_GAP: begin
                // let the hClk write-gray synchronizers catch the last old
                // word before the echo tells the reader to flush/start
                if (gap == 5'd0) begin
                    cmd_pend <= 1'b0;   // pend consumed (a same-cycle cmd_new
                    ack_seq <= pend_seq; // latch below wins if one arrived)
                    ack_pl  <= {pend_play, pend_idx};
                    if (pend_play) begin
                        fxIdx    <= pend_idx;
                        seg_loop <= 1'b0;
                        st <= ST_SEG;
                    end else st <= ST_IDLE;
                end else gap <= gap - 5'd1;
            end
            ST_SEG: begin
                curAddr <= {seg_base[22:1], 1'b0};
                rem_w   <= seg_remw;
                st      <= ST_REQ;
            end
            ST_REQ: begin
                if (cmd_pend) begin
                    st <= ST_ABORT;
                end else if (rem_w == 0) begin
                    if (has_loop) begin
                        seg_loop <= 1'b1;
                        st <= ST_SEG;
                    end else st <= ST_IDLE;      // natural one-shot end
                end else if (xRamReady && space >= 10'd24) begin
                    burst_w = (space - 10'd16 > {4'd0, 9'd128}) ? {1'b0, 9'd128}
                                                                : space - 10'd16;
                    if (rem_w < {4'd0, burst_w}) burst_w = rem_w[FW:0];
                    xRamAddr     <= curAddr;
                    xRamBurstLen <= {burst_w[FW-1:0], 1'b0};
                    xRamReq      <= 1'b1;
                    req_out      <= 1'b1;
                    rem_w        <= rem_w - {4'd0, burst_w};
                    curAddr      <= curAddr + {7'd0, burst_w, 1'b0};
                    st           <= ST_RX;
                end
            end
            ST_RX: begin
                if (cmd_pend) st <= ST_ABORT;
                else if (xRamDone) begin req_out <= 0; st <= ST_REQ; end
            end
            ST_ABORT: begin
                // drain an outstanding burst (data discarded) before the gap
                if (req_out) begin
                    if (xRamDone) req_out <= 0;
                end else st <= ST_GAP;
                gap <= 5'd16;
            end
            endcase

            // new-command latch LAST so it wins over a same-cycle consume
            if (cmd_new) begin
                seq_last  <= seq_x2;
                cmd_pend  <= 1;
                pend_play <= cmd_play;
                pend_idx  <= cmd_idx;
                pend_seq  <= seq_x2;
            end
        end
    end

    // =====================================================================
    // hClk: ack sync
    // =====================================================================
    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            ack_h1 <= 0; ack_h <= 0; ackp_h1 <= 0; ackp_h <= 0; settle_h <= 0;
        end else begin
            ack_h1 <= ack_seq; ack_h <= ack_h1;
            ackp_h1 <= ack_pl; ackp_h <= ackp_h1;
            if (settle_h != 3'd7) settle_h <= settle_h + 3'd1;
        end
    end

    // sync wgray into hClk -> binary writer pointer (lagging = safe)
    always @(posedge hClk) begin wgray_h1 <= wgray; wgray_h <= wgray_h1; end
    wire [FW:0] wbin_h = wgray_h ^ (wgray_h >> 1) ^ (wgray_h >> 2)
                               ^ (wgray_h >> 3) ^ (wgray_h >> 4)
                               ^ (wgray_h >> 5) ^ (wgray_h >> 6)
                               ^ (wgray_h >> 7) ^ (wgray_h >> 8) ^ (wgray_h >> 9);

    // =====================================================================
    // hClk: FIFO byte pump
    // =====================================================================
    wire        words_avail = (wbin_h != rbin);
    wire        sample_tick = (tick_cnt == 12'd0);
    // new-command echo edge this cycle: the flush snapshot reads rbin/byte_sel
    // here, so the pump must NOT advance them on the same edge (else the
    // snapshot counts the byte the old stream is mid-eating -> off-by-one).
    wire        ack_event = (settle_h == 3'd7) && (ack_h != ack_last);
    // one decode step per tick; a latched data byte feeds its 2nd nibble
    // without needing a fresh byte_valid.
    wire        need_byte = (bst == 1'b0) || ((bst == 1'b1) && !have_cur);
    wire        bconsume = run_state && !ack_event && sample_tick &&
                           (need_byte ? byte_valid : 1'b1);
    // take one byte: decoder demand, skip discard, or flush discard
    wire        consume = wbuf_v && !ack_event && (flush_active ||
                          (run_state && !byte_valid && (breq || skip_cnt != 0)));
    wire [FW:0] rbin_next  = rbin + ((consume && byte_sel) ? {{FW{1'b0}},1'b1} : {(FW+1){1'b0}});
    wire [FW:0] rgray_next = (rbin_next >> 1) ^ rbin_next;

    always @(posedge hClk) fword <= fmem[rbin[FW-1:0]];

    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            rbin <= 0; rgray <= 0; byte_sel <= 0; rd_pend <= 0;
            wbuf_v <= 0; skip_cnt <= 0; byte_valid <= 0; bdata <= 0;
        end else begin
            rbin  <= rbin_next;
            rgray <= rgray_next;

            if (flush_start || (hStop && !hStop_q)) begin
                byte_valid <= 1'b0;    // stale byte from an aborted playback
            end else if (byte_valid && bconsume) begin
                byte_valid <= 1'b0;    // decoder took the byte
            end

            if (run_start)       skip_cnt <= skip_start;
            else if (wrap_pulse) skip_cnt <= skip_wrap;
            else if (consume && !flush_active && skip_cnt != 0)
                skip_cnt <= skip_cnt - 2'd1;

            if (consume) begin
                byte_sel <= ~byte_sel;
                if (byte_sel) wbuf_v <= 1'b0;
                if (!flush_active && skip_cnt == 0 && breq && !byte_valid) begin
                    byte_valid <= 1'b1;
                    bdata <= byte_sel ? wbuf[15:8] : wbuf[7:0];
                end
            end

            if (rd_pend) begin
                wbuf    <= fword;
                wbuf_v  <= 1'b1;
                rd_pend <= 1'b0;
            end else if (!wbuf_v && !rd_pend && words_avail &&
                         (run_state || flush_active)) begin
                rd_pend <= 1'b1;
            end
        end
    end

    // =====================================================================
    // hClk: flush / run control
    // =====================================================================
    wire [2:0] par = fx_par(ackp_h[2:0]);

    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            flush_active <= 0; flush_start <= 0; run_start <= 0; run_state <= 0;
            hPlaying <= 0; flush_rem <= 0;
            skip_start <= 0; skip_wrap <= 0; ack_last <= 0;
        end else begin
            flush_start <= 1'b0;
            run_start   <= 1'b0;

            if (settle_h != 3'd7) begin
                ack_last <= ack_h;
            end else if (ack_h != ack_last) begin
                ack_last <= ack_h;
                if (ackp_h[3]) begin
                    // snapshot flush: discard exactly the residue present now
                    flush_start <= 1'b1;
                    hPlaying    <= 1'b1;
                    run_state   <= 1'b0;
                    skip_start  <= {1'b0, par[1]};             // offset[0]
                    skip_wrap   <= (par[2] ^ par[1]) + (par[1] ^ par[0]);
                    if ({wbin_h, 1'b0} == {rbin, byte_sel}) begin
                        flush_active <= 1'b0;
                        run_state    <= 1'b1;
                        run_start    <= 1'b1;
                    end else begin
                        flush_active <= 1'b1;
                        flush_rem    <= {wbin_h, 1'b0} - {rbin, byte_sel};
                    end
                end else begin
                    // stop echo: local hStop already applied; idempotent
                    hPlaying <= 0; run_state <= 0; flush_active <= 0;
                end
            end

            // immediate local stop
            if (hStop && !hStop_q) begin
                hPlaying <= 0; run_state <= 0; flush_active <= 0;
            end

            // decoder natural end (one-shot finished)
            if (end_pulse_q) begin
                hPlaying <= 0; run_state <= 0;
            end

            if (flush_active && consume && flush_rem != 0) begin
                if (flush_rem == 11'd1) begin
                    flush_active <= 1'b0;
                    run_state    <= 1'b1;
                    run_start    <= 1'b1;
                end else flush_rem <= flush_rem - 11'd1;
            end
        end
    end

    // =====================================================================
    // hClk: ~16 kHz sample tick
    // =====================================================================
    always @(posedge hClk or posedge hReset) begin
        if (hReset) tick_cnt <= 12'd0;
        else tick_cnt <= sample_tick ? TICK_PERIOD : tick_cnt - 12'd1;
    end

    // =====================================================================
    // hClk: BRR decode FSM (ares-exact)
    // =====================================================================
    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            bst <= 0; nib_i <= 0; bshift <= 0; bfilter <= 0;
            bend <= 0; bloop <= 0; bprev1 <= 0; bprev2 <= 0;
            cur_byte <= 0; have_cur <= 0; breq <= 0; wrap_pulse <= 0;
            hPcm <= 0; hPcmValid <= 0; end_pulse_q <= 0;
        end else begin
            hPcmValid   <= 1'b0;
            wrap_pulse  <= 1'b0;
            end_pulse_q <= 1'b0;

            if (!run_state) begin
                breq <= 1'b0;
                hPcm <= 16'sd0;
            end else begin
                if (run_start) begin
                    // fresh stream: zero predictor history, prime header read
                    bst <= 1'b0; nib_i <= 0; have_cur <= 0; breq <= 1'b1;
                end else if (!byte_valid && !breq &&
                             (bst == 1'b0 || !have_cur) && skip_cnt == 0) begin
                    breq <= 1'b1;
                end
            end

            if (bconsume) begin
                breq <= 1'b0;
                if (bst == 1'b0) begin
                    bshift   <= bdata[7:4];
                    bfilter  <= bdata[3:2];
                    bend     <= bdata[0];
                    bloop    <= bdata[1];
                    nib_i    <= 4'd0;
                    have_cur <= 1'b0;
                    bst      <= 1'b1;
                end else begin
                    // one output sample per tick, high nibble first
                    if (!have_cur) begin
                        cur_byte <= bdata;
                        have_cur <= 1'b1;
                    end
                    sn = nib_i[0] ? (have_cur ? cur_byte[3:0] : bdata[3:0])
                                  : (have_cur ? cur_byte[7:4] : bdata[7:4]);
                    n16 = {{12{sn[3]}}, sn[3:0]};
                    if (bshift <= 4'd12) half = (n16 <<< bshift) >>> 1;
                    else                 half = n16[3] ? 16'shF800 : 16'sd0;

                    // $signed: a bare concatenation is UNSIGNED, which would
                    // turn the >>> into a logical shift and corrupt negative
                    // predictor history.
                    p1x = $signed({{4{bprev1[15]}}, bprev1});
                    p2x = $signed({{4{bprev2[15]}}, bprev2}) >>> 1;
                    acc = $signed({{2{half[15]}}, half});
                    case (bfilter)
                        2'd1: acc = acc + (p1x >>> 1) + ((-p1x) >>> 5);
                        2'd2: acc = acc + p1x - p2x + (p2x >>> 4)
                                      + ((-(p1x + (p1x <<< 1))) >>> 6);
                        2'd3: begin
                            t13 = p1x + (p1x <<< 2) + (p1x <<< 3);
                            acc = acc + p1x - p2x + ((-t13) >>> 7)
                                      + ((p2x + (p2x <<< 1)) >>> 4);
                        end
                    endcase
                    if (acc >  18'sd32767)      fout = 16'sd32767;
                    else if (acc < -18'sd32768) fout = -18'sd32768;
                    else                        fout = acc[15:0];
                    out16 = fout <<< 1;          // 16-bit wrap, per ares

                    bprev2    <= bprev1;
                    bprev1    <= out16;
                    hPcm      <= out16;
                    hPcmValid <= 1'b1;

                    if (nib_i == 4'd15) begin
                        have_cur <= 1'b0;
                        if (bend && !bloop) begin
                            end_pulse_q <= 1'b1;  // one-shot done
                        end else begin
                            bst <= 1'b0;          // next block header (bprev1/2
                            if (bend && bloop)    // keep running). Only a real
                                wrap_pulse <= 1'b1; // end+loop wrap crosses the
                        end                       // writer's segment padding.
                    end else begin
                        nib_i <= nib_i + 4'd1;
                        if (nib_i[0] == 1'b1) have_cur <= 1'b0;
                    end
                end
            end
        end
    end

endmodule
