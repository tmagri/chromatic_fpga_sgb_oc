// sgb_sfx_play.v -- SGB built-in SFX bank playback engine (PSRAM-streamed PCM)
// ---------------------------------------------------------------------------
// Plays pre-rendered sound effects (the SGB BIOS built-in SFX bank, rendered
// to WAV then packed as UNCOMPRESSED 16-bit mono PCM, see
// sgb_sfx/build_bank_pcm.py) streamed from PSRAM, triggered by effect index.
//
//   xClk -- PSRAM arbiter side. Streams 16-bit PCM words from PSRAM into an
//           async FIFO (gray-coded pointers, BSRAM storage).
//   hClk -- audio side (16.777216 MHz). Emits one PCM sample per tick
//           (~16 kHz) on hPcm/hPcmValid/hPlaying.
//
// Bank = contiguous 16-bit mono PCM at BANK_BASE in PSRAM; per effect:
//   offset / byte length / loop byte offset (0x1FFFF = one-shot).
// PCM is 2 bytes/sample and every stream offset/length/loop offset is even,
// so a FIFO word is always a whole sample -- no byte-alignment bookkeeping.
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
// RESOURCES (GW5A-25 target is ~97% CLS; same discipline as sgb_snd.v):
//   FIFO is one BSRAM (512 x 16, simple-dual-port inference, registered
//   read), sample tick is a 12-bit down-counter (~15993 Hz, -0.04%), no DSP
//   blocks, no BRR prediction -- PCM playback is just a sample read, so this
//   is smaller than the previous BRR-decoding version.
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
    input      [2:0]  hEffectIndex,    // 0..7

    // PCM output (hClk)
    output reg        hPcmValid = 1'b0, // one-hClk pulse per output sample
    output reg signed [15:0] hPcm = 16'sd0, // holds last sample; 0 when idle
    output reg        hPlaying = 1'b0
);

    localparam FW = 9;                 // FIFO: 512 words x 16 bit (1 BSRAM)

    assign xRamDin = 16'd0;
    assign xRamRnW = 1'b1;

    // ---- effect table: {offset[22:0], bytes[16:0], loop[16:0]} ----
    // The 8 effects Kirby Dream Land 2 requests (see sgb_sfx/bank_pcm).
    // Uncompressed 16-bit mono PCM; loop offsets are byte offsets (even).
    // bytes/loop need 17 bits: B0B is 89220 bytes, B08 loops at 79936.
    function [56:0] fx_tab;
        input [2:0] i;
        begin
            case (i)
                3'd0:    fx_tab = {23'd0,      17'd22660, 17'h1FFFF};  // A1F SwordSwing
                3'd1:    fx_tab = {23'd22672,  17'd27140, 17'h1FFFF};  // A26 PictureFloats
                3'd2:    fx_tab = {23'd49824,  17'd17540, 17'h1FFFF};  // A30 SmallLaser
                3'd3:    fx_tab = {23'd67376,  17'd18400, 17'h00BA0};  // B01 ApplauseSmall
                3'd4:    fx_tab = {23'd85776,  17'd37436, 17'h03BC0};  // B04 Wind
                3'd5:    fx_tab = {23'd123216, 17'd14428, 17'h00E20};  // B07 StormThunder
                3'd6:    fx_tab = {23'd137648, 17'd84600, 17'h13840};  // B08 LightningB
                // B0B loops its whole track: on the real SGB every SFX-B is a
                // sustained loop (plays until a SOUND stop), but the bank
                // builder's loop detector saw only one wave + the 250 ms quiet
                // gap and mis-flagged it one-shot. Loop from byte 0.
                3'd7:    fx_tab = {23'd222256, 17'd89220, 17'h00000};  // B0B Wave
                default: fx_tab = 57'd0;
            endcase
        end
    endfunction

    // hClk needs only the one-shot sample count (bytes/2) of the triggered
    // effect. A narrow 16-bit table here instead of a second copy of the
    // full 57-bit fx_tab mux tree saves ~40 mux bits of LUTs (the design
    // routes at ~97% CLS). Keep in sync with fx_tab (byte length / 2).
    function [15:0] fx_samples;
        input [2:0] i;
        begin
            case (i)
                3'd0:    fx_samples = 16'd11330;  // A1F 22660/2
                3'd1:    fx_samples = 16'd13570;  // A26 27140/2
                3'd2:    fx_samples = 16'd8770;   // A30 17540/2
                3'd3:    fx_samples = 16'd9200;   // B01 18400/2
                3'd4:    fx_samples = 16'd18718;  // B04 37436/2
                3'd5:    fx_samples = 16'd7214;   // B07 14428/2
                3'd6:    fx_samples = 16'd42300;  // B08 84600/2
                3'd7:    fx_samples = 16'd44610;  // B0B 89220/2
                default: fx_samples = 16'd0;
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

    // FIFO storage (BSRAM simple dual port: xClk write, hClk read)
    reg  [15:0] fmem [0:(1<<FW)-1];
    reg  [FW:0] wbin = 0, wgray = 0;
    reg  [FW:0] rgray_x1 = 0, rgray_x = 0;
    reg  [FW:0] wgray_h1 = 0, wgray_h = 0;
    reg  [FW:0] rbin = 0, rgray = 0;

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
    reg [16:0] rem_w = 0;              // words left to fetch in this segment
    reg [22:0] curAddr = 0;
    reg [FW:0] burst_w = 0;

    // hClk ack sync + flush/run control
    reg [1:0] ack_h1 = 0, ack_h = 0;
    reg [3:0] ackp_h1 = 0, ackp_h = 0;
    reg [2:0] settle_h = 3'd0;
    reg [1:0] ack_last = 0;
    reg        flush_active = 0;
    reg        run_state = 0;
    reg [10:0] flush_rem = 0;          // bytes left to discard

    // PCM reader
    reg        looping = 0;            // current effect loops (plays until stop)
    reg [15:0] srem = 0;               // samples left (one-shot only)
    reg [15:0] wbuf = 0;               // current sample word from FIFO
    reg        wbuf_v = 0;             // wbuf holds a sample not yet emitted
    reg        rd_pend = 0;            // FIFO read in flight
    reg [11:0] tick_cnt = 12'd0;

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
    // 57-bit layout: {offset[22:0], bytes[16:0], loop[16:0]}
    wire [56:0] fxt      = fx_tab(fxIdx);
    wire [22:0] seg_base = BANK_BASE + fxt[56:34]
                           + (seg_loop ? {6'd0, fxt[16:0]} : 23'd0);
    wire [16:0] seg_len  = seg_loop ? (fxt[33:17] - fxt[16:0]) : fxt[33:17];
    // words to fetch = ceil((len + base[0]) / 2); len is even for PCM and the
    // bank offsets are 16-byte aligned, but keep it general. 17 bits like
    // seg_len; the largest effect (B0B) is 44610 words.
    wire [16:0] seg_remw = ({1'b0, seg_len} + {16'd0, seg_base[0]} + 18'd1) >> 1;
    // Loop presence by index decode: entries 3..7 (all SFX-B) loop, 0..2
    // (SFX-A) are one-shot. Equivalent to fxt[16:0] != 17'h1FFFF but a
    // 3-bit compare instead of a 17-bit one on the muxed table output
    // (~97% CLS budget). Keep in sync with fx_tab.
    wire        has_loop = (fxIdx >= 3'd3);

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
                    if (rem_w < {7'd0, burst_w}) burst_w = rem_w[FW:0];
                    xRamAddr     <= curAddr;
                    xRamBurstLen <= {burst_w[FW-1:0], 1'b0};
                    xRamReq      <= 1'b1;
                    req_out      <= 1'b1;
                    rem_w        <= rem_w - {7'd0, burst_w};
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
    // hClk: FIFO word pump + PCM sample output
    // =====================================================================
    wire        words_avail = (wbin_h != rbin);
    wire        sample_tick = (tick_cnt == 12'd0);
    // new-command echo edge this cycle: the flush snapshot reads rbin here,
    // so the pump must NOT advance it on the same edge.
    wire        ack_event = (settle_h == 3'd7) && (ack_h != ack_last);
    // one sample per tick while running; a looping effect runs until stopped.
    wire        want_sample = run_state && (looping || (srem != 0));
    // take one word from wbuf this cycle: flush discards fast, samples emit
    // only on a tick (one sample per tick).
    wire        emit   = wbuf_v && sample_tick && want_sample && !flush_active;
    wire        fdrop  = wbuf_v && flush_active;         // flush discard
    wire        consume = !ack_event && (emit || fdrop);
    wire [FW:0] rbin_next  = rbin + (consume ? {{FW{1'b0}},1'b1} : {(FW+1){1'b0}});
    wire [FW:0] rgray_next = (rbin_next >> 1) ^ rbin_next;

    // BSRAM read: fword is fmem[rbin] registered (1-cycle latency). rd_pend
    // marks a read in flight; the word lands in wbuf the next cycle.
    reg [15:0] fword;
    always @(posedge hClk) fword <= fmem[rbin[FW-1:0]];

    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            rbin <= 0; rgray <= 0; rd_pend <= 0; wbuf_v <= 0; wbuf <= 0;
        end else begin
            rbin  <= rbin_next;
            rgray <= rgray_next;

            // consume the staged sample word
            if (consume) wbuf_v <= 1'b0;

            // refill wbuf from the FIFO. Never overwrite a staged sample
            // (wbuf_v) and don't read while a read is in flight (rd_pend).
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
    // hClk: flush / run control + sample emit
    // =====================================================================
    // Gowin rejects a part-select directly on a function call, so latch the
    // table lookup for the triggering effect into a wire first.
    wire [15:0] ack_smp = fx_samples(ackp_h[2:0]);

    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            flush_active <= 0; run_state <= 0; hPlaying <= 0; flush_rem <= 0;
            ack_last <= 0; looping <= 0; srem <= 0; hPcm <= 0; hPcmValid <= 0;
        end else begin
            hPcmValid <= 1'b0;

            if (settle_h != 3'd7) begin
                ack_last <= ack_h;
            end else if (ack_h != ack_last) begin
                ack_last <= ack_h;
                if (ackp_h[3]) begin
                    // snapshot flush: discard exactly the residue present now
                    hPlaying  <= 1'b1;
                    run_state <= 1'b0;
                    // entries 3..7 (SFX-B) loop; keep in sync with fx_tab
                    looping   <= (ackp_h[2:0] >= 3'd3);
                    srem      <= ack_smp;             // one-shot sample count
                    if (wbin_h == rbin) begin
                        flush_active <= 1'b0;
                        run_state    <= 1'b1;
                    end else begin
                        flush_active <= 1'b1;
                        flush_rem    <= {wbin_h, 1'b0} - {rbin, 1'b0};
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

            // emit one sample per tick
            if (emit) begin
                hPcm      <= $signed(wbuf);
                hPcmValid <= 1'b1;
                if (!looping) begin
                    srem <= srem - 16'd1;
                    if (srem == 16'd1) begin
                        hPlaying  <= 1'b0;    // one-shot finished after this
                        run_state <= 1'b0;
                    end
                end
            end

            // flush discard: one word per consume, 2 bytes each
            if (fdrop && !ack_event) begin
                if (flush_rem <= 11'd2) begin
                    flush_active <= 1'b0;
                    run_state    <= 1'b1;
                end else flush_rem <= flush_rem - 11'd2;
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

endmodule
