// sgb_sfx_play.v -- SGB built-in SFX bank playback engine (BRR HLE player)
// ---------------------------------------------------------------------------
// Plays the Super Game Boy BIOS's built-in sound-effect bank (all 73 effects:
// SFX-A 0x01..0x30 -> index 0..47, SFX-B 0x01..0x19 -> index 48..72) straight
// from the ROM's original BRR sample data, streamed from PSRAM. This is a
// high-level-emulation player: no SPC700, no Gaussian resampling, no ADSR --
// one dominant BRR voice per effect at the BIOS-recommended pitch (see
// sgb_sfx/extract_apu.py which builds the bank and the per-effect records).
//
//   xClk -- PSRAM arbiter side. Fetches the triggered effect's 16-byte record
//           from the bank table, then streams the sample's BRR bytes (first
//           pass, then the loop region repeatedly) into an async FIFO
//           (gray-coded pointers, BSRAM storage).
//   hClk -- audio side (16.777216 MHz). Reassembles the byte stream, decodes
//           BRR (hardware-validated datapath copied from sgb_snd.v), and emits
//           one sample per per-effect tick on hPcm/hPcmValid/hPlaying.
//
// BANK LAYOUT (loaded at BANK_BASE in PSRAM, built by sgb_sfx/extract_apu.py):
//   +0x000   header  'SFXB', version(5), count, table_off, brr_off, seg_off
//   +0x100   table   73 x 16-byte LE records (8 x u16)
//   +0x800   brr     VERBATIM copy of the ROM's $4DB0 BRR record (41,280 B)
//   +0xA940  segs    pitch-envelope segment table (= brr_off + 41280)
// Record (16 B, little-endian, 8 x u16):
//   u16 brr_off    byte offset of the sample's first BRR block (multiple of 9)
//   u16 loop_off   byte offset of loop start; 0xFFFF = no loop
//   u16 tick       segment-0 hClk divider reload = round(16777216/R_eff)-1
//   u16 blocks     number of first-pass BRR blocks (incl. the terminal block)
//   u16 flags      bit0 = loop region valid; bit1 = loop forever (until stop)
//   u16 count      total BLOCKS to emit; used when flags.bit0 & ~flags.bit1
//   u16 seg_off    entry index of this effect in the segment table
//   u16 seg_count  envelope entries; 0 = fixed pitch (single segment)
// Segment entry (4 B): u16 blocks16 = length of the CURRENT segment in
// 16-sample blocks, u16 tick = divider reload of the NEXT segment. Entry e
// carries segment e's length and segment e+1's tick; the first segment's
// tick is the record's tick field and the final segment has no entry (it
// holds its tick until the effect ends), so the envelope can never desync
// from the block stream.
//
// PLAYBACK MODEL (the "closest enough" HLE contract):
//   * Writer streams [brr_off..end-of-sample] (blocks*9 bytes), then, if the
//     sample loops, [loop_off..end-of-sample] over and over. It never walks
//     BRR headers and never computes the end: for flags.bit1 (loop forever)
//     the reader runs until a stop; for a count-limited one-shot the reader
//     stops after `count` blocks and the writer self-throttles (the FIFO
//     fills, `space` drops below the burst threshold, and it simply stops
//     issuing reads until the next command aborts it).
//   * Pitch envelope (bank v5): when seg_count > 0 the writer fetches the
//     effect's seg_count entries into a small dual-port segment store BEFORE
//     the echo; the reader steps one segment per seg_blk-count block
//     boundaries, reloading the tick divider at each. The envelope only
//     changes the emission RATE -- the emitted sample sequence is identical
//     to the fixed-pitch case. Retriggering overwrites the segment store
//     while the interrupted effect may still be reading it; the resulting
//     sub-100 us glitch on the dying effect is inaudible (it restarts on
//     the echo).
//   * BRR block offsets are multiples of 9, so they alternate odd/even. The
//     writer always fetches from an even-aligned base {base[22:1],1'b0}; the
//     reader skips the base[0] leading pad byte at the start of each segment
//     and the trailing pad at its end (see the byte pump below).
//   * Decode is bit-exact with sgb_sfx/build_bank.py::dec_sample (the
//     reference that generated build/ref_XXX.hex): HIGH-nibble-first, all four
//     predictor filters (shift-add only, no DSP blocks), the invalid-shift
//     rule (shift>12 -> nibble<0 ? -2048 : 0), saturation, and a final x2.
//     Predictor history is zeroed at effect start and CARRIES across the loop
//     jump (the stream is continuous, matching ares/bsnes).
//
// CONTROL
//   hStart pulse -> (re)trigger hEffectIndex; interrupts a playing effect.
//   hStop  pulse -> stop (SGB SOUND bit-7 semantics), applied immediately.
//   Commands cross to xClk on a gray sequence + {play,idx} payload; the
//   writer echoes the processed sequence back. The reader starts a new effect
//   only on that echo, flushing first exactly the FIFO residue snapshot taken
//   at that moment (the writer has already stopped the old stream before the
//   echo, and new-stream words only arrive after the snapshot). The record is
//   fetched and latched BEFORE the echo, so it is stable when the reader
//   samples it on the echo edge.
//
// RESOURCES (GW5A-25 target is ~97% CLS; same discipline as sgb_snd.v):
//   FIFO is one BSRAM (512 x 16, simple-dual-port inference, registered read);
//   decode is a 20-bit adder-only datapath (no DSP); per-effect tick is a
//   16-bit down-counter. If placement is tight, drop FW 9 -> 8 (256 words).
// ---------------------------------------------------------------------------

module sgb_sfx_play #(
    parameter [22:0] BANK_BASE = 23'h100000
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
    input      [6:0]  hEffectIndex,    // 0..47 = SFX-A num-1, 48..72 = SFX-B num-1

    // PCM output (hClk)
    output reg        hPcmValid = 1'b0, // one-hClk pulse per output sample
    output reg signed [15:0] hPcm = 16'sd0, // holds last sample; 0 when idle
    output reg        hPlaying = 1'b0
);

    localparam FW = 9;                 // FIFO: 512 words x 16 bit (1 BSRAM)
    localparam [22:0] TABLE_OFF = 23'h100;
    localparam [22:0] BRR_OFF   = 23'h800;
    localparam [22:0] SEG_OFF   = 23'hA940;  // = BRR_OFF + 41280 (bank v5)
    localparam SW = 10;                // segment store: 1024 x 16 (2 BSRAM)

    assign xRamDin = 16'd0;
    assign xRamRnW = 1'b1;

    // =====================================================================
    // declarations
    // =====================================================================

    // hClk command encoding: gray sequence + {play, idx[6:0]} payload
    reg [1:0] cmd_seq = 2'd0;
    reg [7:0] cmd_pl  = 8'd0;
    reg hStart_q = 1'b0, hStop_q = 1'b0;

    // xClk command sync
    reg [1:0] seq_x1 = 0, seq_x2 = 0, seq_last = 0;
    reg [7:0] pl_x1 = 0, pl_x2 = 0;
    reg [2:0] settle_x = 3'd0;

    // writer echo of processed commands
    reg [1:0] ack_seq = 2'd0;
    reg [7:0] ack_pl  = 8'd0;

    // FIFO storage (BSRAM simple dual port: xClk write, hClk read)
    reg  [15:0] fmem [0:(1<<FW)-1];
    reg  [FW:0] wbin = 0, wgray = 0;
    reg  [FW:0] rgray_x1 = 0, rgray_x = 0;
    reg  [FW:0] wgray_h1 = 0, wgray_h = 0;
    reg  [FW:0] rbin = 0, rgray = 0;

    // latched effect record (xClk), captured during the table fetch
    reg [15:0] rec_brr_off = 0, rec_loop_off = 0, rec_tick = 0;
    reg [15:0] rec_blocks = 0, rec_flags = 0, rec_count = 0;
    reg [15:0] rec_seg_off = 0, rec_seg_cnt = 0;   // bank v5 envelope entry idx/len

    // segment store (bank v5 pitch envelopes): SDPB inference, xClk write,
    // hClk read. Holds ONE effect's envelope entries, refilled per trigger.
    reg  [15:0] smem [0:(1<<SW)-1];
    reg  [SW-1:0] swptr = 0;             // write pointer (entry word pairs)
    reg  [SW-1:0] sraddr = 0;            // reader word address
    reg  [15:0] sword = 0;               // registered smem read data

    // writer streaming FSM
    localparam ST_IDLE=4'd0, ST_TAB=4'd1, ST_TABRX=4'd2, ST_GAP=4'd3,
               ST_SEG=4'd4, ST_REQ=4'd5, ST_RX=4'd6, ST_ABORT=4'd7,
               ST_SEGF=4'd8, ST_SEGRX=4'd9;
    reg [3:0]  st = ST_IDLE;
    reg [4:0]  gap = 5'd0;
    reg        cmd_pend = 0, pend_play = 0;
    reg [6:0]  pend_idx = 0;
    reg [1:0]  pend_seq = 0;
    reg [2:0]  tab_cnt = 0;            // word index within the 16-byte record
    reg        seg_done = 0;           // envelope entries fetched (bank v5)
    reg        seg_loop = 0;           // 0 = first pass, 1 = loop region
    reg        req_out = 0;            // burst request outstanding in arbiter
    reg [16:0] rem_w = 0;              // words left to fetch in this segment
    reg [22:0] curAddr = 0;
    reg [FW:0] burst_w = 0;

    // hClk ack sync + flush/run control
    reg [1:0] ack_h1 = 0, ack_h = 0;
    reg [7:0] ackp_h1 = 0, ackp_h = 0;
    reg [2:0] settle_h = 3'd0;
    reg [1:0] ack_last = 0;
    reg        flush_active = 0;
    reg        run_state = 0;
    reg        need_init = 0;
    reg [10:0] flush_rem = 0;          // bytes left to discard

    // sampled record (hClk), latched on the play-echo edge
    reg [15:0] h_tick = 0, h_blocks = 0, h_flags = 0;
    reg [15:0] h_count = 0, h_brr_off = 0, h_loop_off = 0;
    reg [9:0]  h_seg_cnt = 0;            // envelope entries this effect (bank v5)

    // hClk FIFO word pump
    reg [15:0] wbuf = 0;               // current word from FIFO
    reg        wbuf_v = 0;
    reg        rd_pend = 0;            // FIFO read in flight
    reg [15:0] fword = 0;

    // byte pump (reassembles bytes from FIFO words, drops segment pad bytes)
    reg        bphase = 0;             // 0 = low byte of wbuf, 1 = high byte
    reg [1:0]  skip_cnt = 0;           // pad bytes still to discard
    reg [15:0] seg_rem = 0;            // real bytes left in current segment
    reg        in_loop_seg = 0;        // current segment is the loop region
    reg [7:0]  sb = 0;                 // staged real byte
    reg        sb_v = 0;

    // BRR decoder
    localparam D_HDR = 1'b0, D_DATA = 1'b1;
    reg        dstate = D_HDR;
    reg [3:0]  brr_shift = 0;
    reg [1:0]  brr_filt = 0;
    reg [7:0]  dbyte = 0;              // current data byte (two nibbles)
    reg        have_dbyte = 0;
    reg [3:0]  nib_i = 0;              // sample index within block (0..15)
    reg signed [15:0] prev1 = 0, prev2 = 0;   // predictor history (x2 scale)
    reg [15:0] blk_rem = 0;            // blocks left to emit (one-shot)

    // per-effect sample tick
    reg [15:0] tick_cnt = 0;

    // pitch-envelope sequencer (bank v5): steps through the effect's entries
    // in smem, one entry per segment, on decoded-block boundaries.
    localparam SR_IDLE=3'd0, SR_B0=3'd1, SR_B1=3'd2, SR_B2=3'd3, SR_RUN=3'd4;
    reg [2:0]   seg_st = SR_IDLE;
    reg [15:0]  seg_blk = 0;             // 16-sample blocks left in this segment
    reg [15:0]  seg_ntick = 0;           // tick reload of the NEXT segment
    reg [9:0]   seg_left = 0;            // entries still to fetch after this one
    reg         seg_on = 0;              // envelope armed for the current effect

    // =====================================================================
    // hClk command encoding: gray sequence + {play,idx} payload
    // =====================================================================
    wire [1:0] gray_inc = {cmd_seq[0], ~cmd_seq[0]};  // +1 on a 2-bit gray
    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            cmd_seq <= 2'd0; cmd_pl <= 8'd0; hStart_q <= 0; hStop_q <= 0;
        end else begin
            hStart_q <= hStart;
            hStop_q  <= hStop;
            if (hStart && !hStart_q) begin
                cmd_seq <= cmd_seq ^ gray_inc;
                cmd_pl  <= {1'b1, hEffectIndex};
            end else if (hStop && !hStop_q) begin
                cmd_seq <= cmd_seq ^ gray_inc;
                cmd_pl  <= 8'd0;
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
    wire cmd_play    = pl_x2[7];
    wire [6:0] cmd_idx = pl_x2[6:0];

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

    // segment store write side (bank v5): fills smem during ST_SEGRX
    wire sWrEn = (st == ST_SEGRX) && xRamDoutValid;
    always @(posedge xClk) if (sWrEn) smem[swptr] <= xRamDout;

    // sync rgray into xClk -> binary reader pointer (stale = conservative)
    always @(posedge xClk) begin rgray_x1 <= rgray; rgray_x <= rgray_x1; end
    wire [FW:0] rbin_x = rgray_x ^ (rgray_x >> 1) ^ (rgray_x >> 2)
                               ^ (rgray_x >> 3) ^ (rgray_x >> 4)
                               ^ (rgray_x >> 5) ^ (rgray_x >> 6)
                               ^ (rgray_x >> 7) ^ (rgray_x >> 8) ^ (rgray_x >> 9);
    // free words (see previous revision for the derivation); the stale synced
    // reader pointer only ever understates free space.
    wire [FW:0] free_w = rbin_x - wbin;
    wire [FW:0] space  = (free_w == {(FW+1){1'b0}}) ? {1'b1,{FW{1'b0}}} :
                         (free_w >  {1'b1,{FW{1'b0}}}) ? (free_w - {1'b1,{FW{1'b0}}}) :
                         {(FW+1){1'b0}};

    // ---- segment parameters (combinational from the latched record) ----
    // first-pass byte length = blocks * 9 (max 699*9 = 6291); loop-region
    // length = (blocks*9) - (loop_off - brr_off). Both are multiples of 9.
    wire [15:0] blocks_x9 = rec_blocks * 16'd9;
    wire [15:0] loop_span = rec_loop_off - rec_brr_off;
    wire [15:0] seg_len   = seg_loop ? (blocks_x9 - loop_span) : blocks_x9;
    wire [15:0] seg_off   = seg_loop ? rec_loop_off : rec_brr_off;
    wire [22:0] seg_base  = BANK_BASE + BRR_OFF + {7'd0, seg_off};
    // words to fetch = ceil((len + base[0]) / 2); base[0] is the leading pad
    // byte the writer includes by rounding the base down to even.
    wire [16:0] seg_remw  = ({1'b0, seg_len} + {16'd0, seg_base[0]} + 18'd1) >> 1;
    // the sample loops when flags.bit0 is set (loop region valid)
    wire        has_loop  = rec_flags[0];

    // table fetch address: BANK_BASE + 0x100 + idx*16 (16-byte aligned)
    wire [22:0] tab_addr  = BANK_BASE + TABLE_OFF + {12'd0, pend_idx, 4'd0};

    // =====================================================================
    // xClk: streaming FSM
    // =====================================================================
    always @(posedge xClk or posedge xReset) begin
        if (xReset) begin
            st <= ST_IDLE; xRamReq <= 0; req_out <= 0; cmd_pend <= 0;
            gap <= 0; rem_w <= 0; curAddr <= 0; seg_loop <= 0;
            ack_seq <= 0; ack_pl <= 0; pend_play <= 0; pend_idx <= 0;
            pend_seq <= 0; burst_w <= 0; tab_cnt <= 0; seg_done <= 0;
            rec_brr_off <= 0; rec_loop_off <= 0; rec_tick <= 0;
            rec_blocks <= 0; rec_flags <= 0; rec_count <= 0;
            rec_seg_off <= 0; rec_seg_cnt <= 0; swptr <= 0;
        end else begin
            xRamReq <= 1'b0;

            case (st)
            ST_IDLE: begin
                if (cmd_pend && xRamReady) begin
                    cmd_pend <= 1'b0;                 // pend consumed; a same-cycle
                                                      // cmd_new latch below re-sets it
                    if (pend_play) st <= ST_TAB;      // fetch the record first
                    else begin st <= ST_GAP; gap <= 5'd16; end
                end
            end
            ST_TAB: begin
                // 16-byte read burst on the effect's table record
                xRamAddr     <= tab_addr;
                xRamBurstLen <= 11'd16;
                xRamReq      <= 1'b1;
                req_out      <= 1'b1;
                tab_cnt      <= 3'd0;
                seg_done     <= 1'b0;
                st           <= ST_TABRX;
            end
            ST_TABRX: begin
                // latch the record words (bank v5: 8 x u16, all significant)
                if (xRamDoutValid) begin
                    case (tab_cnt)
                        3'd0: rec_brr_off  <= xRamDout;
                        3'd1: rec_loop_off <= xRamDout;
                        3'd2: rec_tick     <= xRamDout;
                        3'd3: rec_blocks   <= xRamDout;
                        3'd4: rec_flags    <= xRamDout;
                        3'd5: rec_count    <= xRamDout;
                        3'd6: rec_seg_off  <= xRamDout;
                        3'd7: rec_seg_cnt  <= xRamDout;
                        default: ;
                    endcase
                    tab_cnt <= tab_cnt + 3'd1;
                end
                if (cmd_pend) begin                      // superseded mid-fetch
                    st <= ST_ABORT;
                    if (xRamDone) req_out <= 1'b0;       // done landed on the same
                                                         // cycle: don't lose the pulse
                end
                else if (xRamDone) begin req_out <= 0; st <= ST_GAP; gap <= 5'd16; end
            end
            ST_SEGF: begin
                // single read burst on the effect's segment-table entries
                xRamAddr     <= BANK_BASE + SEG_OFF + {5'd0, rec_seg_off, 2'd0};
                xRamBurstLen <= {rec_seg_cnt[8:0], 2'd0};  // entries*4 B, <= 2044
                xRamReq      <= 1'b1;
                req_out      <= 1'b1;
                swptr        <= 0;
                st           <= ST_SEGRX;
            end
            ST_SEGRX: begin
                if (sWrEn) swptr <= swptr + 1'b1;
                if (cmd_pend) begin
                    st <= ST_ABORT;
                    if (xRamDone) req_out <= 1'b0;
                end
                else if (xRamDone) begin
                    req_out  <= 0;
                    seg_done <= 1'b1;
                    st <= ST_GAP; gap <= 5'd16;      // settle, then echo
                end
            end
            ST_GAP: begin
                // let the hClk write-gray synchronizers catch the last old
                // word before the echo tells the reader to flush/start
                if (gap == 5'd0) begin
                    if (cmd_pend) begin
                        // a newer command arrived before we echoed: re-fetch
                        // its record instead of streaming the stale one
                        cmd_pend <= 1'b0;   // consumed; the cmd_new latch below
                                            // re-sets it for a still-newer cmd
                        st <= ST_TAB;
                    end else if (pend_play && !seg_done &&
                                 rec_seg_cnt != 16'd0 && rec_blocks != 16'd0) begin
                        // bank v5: fetch the envelope entries before the echo so
                        // the segment store is settled when the reader starts
                        st <= ST_SEGF;
                    end else begin
                        cmd_pend <= 1'b0;
                        ack_seq  <= pend_seq;
                        ack_pl   <= {pend_play, pend_idx};
                        if (pend_play && rec_blocks != 16'd0) begin
                            seg_loop <= 1'b0;
                            st <= ST_SEG;
                        end else st <= ST_IDLE;
                    end
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
                    if (!seg_loop && has_loop) begin
                        seg_loop <= 1'b1;               // start the loop region
                        st <= ST_SEG;
                    end else if (seg_loop) begin
                        st <= ST_SEG;                   // repeat loop region; for
                                                        // count-limited one-shots the
                                                        // reader stops and the FIFO
                                                        // fills, so `space` throttles us
                    end else st <= ST_IDLE;             // natural one-shot end
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
                if (cmd_pend) begin
                    st <= ST_ABORT;
                    if (xRamDone) req_out <= 1'b0;       // done landed on the same
                                                         // cycle: don't lose the pulse
                end
                else if (xRamDone) begin req_out <= 0; st <= ST_REQ; end
            end
            ST_ABORT: begin
                // drain an outstanding burst (data discarded) before continuing
                if (req_out) begin
                    if (xRamDone) req_out <= 0;
                end else begin
                    cmd_pend <= 1'b0;                 // pend consumed by this
                                                      // handoff; the cmd_new latch
                                                      // below re-sets it for a
                                                      // still-newer cmd
                    if (pend_play) st <= ST_TAB;        // fetch the new record
                    else begin st <= ST_GAP; gap <= 5'd16; end
                end
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
    // hClk: record-derived descriptors (valid once h_* are latched)
    // =====================================================================
    wire [15:0] seg0_len_c   = h_blocks * 16'd9;
    wire [15:0] segL_len_c   = seg0_len_c - (h_loop_off - h_brr_off);
    wire [15:0] total_blk_c  = h_flags[0] ? h_count : h_blocks;
    wire        lead0_c      = h_brr_off[0];
    wire        leadL_c      = h_loop_off[0];
    wire        trail0_c     = seg0_len_c[0] ^ lead0_c;   // (len+lead)&1
    wire        trailL_c     = segL_len_c[0] ^ leadL_c;

    // =====================================================================
    // hClk: FIFO word pump control
    // =====================================================================
    wire        words_avail = (wbin_h != rbin);
    wire        sample_tick = (tick_cnt == 16'd0);
    // new-command echo edge this cycle: the flush snapshot reads rbin here,
    // so the pump must NOT advance it on the same edge.
    wire        ack_event = (settle_h == 3'd7) && (ack_h != ack_last);

    // byte pump steps one byte when it is active, a word is staged, no byte
    // is waiting on the decoder, and there is still something to consume
    wire        bp_active = run_state && !flush_active;
    wire        bp_step   = bp_active && wbuf_v && !sb_v &&
                            ((skip_cnt != 2'd0) || (seg_rem != 16'd0));
    wire        word_done = bp_step && bphase;        // took the high byte
    wire        fdrop     = flush_active && wbuf_v;   // flush discard (1 word/cyc)
    wire        consume   = !ack_event && (fdrop || word_done);
    wire [FW:0] rbin_next  = rbin + (consume ? {{FW{1'b0}},1'b1} : {(FW+1){1'b0}});
    wire [FW:0] rgray_next = (rbin_next >> 1) ^ rbin_next;

    // current byte from the staged word (low byte first, matching the
    // little-endian PSRAM word layout {byte[addr+1], byte[addr]})
    wire [7:0] cur_byte = bphase ? wbuf[15:8] : wbuf[7:0];

    // BSRAM read: fword is fmem[rbin] registered (1-cycle latency).
    always @(posedge hClk) fword <= fmem[rbin[FW-1:0]];

    // segment store read: sword is smem[sraddr] registered (1-cycle latency).
    // The envelope sequencer (SR_B0/B1/B2) walks this pipeline; entries were
    // written by the writer before the echo, so no read/write race exists for
    // the running effect (a retrigger refills smem just before ITS echo).
    always @(posedge hClk) sword <= smem[sraddr];

    // decoder needs a byte for the header, or for a data byte when the
    // current one is exhausted (nib_i even and no byte staged)
    wire dec_needs_byte = (dstate == D_HDR) ||
                          (dstate == D_DATA && !have_dbyte);
    wire dec_take = dec_needs_byte && sb_v;

    // =====================================================================
    // BRR decode datapath (combinational; copied from sgb_snd.v with the
    // ares/build_bank base and x2 feedback -- see the header)
    // =====================================================================
    wire [3:0] nib_raw = nib_i[0] ? dbyte[3:0] : dbyte[7:4];  // HIGH first
    wire signed [19:0] snib20 = {{16{nib_raw[3]}}, nib_raw};
    // base: shift<=12 -> (nib<<shift)>>1; shift>12 -> nib<0 ? -2048 : 0
    wire signed [19:0] base20 = (brr_shift <= 4'd12)
                              ? ((snib20 <<< brr_shift) >>> 1)
                              : (nib_raw[3] ? -20'sd2048 : 20'sd0);
    wire signed [19:0] p1    = {{4{prev1[15]}}, prev1};
    // NOTE: $signed is load-bearing (see sgb_snd.v): a bare concatenation is
    // unsigned, so `>>> 1` would shift logically and corrupt p2h for negative
    // prev2 (audible crackle).
    wire signed [19:0] p2h   = $signed({{4{prev2[15]}}, prev2}) >>> 1;
    wire signed [19:0] t3p1  = (p1 <<< 1) + p1;                 // 3*p1
    wire signed [19:0] t13p1 = (p1 <<< 3) + (p1 <<< 2) + p1;    // 13*p1
    wire signed [19:0] t3p2h = (p2h <<< 1) + p2h;               // 3*(p2>>1)
    reg  signed [19:0] fcon;
    always @(*) begin
        case (brr_filt)
            2'd1:    fcon = (p1 >>> 1) + ((-p1) >>> 5);
            2'd2:    fcon = p1 - p2h + (p2h >>> 4) + ((-t3p1) >>> 6);
            2'd3:    fcon = p1 - p2h + (t3p2h >>> 4) + ((-t13p1) >>> 7);
            default: fcon = 20'sd0;   // filter 0
        endcase
    end
    wire signed [19:0] acc = base20 + fcon;
    wire [15:0] t16 = (acc >  20'sd32767) ? 16'h7FFF :
                      (acc < -20'sd32768) ? 16'h8000 : acc[15:0];
    // final x2 with 16-bit wrap = the emitted sample AND the feedback history
    wire signed [15:0] dec_out = {t16[14:0], 1'b0};

    // last block of a one-shot (infinite effects never set this)
    wire last_block = !h_flags[1] && (blk_rem == 16'd1);

    // envelope segment boundary: the sample being emitted this cycle is the
    // last one of the current segment's final block. The tick reload for the
    // NEXT sample must come from seg_ntick (h_tick updates on the same edge,
    // one nonblocking step too late for the reload below).
    wire seg_boundary = seg_on && (seg_st == SR_RUN) && sample_tick &&
                        run_state && !flush_active &&
                        (dstate == D_DATA) && have_dbyte &&
                        (nib_i == 4'd15) && (seg_blk == 16'd1);

    // =====================================================================
    // hClk: reader (word pump + byte pump + decoder + flush/run + tick)
    // =====================================================================
    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            rbin <= 0; rgray <= 0; rd_pend <= 0; wbuf_v <= 0; wbuf <= 0;
            flush_active <= 0; run_state <= 0; need_init <= 0; flush_rem <= 0;
            ack_last <= 0; hPlaying <= 0; hPcm <= 0; hPcmValid <= 0;
            h_tick <= 0; h_blocks <= 0; h_flags <= 0;
            h_count <= 0; h_brr_off <= 0; h_loop_off <= 0; h_seg_cnt <= 0;
            bphase <= 0; skip_cnt <= 0; seg_rem <= 0; in_loop_seg <= 0;
            sb <= 0; sb_v <= 0;
            dstate <= D_HDR; brr_shift <= 0; brr_filt <= 0; dbyte <= 0;
            have_dbyte <= 0; nib_i <= 0; prev1 <= 0; prev2 <= 0; blk_rem <= 0;
            tick_cnt <= 0;
            seg_st <= SR_IDLE; seg_blk <= 0; seg_ntick <= 0; seg_left <= 0;
            seg_on <= 0; sraddr <= 0;
        end else begin
            hPcmValid <= 1'b0;

            // ---- FIFO read pointer / staged-word control ----
            rbin  <= rbin_next;
            rgray <= rgray_next;
            if (consume) wbuf_v <= 1'b0;
            if (rd_pend) begin
                wbuf    <= fword;
                wbuf_v  <= 1'b1;
                rd_pend <= 1'b0;
            end else if (!wbuf_v && !rd_pend && words_avail &&
                         (run_state || flush_active)) begin
                rd_pend <= 1'b1;
            end

            // ---- record sampling + start/stop on the echo edge ----
            if (ack_event) begin
                ack_last <= ack_h;
                if (ackp_h[7] && rec_blocks != 16'd0) begin
                    // play: sample the (stable) record, then flush + start
                    h_tick    <= rec_tick;
                    h_blocks  <= rec_blocks;
                    h_flags   <= rec_flags;
                    h_count   <= rec_count;
                    h_brr_off <= rec_brr_off;
                    h_loop_off<= rec_loop_off;
                    h_seg_cnt <= rec_seg_cnt[9:0];
                    hPlaying  <= 1'b1;
                    need_init <= 1'b1;
                    run_state <= 1'b0;
                    if (wbin_h == rbin) flush_active <= 1'b0;
                    else begin
                        flush_active <= 1'b1;
                        flush_rem    <= {wbin_h, 1'b0} - {rbin, 1'b0};
                    end
                end else begin
                    // stop echo (or invalid play): idempotent halt
                    hPlaying <= 0; run_state <= 0; flush_active <= 0;
                    need_init <= 0;
                end
            end

            // ---- immediate local stop ----
            if (hStop && !hStop_q) begin
                hPlaying <= 0; run_state <= 0; flush_active <= 0;
                need_init <= 0;
            end

            // ---- flush discard: one word per cycle, 2 bytes each ----
            if (fdrop && !ack_event) begin
                if (flush_rem <= 11'd2) flush_active <= 1'b0;
                else flush_rem <= flush_rem - 11'd2;
            end

            // ---- start decoding once flushed (record already latched) ----
            if (need_init && !flush_active) begin
                need_init   <= 1'b0;
                run_state   <= 1'b1;
                prev1       <= 16'sd0; prev2 <= 16'sd0;   // zero history
                nib_i       <= 4'd0; have_dbyte <= 0; dstate <= D_HDR;
                bphase      <= 1'b0; sb_v <= 0;
                skip_cnt    <= {1'b0, lead0_c};
                seg_rem     <= seg0_len_c;
                in_loop_seg <= 1'b0;
                blk_rem     <= total_blk_c;
                // arm the pitch envelope (bank v5): entry 0 at smem[0] (the
                // writer filled this effect's entries back from word 0)
                if (h_seg_cnt != 10'd0) begin
                    seg_on   <= 1'b1;
                    seg_st   <= SR_B0;
                    sraddr   <= {SW{1'b0}};
                    seg_left <= h_seg_cnt - 10'd1;
                end else begin
                    seg_on <= 1'b0;
                    seg_st <= SR_IDLE;
                end
            end

            // ---- byte pump ----
            if (bp_step) begin
                bphase <= ~bphase;
                if (skip_cnt != 2'd0) begin
                    skip_cnt <= skip_cnt - 2'd1;            // discard pad byte
                end else begin
                    sb   <= cur_byte; sb_v <= 1'b1;          // stage a real byte
                    if (seg_rem == 16'd1) begin
                        // last real byte of the segment: set up the next one
                        if (!in_loop_seg) begin
                            if (h_flags[0]) begin
                                skip_cnt    <= {1'b0, trail0_c} + {1'b0, leadL_c};
                                seg_rem     <= segL_len_c;
                                in_loop_seg <= 1'b1;
                            end else begin
                                skip_cnt <= 2'd0; seg_rem <= 16'd0;   // done
                            end
                        end else begin
                            skip_cnt <= {1'b0, trailL_c} + {1'b0, leadL_c};
                            seg_rem  <= segL_len_c;                  // stay in loop
                        end
                    end else seg_rem <= seg_rem - 16'd1;
                end
            end
            // decoder consumes the staged byte
            if (dec_take) sb_v <= 1'b0;

            // ---- BRR decoder FSM ----
            if (run_state && !flush_active) begin
                case (dstate)
                D_HDR: begin
                    if (dec_take) begin
                        brr_shift  <= sb[7:4];
                        brr_filt   <= sb[3:2];
                        nib_i      <= 4'd0;
                        have_dbyte <= 1'b0;
                        dstate     <= D_DATA;
                    end
                end
                D_DATA: begin
                    if (!have_dbyte) begin
                        if (dec_take) begin
                            dbyte      <= sb;
                            have_dbyte <= 1'b1;
                        end
                    end else if (sample_tick) begin
                        // emit the sample for the current nibble
                        hPcm      <= dec_out;
                        hPcmValid <= 1'b1;
                        prev2     <= prev1;
                        prev1     <= dec_out;
                        if (nib_i == 4'd15) begin
                            // block complete
                            if (last_block) begin
                                run_state <= 1'b0;
                                hPlaying  <= 1'b0;
                                dstate    <= D_HDR;
                            end else begin
                                if (!h_flags[1]) blk_rem <= blk_rem - 16'd1;
                                // pitch envelope: count down the segment in
                                // 16-sample blocks; on its last block apply
                                // the next segment's tick
                                if (seg_on && seg_st == SR_RUN) begin
                                    if (seg_blk == 16'd1) begin
                                        h_tick <= seg_ntick;
                                        if (seg_left == 10'd0) begin
                                            seg_on <= 1'b0;      // final segment:
                                            seg_st <= SR_IDLE;   // hold this tick
                                        end else begin
                                            seg_left <= seg_left - 10'd1;
                                            seg_st   <= SR_B0;   // fetch next entry
                                        end
                                    end else seg_blk <= seg_blk - 16'd1;
                                end
                                nib_i      <= 4'd0;
                                have_dbyte <= 1'b0;
                                dstate     <= D_HDR;
                            end
                        end else begin
                            nib_i <= nib_i + 4'd1;
                            if (nib_i[0]) have_dbyte <= 1'b0; // low nibble used
                        end
                    end
                end
                endcase
            end

            // ---- pitch-envelope entry fetch (bank v5) ----
            // sword lags sraddr by one cycle; SR_B0/B1/B2 walk the pipeline.
            // sraddr already points at the next entry's first word when a
            // boundary re-enters SR_B0 (it was advanced through SR_B1).
            if (run_state && !flush_active) begin
                case (seg_st)
                SR_B0: begin                  // blocks16 word read in flight
                    sraddr <= sraddr + 1'b1;  // -> the entry's tick word
                    seg_st <= SR_B1;
                end
                SR_B1: begin                  // blocks16 valid on sword now
                    seg_blk <= sword;
                    sraddr  <= sraddr + 1'b1; // -> next entry's blocks16 word
                    seg_st  <= SR_B2;
                end
                SR_B2: begin                  // tick word valid on sword now
                    seg_ntick <= sword;
                    seg_st    <= SR_RUN;
                end
                default: ;
                endcase
            end

            // ---- per-effect sample tick ----
            if (need_init && !flush_active) tick_cnt <= h_tick;
            else if (run_state && !flush_active) begin
                if (sample_tick) tick_cnt <= seg_boundary ? seg_ntick : h_tick;
                else             tick_cnt <= tick_cnt - 16'd1;
            end
        end
    end

endmodule
