// ---------------------------------------------------------------------------
// sgb_music.v -- SGB N-SPC music HLE micro-sequencer ("MSGB" tracker player)
// ---------------------------------------------------------------------------
// Plays a song bank produced by sgb_music/sgb_music_compiler.py, which
// offline-translates Nintendo N-SPC (Kankichi-kun) sequence bytecode +
// BRR samples into a compact tracker format. This player replaces the
// SPC700 + S-DSP entirely: no CPU core, no Gaussian interpolation, no
// echo, no ADSR -- just a byte sequencer, 4 BRR voices with zero-order
// hold resampling, and an additive stereo mixer. Gate-budget discipline
// matches sgb_snd.v / sgb_sfx_play.v (design routes at ~97% CLS):
//
//   * single shared BRR predictor (sgb_snd's bit-exact f0..f3 filter set,
//     shift-add only), time-multiplexed across the 4 voices
//   * pitch scaling via ONE shared 16-cycle serial shift-add multiplier
//     used only at note-on (no DSP blocks, ever)
//   * velocity/channel volume quantized to 4-bit gains (the N-SPC velocity
//     table has 16 entries anyway), applied with a shift-add tree
//   * one shared PSRAM fetch FSM services song-table / sample-directory /
//     BRR-block / stream-refill traffic round-robin on the arbiter port
//   * sequencer timing is fully resolved to a fixed tick grid offline, so
//     the hardware needs no tempo arithmetic at all (the TEMPO opcode is
//     consumed and ignored)
//
// CLOCKS
//   xClk -- PSRAM arbiter side (67.108864 MHz = 4 x hClk). Everything
//           (sequencer, voices, decoder, mixer, FIFO writer) runs here.
//           Output rate: xClk/(SAMPLE_DIV+1) ~ 15.99 kHz stereo, one
//           sequencer tick per 16 output samples (~999 Hz; the compiler
//           assumes 1000 Hz, 0.06% tempo error).
//   hClk -- audio side (16.777216 MHz). Reads the stereo FIFO and emits
//           one L/R pair per ~16 kHz tick on hPcmL/hPcmR/hPcmValid.
//
// BANK FORMAT (self-describing; the RTL needs no per-bank parameters)
//   0x000  header: 'MSGB', u16 ver, u16 songs, u32 table_off, u32
//          sample_off, u32 data_off
//   0x100  song table, 16 B/entry: u32 stream_off, u32 stream_len,
//          u32 brr_off (all relative to data_off), u32 reserved
//   0x200  sample directory, 64 x 8 B: u32 brr_off (relative to the BRR
//          region), u32 loop_off (relative to sample start) | 0x80000000
//          if the sample loops. 0xFFFFFFFF brr_off = unused slot.
//   0x800  data: tracker stream, then the BRR region (block chains)
//
// TRACKER OPCODES
//   0x00..0x7E WAIT op+1 ticks          0x7F END (stop)
//   0x80|v NOTE_ON + note, gate u16le, vel, srcn, mult u16le (8.8)
//   0x90|v INSTR + 1 byte (no-op: NOTE_ON is self-contained)
//   0xA0|v PANVOL + vol_l, vol_r        0xF0 LOOP + u24 stream offset
//   0xF1 TEMPO + u16 (ignored)          0xFF STOP
//
// CONTROL (same proven CDC as sgb_sfx_play.v)
//   hStart pulse -> play hSong (interrupts a playing song). Triggered by
//   gb.v on a SGB SOUND packet whose Z byte selects a song. hStop pulse ->
//   stop (SOUND bit-7 / Z=3 semantics). Commands cross to xClk on a gray
//   sequence + payload; the writer echoes the processed sequence back and
//   the reader flushes exactly the FIFO residue snapshotted at that echo.
//   Natural song end (STOP/END opcode) crosses back on a second gray
//   sequence (end_seq) so hPlaying deasserts in the hClk domain.
// ---------------------------------------------------------------------------

module sgb_music #(
    parameter [22:0] BANK_BASE  = 23'h180000,
    // xClk cycles per output sample, minus one. 4196 -> 67.108864 MHz /
    // 4197 ~ 15989.7 Hz: deliberately a hair SLOWER than the hClk reader
    // tick (16.777216/1049 ~ 15993.5 Hz) so the FIFO drains toward empty
    // instead of overflowing; the reader holds its last output on underrun.
    parameter [15:0] SAMPLE_DIV = 16'd4196,
    // hClk reader sample-tick divider (cycles per sample minus one); same
    // value and rationale as sgb_sfx_play.
    parameter [11:0] TICK_PERIOD = 12'd1048
)(
    // PSRAM / arbiter side (xClk)
    input             xClk,
    input             xReset,
    input             xRamReady,       // BIST finished; no traffic before
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
    input             hStart,          // pulse: begin playing hSong
    input             hStop,           // pulse: stop playback
    input      [2:0]  hSong,           // song table index

    // PCM output (hClk)
    output reg        hPcmValid = 1'b0,
    output reg signed [15:0] hPcmL = 16'sd0,
    output reg signed [15:0] hPcmR = 16'sd0,
    output reg        hPlaying = 1'b0
);

    localparam FW = 9;                          // output FIFO: 512 x 16
    localparam [22:0] TABLE_OFF  = 23'h100;     // fixed by the bank format
    localparam [22:0] SAMPLE_OFF = 23'h200;
    localparam [22:0] DATA_OFF   = 23'h800;
    localparam [7:0]  REFILL_LO  = 8'd40;       // stream refill threshold

    assign xRamDin = 16'd0;
    assign xRamRnW = 1'b1;

    // =====================================================================
    // hClk command encoding: gray sequence + {start, song} payload
    // =====================================================================
    reg [1:0] cmd_seq = 2'd0;
    reg [3:0] cmd_pl  = 4'd0;
    reg hStart_q = 1'b0, hStop_q = 1'b0;

    wire [1:0] gray_inc = {cmd_seq[0], ~cmd_seq[0]};  // +1 on a 2-bit gray
    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            cmd_seq <= 2'd0; cmd_pl <= 4'd0; hStart_q <= 0; hStop_q <= 0;
        end else begin
            hStart_q <= hStart;
            hStop_q  <= hStop;
            if (hStart && !hStart_q) begin
                cmd_seq <= cmd_seq ^ gray_inc;
                cmd_pl  <= {1'b1, hSong};
            end else if (hStop && !hStop_q) begin
                cmd_seq <= cmd_seq ^ gray_inc;
                cmd_pl  <= 4'd0;
            end
        end
    end

    // xClk: command sync
    reg [1:0] seq_x1 = 0, seq_x2 = 0, seq_last = 0;
    reg [3:0] pl_x1 = 0, pl_x2 = 0;
    reg [2:0] settle_x = 3'd0;
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
    wire [2:0] cmd_song = pl_x2[2:0];

    // writer echo of processed commands
    reg [1:0] ack_seq = 2'd0;
    reg [3:0] ack_pl  = 4'd0;

    // natural-end status back to hClk (gray toggle)
    reg [1:0] end_seq = 2'd0;

    // =====================================================================
    // stream byte buffer: 256 x 16 BSRAM ring = 512 stream bytes
    // =====================================================================
    reg [15:0] sbmem [0:255];
    reg [7:0]  sb_wpt = 8'd0;            // write word pointer
    reg [15:0] sb_rword = 16'd0;         // registered read word

    // =====================================================================
    // output FIFO storage (BSRAM simple dual port: xClk write, hClk read)
    // =====================================================================
    reg  [15:0] fmem [0:(1<<FW)-1];
    reg  [FW:0] wbin = 0, wgray = 0;
    reg  [FW:0] rgray_x1 = 0, rgray_x = 0;
    reg  [FW:0] wgray_h1 = 0, wgray_h = 0;
    reg  [FW:0] rbin = 0, rgray = 0;

    reg         fifo_wr = 1'b0;
    reg  [15:0] fifo_wd = 16'd0;
    wire [FW:0] wbin_next  = wbin + (fifo_wr ? {{FW{1'b0}},1'b1} : {(FW+1){1'b0}});
    wire [FW:0] wgray_next = (wbin_next >> 1) ^ wbin_next;
    always @(posedge xClk) begin
        if (fifo_wr) fmem[wbin[FW-1:0]] <= fifo_wd;
        wbin  <= wbin_next;
        wgray <= wgray_next;
    end
    always @(posedge xClk) begin rgray_x1 <= rgray; rgray_x <= rgray_x1; end
    wire [FW:0] rbin_x = rgray_x ^ (rgray_x >> 1) ^ (rgray_x >> 2)
                               ^ (rgray_x >> 3) ^ (rgray_x >> 4)
                               ^ (rgray_x >> 5) ^ (rgray_x >> 6)
                               ^ (rgray_x >> 7) ^ (rgray_x >> 8) ^ (rgray_x >> 9);
    // free words (stale synced reader pointer only understates free space)
    wire [FW:0] free_w  = rbin_x - wbin;
    wire [FW:0] fifo_free = (free_w == {(FW+1){1'b0}}) ? {1'b1,{FW{1'b0}}} :
                            (free_w > {1'b1,{FW{1'b0}}}) ?
                                (free_w - {1'b1,{FW{1'b0}}}) : {(FW+1){1'b0}};

    // =====================================================================
    // player state
    // =====================================================================
    reg        running  = 1'b0;
    reg        was_playing = 1'b0;      // a song actually ran (for end_seq)
    reg [22:0] stream_base = 23'd0;     // PSRAM addr of stream byte 0
    reg [22:0] brr_base = 23'd0;        // PSRAM addr of the BRR region
    reg [23:0] stream_len = 24'd0;
    reg [23:0] spos = 24'd0;            // next unconsumed stream byte
    reg [9:0]  fill = 10'd0;            // buffered bytes from spos
    reg [15:0] wait_cnt = 16'd0;        // WAIT countdown (ticks)
    reg        end_pend = 1'b0;         // natural stop -> end_seq toggle
    reg        mreq_tbl = 1'b0;         // master wants the song-table fetch
    reg [2:0]  song_idx = 3'd0;

    // sequencer
    localparam [2:0] SQ_OP = 3'd0, SQ_OPH = 3'd1, SQ_RD = 3'd2,
                     SQ_ARG = 3'd3, SQ_ARGH = 3'd4, SQ_MULT = 3'd5,
                     SQ_EXEC = 3'd6;
    reg [2:0]  sq_st = SQ_OP;
    reg [7:0]  sq_op = 8'd0;
    reg [2:0]  sq_rem = 3'd0;           // arg bytes still to read
    reg [55:0] sq_argbuf = 56'd0;       // args shift in, first arg at top
    reg [4:0]  sq_mult_wait = 5'd0;
    wire [7:0] sb_byte = spos[0] ? sb_rword[15:8] : sb_rword[7:0];

    // serial shift-add multiplier (note-on pitch only)
    reg        m_busy = 1'b0;
    reg [4:0]  m_cnt = 5'd0;
    reg [28:0] m_aa = 29'd0, m_acc = 29'd0;
    reg [15:0] m_bb = 16'd0;
    // latched note-on context while the multiplier runs
    reg [6:0]  m_note = 7'd0;
    reg [1:0]  m_voice = 2'd0;

    // fetch FSM
    localparam [1:0] JT_TBL = 2'd0, JT_DIR = 2'd1, JT_BLK = 2'd2, JT_STR = 2'd3;
    localparam [1:0] F_IDLE = 2'd0, F_REQ = 2'd1, F_RX = 2'd2, F_ABORT = 2'd3;
    reg [1:0]  f_st = F_IDLE;
    reg [1:0]  f_type = JT_TBL;
    reg [1:0]  f_voice = 2'd0, f_rr = 2'd0, f_vgen = 2'd0;
    reg [22:0] f_addr = 23'd0;
    reg [5:0]  f_wrem = 6'd0, f_wcnt = 6'd0;
    reg        f_skip = 1'b0;
    reg        req_out = 1'b0;
    reg [15:0] f_wlo = 16'd0;           // u32 assembly: low word latch
    reg [23:0] t_soff = 24'd0, t_slen = 24'd0, t_boff = 24'd0;
    reg [23:0] d_off = 24'd0;
    reg [31:0] d_loop = 32'd0;           // bit31 = loop flag from the bank
    reg [79:0] b_shift = 80'd0;         // BRR block assembly
    reg        cmd_pend = 0, pend_play = 0;
    reg [2:0]  pend_song = 0;
    reg [1:0]  pend_seq = 0;
    reg [4:0]  gap = 5'd0;
    localparam ST_IDLE=1'd0, ST_GAP=1'd1;
    reg        mst = ST_IDLE;

    // =====================================================================
    // voices (4)
    // =====================================================================
    reg [3:0]  v_act = 4'd0;             // voice sounding (gate running)
    reg [3:0]  v_blk_vld = 4'd0;         // current block loaded
    reg [3:0]  v_nblk_vld = 4'd0;        // prefetched block loaded
    reg [3:0]  v_req = 4'd0;             // fetch request pending
    reg [3:0]  v_job = 4'd0;             // 0 = directory, 1 = block
    reg [3:0]  v_looped = 4'd0;
    reg [71:0] v_blk  [0:3];             // current BRR block (hdr + 8 data)
    reg [71:0] v_nblk [0:3];             // prefetched next block
    reg [3:0]  v_nib  [0:3];             // next nibble to decode (0..15)
    reg signed [15:0] v_pcm [0:3];       // last decoded sample (ZoH)
    reg signed [15:0] v_prev1 [0:3], v_prev2 [0:3];
    reg [23:0] v_inc [0:3];              // phase increment (16.16)
    reg [23:0] v_ph [0:3];               // phase accumulator
    reg [15:0] v_gate [0:3];             // remaining gate (ticks)
    reg [3:0]  v_gain_l [0:3], v_gain_r [0:3];
    reg [7:0]  v_vol_l [0:3], v_vol_r [0:3];   // channel pan/vol (PANVOL)
    reg [22:0] v_cur_addr [0:3];         // next block to fetch
    reg [22:0] v_base [0:3];             // sample start (for loop jumps)
    reg [16:0] v_loop [0:3];             // loop offset from sample start
    reg [5:0]  v_srcn [0:3];
    reg [1:0]  v_gen [0:3];              // note-on generation tag

    integer vi;
    initial begin
        for (vi = 0; vi < 4; vi = vi + 1) begin
            v_blk[vi] = 72'd0; v_nblk[vi] = 72'd0; v_nib[vi] = 4'd0;
            v_pcm[vi] = 16'sd0; v_prev1[vi] = 16'sd0; v_prev2[vi] = 16'sd0;
            v_inc[vi] = 24'd0; v_ph[vi] = 24'd0; v_gate[vi] = 16'd0;
            v_gain_l[vi] = 4'd0; v_gain_r[vi] = 4'd0;
            v_vol_l[vi] = 8'h7F; v_vol_r[vi] = 8'h7F;
            v_cur_addr[vi] = 23'd0; v_base[vi] = 23'd0; v_loop[vi] = 17'd0;
            v_srcn[vi] = 6'd0; v_gen[vi] = 2'd0;
        end
    end

    // =====================================================================
    // N-SPC pitch tables
    // =====================================================================
    // 2^(s/12) * 4096, s = semitone
    function [12:0] noterom;
        input [3:0] s;
        begin
            case (s)
                4'd0:    noterom = 13'd4096;
                4'd1:    noterom = 13'd4340;
                4'd2:    noterom = 13'd4598;
                4'd3:    noterom = 13'd4871;
                4'd4:    noterom = 13'd5161;
                4'd5:    noterom = 13'd5468;
                4'd6:    noterom = 13'd5794;
                4'd7:    noterom = 13'd6139;
                4'd8:    noterom = 13'd6505;
                4'd9:    noterom = 13'd6892;
                4'd10:   noterom = 13'd7303;
                default: noterom = 13'd7737;
            endcase
        end
    endfunction

    // N-SPC note numbers are octave*12 + semitone
    function [3:0] note_oct;
        input [6:0] n;
        begin
            if      (n < 7'd12)  note_oct = 4'd0;
            else if (n < 7'd24)  note_oct = 4'd1;
            else if (n < 7'd36)  note_oct = 4'd2;
            else if (n < 7'd48)  note_oct = 4'd3;
            else if (n < 7'd60)  note_oct = 4'd4;
            else if (n < 7'd72)  note_oct = 4'd5;
            else if (n < 7'd84)  note_oct = 4'd6;
            else if (n < 7'd96)  note_oct = 4'd7;
            else if (n < 7'd108) note_oct = 4'd8;
            else if (n < 7'd120) note_oct = 4'd9;
            else                 note_oct = 4'd10;
        end
    endfunction

    // 4-bit gain from velocity and channel volume: (vel>>4 * vol>>4) >> 4.
    // N-SPC's velocity table only has 16 entries, so this keeps the full
    // musical dynamic range while the mixer needs only a shift-add tree.
    function [3:0] gain4;
        input [7:0] vel, vol;
        reg [8:0] p;                  // wide product: 4x4 needs up to 8 bits
        begin
            p = {5'd0, vel[7:4]} * {5'd0, vol[7:4]};
            gain4 = p[8:4];
        end
    endfunction

    // =====================================================================
    // mixer / voice-scan state (drives the shared BRR predictor below)
    // =====================================================================
    reg [15:0] sdiv = 16'd0;             // output sample divider
    wire       smp_strobe = (sdiv == 16'd0);
    localparam [2:0] MI_IDLE = 3'd0, MI_V = 3'd1, MI_GAIN = 3'd2,
                     MI_PUSH_L = 3'd3, MI_PUSH_R = 3'd4;
    reg [2:0]  mix_st = MI_IDLE;
    reg [1:0]  mix_vi = 2'd0;            // voice being scanned
    reg [2:0]  mix_sub = 3'd0;           // decode steps done this voice
    reg signed [23:0] acc_l = 24'sd0, acc_r = 24'sd0;

    // shared BRR predictor inputs: muxed from the scanned voice, so one
    // filter datapath serves all four voices
    wire [1:0] sv = mix_vi;
    // v_blk packing: byte i of the 9-byte BRR block at bits [8*i +: 8]
    // (header = byte 0 at [7:0]), so nibble k lives in byte 1 + k/2.
    wire [6:0] bpos = 7'd8 + {v_nib[sv][3:1], 3'b000};   // data byte offset
    wire [7:0] sv_byte = v_blk[sv][bpos +: 8];
    wire [3:0] sv_nib  = v_nib[sv][0] ? sv_byte[3:0] : sv_byte[7:4];

    wire signed [15:0] sv_snib  = {{12{sv_nib[3]}}, sv_nib};
    wire signed [15:0] sv_shift = sv_snib <<< v_blk[sv][7:4];
    wire signed [19:0] sv_p1  = {{4{v_prev1[sv][15]}}, v_prev1[sv]};
    wire signed [19:0] sv_p2h = $signed({{4{v_prev2[sv][15]}}, v_prev2[sv]}) >>> 1;
    wire signed [19:0] sv_t3p1  = (sv_p1 <<< 1) + sv_p1;                // 3*p1
    wire signed [19:0] sv_t13p1 = (sv_p1 <<< 3) + (sv_p1 <<< 2) + sv_p1;// 13*p1
    wire signed [19:0] sv_t3p2h = (sv_p2h <<< 1) + sv_p2h;              // 3*(p2>>1)
    reg  signed [19:0] sv_fcon;
    always @(*) begin
        case (v_blk[sv][3:2])                       // BRR filter
            2'd1:    sv_fcon = (sv_p1 >>> 1) + ((-sv_p1) >>> 5);
            2'd2:    sv_fcon = sv_p1 - sv_p2h + (sv_p2h >>> 4) + ((-sv_t3p1) >>> 6);
            2'd3:    sv_fcon = sv_p1 - sv_p2h + (sv_t3p2h >>> 4) + ((-sv_t13p1) >>> 7);
            default: sv_fcon = 20'sd0;
        endcase
    end
    wire signed [19:0] sv_acc = {{4{sv_shift[15]}}, sv_shift} + sv_fcon;
    // bit-exact with sgb_snd.v / bsnes; f2 is load-bearing for timbre
    wire signed [15:0] sv_out = (sv_acc >  20'sd32767) ? 16'sh7FFF :
                                (sv_acc < -20'sd32768) ? 16'sh8000 : sv_acc[15:0];

    // 4-bit gain applied by shift-add tree (no multiplier)
    wire signed [19:0] gterm_l =
        (v_gain_l[sv][3] ? (v_pcm[sv] <<< 3) : 20'sd0) +
        (v_gain_l[sv][2] ? (v_pcm[sv] <<< 2) : 20'sd0) +
        (v_gain_l[sv][1] ? (v_pcm[sv] <<< 1) : 20'sd0) +
        (v_gain_l[sv][0] ?  v_pcm[sv]        : 20'sd0);
    wire signed [19:0] gterm_r =
        (v_gain_r[sv][3] ? (v_pcm[sv] <<< 3) : 20'sd0) +
        (v_gain_r[sv][2] ? (v_pcm[sv] <<< 2) : 20'sd0) +
        (v_gain_r[sv][1] ? (v_pcm[sv] <<< 1) : 20'sd0) +
        (v_gain_r[sv][0] ?  v_pcm[sv]        : 20'sd0);

    // =====================================================================
    // note-on pitch increment from the multiplier result (multiplier itself
    // steps inside the main xClk block, single driver, kick wins):
    //   inc = noterom[semi] * mult * 2^(oct-9)   (16.16; note 48 / mult 1.0
    //   gives 32768 = half a BRR sample per output sample = 8 kHz base)
    wire [3:0] m_oct = note_oct(m_note);
    wire [28:0] m_prod = m_acc;
    wire [35:0] inc_sh = (m_oct >= 4'd9) ? {7'd0, m_prod} << (m_oct - 4'd9)
                                         : {7'd0, m_prod} >> (4'd9 - m_oct);
    wire [23:0] m_inc = (inc_sh[35:24] != 12'd0) ? 24'hFFFFFF : inc_sh[23:0];

    // =====================================================================
    // main xClk block: command handling, master gap, fetch FSM, sequencer,
    // mixer, voices
    // =====================================================================
    wire        byte_avail = (fill != 10'd0);
    wire        str_needed = running && (fill < {2'd0, REFILL_LO}) &&
                             ((spos + {14'd0, fill}) < stream_len);
    reg         tick_evt = 1'b0;
    reg [3:0]   smp_cnt = 4'd0;
    // fill bookkeeping has exactly two sources: sequencer byte consumption
    // (-1) and finished stream-refill bursts (+words*2-skip). Both are
    // merged here so no state branch may assign fill on its own (except
    // whole-buffer resets like LOOP/start, which come later in the block).
    wire        fill_dec = running && ((sq_st == SQ_OPH) || (sq_st == SQ_ARGH));
    wire        fill_add = (f_st == F_RX) && (f_type == JT_STR) && xRamDone &&
                           !cmd_pend;
    // last word's valid can coincide with xRamDone: it is not in f_wcnt yet
    wire [6:0]  f_wc_done = {1'b0, f_wcnt} + {6'd0, xRamDoutValid};

    // semitone of the note shifting in at NOTE_ON kick time (argbuf[47:40])
    wire [3:0] kick_oct = note_oct(sq_argbuf[46:40]);
    wire [3:0] kick_sem = sq_argbuf[46:40] - {kick_oct, 3'd0}
                                             - {kick_oct, 2'd0}; // -oct*12

    always @(posedge xClk or posedge xReset) begin
        if (xReset) begin
            running <= 0; was_playing <= 0; spos <= 0; fill <= 0;
            wait_cnt <= 0; end_pend <= 0; mreq_tbl <= 0; song_idx <= 0;
            sq_st <= SQ_OP; sq_op <= 0; sq_rem <= 0; sq_argbuf <= 0;
            sq_mult_wait <= 0;
            xRamReq <= 0; xRamAddr <= 0; xRamBurstLen <= 0; req_out <= 0;
            cmd_pend <= 0; pend_play <= 0; pend_song <= 0; pend_seq <= 0;
            gap <= 0; mst <= ST_IDLE; ack_seq <= 0; ack_pl <= 0; end_seq <= 0;
            f_st <= F_IDLE; f_type <= JT_TBL; f_voice <= 0; f_rr <= 0;
            f_vgen <= 0; f_addr <= 0; f_wrem <= 0; f_wcnt <= 0; f_skip <= 0;
            f_wlo <= 0; t_soff <= 0; t_slen <= 0; t_boff <= 0;
            d_off <= 0; d_loop <= 0; b_shift <= 0; sb_wpt <= 0;
            stream_base <= 0; brr_base <= 0; stream_len <= 0;
            v_act <= 0; v_blk_vld <= 0; v_nblk_vld <= 0; v_req <= 0;
            v_job <= 0; v_looped <= 0;
            fifo_wr <= 0; fifo_wd <= 0; tick_evt <= 0; smp_cnt <= 0;
            sdiv <= 0; mix_st <= MI_IDLE; mix_vi <= 0; mix_sub <= 0;
            acc_l <= 0; acc_r <= 0; m_note <= 0; m_voice <= 0;
            for (vi = 0; vi < 4; vi = vi + 1) begin
                v_blk[vi] <= 72'd0; v_nblk[vi] <= 72'd0; v_nib[vi] <= 4'd0;
                v_pcm[vi] <= 16'sd0; v_prev1[vi] <= 16'sd0;
                v_prev2[vi] <= 16'sd0;
                v_inc[vi] <= 24'd0; v_ph[vi] <= 24'd0; v_gate[vi] <= 16'd0;
                v_gain_l[vi] <= 4'd0; v_gain_r[vi] <= 4'd0;
                v_vol_l[vi] <= 8'h7F; v_vol_r[vi] <= 8'h7F;
                v_cur_addr[vi] <= 23'd0; v_base[vi] <= 23'd0;
                v_loop[vi] <= 17'd0; v_srcn[vi] <= 6'd0; v_gen[vi] <= 2'd0;
            end
        end else begin
            xRamReq <= 1'b0;
            tick_evt <= 1'b0;
            fifo_wr <= 1'b0;

            // ---- serial multiplier step (shift-add, no DSP); a note-on
            // ---- kick later in this block overrides these assignments
            if (m_busy) begin
                if (m_bb[0]) m_acc <= m_acc + m_aa;
                m_aa  <= m_aa << 1;
                m_bb  <= m_bb >> 1;
                m_cnt <= m_cnt - 5'd1;
                if (m_cnt == 5'd1) m_busy <= 1'b0;
            end

            // ---------------- output sample divider ----------------------
            sdiv <= smp_strobe ? SAMPLE_DIV : sdiv - 16'd1;

            // ---------------- stream buffer fill merge -------------------
            fill <= fill + (fill_add ? ({f_wc_done[5:0], 1'b0} -
                                        {9'd0, f_skip}) : 10'd0)
                         - (fill_dec ? 10'd1 : 10'd0);

            // ---------------- command latch (wins over a consume) --------
            if (cmd_new) begin
                seq_last  <= seq_x2;
                cmd_pend  <= 1;
                pend_play <= cmd_play;
                pend_song <= cmd_song;
                pend_seq  <= seq_x2;
                end_pend  <= 1'b0;
            end

            // ---------------- master: gap / ack / (re)start --------------
            if (mst == ST_IDLE) begin
                if (cmd_pend && xRamReady && f_st == F_IDLE &&
                    sq_st != SQ_MULT && !m_busy) begin
                    cmd_pend <= 0;
                    mst <= ST_GAP;
                    // a full mixer scan can still write up to ~24 words
                    // after running drops; 32 idle cycles guarantee the
                    // FIFO is quiet before the echo fires the flush
                    gap <= 5'd31;
                    // stop the old stream now
                    running  <= 0;
                    v_act    <= 0;
                    v_req    <= 0;
                    v_job    <= 0;
                    v_blk_vld <= 0;
                    v_nblk_vld <= 0;
                end
            end else begin // ST_GAP
                if (gap == 5'd0) begin
                    ack_seq <= pend_seq;
                    ack_pl  <= {pend_play, pend_song};
                    if (pend_play) begin
                        song_idx <= pend_song;
                        mreq_tbl <= 1'b1;
                        // fresh stream state; FIFO residue is flushed by
                        // the reader off its ack-edge snapshot
                        spos <= 0; fill <= 0; wait_cnt <= 0;
                        v_blk_vld <= 0; v_nblk_vld <= 0;
                        v_req <= 0; v_job <= 0;
                        sq_st <= SQ_OP; sq_rem <= 0;
                        for (vi = 0; vi < 4; vi = vi + 1) begin
                            v_vol_l[vi] <= 8'h7F;   // N-SPC channel defaults
                            v_vol_r[vi] <= 8'h7F;
                        end
                    end
                    mst <= ST_IDLE;
                end else gap <= gap - 5'd1;
            end

            // ---------------- natural-end pulse --------------------------
            if (end_pend && !running && mst == ST_IDLE && !cmd_pend &&
                mix_st == MI_IDLE) begin
                end_pend <= 1'b0;
                end_seq  <= end_seq ^ gray_inc;
            end

            // ---------------- fetch FSM ----------------------------------
            case (f_st)
            F_IDLE: begin
                if (!cmd_pend && xRamReady && mst == ST_IDLE) begin
                    if (mreq_tbl) begin
                        mreq_tbl <= 0;
                        f_type <= JT_TBL;
                        f_addr <= BANK_BASE + TABLE_OFF +
                                  {16'd0, song_idx, 4'd0};
                        f_wrem <= 6'd8;
                        f_st <= F_REQ;
                    end else if (|v_req) begin
                        // round-robin over the 4 voices
                        if (v_req[f_rr]) begin
                            f_voice <= f_rr;
                            f_vgen  <= v_gen[f_rr];
                            v_req[f_rr] <= 1'b0;
                            f_type  <= v_job[f_rr] ? JT_BLK : JT_DIR;
                            f_addr  <= v_job[f_rr] ? v_cur_addr[f_rr] :
                                       (BANK_BASE + SAMPLE_OFF +
                                        {14'd0, v_srcn[f_rr], 3'd0});
                            f_wrem  <= v_job[f_rr] ? 6'd5 : 6'd4;
                        end else if (v_req[f_rr + 2'd1]) begin
                            f_voice <= f_rr + 2'd1;
                            f_vgen  <= v_gen[f_rr + 2'd1];
                            v_req[f_rr + 2'd1] <= 1'b0;
                            f_type  <= v_job[f_rr + 2'd1] ? JT_BLK : JT_DIR;
                            f_addr  <= v_job[f_rr + 2'd1] ?
                                       v_cur_addr[f_rr + 2'd1] :
                                       (BANK_BASE + SAMPLE_OFF +
                                        {14'd0, v_srcn[f_rr + 2'd1], 3'd0});
                            f_wrem  <= v_job[f_rr + 2'd1] ? 6'd5 : 6'd4;
                        end else if (v_req[f_rr + 2'd2]) begin
                            f_voice <= f_rr + 2'd2;
                            f_vgen  <= v_gen[f_rr + 2'd2];
                            v_req[f_rr + 2'd2] <= 1'b0;
                            f_type  <= v_job[f_rr + 2'd2] ? JT_BLK : JT_DIR;
                            f_addr  <= v_job[f_rr + 2'd2] ?
                                       v_cur_addr[f_rr + 2'd2] :
                                       (BANK_BASE + SAMPLE_OFF +
                                        {14'd0, v_srcn[f_rr + 2'd2], 3'd0});
                            f_wrem  <= v_job[f_rr + 2'd2] ? 6'd5 : 6'd4;
                        end else begin
                            f_voice <= f_rr + 2'd3;
                            f_vgen  <= v_gen[f_rr + 2'd3];
                            v_req[f_rr + 2'd3] <= 1'b0;
                            f_type  <= v_job[f_rr + 2'd3] ? JT_BLK : JT_DIR;
                            f_addr  <= v_job[f_rr + 2'd3] ?
                                       v_cur_addr[f_rr + 2'd3] :
                                       (BANK_BASE + SAMPLE_OFF +
                                        {14'd0, v_srcn[f_rr + 2'd3], 3'd0});
                            f_wrem  <= v_job[f_rr + 2'd3] ? 6'd5 : 6'd4;
                        end
                        f_rr <= f_rr + 2'd1;
                        f_st <= F_REQ;
                    end else if (str_needed) begin
                        f_type <= JT_STR;
                        f_addr <= ({1'b0, stream_base} + {1'b0, spos[22:0]} +
                                   {14'd0, fill}) & 23'h7FFFFE;
                        f_wrem <= 6'd32;
                        f_skip <= spos[0] ^ fill[0];
                        f_st <= F_REQ;
                    end
                end
            end
            F_REQ: begin
                if (cmd_pend) f_st <= F_ABORT;
                else begin
                    xRamAddr     <= f_addr;
                    xRamBurstLen <= {5'd0, f_wrem, 1'b0};
                    xRamReq      <= 1'b1;
                    req_out      <= 1'b1;
                    f_wcnt       <= 6'd0;
                    f_st         <= F_RX;
                end
            end
            F_RX: begin
                if (cmd_pend) f_st <= F_ABORT;
                else begin
                    if (xRamDoutValid) begin
                        f_wcnt <= f_wcnt + 6'd1;
                        case (f_type)
                        JT_TBL: begin
                            if (!f_wcnt[0]) f_wlo <= xRamDout;
                            else case (f_wcnt[2:1])
                                2'd0: t_soff <= {xRamDout, f_wlo};
                                2'd1: t_slen <= {xRamDout, f_wlo};
                                2'd2: t_boff <= {xRamDout, f_wlo};
                                default: ;
                            endcase
                        end
                        JT_DIR: begin
                            if (!f_wcnt[0]) f_wlo <= xRamDout;
                            else if (f_wcnt == 6'd1) d_off  <= {xRamDout, f_wlo};
                            else if (f_wcnt == 6'd3) d_loop <= {xRamDout, f_wlo};
                        end
                        JT_BLK: b_shift <= {b_shift[63:0], xRamDout};
                        JT_STR: begin
                            sbmem[sb_wpt] <= xRamDout;
                            sb_wpt <= sb_wpt + 8'd1;
                        end
                        endcase
                    end
                    if (xRamDone) begin
                        req_out <= 1'b0;
                        f_st <= F_IDLE;
                        case (f_type)
                        JT_TBL: begin
                            if (t_slen != 24'd0) begin
                                stream_base <= BANK_BASE + DATA_OFF + t_soff[22:0];
                                brr_base    <= BANK_BASE + DATA_OFF + t_boff[22:0];
                                stream_len  <= t_slen;
                                spos <= 0; fill <= 0; wait_cnt <= 0;
                                sq_st <= SQ_OP; sq_rem <= 0;
                                running <= 1'b1;
                                was_playing <= 1'b1;
                            end else begin
                                // empty song entry: play nothing
                                was_playing <= 1'b0;
                                end_seq <= end_seq ^ gray_inc;
                            end
                        end
                        JT_DIR: begin
                            if (f_vgen == v_gen[f_voice]) begin
                                if (d_off == 24'hFFFFFF) begin
                                    v_act[f_voice] <= 1'b0;   // unused slot
                                end else begin
                                    v_base[f_voice] <= brr_base + d_off[22:0];
                                    v_loop[f_voice] <= d_loop[16:0];
                                    v_looped[f_voice] <= d_loop[31];
                                    v_cur_addr[f_voice] <= brr_base + d_off[22:0];
                                    // chain straight into the first block
                                    v_job[f_voice] <= 1'b1;
                                    v_req[f_voice] <= 1'b1;
                                end
                            end
                        end
                        JT_BLK: begin
                            // xRamDone can coincide with the last word's
                            // valid (its shift into b_shift has not
                            // committed yet) or arrive one cycle later
                            // (b_shift already complete): both forms below
                            // yield the same 9-byte block either way.
                            if (f_vgen == v_gen[f_voice]) begin
                                if (!v_blk_vld[f_voice]) begin
                                    v_blk[f_voice] <= xRamDoutValid ?
                                        {xRamDout[7:0], b_shift[63:0]} :
                                        b_shift[71:0];
                                    v_blk_vld[f_voice] <= 1'b1;
                                    v_nib[f_voice] <= 4'd0;
                                end else begin
                                    v_nblk[f_voice] <= xRamDoutValid ?
                                        {xRamDout[7:0], b_shift[63:0]} :
                                        b_shift[71:0];
                                    v_nblk_vld[f_voice] <= 1'b1;
                                end
                                // next fetch address from the block header
                                if (b_shift[0]) begin           // end flag
                                    if (v_looped[f_voice])
                                        v_cur_addr[f_voice] <=
                                            v_base[f_voice] +
                                            {6'd0, v_loop[f_voice]};
                                end else
                                    // 10-byte padded stride (9-byte BRR
                                    // block + 1 pad byte, keeps every block
                                    // even-aligned for the 16-bit PSRAM bus)
                                    v_cur_addr[f_voice] <= f_addr + 23'd10;
                            end
                        end
                        JT_STR: ;   // fill is merged by the fill bookkeeping
                        endcase
                    end
                end
            end
            F_ABORT: begin
                // drain an outstanding burst (data discarded) before the gap
                if (req_out) begin
                    if (xRamDone) req_out <= 1'b0;
                end else f_st <= F_IDLE;
            end
            endcase

            // ---------------- sequencer ----------------------------------
            // stream byte read: sb_rword is sbmem[spos[8:1]] registered one
            // cycle, so consumption is a 2-phase present/then-take dance.
            sb_rword <= sbmem[spos[8:1]];
            if (running) begin
                case (sq_st)
                SQ_OP: if (wait_cnt == 16'd0 && byte_avail)
                           sq_st <= SQ_OPH;           // present read addr
                SQ_OPH: begin                         // sb_rword valid now
                    sq_op <= sb_byte;
                    spos  <= spos + 24'd1;
                    sq_st <= SQ_RD;
                end
                SQ_RD: begin
                    if (sq_op <= 8'h7E) begin         // WAIT n+1 ticks
                        wait_cnt <= {9'd0, sq_op} + 16'd1;
                        sq_st <= SQ_OP;
                    end else if (sq_op == 8'h7F || sq_op == 8'hFF) begin
                        running <= 1'b0;              // END / STOP
                        end_pend <= was_playing;
                        was_playing <= 1'b0;
                    end else if (sq_op[7:2] == 6'b100000) begin
                        sq_rem <= 3'd7; sq_st <= SQ_ARG;   // NOTE_ON (v<4)
                    end else if (sq_op[7:2] == 6'b100100) begin
                        sq_rem <= 3'd1; sq_st <= SQ_ARG;   // INSTR
                    end else if (sq_op[7:2] == 6'b101000) begin
                        sq_rem <= 3'd2; sq_st <= SQ_ARG;   // PANVOL
                    end else if (sq_op == 8'hF0) begin
                        sq_rem <= 3'd3; sq_st <= SQ_ARG;   // LOOP
                    end else if (sq_op == 8'hF1) begin
                        sq_rem <= 3'd2; sq_st <= SQ_ARG;   // TEMPO
                    end else begin
                        running <= 1'b0;              // corrupt bank
                        end_pend <= was_playing;
                        was_playing <= 1'b0;
                    end
                end
                SQ_ARG: if (byte_avail) sq_st <= SQ_ARGH;
                SQ_ARGH: begin
                    sq_argbuf <= {sq_argbuf[47:0], sb_byte};
                    spos <= spos + 24'd1;
                    if (sq_rem == 3'd1) begin
                        if (sq_op[7:2] == 6'b100000) begin
                            // NOTE_ON. This is the 7th arg shift, so the
                            // final positions are one slot up from what is
                            // visible now: note is at [47:40], mult_lo at
                            // [7:0] and mult_hi is the byte shifting in.
                            sq_st <= SQ_MULT;
                            sq_mult_wait <= 5'd19;
                            m_note  <= sq_argbuf[46:40];
                            m_voice <= sq_op[1:0];
                            m_busy  <= 1'b1;
                            m_cnt   <= 5'd16;
                            m_acc   <= 29'd0;
                            m_aa    <= {16'd0, noterom(kick_sem)};
                            m_bb    <= {sb_byte, sq_argbuf[7:0]}; // mult 8.8
                        end else sq_st <= SQ_EXEC;
                    end else begin
                        sq_rem <= sq_rem - 3'd1;
                        // back through SQ_ARG: sb_rword needs one cycle to
                        // register the next spos before it can be taken
                        sq_st <= SQ_ARG;
                    end
                end
                SQ_EXEC: begin
                    // argbuf holds all args now (first arg at the top)
                    if (sq_op[7:2] == 6'b101000) begin       // PANVOL
                        v_vol_l[sq_op[1:0]] <= sq_argbuf[15:8];
                        v_vol_r[sq_op[1:0]] <= sq_argbuf[7:0];
                    end else if (sq_op == 8'hF0) begin       // LOOP
                        spos <= {8'd0, sq_argbuf[7:0], sq_argbuf[15:8],
                                 sq_argbuf[23:16]};
                        fill <= 10'd0;
                    end
                    // INSTR: no-op (NOTE_ON is self-contained)
                    // TEMPO: timing already baked into the tick grid
                    sq_st <= SQ_OP;
                end
                SQ_MULT: begin
                    if (sq_mult_wait != 5'd0)
                        sq_mult_wait <= sq_mult_wait - 5'd1;
                    else begin
                        sq_st <= SQ_OP;
                        // complete the note-on; final arg positions:
                        // note [55:48], gate_lo [47:40], gate_hi [39:32],
                        // vel [31:24], srcn [23:16]
                        if ({sq_argbuf[39:32], sq_argbuf[47:40]} != 16'd0) begin
                            v_act[m_voice]     <= 1'b1;
                            v_gate[m_voice]    <= {sq_argbuf[39:32],
                                                   sq_argbuf[47:40]};
                            v_inc[m_voice]     <= m_inc;
                            v_ph[m_voice]      <= 24'd0;
                            v_pcm[m_voice]     <= 16'sd0;
                            v_prev1[m_voice]   <= 16'sd0;
                            v_prev2[m_voice]   <= 16'sd0;
                            v_blk_vld[m_voice] <= 1'b0;
                            v_nblk_vld[m_voice]<= 1'b0;
                            v_gain_l[m_voice]  <= gain4(sq_argbuf[31:24],
                                                        v_vol_l[m_voice]);
                            v_gain_r[m_voice]  <= gain4(sq_argbuf[31:24],
                                                        v_vol_r[m_voice]);
                            v_srcn[m_voice]    <= sq_argbuf[21:16];
                            v_gen[m_voice]     <= v_gen[m_voice] + 2'd1;
                            v_job[m_voice]     <= 1'b0;        // directory
                            v_req[m_voice]     <= 1'b1;
                        end
                    end
                end
                endcase
            end

            // ---------------- mixer / voice scan -------------------------
            if (smp_strobe && mix_st == MI_IDLE && running) begin
                mix_st <= MI_V;
                mix_vi <= 2'd0;
                mix_sub <= 3'd0;
                acc_l <= 24'sd0;
                acc_r <= 24'sd0;
                for (vi = 0; vi < 4; vi = vi + 1)
                    v_ph[vi] <= v_ph[vi] + v_inc[vi];
            end else case (mix_st)
            MI_V: begin
                // decode-step the scanned voice while its phase overflows
                if (v_act[sv] && v_blk_vld[sv] && (v_ph[sv][23:16] != 8'd0) &&
                    mix_sub != 3'd4) begin
                    mix_sub <= mix_sub + 3'd1;
                    v_ph[sv] <= v_ph[sv] - 24'd65536;
                    v_pcm[sv] <= sv_out;
                    v_prev2[sv] <= v_prev1[sv];
                    v_prev1[sv] <= sv_out;
                    if (v_nib[sv] == 4'd15) begin
                        // block wrap
                        if (v_blk[sv][0] && !v_looped[sv]) begin
                            v_act[sv] <= 1'b0;        // one-shot finished
                        end else if (v_nblk_vld[sv]) begin
                            v_blk[sv] <= v_nblk[sv];
                            v_nblk_vld[sv] <= 1'b0;
                            v_nib[sv] <= 4'd0;
                        end else begin
                            v_act[sv] <= 1'b0;        // underrun (never w/
                        end                           // the prefetch margin)
                    end else begin
                        v_nib[sv] <= v_nib[sv] + 4'd1;
                        // prefetch margin: 8 samples = 0.5 ms of headroom
                        if (v_nib[sv] == 4'd7 && !v_nblk_vld[sv] &&
                            !v_req[sv] && !(v_blk[sv][0] && !v_looped[sv])) begin
                            v_job[sv] <= 1'b1;
                            v_req[sv] <= 1'b1;
                        end
                    end
                end else begin
                    mix_st <= MI_GAIN;
                end
            end
            MI_GAIN: begin
                if (v_act[sv]) begin
                    acc_l <= acc_l + {{4{gterm_l[19]}}, gterm_l};
                    acc_r <= acc_r + {{4{gterm_r[19]}}, gterm_r};
                end
                if (mix_vi == 2'd3) mix_st <= MI_PUSH_L;
                else begin
                    mix_vi <= mix_vi + 2'd1;
                    mix_sub <= 3'd0;
                    mix_st <= MI_V;
                end
            end
            MI_PUSH_L: begin
                if (fifo_free >= 10'd2) begin
                    fifo_wr <= 1'b1;
                    fifo_wd <= (acc_l >  24'sd32767) ? 16'h7FFF :
                               (acc_l < -24'sd32768) ? 16'h8000 : acc_l[15:0];
                    mix_st <= MI_PUSH_R;
                end else mix_st <= MI_IDLE;           // FIFO full: drop pair
            end
            MI_PUSH_R: begin
                fifo_wr <= 1'b1;
                fifo_wd <= (acc_r >  24'sd32767) ? 16'h7FFF :
                           (acc_r < -24'sd32768) ? 16'h8000 : acc_r[15:0];
                mix_st <= MI_IDLE;
                // one sequencer tick every 16 output samples
                if (smp_cnt == 4'd15) begin
                    smp_cnt <= 4'd0;
                    tick_evt <= 1'b1;
                end else smp_cnt <= smp_cnt + 4'd1;
            end
            endcase

            // ---------------- sequencer tick: WAIT + gate countdowns -----
            if (tick_evt) begin
                if (wait_cnt != 16'd0) wait_cnt <= wait_cnt - 16'd1;
                for (vi = 0; vi < 4; vi = vi + 1) begin
                    if (v_act[vi]) begin
                        if (v_gate[vi] == 16'd1) v_act[vi] <= 1'b0;
                        else v_gate[vi] <= v_gate[vi] - 16'd1;
                    end
                end
            end
        end
    end

    // =====================================================================
    // hClk: ack sync + reader
    // =====================================================================
    reg [1:0] ack_h1 = 0, ack_h = 0;
    reg [3:0] ackp_h1 = 0, ackp_h = 0;
    reg [2:0] settle_h = 3'd0;
    reg [1:0] ack_last = 0;
    reg [1:0] end_h1 = 0, end_h = 0, end_last = 0;
    reg        flush_active = 0;
    reg        run_state = 0;
    reg        end_arm = 0;
    reg [10:0] flush_rem = 0;

    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            ack_h1 <= 0; ack_h <= 0; ackp_h1 <= 0; ackp_h <= 0; settle_h <= 0;
            // end_last/ack_last are owned (and reset) by the reader block
            // below; driving them here too makes Gowin see two drivers.
            end_h1 <= 0; end_h <= 0;
        end else begin
            ack_h1 <= ack_seq; ack_h <= ack_h1;
            ackp_h1 <= ack_pl; ackp_h <= ackp_h1;
            end_h1 <= end_seq; end_h <= end_h1;
            if (settle_h != 3'd7) settle_h <= settle_h + 3'd1;
        end
    end

    // sync wgray into hClk -> binary writer pointer (lagging = safe)
    always @(posedge hClk) begin wgray_h1 <= wgray; wgray_h <= wgray_h1; end
    wire [FW:0] wbin_h = wgray_h ^ (wgray_h >> 1) ^ (wgray_h >> 2)
                               ^ (wgray_h >> 3) ^ (wgray_h >> 4)
                               ^ (wgray_h >> 5) ^ (wgray_h >> 6)
                               ^ (wgray_h >> 7) ^ (wgray_h >> 8) ^ (wgray_h >> 9);

    wire        words_avail = (wbin_h != rbin);
    reg  [11:0] tick_cnt = 12'd0;
    wire        sample_tick = (tick_cnt == 12'd0);
    wire        ack_event = (settle_h == 3'd7) && (ack_h != ack_last);
    wire        end_event = (end_h != end_last);

    // FIFO word pump (same staging as sgb_sfx_play)
    reg        wbuf_v = 0, rd_pend = 0;
    reg [15:0] wbuf = 0;
    reg [15:0] fword;
    always @(posedge hClk) fword <= fmem[rbin[FW-1:0]];

    // stereo pair staging
    reg [15:0] l_hold = 16'd0;
    reg        have_l = 0;

    // a stereo pair needs TWO FIFO words per tick: grab the L word as soon
    // as it is available, then emit the pair on the sample tick
    wire want_emit  = run_state && !flush_active;
    wire load_l    = wbuf_v && !have_l && want_emit;
    wire emit_pair = wbuf_v &&  have_l && sample_tick && want_emit;
    wire fdrop     = wbuf_v && flush_active;
    wire consume   = !ack_event && (load_l || emit_pair || fdrop);
    wire [FW:0] rbin_next  = rbin + (consume ? {{FW{1'b0}},1'b1} : {(FW+1){1'b0}});
    wire [FW:0] rgray_next = (rbin_next >> 1) ^ rbin_next;

    always @(posedge hClk or posedge hReset) begin
        if (hReset) begin
            rbin <= 0; rgray <= 0; rd_pend <= 0; wbuf_v <= 0; wbuf <= 0;
            flush_active <= 0; run_state <= 0; hPlaying <= 0; flush_rem <= 0;
            ack_last <= 0; end_arm <= 0; l_hold <= 0; have_l <= 0;
            hPcmL <= 0; hPcmR <= 0; hPcmValid <= 0; tick_cnt <= 0;
        end else begin
            hPcmValid <= 1'b0;
            tick_cnt <= sample_tick ? TICK_PERIOD : tick_cnt - 12'd1;

            rbin  <= rbin_next;
            rgray <= rgray_next;
            if (consume) wbuf_v <= 1'b0;

            // refill wbuf from the FIFO
            if (rd_pend) begin
                wbuf    <= fword;
                wbuf_v  <= 1'b1;
                rd_pend <= 1'b0;
            end else if (!wbuf_v && words_avail && (run_state || flush_active))
                rd_pend <= 1'b1;

            // command echo handling
            if (settle_h != 3'd7) begin
                ack_last <= ack_h;
                end_last <= end_h;
            end else begin
                if (end_event) end_last <= end_h;
                if (ack_event) begin
                    ack_last <= ack_h;
                    have_l <= 1'b0;
                    if (ackp_h[3]) begin
                        // play echo: snapshot flush, then run
                        hPlaying  <= 1'b1;
                        run_state <= 1'b0;
                        end_arm   <= 1'b0;
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
                        end_arm <= 0;
                    end
                end
            end

            // natural end: play out the FIFO tail, then deassert hPlaying
            if (end_event && settle_h == 3'd7) begin
                if (!words_avail && !wbuf_v) begin
                    hPlaying <= 0; run_state <= 0; flush_active <= 0;
                end else begin
                    flush_active <= 0;          // keep samples, stop after
                    end_arm <= 1'b1;
                end
            end
            if (end_arm && sample_tick && !words_avail && !wbuf_v) begin
                end_arm <= 0;
                hPlaying <= 0; run_state <= 0;
            end

            // immediate local stop
            if (hStop && !hStop_q) begin
                hPlaying <= 0; run_state <= 0; flush_active <= 0;
                end_arm <= 0; have_l <= 0;
            end

            // emit one stereo pair per tick
            if (load_l) begin
                l_hold <= wbuf;
                have_l <= 1'b1;
            end
            if (emit_pair) begin
                hPcmL <= $signed(l_hold);
                hPcmR <= $signed(wbuf);
                hPcmValid <= 1'b1;
                have_l <= 1'b0;
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

endmodule
