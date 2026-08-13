// sgb_snd.v -- COMPRESSED SGB custom BRR audio (HLE)  [chromatic GW5A port]
// ---------------------------------------------------------------------------
// Plays the SNES BRR sample the game uploads over SOU_TRN when a SOUND
// packet triggers it (DKGB Pauline-help scream). Derived from the upstream
// Gameboy_MiSTer sgb_snd: aggressively minified for the gate budget of the
// GW5A-25 (Logic/CLS/BSRAM/DSP all >85% used) AND fixed to match real SPC700
// DSP hardware (nibble order + all four predictor filters; see below).
//
// Compression vs. upstream (Gameboy_MiSTer/rtl/sgb_snd.v):
//   * 64-bit blkdata register + variable nibble-select mux REMOVED -- the 8
//     data bytes of each BRR block are re-read from the cache on demand. The
//     cache read-port is idle during playback, and at 11025 Hz there are
//     ~1500 clk_sys of slack per sample, so the BSRAM latency is hidden by
//     presenting the read address combinationally.
//   * 32-bit phase accumulator -> 12-bit /1522 down-counter (11023.1 Hz,
//     -0.017% pitch error vs the exact 11025 Hz -- inaudible).
//   * 8-state FSM -> 4-state (IDLE/HDR/HDR2/PLAY); read address is a pure
//     combinational function of (state, block_ptr, nib_i), removing the
//     rd_addr register entirely.
//   * Unused snd_mute port dropped; invalid-shift (13-15) garbage rule
//     dropped (never present in the uploaded data).
//
// Fidelity fixes vs upstream (both verified against the hardware-validated
// bsnes SPC DSP and the ROM data -- see tb_sgb_snd.v / tb_check_snd.py):
//   * Nibbles are decoded HIGH-nibble-first (the SPC700 hardware order).
//     Upstream decoded low-first, which garbles the waveform's fine
//     structure (envelope/loudness survive, timbre turns harsh).
//   * All four predictor filters (f0-f3), bit-exact bsnes arithmetic,
//     shift-add only (NO multipliers) so it stays in LUTs and does not
//     consume the scarce (>90% full) DSP. f2 is load-bearing: 73 of the
//     scream's 331 blocks are filter-2; an earlier minified build decoded
//     them as f0 (raw residual) -- a loud, harsh buzz.
//   * The predictor datapath is 20 bits (worst-case intermediate is
//     13*prev1 = +/-425984 in f3); saturation to 16 bits at the output.
//   * The end-flag block's final sample is emitted (hardware does not
//     drop it); pcm_out is re-silenced on the following idle tick.
//
// SAMPLE_BASE, the cache-freeze / SOU_TRN-count mechanism and the DKGB
// packet layout are unchanged -- see the upstream header for the full
// derivation (VRAM $8800 -> cache $800, Pauline blob at +$16 = $816, etc.).
// ---------------------------------------------------------------------------

module sgb_snd #(
    // sample tick divider: clk_sys cycles per output sample, minus one.
    // 1521 -> 16.777216 MHz / 1522 ~ 11023.1 Hz (-0.017%, inaudible).
    parameter [11:0] TICK_PERIOD = 12'd1521,
    parameter [7:0]  ENV_ATTACK_RATE  = 8'd2,   // sample ticks per envelope step
    parameter [7:0]  ENV_DECAY_RATE   = 8'd2,
    parameter [7:0]  ENV_SUSTAIN_LVL  = 8'd255, // 0-255 (255 = no decay loss)
    parameter [7:0]  ENV_RELEASE_RATE = 8'd4
)(
    input             clk_sys,
    input             reset,
    input             sgb_en,

    input             snd_trig,       // SOUND packet byte-4 pulse (ce domain)
    input      [7:0]  snd_id,         // SOUND X (byte 1); bit7 = stop
    input             snd_mute, 
    input      [7:0]  snd_z,          // SOUND Z (byte 4); 1=play, 3=stop
    input             sou_trn_valid,  // SOU_TRN transfer-done strobe

    // VRAM snoop -> 4 KB rolling sample cache (frozen once resident)
    input             sp_ce,
    input             sp_wren,
    input      [11:0] sp_addr,
    input      [7:0]  sp_data,

    output reg [15:0] pcm_out         // signed 16-bit PCM, 0 when idle
);

    localparam [11:0] SAMPLE_BASE = 12'h816;       // Pauline-help BRR start
    // chromatic port: clk_sys here is hclk = 16.777216 MHz, NOT the 33.554432
    // MHz the upstream module was written for (TICK_PERIOD 3042 would play the
    // sample at half speed, one octave down). Period = TICK_PERIOD+1 ticks.
    // TICK_PERIOD (module header) is overridable to speed up sims;
    // sample-indexed checks are rate-agnostic.

    localparam [1:0] S_IDLE = 2'd0,
                     S_HDR  = 2'd1,   // present header addr; BSRAM 1-cycle settle
                     S_HDR2 = 2'd2,   // latch shift/filter/end from header byte
                     S_PLAY = 2'd3;   // wait tick -> emit sample, advance nibble

    reg [1:0]  state = 2'd0;
    reg [11:0] block_ptr = 12'd0;        // address of current BRR block header
    reg [3:0]  nib_i = 4'd0;            // sample index within block (0..15)
    reg [3:0]  brr_shift = 4'd0;
    reg [1:0]  brr_filter = 2'd0;
    reg        brr_end = 1'b0;
    reg [11:0] tick_cnt = 12'd0;
    reg signed [15:0] prev1 = 16'sd0, prev2 = 16'sd0;   // BRR predictor history
    reg trig_old = 1'b0, sou_trn_old = 1'b0;
    reg [1:0]  sou_trn_cnt = 2'd0;           // SOU_TRN transfers seen (DKGB sends 2)

    localparam [2:0] E_IDLE    = 3'd0,
                     E_ATTACK  = 3'd1,
                     E_DECAY   = 3'd2,
                     E_SUSTAIN = 3'd3,
                     E_RELEASE = 3'd4;
    reg [2:0] env_state = E_IDLE;
    reg [7:0] env_level = 8'd0;
    reg [7:0] env_tick_cnt = 8'd0;

    wire sample_ready = sou_trn_cnt[1];

    // Combinational cache read address: header byte while fetching, else the
    // data byte holding the current nibble. Held stable for the whole tick
    // wait, so the registered BSRAM output (1-cycle latency) is always valid
    // by the time a sample is consumed.
    wire [11:0] rd_addr = (state == S_PLAY) ? block_ptr + 12'd1 + {7'd0, nib_i[3:1]}
                                            : block_ptr;

    // Register the VRAM snoop inputs: decouples the 4 KB BSRAM write-port
    // load from the hot vram_addr/vram_di/vram_wren nets (which also drive
    // both VRAM banks and the HDMA mux, adjacent to the hclk critical
    // paths). One cycle of write latency is invisible: the SOU_TRN strobe
    // that freezes the cache arrives a full bit-banged JOYP packet
    // (milliseconds) after the last payload write.
    reg        sp_ce_r = 1'b0, sp_wren_r = 1'b0;
    reg [11:0] sp_addr_r = 12'd0;
    reg [7:0]  sp_data_r = 8'd0;
    always @(posedge clk_sys) begin
        sp_ce_r   <= sp_ce;
        sp_wren_r <= sp_wren;
        sp_addr_r <= sp_addr;
        sp_data_r <= sp_data;
    end

    // 4 KB x8 simple-dual-port sample cache: one synchronous write port
    // (snoops VRAM writes until the sample is resident) + one synchronous
    // read port (decode). Written as an explicit SDP pattern so Gowin infers
    // a BSRAM block (the design already uses SDPB blocks) rather than the
    // ~2000-LUT distributed RAM that dpramV's unused read-port can collapse
    // into under rw_check_on_ram on this >90%-full device.
    reg [7:0] cmem [0:4095];
    reg [7:0] rd_byte;
    always @(posedge clk_sys) begin
        if (sp_ce_r & sp_wren_r & ~sample_ready)
            cmem[sp_addr_r] <= sp_data_r;
        rd_byte <= cmem[rd_addr];
    end

    wire trig       = snd_trig      & ~trig_old;
    wire sou_strobe = sou_trn_valid & ~sou_trn_old;
    wire tick       = (tick_cnt == 12'd0);

    // --- BRR decode (combinational, adders only -- no DSP) -----------------
    // HIGH nibble first: the first sample of each data byte is bits 7-4 (the
    // SPC700 hardware order, per fullsnes/SNESdev and the hardware-validated
    // bsnes SPC DSP). The upstream Gameboy_MiSTer sgb_snd decoded low-nibble
    // first -- wrong for hardware, and a major source of harshness alongside
    // the missing f2/f3 filters.
    wire [3:0] nib_raw = nib_i[0] ? rd_byte[3:0] : rd_byte[7:4];
    wire signed [15:0] snib     = {{12{nib_raw[3]}}, nib_raw};
    wire signed [15:0] shifted  = snib <<< brr_shift;

    // All four BRR predictor filters, BIT-EXACT with the hardware-validated
    // bsnes SPC DSP (blargg SPC_DSP.cpp decode_brr), including its
    // sub-expression flooring order. Note the real hardware coefficients are
    //   f1: +15/32 p1                      (0.46875)
    //   f2: +61/64 p1 - 15/32 p2           (0.953125, -0.46875)
    //   f3: +115/128 p1 - 13/32 p2         (0.8984375, -0.40625)
    // NOT the old SPC700 datasheet's 15/16 family. f2 is load-bearing for the
    // DKGB scream: 73 of its 331 blocks are filter-2; decoding them as raw
    // excitation (the old minified behaviour) is a loud, harsh buzz.
    // 20-bit datapath: the widest intermediate is 13*prev1 = +/-425984 (f3).
    wire signed [19:0] p1    = {{4{prev1[15]}}, prev1};
    // NOTE: $signed is load-bearing -- a bare concatenation is UNSIGNED, so
    // `{...} >>> 1` would shift logically and corrupt p2h for every negative
    // prev2 (audible crackle). Same trap does not apply to p1 (no shift in
    // its expression; the bit pattern is right either way).
    wire signed [19:0] p2h   = $signed({{4{prev2[15]}}, prev2}) >>> 1;
    wire signed [19:0] t3p1  = (p1 <<< 1) + p1;                 // 3*p1
    wire signed [19:0] t13p1 = (p1 <<< 3) + (p1 <<< 2) + p1;    // 13*p1
    wire signed [19:0] t3p2h = (p2h <<< 1) + p2h;               // 3*(p2>>1)
    reg  signed [19:0] fcon;
    always @(*) begin
        case (brr_filter)
            2'd1:    fcon = (p1 >>> 1) + ((-p1) >>> 5);
            2'd2:    fcon = p1 - p2h + (p2h >>> 4) + ((-t3p1) >>> 6);
            2'd3:    fcon = p1 - p2h + (t3p2h >>> 4) + ((-t13p1) >>> 7);
            default: fcon = 20'sd0;   // filter 0
        endcase
    end

    wire signed [19:0] acc = {{4{shifted[15]}}, shifted} + fcon;
    wire [15:0] out16 = (acc >  20'sd32767) ? 16'h7FFF :
                        (acc < -20'sd32768) ? 16'h8000 : acc[15:0];

    wire signed [15:0] signed_out16 = out16;
    wire signed [24:0] pcm_mult = signed_out16 * $signed({1'b0, env_level});
    wire [15:0] env_out16 = pcm_mult[23:8];

    // --- ADSR Envelope Generator -------------------------------------------
    always @(posedge clk_sys) begin
        if (reset || !sgb_en) begin
            env_state    <= E_IDLE;
            env_level    <= 8'd0;
            env_tick_cnt <= 8'd0;
        end else if (trig) begin
            if (snd_id[7] || snd_z == 8'd3) begin
                if (env_state != E_IDLE) begin
                    env_state    <= E_RELEASE;
                    env_tick_cnt <= ENV_RELEASE_RATE;
                end
            end else if (snd_z == 8'd1 && sample_ready) begin
                env_state    <= E_ATTACK;
                env_level    <= 8'd0;
                env_tick_cnt <= ENV_ATTACK_RATE;
            end
        end else if (tick) begin
            case (env_state)
                E_ATTACK: begin
                    if (env_tick_cnt == 0) begin
                        env_tick_cnt <= ENV_ATTACK_RATE;
                        if (env_level == 8'd255) env_state <= E_DECAY;
                        else env_level <= env_level + 8'd1;
                    end else env_tick_cnt <= env_tick_cnt - 8'd1;
                end
                E_DECAY: begin
                    if (env_tick_cnt == 0) begin
                        env_tick_cnt <= ENV_DECAY_RATE;
                        if (env_level <= ENV_SUSTAIN_LVL) begin
                            env_level <= ENV_SUSTAIN_LVL;
                            env_state <= E_SUSTAIN;
                        end else env_level <= env_level - 8'd1;
                    end else env_tick_cnt <= env_tick_cnt - 8'd1;
                end
                E_SUSTAIN: begin
                    // Hold level until release triggered
                end
                E_RELEASE: begin
                    if (env_tick_cnt == 0) begin
                        env_tick_cnt <= ENV_RELEASE_RATE;
                        if (env_level == 8'd0) env_state <= E_IDLE;
                        else env_level <= env_level - 8'd1;
                    end else env_tick_cnt <= env_tick_cnt - 8'd1;
                end
                default: env_level <= 8'd0;
            endcase
        end
    end

    // --- main FSM ----------------------------------------------------------
    always @(posedge clk_sys) begin
        trig_old    <= snd_trig;
        sou_trn_old <= sou_trn_valid;
        if (sou_strobe & ~sample_ready) sou_trn_cnt <= sou_trn_cnt + 2'd1;
        if (tick) tick_cnt <= TICK_PERIOD;
        else      tick_cnt <= tick_cnt - 12'd1;

        if (reset || !sgb_en) begin
            state        <= S_IDLE;
            pcm_out      <= 16'd0;
            prev1        <= 16'sd0;
            prev2        <= 16'sd0;
            block_ptr    <= 12'd0;
            nib_i        <= 4'd0;
            tick_cnt     <= 12'd0;
            sou_trn_cnt  <= 2'd0;
        end else if (trig) begin
            // DKGB SOUND packets: Z=1 -> play from SAMPLE_BASE; X bit7 or Z=3
            // -> stop. Other Z leave the playing sample alone.
            // ADSR: Stop commands trigger release instead of jumping to IDLE instantly.
            if (snd_id[7] || snd_z == 8'd3) begin
                // Do not instantly mute; let the ADSR release phase handle it
            end else if (snd_z == 8'd1 && sample_ready) begin
                block_ptr <= SAMPLE_BASE;
                prev1     <= 16'sd0;
                prev2     <= 16'sd0;
                pcm_out   <= 16'd0;
                nib_i     <= 4'd0;
                state     <= S_HDR;
            end
        end else begin
            case (state)
                S_IDLE:  if (tick) pcm_out <= 16'd0;  // release held last sample
                S_HDR:   state <= S_HDR2;            // BSRAM read settles
                S_HDR2: begin
                    brr_shift  <= rd_byte[7:4];
                    brr_filter <= rd_byte[3:2];
                    brr_end    <= rd_byte[0];
                    nib_i      <= 4'd0;
                    state      <= S_PLAY;
                end
                S_PLAY: if (tick) begin
                    pcm_out <= env_out16; // Use the envelope-scaled output
                    prev2   <= prev1;
                    prev1   <= out16;     // Predictor still uses raw out16
                    if (env_state == E_IDLE) begin
                        state <= S_IDLE;
                    end else if (nib_i == 4'd15) begin
                        // The end-flag block's samples are real audio: emit
                        // them all (pcm_out <= env_out16 above).
                        if (brr_end) state <= S_IDLE;
                        else begin
                            block_ptr <= block_ptr + 12'd9;   // next block header
                            state     <= S_HDR;
                        end
                    end else begin
                        nib_i <= nib_i + 4'd1;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
