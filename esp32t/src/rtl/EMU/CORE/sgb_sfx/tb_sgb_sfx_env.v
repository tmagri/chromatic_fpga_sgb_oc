`timescale 1ns/1ps
// tb_sgb_sfx_env.v -- verify the bank-v5 pitch-envelope sequencer in
// sgb_sfx_play at REAL tick rates (no sim-speed tick patching).
//
// The sweep TB (tb_sgb_sfx_play.v) patches every tick to 3 to run fast; that
// proves sample VALUES but not envelope TIMING. This TB plays a few envelope
// effects with their true per-segment tick dividers and checks, sample by
// sample, that the tick in force for each emitted sample matches the bank's
// piecewise-constant envelope: segment 0 holds the record's tick for
// entry[0].blocks16*16 samples, then segment 1 holds entry[0].tick for
// entry[1].blocks16*16 samples, and so on; the final segment holds its tick
// to the end. Any desync in the block-boundary countdown, the smem fetch
// pipeline, or the tick reload mux shows up as a per-sample tick mismatch.
//
// Run from esp32t/src/rtl/EMU/CORE:
//   iverilog -g2001 -o /tmp/tb_env.out sgb_sfx/tb_sgb_sfx_env.v sgb_sfx_play.v
//   vvp /tmp/tb_env.out
module tb_sgb_sfx_env;
    localparam [22:0] BANK_BASE = 23'h100000;
    localparam BANK_WORDS = 23788;        // 47576 bytes / 2 (bank v5)
    localparam SEG_WBASE  = 21664;        // segment region word base (0xA940/2)
    localparam MAXS       = 120000;

    reg xClk = 0, hClk = 0;
    always #6.667  xClk = ~xClk;          // 75 MHz
    always #29.801 hClk = ~hClk;          // 16.777 MHz
    reg xReset = 1, hReset = 1;
    initial begin xReset = 1; hReset = 1; #400; xReset = 0; hReset = 0; end

    wire        xRamReq_w; wire [22:0] xRamAddr_w; wire [10:0] xRamBurstLen_w;
    reg         xRamReady = 0;
    reg         xRamDone = 0, xRamDoutValid = 0;
    reg  [15:0] xRamDout = 0;
    reg         hStart = 0, hStop = 0;
    reg  [6:0]  hEffectIndex = 0;
    wire        hPcmValid, hPlaying; wire signed [15:0] hPcm;

    sgb_sfx_play #(.BANK_BASE(BANK_BASE)) dut (
        .xClk(xClk), .xReset(xReset), .xRamReady(xRamReady),
        .xRamReq(xRamReq_w), .xRamRnW(), .xRamAddr(xRamAddr_w),
        .xRamDin(), .xRamBurstLen(xRamBurstLen_w),
        .xRamDone(xRamDone), .xRamDoutValid(xRamDoutValid), .xRamDout(xRamDout),
        .hClk(hClk), .hReset(hReset),
        .hStart(hStart), .hStop(hStop), .hEffectIndex(hEffectIndex),
        .hPcmValid(hPcmValid), .hPcm(hPcm), .hPlaying(hPlaying)
    );

    // ---- PSRAM contents: the real bank, NO tick patching ----
    reg [15:0] psram [0:BANK_WORDS-1];
    integer i;
    initial begin
        $readmemh("sgb_sfx/build/sim_bank.hex", psram);
        xRamReady <= 1;
    end

    function [15:0] rd_word; input [22:0] a; integer w;
        begin w = (a - BANK_BASE) >> 1;
              rd_word = (w >= 0 && w < BANK_WORDS) ? psram[w] : 16'h0000; end
    endfunction

    function [15:0] rec_word; input [6:0] idx; input [2:0] w;
        begin rec_word = psram[128 + idx*8 + w]; end
    endfunction

    // ---- arbiter / PSRAM read model (same as the sweep TB) ----
    localparam ARB_IDLE=0, ARB_LAT=1, ARB_RUN=2;
    reg [1:0] arb_st = ARB_IDLE; reg [5:0] arb_lat = 0;
    reg [22:0] rd_addr = 0; reg [10:0] rd_rem = 0; reg [2:0] bubble = 0;
    always @(posedge xClk) begin
        if (xReset) begin
            arb_st <= ARB_IDLE; xRamDone <= 0; xRamDoutValid <= 0; rd_rem <= 0;
        end else begin
            xRamDoutValid <= 0; xRamDone <= 0;
            case (arb_st)
            ARB_IDLE: if (xRamReq_w && xRamReady) begin
                arb_st <= ARB_LAT; arb_lat <= $urandom_range(20, 2);
                rd_addr <= xRamAddr_w; rd_rem <= xRamBurstLen_w;
            end
            ARB_LAT: if (arb_lat == 0) arb_st <= ARB_RUN;
                     else arb_lat <= arb_lat - 1;
            ARB_RUN: begin
                if (bubble != 0) bubble <= bubble - 1;
                else if (rd_rem != 0) begin
                    xRamDout <= rd_word(rd_addr); xRamDoutValid <= 1;
                    rd_addr <= rd_addr + 2; rd_rem <= rd_rem - 2;
                    if ($urandom_range(7,0) == 0) bubble <= $urandom_range(4,1);
                    if (rd_rem == 2) begin xRamDone <= 1; arb_st <= ARB_IDLE; end
                end
            end
            endcase
        end
    end

    // ---- capture the tick in force for each emitted sample ----
    integer scnt = 0;
    reg [15:0] obs_tick [0:MAXS-1];
    always @(posedge hClk) begin
        if (hPcmValid && scnt < MAXS) begin
            obs_tick[scnt] <= dut.h_tick;      // pre-update value this sample
            scnt <= scnt + 1;
        end
    end

    integer fails = 0;

    task start_fx; input [6:0] idx;
        begin
            @(posedge hClk); hEffectIndex <= idx; hStart <= 1;
            @(posedge hClk); hStart <= 0;
        end
    endtask

    // play effect idx (a count-limited one-shot) to its natural end, then
    // verify the observed per-sample tick matches the bank envelope exactly
    task check_env;
        input [6:0] idx;
        input integer timeout_ms;
        integer t, tmax, total, s, e, segb, mm, first_mm;
        reg [15:0] tick0, etick, r_flags;
        reg [15:0] r_count, r_blocks;
        reg [15:0] seg_off_r, seg_cnt_r;
        begin
            r_flags   = rec_word(idx, 4);
            r_count   = rec_word(idx, 5);
            r_blocks  = rec_word(idx, 3);
            tick0     = rec_word(idx, 2);
            seg_off_r = rec_word(idx, 6);
            seg_cnt_r = rec_word(idx, 7);
            total     = (r_flags[0] ? r_count : r_blocks) * 16;

            scnt = 0;
            start_fx(idx);
            t = 0; tmax = timeout_ms * 75000;
            while (!hPlaying && t < tmax) begin @(posedge xClk); t=t+1; end
            while ( hPlaying && t < tmax) begin @(posedge xClk); t=t+1; end
            repeat (200) @(posedge hClk);
            if (t >= tmax) begin
                fails = fails + 1;
                $display("FAIL env fx%0d: timed out (playing=%b scnt=%0d/%0d)",
                         idx, hPlaying, scnt, total);
            end else if (scnt !== total) begin
                fails = fails + 1;
                $display("FAIL env fx%0d: %0d samples, expected %0d",
                         idx, scnt, total);
            end else begin
                // walk the expected piecewise-constant tick and compare.
                // Alignment note: hPcmValid is registered, so the TB samples
                // h_tick one hClk after the emit edge; on a boundary emit the
                // RTL updates h_tick on that same edge (the tick reload for the
                // NEXT interval is correct), so the boundary sample itself is
                // observed with the NEW tick. Expected tick of sample s is
                // therefore the envelope value of sample min(s+1, total-1):
                // each segment's observed span is shifted one sample earlier.
                mm = 0; first_mm = -1; s = 0;
                if (seg_cnt_r == 0) begin
                    // single fixed segment for the whole effect
                    for (s = 0; s < total; s = s + 1)
                        if (obs_tick[s] !== tick0) begin
                            mm = mm + 1; if (first_mm < 0) first_mm = s;
                        end
                end else begin
                    segb = 0;   // envelope-space boundary (sample index)
                    for (e = 0; e < seg_cnt_r; e = e + 1) begin : per_entry
                        integer blk16; integer end_s; integer obs_end;
                        blk16 = psram[SEG_WBASE + (seg_off_r + e)*2];
                        etick = (e == 0) ? tick0
                                         : psram[SEG_WBASE + (seg_off_r + e - 1)*2 + 1];
                        end_s = segb + blk16*16;
                        if (end_s > total) end_s = total;
                        // observed span of this tick: [segb-1, end_s-1)
                        obs_end = end_s - 1;
                        for (s = (e == 0 ? 0 : segb - 1); s < obs_end; s = s + 1)
                            if (obs_tick[s] !== etick) begin
                                mm = mm + 1; if (first_mm < 0) first_mm = s;
                            end
                        segb = end_s;
                    end
                    // final segment holds the last entry's next-tick to the end
                    etick = psram[SEG_WBASE + (seg_off_r + seg_cnt_r - 1)*2 + 1];
                    for (s = segb - 1; s < total; s = s + 1)
                        if (obs_tick[s] !== etick) begin
                            mm = mm + 1; if (first_mm < 0) first_mm = s;
                        end
                end
                if (mm == 0)
                    $display("  ENV EXACT  fx%0d: %0d samples, %0d segments",
                             idx, total, seg_cnt_r + 1);
                else begin
                    fails = fails + 1;
                    $display("  ENV MISMATCH fx%0d: %0d/%0d samples (first @%0d)",
                             idx, mm, total, first_mm);
                end
            end
        end
    endtask

    initial begin
        $display("TB ENV START (real-tick envelope timing)");
        wait (!xReset && !hReset);
        repeat (32) @(posedge xClk);

        // A01 (2-segment coin), A02 (melody), A03 (long glide)
        $display("fx A01 Nintendo (2 segments)");   check_env(7'd0, 60000);
        $display("fx A02 GameOver (82 entries)");   check_env(7'd1, 60000);
        $display("fx A03 Drop (114 entries)");      check_env(7'd2, 60000);

        if (fails) $display("RESULT: FAIL (%0d)", fails);
        else       $display("RESULT: PASS");
        $finish;
    end

    initial begin
        #20_000_000_000;
        $display("GLOBAL TIMEOUT (scnt=%0d)", scnt);
        $display("RESULT: FAIL (timeout)");
        $finish;
    end

endmodule
