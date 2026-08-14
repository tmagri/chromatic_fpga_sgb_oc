`timescale 1ns/1ps
// tb_sgb_sfx_hang.v -- repro of the sweep-TB hang: after
//   play B01 (loop) -> interrupt with A09 (count one-shot) ->
//   A09 natural end -> belt-and-braces stop -> play A03 2 hClk later,
// the fresh play (A03, enveloped) emitted zero samples in the sweep.
//
// Trigger (found): the stop's pend is consumed into ST_GAP(16 xClk); the
// play arrives ~2 hClk (~9 xClk) later, INSIDE that gap, and the cmd_new
// latch overwrites pend_seq before the echo is emitted -- the stop's echo
// is skipped. With the original 2-state oscillating "gray" sequence the
// play then echoes the SAME sequence value the previous play echoed, the
// reader's edge detector sees no change, and the play is silently lost.
// (An earlier version of this TB waited 300 xClk after the stop and so
// never hit the race; the sweep does not wait.)
//
// The sequence is iterated MANY times without reset (FIFO gray pointers and
// ack/cmd sequences accumulate exactly like in the sweep TB) behind the same
// $urandom latency/bubble arbiter, to expose any seed/state-dependent race.
//
// Run from esp32t/src/rtl/EMU/CORE:
//   iverilog -g2001 -o /tmp/tb_hang.out sgb_sfx/tb_sgb_sfx_hang.v sgb_sfx_play.v
//   vvp /tmp/tb_hang.out
module tb_sgb_sfx_hang;
    localparam [22:0] BANK_BASE = 23'h100000;
    localparam BANK_WORDS = 31867;        // 63734 bytes / 2 (bank v6.1)
    localparam SEG_WBASE  = 25768;        // segment region word base (0xC950/2)
    localparam ITERS      = 200;

    reg xClk = 0, hClk = 0;
    always #6.667  xClk = ~xClk;
    always #29.801 hClk = ~hClk;
    reg xReset = 1, hReset = 1;
    initial begin xReset = 1; hReset = 1; #400; xReset = 0; hReset = 0; end

    wire        xRamReq_w; wire [22:0] xRamAddr_w; wire [10:0] xRamBurstLen_w;
    reg         xRamReady = 1;
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

    reg [15:0] psram [0:BANK_WORDS-1];
    integer i;
    initial begin
        $readmemh("sgb_sfx/build/sim_bank.hex", psram);
        for (i = 0; i < 73; i = i + 1) psram[128 + i*8 + 2] = 16'd3;
        // bank v6: entries are 3 words = blocks16, next_tick, next_amp
        for (i = SEG_WBASE + 1; i < BANK_WORDS; i = i + 3) psram[i] = 16'd3;
    end

    function [15:0] rd_word; input [22:0] a; integer w;
        begin w = (a - BANK_BASE) >> 1;
              rd_word = (w >= 0 && w < BANK_WORDS) ? psram[w] : 16'h0000; end
    endfunction

    // same $urandom arbiter as the sweep TB
    localparam ARB_IDLE=0, ARB_LAT=1, ARB_RUN=2;
    reg [1:0] arb_st = ARB_IDLE; reg [5:0] arb_lat = 0;
    reg [22:0] rd_addr = 0; reg [10:0] rd_rem = 0; reg [2:0] bubble = 0;
    always @(posedge xClk) begin
        if (xReset) begin arb_st <= ARB_IDLE; xRamDone <= 0; xRamDoutValid <= 0; rd_rem <= 0; end
        else begin
            xRamDoutValid <= 0; xRamDone <= 0;
            case (arb_st)
            ARB_IDLE: if (xRamReq_w && xRamReady) begin
                arb_st <= ARB_LAT; arb_lat <= $urandom_range(20, 2);
                rd_addr <= xRamAddr_w; rd_rem <= xRamBurstLen_w;
            end
            ARB_LAT: if (arb_lat == 0) arb_st <= ARB_RUN; else arb_lat <= arb_lat - 1;
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

    integer ncap = 0;
    always @(posedge hClk) if (hPcmValid) ncap <= ncap + 1;

    task start_fx; input [6:0] idx;
        begin
            @(posedge hClk); hEffectIndex <= idx; hStart <= 1;
            @(posedge hClk); hStart <= 0;
        end
    endtask
    task stop_fx;
        begin
            @(posedge hClk); hStop <= 1;
            @(posedge hClk); hStop <= 0;
        end
    endtask

    integer iter, t, fails = 0;
    initial begin
        wait (!xReset && !hReset);
        repeat (32) @(posedge xClk);

        begin : main_loop
        for (iter = 0; iter < ITERS; iter = iter + 1) begin
            // step 1: play B01 (loop-forever), let it run
            ncap = 0;
            start_fx(7'd48);
            t = 0; while (ncap < 6000 && t < 2000000) begin @(posedge xClk); t=t+1; end
            if (ncap < 6000) begin
                fails = fails + 1;
                $display("iter %0d: B01 stalled at %0d samples", iter, ncap);
            end
            // step 2: interrupt with A09 (count one-shot), let it end
            start_fx(7'd8);
            t = 0; while (!hPlaying && t < 2000000) begin @(posedge xClk); t=t+1; end
            t = 0; while ( hPlaying && t < 2000000) begin @(posedge xClk); t=t+1; end
            repeat (200) @(posedge hClk);
            // step 3: belt-and-braces stop (DUT idle here)
            stop_fx;
            // NOTE: NO breather here -- the sweep issues the next play ~2 hClk
            // after the stop (task call overhead only). A >=17 xClk gap hides
            // the race (the stop's echo settles before the play arrives).
            // step 4: fresh play A03 (enveloped); must emit samples
            ncap = 0;
            start_fx(7'd2);
            t = 0; while (ncap < 100 && t < 300000) begin @(posedge xClk); t=t+1; end
            if (ncap < 100) begin
                fails = fails + 1;
                $display("iter %0d: A03 HANG after %0d cyc (ncap=%0d)", iter, t, ncap);
                // dump state for diagnosis
                $display("  st=%0d req=%b cmdp=%b pplay=%b sdone=%b ack=%0d acklast=%0d",
                         dut.st, dut.req_out, dut.cmd_pend, dut.pend_play,
                         dut.seg_done, dut.ack_seq, dut.ack_last);
                $display("  run=%b flush=%b ninit=%b play=%b wbin=%0d rbin=%0d wbin_h=%0d",
                         dut.run_state, dut.flush_active, dut.need_init, hPlaying,
                         dut.wbin, dut.rbin, dut.wbin_h);
                $display("  segst=%0d segon=%b segblk=%0d segleft=%0d hsegcnt=%0d htick=%0d tickcnt=%0d",
                         dut.seg_st, dut.seg_on, dut.seg_blk, dut.seg_left,
                         dut.h_seg_cnt, dut.h_tick, dut.tick_cnt);
                $display("  dstate=%0d nib=%0d haveb=%b sbv=%b wbufv=%b rdp=%b segrem=%0d skip=%0d blkrem=%0d",
                         dut.dstate, dut.nib_i, dut.have_dbyte, dut.sb_v,
                         dut.wbuf_v, dut.rd_pend, dut.seg_rem, dut.skip_cnt, dut.blk_rem);
                disable main_loop;
            end
            // drain A03: stop it before the next iteration
            stop_fx;
            repeat (300) @(posedge xClk);
            if (iter % 25 == 0)
                $display("iter %0d OK (wbin=%0d rbin=%0d)", iter, dut.wbin, dut.rbin);
        end
        end // main_loop

        if (fails) $display("RESULT: FAIL (%0d hangs)", fails);
        else       $display("RESULT: PASS (%0d iterations)", ITERS);
        $finish;
    end

    initial begin #50_000_000_000; $display("GLOBAL TIMEOUT"); $finish; end
endmodule
