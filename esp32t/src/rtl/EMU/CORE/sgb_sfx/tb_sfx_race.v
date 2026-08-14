`timescale 1ns/1ps
// tb_sfx_race.v -- supersede-vs-burst-completion race regression for
// sgb_sfx_play.
//
// A stop command whose cmd_pend lands on the exact xClk where the current
// PSRAM burst completes must not lose the xRamDone pulse: ST_RX/ST_TABRX
// branch to ST_ABORT on cmd_pend and have to clear req_out in the same
// cycle, or the writer deadlocks in ST_ABORT waiting for a done that the
// arbiter already dropped (observed as a never-echoed stop).
//
// For each phase offset: reset, play the looping idx51 (B04), wait until
// the writer is mid-burst (ST_RX + req_out), wait `phase` cycles, inject
// hStop, and require the stop echo (ack_seq change) within 3000 xClk.
//
// Run from esp32t/src/rtl/EMU/CORE:
//   iverilog -g2001 -o /tmp/tb_race.out sgb_sfx/tb_sfx_race.v sgb_sfx_play.v
//   vvp /tmp/tb_race.out
module tb_sfx_race;
    localparam [22:0] BANK_BASE = 23'h100000;
    localparam BANK_WORDS = 31867;        // 63734 bytes / 2 (bank v6.1)
    localparam SEG_WBASE  = 25768;        // segment region word base (0xC950/2)

    reg xClk = 0, hClk = 0;
    always #6.667  xClk = ~xClk;
    always #29.801 hClk = ~hClk;
    reg xReset = 1, hReset = 1;

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
    integer i, w;
    initial begin
        $readmemh("sgb_sfx/build/sim_bank.hex", psram);
        // tick is record word 2; also patch every seg-entry next-tick word
        // (bank v6: entries are 3 words = blocks16, next_tick, next_amp)
        for (i = 0; i < 73; i = i + 1) psram[128 + i*8 + 2] = 16'd3;
        for (w = SEG_WBASE + 1; w < BANK_WORDS; w = w + 3) psram[w] = 16'd3;
    end

    function [15:0] rd_word; input [22:0] a; integer w;
        begin w = (a - BANK_BASE) >> 1;
              rd_word = (w >= 0 && w < BANK_WORDS) ? psram[w] : 16'h0000; end
    endfunction

    localparam ARB_IDLE=0, ARB_LAT=1, ARB_RUN=2;
    reg [1:0] arb_st = ARB_IDLE; reg [5:0] arb_lat = 0;
    reg [22:0] rd_addr = 0; reg [10:0] rd_rem = 0; reg [2:0] bubble = 0;
    always @(posedge xClk) begin
        if (xReset) begin arb_st <= ARB_IDLE; xRamDone <= 0; xRamDoutValid <= 0; rd_rem <= 0; end
        else begin
            xRamDoutValid <= 0; xRamDone <= 0;
            case (arb_st)
            ARB_IDLE: if (xRamReq_w && xRamReady) begin
                arb_st <= ARB_LAT; arb_lat <= 4;
                rd_addr <= xRamAddr_w; rd_rem <= xRamBurstLen_w;
            end
            ARB_LAT: if (arb_lat == 0) arb_st <= ARB_RUN; else arb_lat <= arb_lat - 1;
            ARB_RUN: begin
                if (bubble != 0) bubble <= bubble - 1;
                else if (rd_rem != 0) begin
                    xRamDout <= rd_word(rd_addr); xRamDoutValid <= 1;
                    rd_addr <= rd_addr + 2; rd_rem <= rd_rem - 2;
                    if (rd_rem == 2) begin xRamDone <= 1; arb_st <= ARB_IDLE; end
                end
            end
            endcase
        end
    end

    integer fails = 0, phase, t;
    reg [1:0] ack0;

    initial begin
        // reset pulse before each phase
        for (phase = 0; phase < 800; phase = phase + 1) begin
            xReset = 1; hReset = 1; hStart = 0; hStop = 0;
            #400;
            xReset = 0; hReset = 0;
            repeat (32) @(posedge xClk);

            // play idx51 (B04, loop-forever)
            @(posedge hClk); hEffectIndex <= 7'd51; hStart <= 1;
            @(posedge hClk); hStart <= 0;

            // wait for the writer to be mid-burst with req_out
            t = 0;
            while (!(dut.st == 3'd6 && dut.req_out) && t < 20000) begin
                @(posedge xClk); t = t + 1;
            end
            if (t >= 20000) begin
                $display("phase %0d: writer never reached ST_RX", phase);
                fails = fails + 1;
            end else begin
                repeat (phase) @(posedge xClk);
                ack0 = dut.ack_seq;
                @(posedge hClk); hStop <= 1;
                @(posedge hClk); hStop <= 0;
                // stop must be echoed within 3000 xClk
                t = 0;
                while (dut.ack_seq == ack0 && t < 3000) begin
                    @(posedge xClk); t = t + 1;
                end
                if (t >= 3000) begin
                    $display("phase %0d: DEADLOCK (st=%0d req_out=%b cmd_pend=%b ack=%0d->%0d)",
                             phase, dut.st, dut.req_out, dut.cmd_pend, ack0, dut.ack_seq);
                    fails = fails + 1;
                end
            end
        end
        // ---- same race, but against the bank-v5 segment-table fetch ----
        // A03 (idx2) has 114 envelope entries, so the writer runs a long
        // ST_SEGRX burst before the echo; a stop landing mid-seg-fetch must
        // also drain and echo without deadlock.
        for (phase = 0; phase < 800; phase = phase + 1) begin
            xReset = 1; hReset = 1; hStart = 0; hStop = 0;
            #400;
            xReset = 0; hReset = 0;
            repeat (32) @(posedge xClk);

            @(posedge hClk); hEffectIndex <= 7'd2; hStart <= 1;
            @(posedge hClk); hStart <= 0;

            t = 0;
            while (!(dut.st == 4'd9 && dut.req_out) && t < 20000) begin
                @(posedge xClk); t = t + 1;
            end
            if (t >= 20000) begin
                $display("seg phase %0d: writer never reached ST_SEGRX", phase);
                fails = fails + 1;
            end else begin
                repeat (phase) @(posedge xClk);
                ack0 = dut.ack_seq;
                @(posedge hClk); hStop <= 1;
                @(posedge hClk); hStop <= 0;
                t = 0;
                while (dut.ack_seq == ack0 && t < 3000) begin
                    @(posedge xClk); t = t + 1;
                end
                if (t >= 3000) begin
                    $display("seg phase %0d: DEADLOCK (st=%0d req_out=%b cmd_pend=%b ack=%0d->%0d)",
                             phase, dut.st, dut.req_out, dut.cmd_pend, ack0, dut.ack_seq);
                    fails = fails + 1;
                end
            end
        end

        if (fails) $display("RESULT: FAIL (%0d deadlocks)", fails);
        else       $display("RESULT: PASS (1600 phases, all stops echoed)");
        $finish;
    end

    initial begin
        #4_000_000_000;
        $display("GLOBAL TIMEOUT");
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
