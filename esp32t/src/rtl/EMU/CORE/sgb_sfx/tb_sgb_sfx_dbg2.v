`timescale 1ns/1ps
// tb_sgb_sfx_dbg2.v -- focused probe: play B19 (idx 72, forced full-track
// loop: brr=loop=0x9d65, 109 blocks, flags=3). The play TB shows the first
// pass bit-exact and the loop pass all zeros. Dump every FIFO write and the
// reader's segment state to find where the zeros come from.
module tb_sgb_sfx_dbg2;
    localparam [22:0] BANK_BASE = 23'h100000;
    localparam BANK_WORDS = 31867;        // 63734 bytes / 2 (bank v6.1)
    localparam SEG_WBASE  = 25768;        // segment region word base (0xC950/2)

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

    // deterministic arbiter: fixed latency, no bubbles (reproducible trace)
    localparam ARB_IDLE=0, ARB_LAT=1, ARB_RUN=2;
    reg [1:0] arb_st = ARB_IDLE; reg [5:0] arb_lat = 0;
    reg [22:0] rd_addr = 0; reg [10:0] rd_rem = 0;
    always @(posedge xClk) begin
        if (xReset) begin arb_st <= ARB_IDLE; xRamDone <= 0; xRamDoutValid <= 0; rd_rem <= 0; end
        else begin
            xRamDoutValid <= 0; xRamDone <= 0;
            case (arb_st)
            ARB_IDLE: if (xRamReq_w && xRamReady) begin
                arb_st <= ARB_LAT; arb_lat <= 6'd3;
                rd_addr <= xRamAddr_w; rd_rem <= xRamBurstLen_w;
                $display("[%0t] REQ addr=%06x len=%0d", $time, xRamAddr_w, xRamBurstLen_w);
            end
            ARB_LAT: if (arb_lat == 0) arb_st <= ARB_RUN; else arb_lat <= arb_lat - 1;
            ARB_RUN: if (rd_rem != 0) begin
                xRamDout <= rd_word(rd_addr); xRamDoutValid <= 1;
                rd_addr <= rd_addr + 2; rd_rem <= rd_rem - 2;
                if (rd_rem == 2) begin xRamDone <= 1; arb_st <= ARB_IDLE; end
            end
            endcase
        end
    end

    // ---- probes ----
    integer ncap = 0;
    always @(posedge hClk) begin
        if (hPcmValid) begin
            ncap <= ncap + 1;
            if (ncap >= 1736 && ncap < 1760)   // around the 1744 boundary
                $display("[%0t] sample #%0d pcm=%04x seg_rem=%0d inloop=%b skip=%0d bphase=%b wbuf_v=%b dstate=%0d nib=%0d",
                         $time, ncap, hPcm, dut.seg_rem, dut.in_loop_seg, dut.skip_cnt,
                         dut.bphase, dut.wbuf_v, dut.dstate, dut.nib_i);
        end
    end
    // writer FSM transitions of interest
    always @(posedge xClk) begin
        if (dut.st == 4'd5 && dut.xRamReq)     // ST_REQ issuing a burst
            $display("[%0t] BURST addr=%06x bytes=%0d seg_loop=%b rem_w=%0d",
                     $time, dut.xRamAddr, dut.xRamBurstLen, dut.seg_loop, dut.rem_w);
        if (dut.st == 4'd4)                    // ST_SEG (re)load
            $display("[%0t] ST_SEG seg_loop=%b seg_base=%06x seg_remw=%0d seg_len=%0d",
                     $time, dut.seg_loop, dut.seg_base, dut.seg_remw, dut.seg_len);
    end
    // FIFO write stream around the loop-pass boundary (wbin 488..520)
    always @(posedge xClk) begin
        if (dut.xWrEn && dut.wbin >= 488 && dut.wbin <= 520)
            $display("[%0t] WRITE wbin=%0d data=%04x", $time, dut.wbin, dut.xRamDout);
    end
    // FIFO read stream around the boundary (rbin 488..520)
    always @(posedge hClk) begin
        if (dut.consume && dut.rbin >= 488 && dut.rbin <= 520)
            $display("[%0t] READ  rbin=%0d wbuf=%04x (wbin_h=%0d avail=%b)",
                     $time, dut.rbin, dut.wbuf, dut.wbin_h, dut.words_avail);
    end

    initial begin
        wait (!xReset && !hReset);
        repeat (32) @(posedge xClk);
        @(posedge hClk); hEffectIndex <= 7'd72; hStart <= 1;
        @(posedge hClk); hStart <= 0;
        wait (ncap >= 1760);
        $display("DONE ncap=%0d st=%0d seg_loop=%b wbin=%0d rbin=%0d", ncap, dut.st, dut.seg_loop, dut.wbin, dut.rbin);
        $finish;
    end
    initial begin #200_000_000; $display("TIMEOUT ncap=%0d st=%0d", ncap, dut.st); $finish; end
endmodule
