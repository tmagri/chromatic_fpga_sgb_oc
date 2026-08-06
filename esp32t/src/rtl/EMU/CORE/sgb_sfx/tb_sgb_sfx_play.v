// tb_sgb_sfx_play.v -- verify sgb_sfx_play against the real bank + WAV refs.
// Models the PSRAM arbiter (request/grant/done, with realistic stall cycles)
// over a word memory loaded from bank7/sim_bank.hex, plays every effect, and
// dumps the decoded PCM so sgb_sfx/tb_check.py can compare it sample-exact
// against a software BRR decode of the SAME bank bytes, and correlation-wise
// against the rendered WAVs.
`timescale 1ns/1ps
module tb_sgb_sfx_play;

    localparam [22:0] BANK_BASE = 23'h100000;

    reg xClk = 0, hClk = 0;
    always #6.667  xClk = ~xClk;   // 75 MHz
    always #29.801 hClk = ~hClk;   // 16.777 MHz

    reg xReset = 1, hReset = 1;
    initial begin xReset = 1; hReset = 1; #400; xReset = 0; hReset = 0; end

    // ---- DUT interface ----
    wire        xRamReq_w;
    wire [22:0] xRamAddr_w;
    wire [10:0] xRamBurstLen_w;
    reg         xRamReady = 0;
    reg         xRamDone = 0, xRamDoutValid = 0;
    reg  [15:0] xRamDout = 0;
    reg         hStart = 0, hStop = 0;
    reg  [2:0]  hEffectIndex = 0;
    wire        hPcmValid;
    wire signed [15:0] hPcm;
    wire        hPlaying;

    // TICK_PERIOD shrunk for sim speed: comparison is sample-indexed, so the
    // decode rate is irrelevant; 3 -> tick every 4 hClk.
    sgb_sfx_play #(.BANK_BASE(BANK_BASE), .TICK_PERIOD(12'd3)) dut (
        .xClk(xClk), .xReset(xReset), .xRamReady(xRamReady),
        .xRamReq(xRamReq_w), .xRamRnW(), .xRamAddr(xRamAddr_w),
        .xRamDin(), .xRamBurstLen(xRamBurstLen_w),
        .xRamDone(xRamDone), .xRamDoutValid(xRamDoutValid), .xRamDout(xRamDout),
        .hClk(hClk), .hReset(hReset),
        .hStart(hStart), .hStop(hStop), .hEffectIndex(hEffectIndex),
        .hPcmValid(hPcmValid), .hPcm(hPcm), .hPlaying(hPlaying)
    );

    // ---- PSRAM contents: the uncompressed Kirby2 8-effect PCM bank ----
    localparam BANK_WORDS = 155808;                // 311616 bytes
    reg [15:0] psram [0:BANK_WORDS-1];
    initial $readmemh("sgb_sfx/bank_pcm/sim_bank.hex", psram);

    function [15:0] rd_word;
        input [22:0] a;
        integer w;
        begin
            w = (a - BANK_BASE) >> 1;
            rd_word = (w >= 0 && w < BANK_WORDS) ? psram[w] : 16'h0000;
        end
    endfunction

    // ---- arbiter / PSRAM read model ----
    // grant after a random latency, then stream words with random bubbles.
    localparam ARB_IDLE=0, ARB_LAT=1, ARB_RUN=2;
    reg [1:0]  arb_st = ARB_IDLE;
    reg [5:0]  arb_lat = 0;
    reg [22:0] rd_addr = 0;
    reg [10:0] rd_rem = 0;
    reg [1:0]  bubble = 0;

    always @(posedge xClk) begin
        if (xReset) begin
            arb_st <= ARB_IDLE; xRamDone <= 0; xRamDoutValid <= 0; rd_rem <= 0;
        end else begin
            xRamDoutValid <= 0;
            xRamDone      <= 0;
            case (arb_st)
            ARB_IDLE: begin
                if (xRamReq_w && xRamReady) begin
                    arb_st  <= ARB_LAT;
                    arb_lat <= $urandom_range(20, 2);   // arbiter/PSRAM latency
                    rd_addr <= xRamAddr_w;
                    rd_rem  <= xRamBurstLen_w;
                end
            end
            ARB_LAT: begin
                if (arb_lat == 0) arb_st <= ARB_RUN;
                else arb_lat <= arb_lat - 1;
            end
            ARB_RUN: begin
                if (bubble != 0) begin
                    bubble <= bubble - 1;
                end else if (rd_rem != 0) begin
                    xRamDout      <= rd_word(rd_addr);
                    xRamDoutValid <= 1;
                    rd_addr       <= rd_addr + 2;
                    rd_rem        <= rd_rem - 2;
                    if ($urandom_range(7,0) == 0) bubble <= $urandom_range(4,1);
                    if (rd_rem == 2) begin
                        xRamDone <= 1;
                        arb_st   <= ARB_IDLE;
                    end
                end
            end
            endcase
        end
    end

    // ---- capture buffer ----
    localparam MAXS = 16384*8;
    reg signed [15:0] pcm_buf [0:MAXS-1];
    integer ncap = 0;
    reg cap_en = 0;
    always @(posedge hClk) begin
        if (cap_en && hPcmValid && ncap < MAXS) begin
            pcm_buf[ncap] <= hPcm;
            ncap <= ncap + 1;
        end
    end

    task play_oneshot;
        input [2:0] idx;
        input integer timeout_ms;
        integer t, tmax;
        begin
            ncap = 0; cap_en = 1;
            tmax = timeout_ms * 75000;         // xClk cycles (75 MHz)
            @(posedge hClk); hEffectIndex <= idx; hStart <= 1;
            @(posedge hClk); hStart <= 0;
            t = 0;
            // wait for it to start, then for it to end on its own
            while (!hPlaying && t < tmax) begin @(posedge xClk); t=t+1; end
            while (hPlaying  && t < tmax) begin @(posedge xClk); t=t+1; end
            repeat (400) @(posedge hClk);      // drain
            cap_en = 0;
            if (t >= tmax)
                $display("FAIL fx%0d: timed out (playing=%b)", idx, hPlaying);
        end
    endtask

    task play_loop_for;
        input [2:0] idx;
        input integer nsamples;     // capture this many samples then stop
        input integer timeout_ms;
        integer t, tmax;
        begin
            ncap = 0; cap_en = 1;
            tmax = timeout_ms * 75000;         // xClk cycles (75 MHz)
            @(posedge hClk); hEffectIndex <= idx; hStart <= 1;
            @(posedge hClk); hStart <= 0;
            t = 0;
            while (ncap < nsamples && t < tmax) begin @(posedge xClk); t=t+1; end
            @(posedge hClk); hStop <= 1;
            @(posedge hClk); hStop <= 0;
            repeat (400) @(posedge hClk);      // drain / playing should drop
            cap_en = 0;
            if (hPlaying) $display("FAIL fx%0d: still playing after stop", idx);
            if (t >= tmax) $display("FAIL fx%0d: capture timed out", idx);
        end
    endtask

    integer fh, i;
    task dump_raw_from;
        input [255:0] path;
        input integer first;
        begin
            fh = $fopen(path, "w");
            for (i = first; i < ncap; i = i + 1)
                $fwrite(fh, "%c%c", pcm_buf[i][7:0], pcm_buf[i][15:8]);
            $fclose(fh);
            $display("  wrote %0s : %0d samples", path, ncap - first);
        end
    endtask
    task dump_raw;
        input [255:0] path;
        begin
            dump_raw_from(path, 0);
        end
    endtask

    // interrupt A mid-play with B; capture only B's samples (from mark)
    integer mark;
    task play_interrupt;
        input [2:0] idx_a;
        input [2:0] idx_b;
        input integer hold;        // samples of A before the interrupt
        input integer expect_b;
        input integer timeout_ms;
        integer t, tmax;
        begin
            ncap = 0; cap_en = 1;
            tmax = timeout_ms * 75000;
            @(posedge hClk); hEffectIndex <= idx_a; hStart <= 1;
            @(posedge hClk); hStart <= 0;
            t = 0;
            while (ncap < hold && t < tmax) begin @(posedge xClk); t=t+1; end
            @(posedge hClk); hEffectIndex <= idx_b; hStart <= 1; mark = ncap;
            @(posedge hClk); hStart <= 0;
            // the retriggered one-shot idx_b plays out in full; hPlaying is
            // already high (old stream) and falls when idx_b finishes. A few
            // old-stream tail samples leak into the capture before the flush
            // lands (retrigger CDC latency) -- tb_check.py aligns them out.
            while (hPlaying && t < tmax) begin @(posedge xClk); t=t+1; end
            repeat (4) @(posedge hClk);
            @(posedge hClk); hStop <= 1;
            @(posedge hClk); hStop <= 0;
            repeat (400) @(posedge hClk);
            cap_en = 0;
            if (ncap - mark < expect_b - 8)
                $display("FAIL int %0d->%0d: only %0d of %0d new samples",
                         idx_a, idx_b, ncap - mark, expect_b);
            if (hPlaying) $display("FAIL int %0d->%0d: still playing after stop", idx_a, idx_b);
            if (t >= tmax) $display("FAIL int %0d->%0d: timed out", idx_a, idx_b);
        end
    endtask

    // sample counts for the loop-run tests (see tb_check.py; 16 kHz)
    // B04 wind: first-pass ~13120 + loop tail; run enough to prove looping
    // heartbeat: catch hangs early
    integer hb = 0;
    always @(posedge xClk) begin
        hb <= hb + 1;
        if (hb % 1000000 == 0)
            $display("hb: xClk cycles=%0d simtime=%t", hb, $time);
    end

    initial begin
        $display("TB START");
        // hold the PSRAM port in reset/unready briefly to exercise the gate
        repeat (300) @(posedge xClk);
        xRamReady <= 1;
        repeat (100) @(posedge xClk);

        // Kirby Dream Land 2 bank:
        //   one-shots: 0=A1F 1=A26 2=A30
        //   loops:     3=B01 4=B04 5=B07 6=B08 7=B0B(Wave, loops full track)
        $display("== one-shot effects ==");
        play_oneshot(3'd0, 3000); dump_raw("sgb_sfx/bank_pcm/tb_fx0.raw");
        play_oneshot(3'd1, 3000); dump_raw("sgb_sfx/bank_pcm/tb_fx1.raw");
        play_oneshot(3'd2, 3000); dump_raw("sgb_sfx/bank_pcm/tb_fx2.raw");

        // capture far enough to cross the first-pass->loop boundary, which
        // exercises the odd-offset loop re-alignment (skip_wrap).
        $display("== loop effects (capture first pass + loop tail, then stop) ==");
        play_loop_for(3'd3, 12000, 3000); dump_raw("sgb_sfx/bank_pcm/tb_fx3.raw");
        play_loop_for(3'd4, 30000, 3000); dump_raw("sgb_sfx/bank_pcm/tb_fx4.raw");
        play_loop_for(3'd5, 14000, 3000); dump_raw("sgb_sfx/bank_pcm/tb_fx5.raw");
        play_loop_for(3'd6, 43000, 3000); dump_raw("sgb_sfx/bank_pcm/tb_fx6.raw");
        // B0B Wave: one pass is 44610 samples; capture past it to prove the
        // full-track loop restarts.
        play_loop_for(3'd7, 46000, 3000); dump_raw("sgb_sfx/bank_pcm/tb_fx7.raw");

        $display("== interrupt/retrigger ==");
        // one-shot (A1F) interrupted by one-shot (A30)
        play_interrupt(3'd0, 3'd2, 2000, 8770, 3000);
        dump_raw_from("sgb_sfx/bank_pcm/tb_fxR1.raw", mark);
        // looping effect (B04 Wind) interrupted by one-shot (A26)
        play_interrupt(3'd4, 3'd1, 20000, 13570, 3000);
        dump_raw_from("sgb_sfx/bank_pcm/tb_fxR2.raw", mark);

        $display("DONE");
        $finish;
    end

endmodule
