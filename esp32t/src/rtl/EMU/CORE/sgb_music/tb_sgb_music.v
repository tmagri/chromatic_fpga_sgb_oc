// tb_sgb_music.v -- self-checking smoke test for sgb_music.v.
// Serves build/sim_bank.hex from a fake PSRAM (fixed read latency), starts
// song 0, watches the sequencer/voices/mixer run, then stops.
`timescale 1ns / 1ps

module tb_sgb_music;
    `include "build/sgb_music_bank.vh"

    localparam T_X = 14.9;   // 67.1 MHz
    localparam T_H = 59.6;   // 16.78 MHz
    localparam LAT = 6;      // fake PSRAM read latency (xClk cycles)

    reg xClk = 0, hClk = 0;
    always #(T_X / 2.0) xClk = ~xClk;
    always #(T_H / 2.0) hClk = ~hClk;

    reg [22:0] mem [0:(`SGB_MUSIC_BANK_BYTES / 2 - 1)];
    initial begin
        $readmemh("build/sim_bank.hex", mem);
    end

    // ---------------- fake arbiter ----------------------------------------
    reg         xReset = 1, hReset = 1;
    reg         hStart = 0, hStop = 0;
    reg  [2:0]  hSong = 0;

    wire        xRamReq, xRamRnW, xRamDone_t, xRamDoutValid_t;
    wire [22:0] xRamAddr;
    wire [15:0] xRamDin, xRamDout_t;
    wire [10:0] xRamBurstLen;

    reg [4:0] lat_cnt = 0;
    reg [10:0] lat_rem = 0;
    reg [22:0] lat_addr = 0;
    reg        lat_busy = 0;
    assign xRamDoutValid_t = lat_busy && (lat_cnt == 0);
    assign xRamDout_t = mem[lat_addr[22:1]];
    assign xRamDone_t  = lat_busy && xRamDoutValid_t && (lat_rem == 11'd2);

    always @(posedge xClk) begin
        if (xReset) begin
            lat_busy <= 0;
        end else if (!lat_busy && xRamReq) begin
            lat_busy  <= 1;
            lat_cnt   <= LAT;
            lat_rem   <= xRamBurstLen;
            lat_addr  <= xRamAddr;
        end else if (lat_busy) begin
            if (lat_cnt != 0) lat_cnt <= lat_cnt - 5'd1;
            else begin
                lat_addr <= lat_addr + 23'd2;
                if (lat_rem == 11'd2) lat_busy <= 0;
                else lat_rem <= lat_rem - 11'd2;
            end
        end
    end

    // ---------------- DUT ---------------------------------------------------
    wire hPcmValid, hPlaying;
    wire signed [15:0] hPcmL, hPcmR;

    sgb_music #(
        .BANK_BASE(23'h000000)
    ) dut (
        .xClk(xClk), .xReset(xReset), .xRamReady(1'b1),
        .xRamReq(xRamReq), .xRamRnW(xRamRnW), .xRamAddr(xRamAddr),
        .xRamDin(xRamDin), .xRamBurstLen(xRamBurstLen),
        .xRamDone(xRamDone_t), .xRamDoutValid(xRamDoutValid_t),
        .xRamDout(xRamDout_t),
        .hClk(hClk), .hReset(hReset),
        .hStart(hStart), .hStop(hStop), .hSong(hSong),
        .hPcmValid(hPcmValid), .hPcmL(hPcmL), .hPcmR(hPcmR),
        .hPlaying(hPlaying)
    );

    // ---------------- scoreboard --------------------------------------------
    integer n_pairs = 0;
    integer n_nonzero = 0;
    always @(posedge hClk) begin
        if (hPcmValid) begin
            n_pairs <= n_pairs + 1;
            if (hPcmL != 0 || hPcmR != 0) n_nonzero <= n_nonzero + 1;
        end
    end

    initial begin
        // start song 0
        #(T_X * 20);
        xReset = 0; hReset = 0;
        #(T_H * 20);
        @(posedge hClk);
        hStart <= 1; @(posedge hClk); hStart <= 0;

        // wait for playback to begin
        fork : wait_play
            begin
                repeat (400000) @(posedge hClk);
                $display("FAIL: hPlaying never asserted");
                $finish;
            end
            begin
                @(posedge hPlaying);
                disable wait_play;
            end
        join
        $display("PASS: playback started (hPlaying=1)");

        // let it run ~300 ms of audio
        repeat (300000000 / T_H) @(posedge hClk);

        if (n_pairs < 2000) begin
            $display("FAIL: only %0d sample pairs emitted", n_pairs);
            $finish;
        end
        $display("PASS: %0d sample pairs emitted", n_pairs);
        if (n_nonzero < 100) begin
            $display("FAIL: only %0d nonzero pairs (voices silent?)",
                     n_nonzero);
            $finish;
        end
        $display("PASS: %0d nonzero pairs (audio present)", n_nonzero);

        // stop command
        @(posedge hClk);
        hStop <= 1; @(posedge hClk); hStop <= 0;
        fork : wait_stop
            begin
                repeat (600000) @(posedge hClk);
                $display("FAIL: hPlaying never deasserted after hStop");
                $finish;
            end
            begin
                @(negedge hPlaying);
                disable wait_stop;
            end
        join
        $display("PASS: hStop ended playback");
        $display("ALL TESTS PASSED");
        $finish;
    end

endmodule
