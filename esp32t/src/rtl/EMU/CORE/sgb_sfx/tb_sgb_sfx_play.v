// tb_sgb_sfx_play.v -- verify sgb_sfx_play BRR decode against known-good WAVs
// Models the PSRAM arbiter with a byte memory loaded from sim_bank.hex.
`timescale 1ns/1ps
module tb_sgb_sfx_play;

    // xClk ~75MHz -> 6.667ns; hClk ~16.777MHz -> 29.802ns
    reg xClk = 0, hClk = 0;
    always #3.3335 xClk = ~xClk;     // ~75 MHz
    always #14.901 hClk = ~hClk;     // ~16.777 MHz

    reg xReset = 1, hReset = 1;
    initial begin
        xReset = 1; hReset = 1;
        #200; xReset = 0; hReset = 0;
    end

    // PSRAM model: word-addressable memory loaded with the bank.
    localparam BANK_WORDS = 29656;
    reg [15:0] psram [0:BANK_WORDS-1];
    initial $readmemh("sgb_sfx/bank7/sim_bank.hex", psram);

    // arbiter model: respond to xRamReq with a burst of words
    reg        xRamReq_w;
    reg [22:0] xRamAddr_w;
    reg [10:0] xRamBurstLen_w;
    reg        xRamDone, xRamDoutValid;
    reg [15:0] xRamDout;

    sgb_sfx_play dut (
        .xClk(xClk), .xReset(xReset),
        .xRamReq(xRamReq_w), .xRamRnW(), .xRamAddr(xRamAddr_w),
        .xRamDin(), .xRamBurstLen(xRamBurstLen_w),
        .xRamDone(xRamDone), .xRamDoutValid(xRamDoutValid), .xRamDout(xRamDout),
        .hClk(hClk), .hReset(hReset),
        .hStart(hStart), .hEffectIndex(hEffectIndex),
        .hPcmValid(hPcmValid), .hPcm(hPcm), .hPlaying(hPlaying)
    );

    reg hStart = 0;
    reg [2:0] hEffectIndex = 0;

    // arbiter behavioural model
    reg [10:0] burst_rem;
    reg [22:0] rd_addr;
    reg        responding;
    initial begin responding=0; xRamDone=0; xRamDoutValid=0; xRamDout=0; burst_rem=0; rd_addr=0; end
    always @(posedge xClk) begin
        if (xReset) begin
            responding<=0; xRamDone<=0; xRamDoutValid<=0; burst_rem<=0;
        end else begin
            xRamDoutValid <= 0;
            xRamDone      <= 0;
            if (!responding && xRamReq_w) begin
                responding <= 1;
                rd_addr    <= xRamAddr_w;
                burst_rem  <= xRamBurstLen_w;
            end else if (responding) begin
                if (burst_rem >= 2) begin
                    xRamDout      <= psram[rd_addr[22:1]];
                    xRamDoutValid <= 1;
                    rd_addr       <= rd_addr + 2;
                    burst_rem     <= burst_rem - 2;
                    if (burst_rem == 2) begin
                        responding <= 0;
                        xRamDone   <= 1;
                    end
                end else begin
                    responding <= 0;
                    xRamDone   <= 1;
                end
            end
        end
    end

    // capture PCM into an array, dump to file at end
    localparam MAXS = 16384*4;
    reg signed [15:0] pcm_buf [0:MAXS-1];
    integer ncap = 0;
    always @(posedge hClk) begin
        if (hPcmValid && ncap < MAXS) begin
            pcm_buf[ncap] <= hPcm;
            ncap <= ncap + 1;
        end
    end

    integer i;
    integer fh;
    initial begin
        // play effect index 5 (B04 Wind, a long loop)
        #500;
        @(posedge hClk); hEffectIndex <= 5; hStart <= 1;
        @(posedge hClk); hStart <= 0;
        // let it play ~1.5s of audio
        repeat (16000*3/2) @(posedge hClk);
        // dump captured PCM as raw signed-16 for offline compare
        fh = $fopen("sgb_sfx/bank7/tb_pcm_out.raw","w");
        for (i=0;i<ncap;i=i+1) $fwrite(fh, "%c%c", pcm_buf[i][7:0], pcm_buf[i][15:8]);
        $fclose(fh);
        $display("captured %0d samples", ncap);
        $finish;
    end

endmodule
