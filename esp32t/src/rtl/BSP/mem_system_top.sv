// mem_system_top.v

module mem_system_top #(parameter ISSIMU=0)
(
    input               xClk,
    input               fClk,
    input               hClk,
    input               reset,
    
    input               QSPI_CLK,
    input               QSPI_MOSI,
    input               QSPI_MISO,
    input               QSPI_CS,
    input               QSPI_WP,
    input               QSPI_HD,

    output              PS_CE_N,
    output              PS_CLK,
    inout   [7:0]       PS_DQ,
    inout               PS_DQS,
    
    input               hGBNewLine,
    input   [22:0]      hGBAddress,
    input               hGBWrite,
    input   [15:0]      hGBData,
    
    input               hValid,
    input               hHsync,
    input               hVsync,
    output              qMenuInit,
    output  [15:0]      hWrBurstQ,
    output  [15:0]      hWrBurstQ2,
    
    output              BIST_failed,
    output              BIST_finished,

    // SGB built-in SFX bank playback engine (sgb_sfx_play). Control and PCM
    // live in the hClk (audio) domain; the PSRAM streaming happens internally
    // on xClk through the 6th arbiter port.
    input               hSfxStart,
    input               hSfxStop,
    input   [2:0]       hSfxIndex,
    output signed [15:0] hSfxPcm,
    output              hSfxPcmValid,
    output              hSfxPlaying,

    // SGB N-SPC music engine (sgb_music): sequence + BRR bank lives in
    // PSRAM, streamed on the 7th arbiter port; stereo PCM out in hClk.
    input               hMusicStart,
    input               hMusicStop,
    input   [2:0]       hMusicSong,
    output signed [15:0] hMusicPcmL,
    output signed [15:0] hMusicPcmR,
    output              hMusicPcmValid,
    output              hMusicPlaying
);
    
    localparam RAMPORTCOUNT = 7;
    localparam RAMPORT_BIST = 0;
    localparam RAMPORT_QSPI = 1;
    localparam RAMPORT_FBRD = 2;
    localparam RAMPORT_FBWR = 3;
    localparam RAMPORT_FBRDOSD = 4;
    localparam RAMPORT_SFX = 5;
    localparam RAMPORT_MUSIC = 6;
    
    typedef logic tRAMIn_request     [RAMPORTCOUNT];
    typedef logic tRAMIn_RnW         [RAMPORTCOUNT];
    typedef logic [22:0] tRAMIn_addr        [RAMPORTCOUNT];
    typedef logic [15:0] tRAMIn_din         [RAMPORTCOUNT];
    typedef logic [10:0] tRAMIn_burst_length[RAMPORTCOUNT];
                  
    typedef logic tRAMOut_writeNext   [RAMPORTCOUNT];
    typedef logic tRAMOut_done        [RAMPORTCOUNT];
    typedef logic tRAMOut_dout_valid  [RAMPORTCOUNT];

    tRAMIn_request      RAMIn_request;     
    tRAMIn_RnW          RAMIn_RnW;     
    tRAMIn_addr         RAMIn_addr;     
    tRAMIn_din          RAMIn_din;     
    tRAMIn_burst_length RAMIn_burst_length; 
                            
    tRAMOut_writeNext   RAMOut_writeNext; 
    tRAMOut_done        RAMOut_done; 
    tRAMOut_dout_valid  RAMOut_dout_valid; 
   
    wire BIST_req_read;
    wire BIST_req_write;
    assign RAMIn_request[RAMPORT_BIST] = BIST_req_read | BIST_req_write;
    assign RAMIn_RnW[RAMPORT_BIST]     = BIST_req_read;
    
    assign RAMIn_RnW[RAMPORT_QSPI]          = 1'b0;
    assign RAMIn_burst_length[RAMPORT_QSPI] = 11'd1024;
    
    assign RAMIn_RnW[RAMPORT_FBRD]          = 1'b1;
    assign RAMIn_burst_length[RAMPORT_FBRD] = 11'd320;
    assign RAMIn_din[RAMPORT_FBRD] = 16'd0;   

    assign RAMIn_RnW[RAMPORT_FBRDOSD]          = 1'b1;
    assign RAMIn_burst_length[RAMPORT_FBRDOSD] = 11'd320;
    assign RAMIn_din[RAMPORT_FBRDOSD] = 16'd0;   

    assign RAMIn_RnW[RAMPORT_FBWR]          = 1'b0;
    assign RAMIn_burst_length[RAMPORT_FBWR] = 11'd320;

    wire RAM_ready;
    wire [15:0] RAM_dout;    
    
    MultiPortRamCtrl #(ISSIMU)
    iMultiPortRamCtrl
    (
       .clk_sys            (xClk),      
       .clk_fsys           (fClk),
       .rst                (reset),      
       
       .RAMIn_request      (RAMIn_request),  
       .RAMIn_RnW          (RAMIn_RnW),         
       .RAMIn_addr         (RAMIn_addr),        
       .RAMIn_din          (RAMIn_din),         
       .RAMIn_burst_length (RAMIn_burst_length),
            
       .RAMOut_writeNext   (RAMOut_writeNext),   
       .RAMOut_done        (RAMOut_done),        
       .RAMOut_dout_valid  (RAMOut_dout_valid),  
 
       .ram_ready          (RAM_ready), 
       .ram_dout           (RAM_dout),  
            
       .psram_clk          (PS_CLK), 
       .psram_cs_n         (PS_CE_N),
       .psram_rwds         (PS_DQS),
       .psram_dq           (PS_DQ)
    );
    
    PSRAMBIST_Burst #(
        .BIST_BURSTLENGTH(512), 
        .SHORTTEST(1)
        )
    iPSRAMBIST_Burst
    (
       .clk                  (xClk),  
       .rst                  (reset),
                                    
       .test_finished        (BIST_finished),   
       .test_failed          (BIST_failed),
                                    
       .ram_req_read         (BIST_req_read),
       .ram_req_write        (BIST_req_write),
       .ram_addr             (RAMIn_addr[RAMPORT_BIST]),    
       .ram_din              (RAMIn_din[RAMPORT_BIST]),
       .burst_length         (RAMIn_burst_length[RAMPORT_BIST]),
       .ram_ready            (RAM_ready),
       .ram_writeNext        (RAMOut_writeNext[RAMPORT_BIST]),
       .ram_done             (RAMOut_done[RAMPORT_BIST]),
       .ram_dout             (RAM_dout),
       .ram_dout_valid       (RAMOut_dout_valid[RAMPORT_BIST])
    );  

    wire [15:0] qData;
    wire [31:0] qAddress;
    wire qDataValid;

    QSPI_Slave u_QSPI_Slave(
        .QSPI_CLK(QSPI_CLK),
        .QSPI_CS(QSPI_CS),
        .QSPI_MOSI(QSPI_MOSI),
        .QSPI_MISO(QSPI_MISO),
        .QSPI_WP(QSPI_WP),
        .QSPI_HD(QSPI_HD),
        
        .qMenuInit(qMenuInit),
        .qDataValid(qDataValid),
        .qData(qData),
        .qAddress(qAddress)
    );

    mm_burst_write u_mm_burst_write(
        .QSPI_CLK(QSPI_CLK),
        .QSPI_CS(QSPI_CS),
        .qAddress(qAddress),
        .qDataValid(qDataValid),
        .qData(qData),

        .xClk(xClk),
        // Standard FIFO uses xRegWrite here (not FWFT FIFO)
        .xRdEn(RAMOut_writeNext[RAMPORT_QSPI]),
        .xRamReady(BIST_finished),
        .xMcuReqWrite(RAMIn_request[RAMPORT_QSPI]),
        .xDout(RAMIn_din[RAMPORT_QSPI]),
        .xAddress(RAMIn_addr[RAMPORT_QSPI])
    );
    
    mm_burst_read_to_stream #(
        .base_pointer(23'h10000)
    )   u_mm_burst_read_to_stream(
        .hClk(hClk),
        .hVsync(hVsync),
        .hHsync(hHsync),
        .hValid(hValid),
        .xClk(xClk),
        
        .xRamReady(BIST_finished),
        .xStreamValid(RAMOut_dout_valid[RAMPORT_FBRD]),
        .xStreamData(RAM_dout),
        .xWrBurstDone(RAMOut_done[RAMPORT_FBRD]),
        .xGbReqRead(RAMIn_request[RAMPORT_FBRD]),
        
        .hWrBurstQ(hWrBurstQ),
        .xGbAddress(RAMIn_addr[RAMPORT_FBRD])
    );

    reg hVsync_r1;
    reg hHsync_r1;
    reg hValid_r1;
    always@(posedge hClk)
        begin
        hVsync_r1 <= hVsync;
        hHsync_r1 <= hHsync;
        hValid_r1 <= hValid;
    end

    mm_burst_read_to_stream #(
        .base_pointer(23'h0)
    ) u_mm_burst_read_to_stream_osd(
        .hClk(hClk),
        .hVsync(hVsync_r1),
        .hHsync(hHsync_r1),
        .hValid(hValid_r1),
        .xClk(xClk),
                
        .xRamReady(BIST_finished),
        .xStreamValid(RAMOut_dout_valid[RAMPORT_FBRDOSD]),
        .xStreamData(RAM_dout),
        .xWrBurstDone(RAMOut_done[RAMPORT_FBRDOSD]),
        .xGbReqRead(RAMIn_request[RAMPORT_FBRDOSD]),
        
        .hWrBurstQ(hWrBurstQ2),
        .xGbAddress(RAMIn_addr[RAMPORT_FBRDOSD])
    );

    
    gb_burst_write u_gb_burst_write(
        .hClk(hClk),
        .hVsync(hVsync),
        .hNewLine(hGBNewLine),
        .hAddress(hGBAddress),
        .hWrite(hGBWrite),
        .hData(hGBData),

        .xClk(xClk),
        .xRdEn(RAMOut_writeNext[RAMPORT_FBWR]),
        .xRamReady(BIST_finished),
        .xMcuReqWrite(RAMIn_request[RAMPORT_FBWR]),
        .xDout(RAMIn_din[RAMPORT_FBWR]),
        .xAddress(RAMIn_addr[RAMPORT_FBWR])
    );

    // SGB built-in SFX bank playback: streams the pre-rendered BRR bank from
    // PSRAM (BANK_BASE, see sgb_sfx_play.v) on the 6th arbiter port and decodes
    // it to PCM in the hClk domain. Ready is gated on BIST_finished like the
    // other PSRAM clients.
    sgb_sfx_play u_sgb_sfx_play(
        .xClk(xClk),
        .xReset(reset),
        .xRamReady(BIST_finished),

        .xRamReq(RAMIn_request[RAMPORT_SFX]),
        .xRamRnW(RAMIn_RnW[RAMPORT_SFX]),
        .xRamAddr(RAMIn_addr[RAMPORT_SFX]),
        .xRamDin(RAMIn_din[RAMPORT_SFX]),
        .xRamBurstLen(RAMIn_burst_length[RAMPORT_SFX]),
        .xRamDone(RAMOut_done[RAMPORT_SFX]),
        .xRamDoutValid(RAMOut_dout_valid[RAMPORT_SFX]),
        .xRamDout(RAM_dout),

        .hClk(hClk),
        .hReset(reset),
        .hStart(hSfxStart),
        .hStop(hSfxStop),
        .hEffectIndex(hSfxIndex),

        .hPcmValid(hSfxPcmValid),
        .hPcm(hSfxPcm),
        .hPlaying(hSfxPlaying)
    );

    // SGB N-SPC music bank playback: streams the MSGB bank (tracker
    // sequence + BRR samples, BANK_BASE inside sgb_music.v) from PSRAM on
    // the 7th arbiter port, decodes to stereo PCM in the hClk domain.
    sgb_music u_sgb_music(
        .xClk(xClk),
        .xReset(reset),
        .xRamReady(BIST_finished),

        .xRamReq(RAMIn_request[RAMPORT_MUSIC]),
        .xRamRnW(RAMIn_RnW[RAMPORT_MUSIC]),
        .xRamAddr(RAMIn_addr[RAMPORT_MUSIC]),
        .xRamDin(RAMIn_din[RAMPORT_MUSIC]),
        .xRamBurstLen(RAMIn_burst_length[RAMPORT_MUSIC]),
        .xRamDone(RAMOut_done[RAMPORT_MUSIC]),
        .xRamDoutValid(RAMOut_dout_valid[RAMPORT_MUSIC]),
        .xRamDout(RAM_dout),

        .hClk(hClk),
        .hReset(reset),
        .hStart(hMusicStart),
        .hStop(hMusicStop),
        .hSong(hMusicSong),

        .hPcmValid(hMusicPcmValid),
        .hPcmL(hMusicPcmL),
        .hPcmR(hMusicPcmR),
        .hPlaying(hMusicPlaying)
    );

endmodule
