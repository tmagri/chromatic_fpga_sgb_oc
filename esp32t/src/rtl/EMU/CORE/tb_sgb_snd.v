// tb_sgb_snd.v -- verify sgb_snd's BRR decode against the real DKGB sample.
// Snoop-writes the Pauline-help scream BRR (sgb_pauline_help_scream.hex,
// from DKGBDisasm) into the sample cache at SAMPLE_BASE, strobes SOU_TRN
// twice, triggers playback (SOUND Z=1), captures every emitted sample and
// dumps tb_sgb_snd.raw for tb_check_snd.py, which compares SAMPLE-EXACTLY
// against a software BRR decode and against the rendered 11025 Hz WAV.
`timescale 1ns/1ps
module tb_sgb_snd;

    reg clk = 0;
    always #29.801 clk = ~clk;   // 16.777216 MHz hclk

    reg reset = 1, sgb_en = 1;
    initial begin reset = 1; #400; reset = 0; end

    reg        snd_trig = 0, snd_mute = 0, sou_trn_valid = 0;
    reg [7:0]  snd_id = 0, snd_z = 0;
    reg        sp_ce = 0, sp_wren = 0;
    reg [11:0] sp_addr = 0;
    reg [7:0]  sp_data = 0;
    wire [15:0] pcm_out;

    // TICK_PERIOD shrunk for sim speed: checks are sample-indexed, so the
    // decode rate is irrelevant.
    sgb_snd #(.TICK_PERIOD(12'd3)) dut (
        .clk_sys(clk), .reset(reset), .sgb_en(sgb_en),
        .snd_trig(snd_trig), .snd_id(snd_id), .snd_mute(snd_mute),
        .snd_z(snd_z), .sou_trn_valid(sou_trn_valid),
        .sp_ce(sp_ce), .sp_wren(sp_wren), .sp_addr(sp_addr), .sp_data(sp_data),
        .pcm_out(pcm_out)
    );

    // the uploaded BRR stream (2979 bytes = 331 blocks)
    localparam NB = 2979;
    reg [7:0] brr [0:NB-1];
    initial $readmemh("sgb_pauline_help_scream.hex", brr);

    // ---- capture: one sample per emit (S_PLAY & tick, 1-cycle delayed so
    // pcm_out already holds the freshly emitted value) ----
    localparam MAXS = 8192;
    reg signed [15:0] cap [0:MAXS-1];
    integer ncap = 0;
    wire emit = (dut.state == 2'd3) && dut.tick;
    reg emit_q = 0;
    always @(posedge clk) begin
        emit_q <= emit;
        if (emit_q && ncap < MAXS) begin
            cap[ncap] <= pcm_out;
            ncap <= ncap + 1;
        end
    end

    integer i, fh;
    initial begin
        // 1. VRAM-snoop the BRR bytes into the cache at SAMPLE_BASE (0x816)
        repeat (8) @(posedge clk);
        for (i = 0; i < NB; i = i + 1) begin
            @(negedge clk);
            sp_ce   <= 1'b1; sp_wren <= 1'b1;
            sp_addr <= 12'h816 + i[11:0];
            sp_data <= brr[i];
        end
        @(negedge clk); sp_ce <= 1'b0; sp_wren <= 1'b0;

        // 2. two SOU_TRN strobes freeze the cache (sample_ready)
        repeat (4) @(posedge clk);
        @(negedge clk); sou_trn_valid <= 1'b1;
        @(negedge clk); sou_trn_valid <= 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk); sou_trn_valid <= 1'b1;
        @(negedge clk); sou_trn_valid <= 1'b0;

        // 3. SOUND packet: Z=1 -> play from SAMPLE_BASE
        repeat (4) @(posedge clk);
        @(negedge clk); snd_trig <= 1'b1; snd_id <= 8'h00; snd_z <= 8'd1;
        @(negedge clk); snd_trig <= 1'b0;

        // 4. wait for the end flag: 500 quiet cycles after the last emit
        begin : WAIT_DONE
            integer quiet, guard;
            quiet = 0; guard = 0;
            while (quiet < 500 && guard < 400000) begin
                @(posedge clk);
                quiet = emit_q ? 0 : quiet + 1;
                guard = guard + 1;
            end
            if (guard >= 400000) $display("FAIL: playback never finished");
        end
        if (ncap == 0) $display("FAIL: no samples captured");

        fh = $fopen("tb_sgb_snd.raw", "w");
        for (i = 0; i < ncap; i = i + 1)
            $fwrite(fh, "%c%c", cap[i][7:0], cap[i][15:8]);
        $fclose(fh);
        $display("captured %0d samples (expect 5296 = 331 blocks x 16)", ncap);
        $display("DONE");
        $finish;
    end

endmodule
