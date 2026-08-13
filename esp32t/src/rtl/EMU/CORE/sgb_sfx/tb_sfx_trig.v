`timescale 1ns/1ps
// tb_sfx_trig.v -- verify gb.v's CMD_SOUND -> sfx_start/index/stop trigger path.
// This replicates (verbatim) the SFX trigger logic in gb.v so it can be
// exercised without the whole GB SoC. It drives SOUND packets (byte_cnt 1..4)
// exactly as the packet engine presents them and checks sfx_start/index/stop.
//
// Full 73-effect bank map (record v4 / sgb_sfx_play.v):
//   SFX-A num 0x01..0x30 -> index num-1       (0..47)
//   SFX-B num 0x01..0x19 -> index 47+num      (48..72)
//   B valid beats A; bit 7 of the aimed byte stops the playing channel.
module tb_sfx_trig;

    reg clk = 0;
    always #10 clk = ~clk;

    // ---- replicated trigger state (mirrors gb.v) ----
    reg       isSGB = 1'b1;
    reg [4:0] byte_cnt = 0;
    reg [7:0] data = 0;
    reg       byte_done = 0;

    reg       sfx_start_r = 1'b0;
    reg       sfx_stop_r  = 1'b0;
    reg [6:0] sfx_index_r = 7'd0;
    reg       sfx_is_b    = 1'b0;
    reg [7:0] sfx_a_num   = 8'd0;
    reg [7:0] sfx_b_num   = 8'd0;
    wire      sfx_start = sfx_start_r;
    wire      sfx_stop  = sfx_stop_r;
    wire [6:0] sfx_index = sfx_index_r;

    wire [6:0] sfx_a_idx  = sfx_a_num[6:0] - 7'd1;
    wire       sfx_hit_a  = (sfx_a_num[6:0] >= 7'h01) && (sfx_a_num[6:0] <= 7'h30);
    wire [6:0] sfx_b_idx  = 7'd47 + sfx_b_num[6:0];
    wire       sfx_hit_b  = (sfx_b_num[6:0] >= 7'h01) && (sfx_b_num[6:0] <= 7'h19);

    // ---- replicated CMD_SOUND handling, clocked like gb.v's packet engine ----
    reg reset_ss = 0;
    always @(posedge clk) begin
        if (reset_ss) begin
            // gb.v's packet-engine reset branch: stop the SFX engine and
            // clear the latched SOUND state. The engine lives in
            // mem_system_top on the memrst domain, which this reset does
            // not reach, so looping SFX-B ambience (e.g. B0B Wave) would
            // otherwise keep playing after Everdrive-menu exit / reset.
            sfx_stop_r <= 1'b1;
            sfx_a_num  <= 8'd0;
            sfx_b_num  <= 8'd0;
            sfx_is_b   <= 1'b0;
        end else begin
            // per-cycle default pulse clears (gb.v does this at the top of the ce block)
            sfx_start_r <= 0;
            sfx_stop_r  <= 0;
            if (byte_done) begin
                if (isSGB) begin
                    if (byte_cnt == 5'd1) sfx_a_num <= data;
                    if (byte_cnt == 5'd2) sfx_b_num <= data;
                    if (byte_cnt == 5'd4) begin
                        if ((sfx_a_num[7] && !sfx_is_b) ||
                            (sfx_b_num[7] &&  sfx_is_b)) begin
                            sfx_stop_r  <= 1'b1;
                        end else if (!sfx_b_num[7] && sfx_hit_b) begin
                            sfx_start_r <= 1'b1;
                            sfx_index_r <= sfx_b_idx;
                            sfx_is_b    <= 1'b1;
                        end else if (!sfx_a_num[7] && sfx_hit_a) begin
                            sfx_start_r <= 1'b1;
                            sfx_index_r <= sfx_a_idx;
                            sfx_is_b    <= 1'b0;
                        end
                    end
                end
            end
        end
    end

    // ---- packet-injection task: present bytes 1..4 of a SOUND packet ----
    task send_sound;
        input [7:0] a;   // byte1 SFX-A
        input [7:0] b;   // byte2 SFX-B
        input [7:0] z;   // byte4 Z
        begin
            @(posedge clk); byte_done<=0; byte_cnt<=5'd1; data<=a;
            @(posedge clk); byte_done<=1;
            @(posedge clk); byte_done<=0; byte_cnt<=5'd2; data<=b;
            @(posedge clk); byte_done<=1;
            @(posedge clk); byte_done<=0; byte_cnt<=5'd3; data<=8'h00;
            @(posedge clk); byte_done<=1;
            @(posedge clk); byte_done<=0; byte_cnt<=5'd4; data<=z;
            @(posedge clk); byte_done<=1;
            @(posedge clk); byte_done<=0;
        end
    endtask

    integer fails = 0;

    // capture the stop pulse across the reset window
    reg saw_stop = 0;
    always @(posedge clk) if (sfx_stop) saw_stop <= 1;

    task check;
        input [6:0] exp_idx;
        input       exp_start;
        input       exp_stop;
        begin
            // sfx_start_r/stop_r are 1-cycle pulses set on the byte_cnt==4 edge
            if (sfx_start !== exp_start || sfx_stop !== exp_stop ||
                (exp_start && sfx_index !== exp_idx)) begin
                fails = fails + 1;
                $display("  FAIL got start=%b stop=%b idx=%0d  want start=%b stop=%b idx=%0d",
                         sfx_start, sfx_stop, sfx_index, exp_start, exp_stop, exp_idx);
            end else begin
                $display("  ok   start=%b stop=%b idx=%0d", sfx_start, sfx_stop, sfx_index);
            end
        end
    endtask

    task check_is_b;
        input exp;
        begin
            if (sfx_is_b !== exp) begin
                fails = fails + 1;
                $display("  FAIL sfx_is_b=%b want %b", sfx_is_b, exp);
            end
        end
    endtask

    initial begin
        $display("== SFX-A range: num-1 -> index 0..47 ==");
        send_sound(8'h00, 8'h00, 8'h00); @(posedge clk); check(7'd0,0,0);  // 0 = none
        send_sound(8'h01, 8'h00, 8'h00); @(posedge clk); check(7'd0,1,0);  // low edge
        check_is_b(0);
        send_sound(8'h1F, 8'h00, 8'h00); @(posedge clk); check(7'd30,1,0); // KDL2 SwordSwing
        send_sound(8'h26, 8'h00, 8'h00); @(posedge clk); check(7'd37,1,0);
        send_sound(8'h30, 8'h00, 8'h00); @(posedge clk); check(7'd47,1,0); // high edge
        send_sound(8'h31, 8'h00, 8'h00); @(posedge clk); check(7'd0,0,0);  // out of range
        send_sound(8'h40, 8'h00, 8'h00); @(posedge clk); check(7'd0,0,0);

        $display("== SFX-B range: 47+num -> index 48..72 ==");
        send_sound(8'h00, 8'h01, 8'h00); @(posedge clk); check(7'd48,1,0); // low edge
        check_is_b(1);
        send_sound(8'h00, 8'h0B, 8'h00); @(posedge clk); check(7'd58,1,0); // B0B Wave
        send_sound(8'h00, 8'h19, 8'h00); @(posedge clk); check(7'd72,1,0); // high edge
        send_sound(8'h00, 8'h1A, 8'h00); @(posedge clk); check(7'd0,0,0);  // out of range
        send_sound(8'h00, 8'h20, 8'h00); @(posedge clk); check(7'd0,0,0);

        $display("== priority: valid B beats valid A; A fallback when B invalid ==");
        send_sound(8'h05, 8'h02, 8'h00); @(posedge clk); check(7'd49,1,0);
        check_is_b(1);
        send_sound(8'h05, 8'h00, 8'h00); @(posedge clk); check(7'd4,1,0);
        check_is_b(0);

        $display("== stop semantics (bit 7, aimed via sfx_is_b) ==");
        // A playing (is_b=0): stop-A via 0x80 in byte1
        send_sound(8'h02, 8'h00, 8'h00); @(posedge clk); check(7'd1,1,0);   // start A02
        send_sound(8'h80, 8'h00, 8'h00); @(posedge clk); check(7'd0,0,1);
        // B playing (is_b=1): stop-B via bit7 in byte2
        send_sound(8'h00, 8'h04, 8'h00); @(posedge clk); check(7'd51,1,0);  // start B04
        check_is_b(1);
        send_sound(8'h00, 8'h84, 8'h00); @(posedge clk); check(7'd0,0,1);
        // B playing: stop-A (byte1 bit7) must NOT stop (wrong channel)...
        send_sound(8'h00, 8'h04, 8'h00); @(posedge clk); check(7'd51,1,0);
        send_sound(8'h80, 8'h00, 8'h00); @(posedge clk); check(7'd0,0,0);
        // ...and a stop byte must never itself restart playback
        check_is_b(1);
        // A playing: stop-B (byte2 bit7) must NOT stop either
        send_sound(8'h03, 8'h00, 8'h00); @(posedge clk); check(7'd2,1,0);   // start A03 (is_b=0)
        send_sound(8'h00, 8'h80, 8'h00); @(posedge clk); check(7'd0,0,0);
        check_is_b(0);
        // stop takes precedence even when the other byte carries a valid number
        send_sound(8'h06, 8'h00, 8'h00); @(posedge clk); check(7'd5,1,0);   // start A06
        send_sound(8'h80, 8'h05, 8'h00); @(posedge clk); check(7'd0,0,1);   // stop beats B05

        $display("== core reset must stop the SFX engine (Everdrive menu exit) ==");
        // start a looping SFX-B (B0B Wave), then assert the core reset
        send_sound(8'h00, 8'h0B, 8'h00); @(posedge clk); check(7'd58,1,0);
        saw_stop = 0;
        reset_ss = 1;
        repeat (5) @(posedge clk);
        if (!saw_stop) begin
            fails = fails + 1;
            $display("  FAIL reset did not assert sfx_stop");
        end else $display("  ok   reset asserted sfx_stop");
        if (sfx_is_b !== 1'b0) begin
            fails = fails + 1;
            $display("  FAIL reset did not clear sfx_is_b");
        end else $display("  ok   reset cleared sfx_is_b");
        if (sfx_a_num !== 8'd0 || sfx_b_num !== 8'd0) begin
            fails = fails + 1;
            $display("  FAIL reset did not clear latched SOUND numbers");
        end else $display("  ok   reset cleared latched SOUND numbers");
        // deassert: the default clears re-arm sfx_stop on the first ce
        reset_ss = 0;
        @(posedge clk); #1;
        if (sfx_stop !== 1'b0) begin
            fails = fails + 1;
            $display("  FAIL sfx_stop not re-armed after reset");
        end else $display("  ok   sfx_stop re-armed after reset");
        // a new game after reset can still trigger effects (both sides)
        send_sound(8'h1F, 8'h00, 8'h00); @(posedge clk); check(7'd30,1,0);
        send_sound(8'h00, 8'h19, 8'h00); @(posedge clk); check(7'd72,1,0);

        if (fails) $display("RESULT: FAIL (%0d)", fails);
        else       $display("RESULT: PASS");
        $finish;
    end

endmodule
