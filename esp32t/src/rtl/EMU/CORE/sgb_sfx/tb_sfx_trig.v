`timescale 1ns/1ps
// tb_sfx_trig.v -- verify gb.v's CMD_SOUND -> sfx_start/index/stop trigger path.
// This replicates (verbatim) the SFX trigger logic added to gb.v so it can be
// exercised without the whole GB SoC. It drives SOUND packets (byte_cnt 1..4)
// exactly as the packet engine presents them and checks sfx_start/index/stop.
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
    reg [2:0] sfx_index_r = 3'd0;
    reg       sfx_is_b    = 1'b0;
    reg [7:0] sfx_a_num   = 8'd0;
    reg [7:0] sfx_b_num   = 8'd0;
    wire      sfx_start = sfx_start_r;
    wire      sfx_stop  = sfx_stop_r;
    wire [2:0] sfx_index = sfx_index_r;

    function [3:0] sfx_map_a;
        input [6:0] num;
        begin
            case (num)
                7'h1F:   sfx_map_a = 4'b1_000;
                7'h26:   sfx_map_a = 4'b1_001;
                7'h30:   sfx_map_a = 4'b1_010;
                default: sfx_map_a = 4'b0_000;
            endcase
        end
    endfunction
    function [3:0] sfx_map_b;
        input [6:0] num;
        begin
            case (num)
                7'h01:   sfx_map_b = 4'b1_011;
                7'h04:   sfx_map_b = 4'b1_100;
                7'h07:   sfx_map_b = 4'b1_101;
                7'h08:   sfx_map_b = 4'b1_110;
                7'h0B:   sfx_map_b = 4'b1_111;
                default: sfx_map_b = 4'b0_000;
            endcase
        end
    endfunction
    wire [3:0] sfx_hit_a  = sfx_map_a(sfx_a_num[6:0]);
    wire [3:0] sfx_hit_b  = sfx_map_b(sfx_b_num[6:0]);
    wire       sfx_do_stop = (sfx_a_num[7] && !sfx_is_b) ||
                             (sfx_b_num[7] &&  sfx_is_b);

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
                        end else if (!sfx_b_num[7] && sfx_hit_b[3]) begin
                            sfx_start_r <= 1'b1;
                            sfx_index_r <= sfx_hit_b[2:0];
                            sfx_is_b    <= 1'b1;
                        end else if (!sfx_a_num[7] && sfx_hit_a[3]) begin
                            sfx_start_r <= 1'b1;
                            sfx_index_r <= sfx_hit_a[2:0];
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
    task check;
        input [2:0] exp_idx;
        input       exp_start;
        input       exp_stop;
        begin
            // sfx_start_r/stop_r are 1-cycle pulses set on the byte_cnt==4 edge
            if (sfx_start !== exp_start || sfx_stop !== exp_stop ||
                (exp_start && sfx_index !== exp_idx)) begin
                fails = fails + 1;
                $display("  FAIL got start=%b stop=%b idx=%d  want start=%b stop=%b idx=%d",
                         sfx_start, sfx_stop, sfx_index, exp_start, exp_stop, exp_idx);
            end else begin
                $display("  ok   start=%b stop=%b idx=%d", sfx_start, sfx_stop, sfx_index);
            end
        end
    endtask

    // capture the pulse on the cycle after the final byte_done
    reg saw_start, saw_stop; reg [2:0] cap_idx;
    always @(posedge clk) begin
        if (sfx_start) begin saw_start<=1; cap_idx<=sfx_index; end
        if (sfx_stop)  saw_stop<=1;
    end

    initial begin
        // SFX-A triggers -> indices 0..2
        send_sound(8'h1F, 8'h00, 8'h00); @(posedge clk); check(3'd0,1,0);
        send_sound(8'h26, 8'h00, 8'h00); @(posedge clk); check(3'd1,1,0);
        send_sound(8'h30, 8'h00, 8'h00); @(posedge clk); check(3'd2,1,0);
        // SFX-B triggers -> indices 3..7
        send_sound(8'h00, 8'h01, 8'h00); @(posedge clk); check(3'd3,1,0);
        send_sound(8'h00, 8'h04, 8'h00); @(posedge clk); check(3'd4,1,0);
        send_sound(8'h00, 8'h07, 8'h00); @(posedge clk); check(3'd5,1,0);
        send_sound(8'h00, 8'h08, 8'h00); @(posedge clk); check(3'd6,1,0);
        send_sound(8'h00, 8'h0B, 8'h00); @(posedge clk); check(3'd7,1,0);
        // unmapped numbers -> no trigger (incl. removed A0C)
        send_sound(8'h0C, 8'h02, 8'h00); @(posedge clk); check(3'd0,0,0);
        // stop: A playing (is_b=0), stop-A bit set
        send_sound(8'h1F, 8'h00, 8'h00); @(posedge clk);        // start A
        send_sound(8'h80, 8'h00, 8'h00); @(posedge clk); check(3'd0,0,1);
        // start a B (is_b=1), then stop-B via 0x80|number
        send_sound(8'h00, 8'h04, 8'h00); @(posedge clk);        // start B (is_b=1)
        send_sound(8'h00, 8'h84, 8'h00); @(posedge clk); check(3'd0,0,1);
        // stop-A while B playing -> should NOT stop (is_b=1)
        send_sound(8'h00, 8'h04, 8'h00); @(posedge clk);        // start B
        send_sound(8'h80, 8'h00, 8'h00); @(posedge clk); check(3'd0,0,0);

        // ---- core reset must stop the SFX engine (Everdrive menu exit) ----
        // start a looping SFX-B (B0B Wave), then assert the core reset
        send_sound(8'h00, 8'h0B, 8'h00); @(posedge clk); check(3'd7,1,0);
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
        // deassert: the default clears re-arm sfx_stop on the first ce
        reset_ss = 0;
        @(posedge clk); #1;
        if (sfx_stop !== 1'b0) begin
            fails = fails + 1;
            $display("  FAIL sfx_stop not re-armed after reset");
        end else $display("  ok   sfx_stop re-armed after reset");
        // a new game after reset can still trigger effects
        send_sound(8'h1F, 8'h00, 8'h00); @(posedge clk); check(3'd0,1,0);

        if (fails) $display("RESULT: FAIL (%0d)", fails);
        else       $display("RESULT: PASS");
        $finish;
    end
endmodule
