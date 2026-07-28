//
// gb.v
//
// Gameboy for the MIST board https://github.com/mist-devel
// 
// Copyright (c) 2015 Till Harbaum <till@harbaum.org> 
// 
// This source file is free software: you can redistribute it and/or modify 
// it under the terms of the GNU General Public License as published 
// by the Free Software Foundation, either version 3 of the License, or 
// (at your option) any later version. 
// 
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License 
// along with this program.  If not, see <http://www.gnu.org/licenses/>. 
//

module gb (
   input reset,
   
    input clk_sys,
    input ce,
    input ce_n,
    input ce_2x,
    input ce_4x,
    input [1:0] oc_lvl,   // 0=1x  1=2x  2=4x

    input isGBC,
    input isSGB,        // native SGB enable: cart is an SGB game (static, from header detect)
    input isGBC_game,   // cart uses CGB features (SGB palette is bypassed when set; 0 for SGB games)
    input real_cgb_boot,
    input paletteOff,
    input customPaletteEna,
    input [63:0] paletteBGIn,
    input [63:0] paletteOBJ0In,
    input [63:0] paletteOBJ1In,
    output gbc_mode,
    output [63:0] gpd,

    // cartridge interface
    // can adress up to 1MB ROM
    output [14:0] ext_bus_addr,
    output ext_bus_a15,
    output cart_rd,
    output cart_wr,
    input [7:0] cart_do,
    output [7:0] cart_di,
    input  cart_oe,
    // High while a physical cartridge bus cycle is in flight (from cart.v).
    // Used to gate ce_cpu to 1× so the CPU never advances past T3 before
    // the ROM has had its full tACC window.
    input  cart_busy,
    output  [2:0] TSTATEo,
    output  TSTATE1,
    output  TSTATE2,
    output  TSTATE3,
    output  TSTATE4,

    input   IR_RX,
    output  IR_LED,

    // WRAM or Cart RAM CS
    output nCS,

    input cgb_boot_download,
    input dmg_boot_download,
    input         ioctl_wr,
    input  [24:0] ioctl_addr,
    input  [15:0] ioctl_dout,

    // Bootrom features
    input boot_gba_en,
    input fast_boot_en,
    input skip_boot_rom,  // skip boot ROM on reset (reserved; tied 0 -- SGB now boots natively, no reboot)

    // audio
    output [15:0] audio_l,
    output [15:0] audio_r,

    // Megaduck?
    input megaduck,

    // lcd interface
    output reg lcd_clkena,
    output [14:0] lcd_data,   // final LCD colour (SGB palette overlay applied when isSGB & sgb_pal_en)
    output reg [1:0] lcd_data_gb,
    output reg [7:0] lcd_pix_x,   // screen x (0-159) of lcd_data_gb pixel, in lockstep (for SGB)
    output reg [7:0] lcd_pix_y,   // screen y (0-143) of lcd_data_gb pixel, in lockstep (for SGB)
    output reg [1:0] lcd_mode,
    output reg lcd_on,
    output reg lcd_vsync,

    // SGB taps: main-video LCD state only, never videoBypass outputs.
    // The bypass free-runs while the game's LCD is off (and during the
    // LCD-on alignment window): lcd_on=1, mode cycling through vblank,
    // lcd_clkena pulsing and lcd_data_gb forced to 0. Feeding that to
    // sgb.v makes its frame scanner see phantom frames and overwrite
    // PAL_TRN/ATTR_TRN captures with zeros (Pokemon Red/Blue and
    // Donkey Kong '94 boot to a black screen). MiSTer feeds sgb.v
    // directly from the video module; these taps restore that.
    output reg lcd_clkena_sgb,
    output reg [1:0] lcd_data_gb_sgb,
    output reg [1:0] lcd_mode_sgb,
    output reg lcd_on_sgb,

    output [1:0] joy_p54,
    input  [3:0] joy_din,
    
    output speed,   //GBC
    output DMA_on,
    output hdma_active,
    output cpu_stop,
    output cpu_halt,
    
    input          gg_reset,
    input          gg_en,
    input  [128:0] gg_code,
    output         gg_available,

    //serial port
    output sc_int_clock2,
    input serial_clk_in,
    output serial_clk_out,
    input  serial_data_in,
    output serial_data_out,
    output reg boot_rom_enabled = 1'd1,
    // savestates
    input        increaseSSHeaderCount,
    input  [7:0] cart_ram_size,
    input        save_state,
    input        load_state,
    input  [1:0] savestate_number,
    output       sleep_savestate,
    
    output [63:0] SaveStateExt_Din, 
    output [9:0]  SaveStateExt_Adr, 
    output        SaveStateExt_wren,
    output        SaveStateExt_rst, 
    input  [63:0] SaveStateExt_Dout,
    output        SaveStateExt_load,
    
    output [19:0] Savestate_CRAMAddr,     
    output        Savestate_CRAMRWrEn,    
    output [7:0]  Savestate_CRAMWriteData,
    input  [7:0]  Savestate_CRAMReadData,

    output [63:0] SAVE_out_Din,     // data read from savestate
    input  [63:0] SAVE_out_Dout,    // data written to savestate
    output [25:0] SAVE_out_Adr,     // all addresses are DWORD addresses!
    output        SAVE_out_rnw,     // read = 1, write = 0
    output        SAVE_out_ena,     // one cycle high for each action
    output  [7:0] SAVE_out_be,     
    input         SAVE_out_done,    // should be one cycle high when write is done or read value is valid
    
    output lcd_on_int,
    output lcd_off_overwrite,

    
    input         rewind_on,
    input         rewind_active
);

// savestates
wire [63:0] SaveStateBus_Din;
wire [9:0] SaveStateBus_Adr;
wire SaveStateBus_wren, SaveStateBus_rst;
  
wire [7:0] Savestate_RAMWriteData;
wire [7:0] Savestate_RAMReadData_WRAM, Savestate_RAMReadData_VRAM, Savestate_RAMReadData_ORAM, Savestate_RAMReadData_ZRAM;
wire [19:0] Savestate_RAMAddr;
wire [4:0] Savestate_RAMRWrEn;

localparam SAVESTATE_MODULES    = 8;
wire [63:0] SaveStateBus_wired_or[0:SAVESTATE_MODULES-1];

wire [56:0] SS_Top;
wire [56:0] SS_Top_BACK;
eReg_SavestateV #(0, 31, 56, 0, 64'h0000000000800000) iREG_SAVESTATE_Top (clk_sys, SaveStateBus_Din, SaveStateBus_Adr, SaveStateBus_wren, SaveStateBus_rst, SaveStateBus_wired_or[6], SS_Top_BACK, SS_Top);  

wire [10:0] SS_Top2;
wire [10:0] SS_Top2_BACK;
eReg_SavestateV #(0, 38, 10, 0, 64'h0000000000000000) iREG_SAVESTATE_Top2 (clk_sys, SaveStateBus_Din, SaveStateBus_Adr, SaveStateBus_wren, SaveStateBus_rst, SaveStateBus_wired_or[7], SS_Top2_BACK, SS_Top2);

// include cpu

reg [1:0] ff4c_key0; // GBC DMG mode register
wire isGBC_mode = !ff4c_key0 | boot_rom_enabled;

wire [15:0] cpu_addr;
wire [7:0] cpu_do;

wire sel_timer = (cpu_addr[15:4] == 12'hff0) && (cpu_addr[3:2] == 2'b01);
wire sel_video_reg = (cpu_addr[15:4] == 12'hff4) || (isGBC && (cpu_addr[15:4] == 12'hff6) && (cpu_addr[3:0] >= 4'h8 && cpu_addr[3:0] <= 4'hC)); //video and oam dma (+ ff68-ff6C when gbc)
wire sel_video_oam = cpu_addr[15:8] == 8'hfe;
wire sel_joy  = cpu_addr == 16'hff00;                // joystick controller
wire sel_sb  = cpu_addr == 16'hff01;                    // serial SB - Serial transfer data
wire sel_sc  = cpu_addr == 16'hff02;                    // SC - Serial Transfer Control (R/W)
wire sel_rom  = !cpu_addr[15];                       // lower 32k are rom
wire sel_cram = cpu_addr[15:13] == 3'b101;           // 8k cart ram at $a000
wire sel_vram = cpu_addr[15:13] == 3'b100;           // 8k video ram at $8000
wire sel_ie   = cpu_addr == 16'hffff;                // interupt enable
wire sel_if   = cpu_addr == 16'hff0f;                // interupt flag
wire sel_wram = (cpu_addr[15:14] == 2'b11) && (~&cpu_addr[13:9]); // 8k internal ram at $C000-DFFF. C000-DDFF mirrored to E000-FDFF
wire sel_zpram = (cpu_addr[15:7] == 9'b111111111) && // 127 bytes zero pageram at $ff80
                     (cpu_addr != 16'hffff);
wire sel_audio = (cpu_addr[15:8] == 8'hff) &&        // audio reg ff10 - ff3f and ff76/ff77 PCM12/PCM34 (undocumented registers)
                    ((cpu_addr[7:5] == 3'b001) || (cpu_addr[7:4] == 4'b0001) || (cpu_addr[7:0] == 8'h76) || (cpu_addr[7:0] == 8'h77));
wire sel_nr32  = (cpu_addr == 16'hff1c);             // NR32 - Wave channel output level (volume)

wire sel_ext_bus = sel_rom | sel_cram | sel_wram;

wire sel_boot_rom, sel_boot_rom_cgb;

// unused cgb registers
wire sel_FF72  = isGBC && cpu_addr == 16'hff72;            // unused register, all bits read/write
wire sel_FF73  = isGBC && cpu_addr == 16'hff73;            // unused register, all bits read/write
wire sel_FF74  = isGBC && isGBC_mode && (cpu_addr == 16'hff74); // unused register, all bits read/write, only in CGB mode
wire sel_FF75  = isGBC && cpu_addr == 16'hff75;            // unused register, bits 4-6 read/write

// Special MiSTer register for signalling to cpu bootrom
wire sel_FF50  = boot_rom_enabled && cpu_addr == 16'hff50;

wire ext_bus_wram_sel, ext_bus_cram_sel, ext_bus_rom_sel;
wire ext_bus_rd, ext_bus_wr;
wire [7:0] ext_bus_di;
wire cart_sel;


//  From Game Boy: Complete Technical Reference:
//  DMA register value, Selected bus, Asserted CS:
//  0x00-0x7F, external bus,  external ROM (A15)
//  0x80-0x9F, video RAM bus, video RAM (MCS)
//  0xA0-0xFF, external bus,  external RAM (CS)
wire [15:0] dma_addr;               
wire dma_sel_rom = ~dma_addr[15];                       // lower 32k are rom
wire dma_sel_vram = dma_addr[15:13] == 3'b100;          // 8k video ram at $8000-$9FFF
wire dma_sel_ext_bus_dmg = ~dma_sel_vram;               // WRAM or Cart ROM/RAM
wire dma_sel_ext_ram = ~dma_sel_rom;                    // External RAM CS

//CGB
wire dma_sel_cram = dma_addr[15:13] == 3'b101;          // 8k cart ram at $a000
wire dma_sel_wram = dma_addr[15:13] == 3'b110;          // 8k WRAM at $c000-$dfff
wire dma_sel_ext_bus_cgb = dma_sel_rom | dma_sel_cram;  // Cart ROM/RAM

wire dma_sel_ext_bus = isGBC ? dma_sel_ext_bus_cgb : dma_sel_ext_bus_dmg;

//CGB
wire sel_vram_bank = (cpu_addr==16'hff4f);
wire sel_wram_bank = isGBC && isGBC_mode && (cpu_addr==16'hff70);
wire sel_hdma = isGBC && isGBC_mode && (cpu_addr[15:4]==12'hff5) && 
                    ((cpu_addr[3:0]!=4'd0)&&(cpu_addr[3:0]< 4'd6)); //HDMA FF51-FF55 
wire sel_key0 = isGBC && boot_rom_enabled && (cpu_addr == 16'hff4c); // KEY0 - CGB / DMG mode register
wire sel_key1 = isGBC && isGBC_mode && (cpu_addr == 16'hff4d); // KEY1 - CGB Mode Only - Prepare Speed Switch
wire sel_rp = isGBC && isGBC_mode && (cpu_addr == 16'hff56); //FF56 - RP - CGB Mode Only - Infrared Communications Port

//HDMA can select from $0000 to $7ff0 or A000-DFF0
wire [15:0] hdma_source_addr;
wire hdma_sel_rom = !hdma_source_addr[15];                  // lower 32k are rom
wire hdma_sel_cram = hdma_source_addr[15:13] == 3'b101;     // 8k cart ram at $a000
wire hdma_sel_wram = hdma_source_addr[15:13] == 3'b110;     // 8k WRAM at $c000-$dff0
wire hdma_sel_ext_bus = hdma_sel_rom | hdma_sel_cram;

wire sc_start;
wire sc_shiftclock;
wire [7:0] sc_r;

wire irq_ack;
wire [7:0] irq_vec;
reg [4:0] if_r;
reg [7:0] ie_r; // writing  $ffff sets the irq enable mask
wire irq_n;
                
reg [2:0] wram_bank; //1-7 FF70 - SVBK
reg vram_bank; //0-1 FF4F - VBK

wire [7:0] hdma_do;

reg cpu_speed; // - 0 Normal mode (4MHz) - 1 Double Speed Mode (8MHz)
reg prepare_switch; // set to 1 to toggle speed

wire oam_cpu_allow;
wire vram_cpu_allow;

wire [7:0] joy_do;
wire [7:0] sb_o;
wire [7:0] timer_do;
wire [7:0] video_do;
wire [7:0] audio_do;
wire [7:0] boot_do;
wire [7:0] vram_do;
wire [7:0] vram1_do;
wire [7:0] zpram_do;
wire [7:0] wram_do;

reg[7:0] FF72;
reg[7:0] FF73;
reg[7:0] FF74;
reg[2:0] FF75;


reg [1:0] ir_reg76;
reg ir_reg0;
reg ir_reg1;

wire [7:0] ir_reg_mask = ir_reg76 == 2'b11 ? 
                            {ir_reg76, 4'd0, ir_reg1, ir_reg0} : 
                            {ir_reg76, 4'd0, 1'b1, ir_reg0};
                            
assign IR_LED = ir_reg0;

wire reset_ss;
wire cpu_wr_n_edge;

always @(posedge clk_sys) begin
    if(reset_ss)
    begin
        ir_reg0 <= 1'd0;
        ir_reg76 <= 2'd0;
    end
    else 
        if (ce_2x) 
        begin
            ir_reg1  <= IR_RX; 
        
            if(sel_rp && !cpu_wr_n_edge)
            begin
                ir_reg76 <= cpu_do[7:6];
                ir_reg0  <= cpu_do[0];
            end
        end
    end


// http://gameboy.mongenel.com/dmg/asmmemmap.html
wire [7:0] cpu_di = 
        irq_ack?irq_vec:
        sel_if?{3'b111, if_r}:  // interrupt flag register
        sel_rp? ir_reg_mask:
        sel_wram_bank?{5'h1f,wram_bank}:
        isGBC&&sel_vram_bank?{7'h7f,vram_bank}:
        sel_hdma?{hdma_do}:  //hdma GBC
        sel_key0 ? { 4'hF, ff4c_key0, 2'b10 } :
        sel_key1?{cpu_speed,6'h3f,prepare_switch}: //key1 cpu speed register(GBC)
        sel_joy?joy_do:         // joystick register
        sel_sb?sb_o:                // serial transfer data register
        sel_sc?sc_r:                // serial transfer control register
        sel_timer?timer_do:     // timer registers
        sel_video_reg?video_do: // video registers
        (sel_video_oam&&oam_cpu_allow)?video_do: // video object attribute memory
        sel_audio?audio_do:                                // audio registers
        sel_boot_rom?boot_do:                             // boot rom
        isGBC&sel_wram ? wram_do:                         // wram on GBC
        sel_ext_bus?ext_bus_di:                           // wram (DMG) + cartridge rom/ram
        (sel_vram&&vram_cpu_allow)?(isGBC&&vram_bank)?vram1_do:vram_do:       // vram (GBC bank 0+1)
        sel_zpram?zpram_do:     // zero page ram
        sel_ie?ie_r:  // interrupt enable register
        sel_FF72?FF72: // unused register, all bits read/write
        sel_FF73?FF73: // unused register, all bits read/write
        sel_FF74?FF74: // unused register, all bits read/write, only in CGB mode
        sel_FF75?{1'b1,FF75, 4'b1111}: // unused register, bits 4-6 read/write
        sel_FF50?{6'b0, fast_boot_en, boot_gba_en}: // MiSTer special instruction register 
        8'hff;

wire cpu_wr_n;
wire cpu_rd_n;
wire cpu_iorq_n;
wire cpu_m1_n;
wire cpu_mreq_n;

// Clean, glitch-free clock enable. 
// DMA and BootROM always run at 1x. Overclock is applied cleanly otherwise.
// Detect CPU cart-bound machine cycles early, at T1, using the address
// bus (which is stable from T1 onwards). This gives us a perfectly
// registered flag to drop ce_cpu to native speed for the entire cycle.
wire cart_addr_match = sel_rom | sel_cram;
reg  cart_access_pending;
always @(posedge clk_sys) begin
   if (TSTATE1)
      cart_access_pending <= cart_addr_match;
end

// Force native speed for DMA, BootROM, and physical cartridge accesses
wire force_native = dma_rd || hdma_active || boot_rom_enabled || cart_access_pending;

// gbc_snd runs at ce_2x. In 4x OC the CPU runs at ce_4x, so an audio
// register access could land on a non-ce_2x tick and the audio module
// would never see it. Detect the access at T1 (address is stable from
// T1) and drop ce_cpu to ce_2x for the entire machine cycle.
wire audio_addr_match = sel_audio;
reg   audio_access_pending;
always @(posedge clk_sys) begin
   if (TSTATE1)
      audio_access_pending <= audio_addr_match;
end
wire audio_slow = (oc_lvl == 2'd2) & audio_access_pending;

// Detect CPU-timed PCM playback via rapid writes to NR32 (FF1C).
// Pokémon Yellow / Pinball bit-bang Pikachu's voice through the wave
// channel volume register — the CPU IS the DAC, so playback pitch is
// directly proportional to CPU clock rate.  When overclocked, force
// native speed during PCM to preserve correct pitch.
// Normal gameplay writes NR32 at most once per note trigger; the PCM
// loop writes it every ~179 CPU cycles.  A saturating counter that
// increments per write and slowly decays cleanly separates the two.
wire nr32_wr = !cpu_wr_n_edge & sel_nr32;

reg [3:0] pcm_cnt;
reg [8:0] pcm_decay;

always @(posedge clk_sys) begin
   if (reset_ss) begin
      pcm_cnt   <= 0;
      pcm_decay <= 0;
   end else begin
      if (ce_cpu & nr32_wr) begin
         if (pcm_cnt < 4'hF) pcm_cnt <= pcm_cnt + 4'd1;
         pcm_decay <= 0;
      end else if (ce_2x & (pcm_cnt != 0)) begin
         if (pcm_decay == 9'd511) begin
            pcm_cnt   <= pcm_cnt - 4'd1;
            pcm_decay <= 0;
         end else begin
            pcm_decay <= pcm_decay + 9'd1;
         end
      end
   end
end

wire pcm_active = (pcm_cnt >= 4'd4) & (oc_lvl != 2'd0);

// Clean, glitch-free clock enable.
wire ce_cpu = force_native ? (cpu_speed ? ce_2x : ce) :
              pcm_active   ? (cpu_speed ? ce_2x : ce) :
              audio_slow   ? ce_2x :
              (oc_lvl == 2'd2) ? ce_4x :
              (oc_lvl == 2'd1 || cpu_speed) ? ce_2x : ce;

// when hdma is enabled stop CPU (GBC). Finish read/write before stopping CPU
wire hdma_cpu_stop = (isGBC & hdma_active & cpu_rd_n & cpu_wr_n);

// OC bus stall: hold the CPU while it holds a memory access to a region the
// PPU has bus priority over - OAM during modes 2/3 (oam_cpu_allow low) and
// VRAM during mode 3 (vram_cpu_allow low). Real DMG/CGB hardware freezes the
// CPU for the duration of these accesses (WAIT states). This core ties
// WAIT_n high and instead gates the access itself (oam_wr / cpu_wr_vram and
// the cpu_di read mux require the *_allow flags), so a blocked access is
// silently DROPPED rather than delayed.
//
// At 1x (and CGB double speed) the CPU advances lock-step with the PPU, so a
// game's OAM/VRAM writes naturally land in the accessible windows (HBlank/
// VBlank) and nothing is lost - that is why the drop behaviour is harmless
// there and upstream keeps it. Under the 2x/4x overclock the CPU laps the PPU
// and issues OAM/VRAM writes *inside* mode 2/3; those were dropped, corrupting
// sprite attributes/tiles for a frame (Donkey Kong Land I/II sprite flicker).
//
// Gating cpu_clken low simply drops ce_cpu ticks (the same clock-enable
// mechanism the rest of the core uses) until the PPU leaves the blocking mode,
// so the access completes only once it is legal - matching hardware. No
// deadlock is possible: the PPU always advances on ce independently of the
// CPU, so mode 2/3 always ends and the block clears. LCD-off accesses are
// unaffected (mode3/oam_eval are 0 there). Scoped to oc_lvl!=0 so the
// validated 1x / CGB-double-speed paths stay identical to upstream.
wire cpu_bus_blocked = (oc_lvl != 2'd0) & ~cpu_mreq_n &
                       ( (sel_video_oam & ~oam_cpu_allow) |
                         (sel_vram      & ~vram_cpu_allow) );

wire cpu_clken = ~hdma_cpu_stop & ce_cpu & ~cpu_bus_blocked;

reg reset_r  = 1;

//sync reset with clock
always  @ (posedge clk_sys) begin
    if(ce)
        reset_r <= reset;
end

reg old_cpu_wr_n;

assign SS_Top_BACK[54] = old_cpu_wr_n;

always @(posedge clk_sys) begin
    if(reset_ss) old_cpu_wr_n <= SS_Top[54]; // 1'b0
    else if (ce_cpu) old_cpu_wr_n <= cpu_wr_n;
end
assign cpu_wr_n_edge = ~(old_cpu_wr_n & ~cpu_wr_n);

wire genie_ovr;
wire [7:0] genie_data;
wire WRITE_CYCLE;

GBse cpu (
    .RESET_n           ( !reset_ss        ),
    .CLK_n             ( clk_sys         ),
    .CLKEN             ( cpu_clken       ),
    .WAIT_n            ( 1'b1            ),
    .INT_n             ( irq_n           ),
    .NMI_n             ( 1'b1            ),
    .BUSRQ_n           ( 1'b1            ),
   .M1_n              ( cpu_m1_n        ),
   .MREQ_n            ( cpu_mreq_n      ),
   .IORQ_n            ( cpu_iorq_n      ),
   .RD_n              ( cpu_rd_n        ),
   .WR_n              ( cpu_wr_n        ),
   .RFSH_n            (                 ),
   .HALT_n            ( cpu_halt        ),
   .BUSAK_n           (                 ),
   .A                 ( cpu_addr        ),
   .DI                ( genie_ovr ? genie_data : cpu_di),
   .DO                ( cpu_do          ),
   .WRITE_CYCLE       ( WRITE_CYCLE),
   .TSTATEo           (TSTATEo),
   .TSTATE1           (TSTATE1),
   .TSTATE2           (TSTATE2),
   .TSTATE3           (TSTATE3),
   .TSTATE4           (TSTATE4),
    .STOP              ( cpu_stop        ),
    .isGBC             ( isGBC           ),
   // savestates
   .SaveStateBus_Din  (SaveStateBus_Din ), 
   .SaveStateBus_Adr  (SaveStateBus_Adr ),
   .SaveStateBus_wren (SaveStateBus_wren),
   .SaveStateBus_rst  (SaveStateBus_rst ),
   .SaveStateBus_Dout (SaveStateBus_wired_or[0])
);

// --------------------------------------------------------------------
// --------------------------- Cheat Engine ---------------------------
// --------------------------------------------------------------------

CODES codes (
    .clk        (clk_sys),
    .reset      (gg_reset),
    .enable     (gg_en),
    .addr_in    (cpu_addr),
    .data_in    (cpu_di),
    .available  (gg_available),
    .code       (gg_code),
    .genie_ovr  (genie_ovr),
    .genie_data (genie_data)
);

// --------------------------------------------------------------------
// --------------------- GBC/DMG mode KEY0 (GBC) ----------------------
// --------------------------------------------------------------------

assign SS_Top_BACK[56:55] = ff4c_key0;
assign gbc_mode = isGBC_mode;

// KEY0 FF4C CPU mode register
// Bits 2 and 3 CPU mode select
// 00: CGB mode (Mode used by carts supporting CGB)
// 01: DMG/MGB mode (Mode used by DMG/MGB-only carts)
 //10: PGB1 mode (STOP the CPU, with the LCD operated by an external signal)
// 11: PGB2 mode (CPU still running, with the LCD operated by an external signal)
// R/W only when boot rom is enabled
// https://forums.nesdev.org/viewtopic.php?t=19888
always @(posedge clk_sys) begin
    if(reset_ss) begin
        ff4c_key0 <= SS_Top[56:55]; // 2'd0;
    end else if(sel_key0 & ce_cpu & ~cpu_wr_n_edge) begin
        ff4c_key0 <= cpu_do[3:2];
    end
end

// --------------------------------------------------------------------
// --------------------- Speed Toggle KEY1 (GBC)-----------------------
// --------------------------------------------------------------------

// The physical cartridge controller (cart.v) changes its sample timing
// based on the 'speed' flag. If we overclock the CPU to 2x or 4x, we MUST
// tell the cart controller to use the faster GBC sample window, otherwise
// the CPU cycle will end before cart.v even tries to latch the data!
wire is_overclocked = (oc_lvl == 2'd1 || oc_lvl == 2'd2) && !boot_rom_enabled && !dma_rd && !hdma_active;
assign speed = cpu_speed | is_overclocked;

assign SS_Top_BACK[3] = cpu_speed;
assign SS_Top_BACK[4] = prepare_switch;

always @(posedge clk_sys) begin
   if(reset_ss) begin
        cpu_speed      <= SS_Top[3]; // 1'b0;
        prepare_switch <= SS_Top[4]; // 1'b0;
    end
    else if (ce_2x && sel_key1 && !cpu_wr_n_edge)begin
        prepare_switch <= cpu_do[0];
    end
    
    if (ce_2x && isGBC && prepare_switch && cpu_stop) begin
        cpu_speed <= !cpu_speed;
        prepare_switch <= 1'b0;
    end
end

// --------------------------------------------------------------------
// ------------------------------ audio -------------------------------
// --------------------------------------------------------------------

wire audio_rd = !cpu_rd_n && sel_audio;
wire audio_wr = !cpu_wr_n_edge && sel_audio;

gbc_snd audio (
    .clk                ( clk_sys           ),
    .ce            ( ce_2x           ),
    .reset          ( reset_ss          ),
    
    .is_gbc        ( isGBC           ),
    //.remove_pops   ( 1'b0   ),

    .s1_read        ( audio_rd          ),
    .s1_write       ( audio_wr          ),
    .s1_addr        ( cpu_addr[6:0] ),
   .s1_readdata     ( audio_do        ),
    .s1_writedata  ( cpu_do         ),

   .snd_left        ( audio_l           ),
    .snd_right      ( audio_r           ),
    
  .SaveStateBus_Din  (SaveStateBus_Din ), 
  .SaveStateBus_Adr  (SaveStateBus_Adr ),
  .SaveStateBus_wren (SaveStateBus_wren),
  .SaveStateBus_rst  (SaveStateBus_rst ),
  .SaveStateBus_Dout (SaveStateBus_wired_or[5])
);

// --------------------------------------------------------------------
// -----------------------serial port()--------------------------------
// --------------------------------------------------------------------

// SNAC
wire serial_irq;

assign sc_int_clock2 = sc_shiftclock;

link link (
  .clk_sys(clk_sys),
  .ce(ce_cpu),
  .rst(reset_ss),

  .sel_sc(sel_sc),
  .sel_sb(sel_sb),
  .cpu_wr_n(cpu_wr_n_edge),
  .sc_start_in(cpu_do[7]),
  .sc_speed_in(cpu_do[1]),
  .sc_int_clock_in(cpu_do[0]),

  .sb_in(cpu_do),
  .sc_r(sc_r),

  .serial_clk_in(serial_clk_in),
  .serial_data_in(serial_data_in),

  .serial_clk_out(serial_clk_out),
  .serial_data_out(serial_data_out),
  .sb(sb_o),
  .serial_irq(serial_irq),
  .sc_start(sc_start),
  .sc_int_clock(sc_shiftclock),
    
  .SaveStateBus_Din  (SaveStateBus_Din ), 
  .SaveStateBus_Adr  (SaveStateBus_Adr ),
  .SaveStateBus_wren (SaveStateBus_wren),
  .SaveStateBus_rst  (SaveStateBus_rst ),
  .SaveStateBus_Dout (SaveStateBus_wired_or[3])
);

// --------------------------------------------------------------------
// ------------------------------ inputs ------------------------------
// --------------------------------------------------------------------

reg  [1:0] p54;

assign SS_Top_BACK[6:5] = p54;

always @(posedge clk_sys) begin
    if(reset_ss) p54 <= SS_Top[6:5]; //2'b00 for DMG, 2'b11 for CGB/SGB will be written by BIOS
    else if(ce_cpu && sel_joy && !cpu_wr_n_edge)    p54 <= cpu_do[5:4];
end

// When an SGB game is active, sgb_joy_do (below) supplies the joypad nibble:
// the SGB multitap player id when both P14/P15 are high (needed for SGB
// detection), otherwise the normal joy_din decode; non-SGB games use joy_din
// directly.
//
// sgb_joy_do is registered before use (sgb_joy_do_r), matching the original
// design which registered the SGB joypad in emu_system_top before feeding the
// core. The SGB multitap read protocol expects this one-cycle joypad latency,
// and feeding sgb_joy_do to the CPU combinationally broke SGB input.
reg [3:0] sgb_joy_do_r;
always @(posedge clk_sys) sgb_joy_do_r <= sgb_joy_do;
wire [3:0] joy_nibble = isSGB ? sgb_joy_do_r : joy_din;
assign joy_do = { 2'b11, p54, joy_nibble };
assign joy_p54 = p54;

// --------------------------------------------------------------------
// ------------------------------ unused regs -------------------------
// --------------------------------------------------------------------

assign SS_Top_BACK[34:27] = FF72;
assign SS_Top_BACK[42:35] = FF73;
assign SS_Top_BACK[50:43] = FF74;
assign SS_Top_BACK[53:51] = FF75;

always @(posedge clk_sys) begin
    if(reset_ss) begin
        FF72 <= SS_Top[34:27]; // 0;  
        FF73 <= SS_Top[42:35]; // 0;  
        FF74 <= SS_Top[50:43]; // 0;  
        FF75 <= SS_Top[53:51]; // 0;  
    end else if(ce_cpu && !cpu_wr_n_edge)   begin
        if (sel_FF72) FF72 <= cpu_do;
        if (sel_FF73) FF73 <= cpu_do;
        if (sel_FF74) FF74 <= cpu_do;
        if (sel_FF75) FF75 <= cpu_do[6:4];
    end
end

// --------------------------------------------------------------------
// ---------------------------- interrupts ----------------------------
// --------------------------------------------------------------------

// interrupt flags are set when the event happens or when the cpu writes
// the register to 1. The "highest" one active is cleared when the cpu
// runs an interrupt ack cycle or when it writes a 0 to the register

assign irq_ack = !cpu_iorq_n && !cpu_m1_n;

// irq vector
assign irq_vec = 
            if_r[0]&&ie_r[0]?8'h40:   // vsync
            if_r[1]&&ie_r[1]?8'h48:   // lcdc
            if_r[2]&&ie_r[2]?8'h50:   // timer
            if_r[3]&&ie_r[3]?8'h58:   // serial
            if_r[4]&&ie_r[4]?8'h60:   // input
            8'h00;

//wire vs = (lcd_mode == 2'b01);
//reg vsD, vsD2;
reg [3:0] inputD, inputD2;

// irq is low when an enable irq is active
assign irq_n = !(ie_r & if_r);

wire video_irq,vblank_irq;
wire timer_irq;

reg old_vblank_irq, old_video_irq, old_timer_irq, old_serial_irq;
reg old_ack = 0;

assign SS_Top_BACK[11: 7] = ie_r[4:0];
assign SS_Top_BACK[16:12] = if_r;
assign SS_Top_BACK[   17] = old_vblank_irq;
assign SS_Top_BACK[   18] = old_video_irq;
assign SS_Top_BACK[   19] = old_timer_irq;
assign SS_Top_BACK[   20] = old_serial_irq;
assign SS_Top_BACK[   21] = old_ack;
assign SS_Top_BACK[26:24] = ie_r[7:5];

always @(negedge clk_sys) begin //negedge to trigger interrupt earlier

    if(reset_ss) begin
        ie_r[4:0]        <= SS_Top[11: 7]; // 5'h00;
        if_r                <= SS_Top[16:12]; // 5'h00;
        old_vblank_irq  <= SS_Top[   17]; // 1'b0;
        old_video_irq   <= SS_Top[   18]; // 1'b0;
        old_timer_irq   <= SS_Top[   19]; // 1'b0;
        old_serial_irq  <= SS_Top[   20]; // 1'b0;
        old_ack         <= SS_Top[   21]; // 1'b0;
        ie_r[7:5]        <= SS_Top[26:24]; // 3'h00;
    end else if (ce_cpu) begin

        // "When an interrupt signal changes from low to high,
        //  then the corresponding bit in the IF register becomes set."
        old_vblank_irq <= vblank_irq;
        if(~old_vblank_irq & vblank_irq) if_r[0] <= 1'b1;
    
        // video irq already is a 1 clock event
        old_video_irq <= video_irq;
        if(~old_video_irq & video_irq) if_r[1] <= 1'b1;
        
        // timer_irq already is a 1 clock event
        old_timer_irq <= timer_irq;
        if(~old_timer_irq & timer_irq) if_r[2] <= 1'b1;
        
        // serial irq already is a 1 clock event
        old_serial_irq <= serial_irq;
        if(~old_serial_irq & serial_irq) if_r[3] <= 1'b1;
    
        // falling edge on any input line P10..P13
    
        inputD <= joy_din;
        inputD2 <= inputD;
        if(~inputD & inputD2) if_r[4] <= 1'b1;
    
        // cpu acknowledges irq. this clears the active irq with hte 
        // highest priority
        old_ack <= irq_ack;
        
        if(old_ack & ~irq_ack) begin
            if(if_r[0] && ie_r[0]) if_r[0] <= 1'b0;
            else if(if_r[1] && ie_r[1]) if_r[1] <= 1'b0;
            else if(if_r[2] && ie_r[2]) if_r[2] <= 1'b0;
            else if(if_r[3] && ie_r[3]) if_r[3] <= 1'b0;
            else if(if_r[4] && ie_r[4]) if_r[4] <= 1'b0;
        end
    
        // cpu writes interrupt enable register
        if(sel_ie && !cpu_wr_n_edge)
            ie_r <= cpu_do;
    
        // cpu writes interrupt flag register
        if(sel_if && !cpu_wr_n_edge)
            if_r <= cpu_do[4:0];
            
    end
end
            
// --------------------------------------------------------------------
// ------------------------------ timer -------------------------------
// --------------------------------------------------------------------

timer timer (
    .reset               ( reset_ss      ),
    .clk_sys               ( clk_sys       ),
    .ce                  ( ce_cpu        ), //2x in fast mode
         
    .irq                 ( timer_irq     ),
                 
    .cpu_sel             ( sel_timer     ),
    .cpu_addr            ( cpu_addr[1:0] ),
    .cpu_wr              ( !cpu_wr_n_edge ),
    .cpu_di              ( cpu_do        ),
    .cpu_do              ( timer_do      ),
    
    .SaveStateBus_Din  (SaveStateBus_Din ), 
    .SaveStateBus_Adr  (SaveStateBus_Adr ),
    .SaveStateBus_wren (SaveStateBus_wren),
    .SaveStateBus_rst  (SaveStateBus_rst ),
    .SaveStateBus_Dout (SaveStateBus_wired_or[1])
);
    
// --------------------------------------------------------------------
// ------------------------------ video -------------------------------
// --------------------------------------------------------------------

// cpu tries to read or write the lcd controller registers
wire [12:0] video_addr;
wire video_rd, dma_rd;

wire [7:0] dma_data = (isGBC & dma_sel_wram) ? wram_do :
                        dma_sel_ext_bus ? ext_bus_di :
                            dma_sel_vram ? (isGBC&vram_bank ? vram1_do : vram_do) :
                                8'hFF;

wire lcd_clkena_int;
wire [14:0] lcd_data_int;
wire [1:0] lcd_data_gb_int;
wire [7:0] lcd_pix_x_int;
wire [7:0] lcd_pix_y_int;
wire [1:0] lcd_mode_int;
wire lcd_vsync_int;

// Registered core LCD colour, before the SGB palette overlay (see lcd_data assign below).
reg  [14:0] lcd_data_core;

wire lcd_clkena_off;
wire [14:0] lcd_data_off;
wire [1:0] lcd_mode_off;
wire lcd_on_off;
wire lcd_vsync_off;
wire lcd_vsync_overwrite;

reg lcd_blankwait = 1'b0;

always@(posedge clk_sys)
begin
   if(reset) begin
       lcd_on <= 1'd0;
       lcd_vsync <= 1'd0;
       lcd_clkena <= 1'd0;
   end else if(ce) begin
      if (lcd_on_int && ~lcd_off_overwrite) begin
         lcd_clkena <= lcd_clkena_int;
         lcd_data_core <= lcd_data_int;
         lcd_data_gb <= lcd_data_gb_int;
         lcd_pix_x  <= lcd_pix_x_int;
         lcd_pix_y  <= lcd_pix_y_int;
         lcd_mode <= lcd_mode_int;
         lcd_on <= lcd_on_int;
         lcd_vsync <= lcd_vsync_int | lcd_vsync_overwrite;
         if (lcd_blankwait) begin
            if (lcd_vsync_int) begin
               lcd_blankwait <= 1'b0;
            end
            lcd_data_core <= lcd_data_off;
            lcd_data_gb <= 2'b00;
         end
      end else begin
         lcd_clkena <= lcd_clkena_off;
         lcd_data_core <= lcd_data_off;
         lcd_data_gb <= 2'b00;
         lcd_mode <= lcd_mode_off;
         lcd_on <= lcd_on_off;
         lcd_vsync <= lcd_vsync_off | lcd_vsync_overwrite;
         lcd_blankwait  <= 1'b1;
      end
   end else begin
      lcd_clkena <= 1'd0;
   end
end

// SGB palette overlay (merged from sgb.v): when an SGB game's palette is
// active, replace the core LCD colour with the combinational SGB palette
// lookup (keyed on lcd_pix_x/y and the 2-bit lcd_data_gb index). Otherwise
// pass the registered core colour straight through. sgb_pal_out / sgb_pal_en
// are produced by the SGB engine inlined at the bottom of this module.
assign lcd_data = (isSGB & sgb_pal_en) ? sgb_pal_out : lcd_data_core;

// SGB taps: mirror of the output stage above, but sourced from the main
// video path only. Never muxes in videoBypass, so sgb.v sees a clean
// constant "LCD off" while the game's LCD is disabled and true frame
// timing otherwise (see port comment above). Same register shape as
// lcd_clkena (1-cycle pulse on ce edges) so sgb.v's lcd_clkena_captured
// logic keeps its alignment.
always@(posedge clk_sys)
begin
   if(reset) begin
       lcd_clkena_sgb   <= 1'd0;
       lcd_data_gb_sgb  <= 2'd0;
       lcd_mode_sgb     <= 2'd0;
       lcd_on_sgb       <= 1'd0;
   end else if(ce) begin
       lcd_clkena_sgb   <= lcd_on_int ? lcd_clkena_int  : 1'd0;
       lcd_data_gb_sgb  <= lcd_on_int ? lcd_data_gb_int : 2'd0;
       lcd_mode_sgb     <= lcd_on_int ? lcd_mode_int    : 2'd0;
       lcd_on_sgb       <= lcd_on_int;
   end else begin
       lcd_clkena_sgb   <= 1'd0;
   end
end

video video (
    .reset       ( reset_ss         ),
    .clk         ( clk_sys       ),
    .ce          ( ce            ),   // 4Mhz
    .ce_cpu      ( ce_cpu        ),   //can be 2x in cgb double speed mode
    .isGBC       ( isGBC         ),
    .isGBC_mode  ( isGBC_mode    ),  //enable GBC mode during bootstrap rom
    .megaduck    ( megaduck      ),

    .boot_rom_en ( boot_rom_enabled ),
    .customPaletteEna ( customPaletteEna ),
    .paletteBGIn      ( paletteBGIn ),
    .paletteOBJ0In     ( paletteOBJ0In ),
    .paletteOBJ1In     ( paletteOBJ1In ),
    .gpd_out         ( gpd     ),

    .irq         ( video_irq     ),
    .vblank_irq  ( vblank_irq    ),


    .cpu_sel_reg ( sel_video_reg ),
    .cpu_sel_oam ( sel_video_oam ),
    .cpu_addr    ( cpu_addr[7:0] ),
    .cpu_wr      ( !cpu_wr_n_edge ),
    .cpu_di      ( cpu_do        ),
    .cpu_do      ( video_do      ),
    
    .lcd_on      ( lcd_on_int        ),
    .lcd_clkena  ( lcd_clkena_int),
    .lcd_data    ( lcd_data_int      ),
    .lcd_data_gb ( lcd_data_gb_int   ),
    .lcd_pix_x   ( lcd_pix_x_int     ),
    .lcd_pix_y   ( lcd_pix_y_int     ),
    .mode        ( lcd_mode_int      ),
    .lcd_vsync   ( lcd_vsync_int     ),

    .oam_cpu_allow  ( oam_cpu_allow  ),
    .vram_cpu_allow ( vram_cpu_allow ),
    .vram_rd     ( video_rd      ),
    .vram_addr   ( video_addr    ),
    .vram_data   ( vram_do       ),
    
    // vram connection bank1 (GBC)
    .vram1_data  ( vram1_do      ),
    
    .dma_rd      ( dma_rd        ),
    .dma_addr    ( dma_addr      ),
    .dma_data    ( dma_data      ),
   
   .Savestate_OAMRAMAddr      (Savestate_RAMAddr[7:0]),
   .Savestate_OAMRAMRWrEn     (Savestate_RAMRWrEn[2]),
   .Savestate_OAMRAMWriteData (Savestate_RAMWriteData),
   .Savestate_OAMRAMReadData  (Savestate_RAMReadData_ORAM),
    
    .SaveStateBus_Din  (SaveStateBus_Din ), 
    .SaveStateBus_Adr  (SaveStateBus_Adr ),
    .SaveStateBus_wren (SaveStateBus_wren),
    .SaveStateBus_rst  (SaveStateBus_rst ),
    .SaveStateBus_Dout (SaveStateBus_wired_or[4])
);

videoBypass videoBypass (
    .reset       ( reset_ss      ),
    .clk         ( clk_sys       ),
    .ce          ( ce            ),   // 4Mhz
    .ce_cpu      ( ce_cpu        ),   //can be 2x in cgb double speed mode
    .isGBC       ( isGBC         ),
    .isGBC_mode  ( isGBC_mode    ),  //enable GBC mode during bootstrap rom
    .megaduck    ( megaduck      ),

    .boot_rom_en ( boot_rom_enabled ),
    .paletteOff ( paletteOff ),
    .customPaletteEna ( customPaletteEna ),
    .paletteBGIn      ( paletteBGIn ),

    .cpu_sel_reg ( sel_video_reg ),
    .cpu_sel_oam ( sel_video_oam ),
    .cpu_addr    ( cpu_addr[7:0] ),
    .cpu_wr      ( !cpu_wr_n_edge),
    .cpu_di      ( cpu_do        ),

    .lcd_on      ( lcd_on_off     ),
    .lcd_clkena  ( lcd_clkena_off ),
    .lcd_data    ( lcd_data_off   ),
    .mode        ( lcd_mode_off   ),
    .lcd_vsync       ( lcd_vsync_off  ),
    .overwrite       ( lcd_off_overwrite  ),
    .vsync_overwrite ( lcd_vsync_overwrite  ),
    
    .mode_real       (lcd_mode_int)
);

// total 8k/16k (CGB) vram from $8000 to $9fff
wire cpu_wr_vram = sel_vram && !cpu_wr_n && vram_cpu_allow;

wire hdma_rd;
wire hdma_read_wram_bus;
wire hdma_read_ext_bus;
wire [7:0] vram_di = (hdma_read_wram_bus) ? wram_do :
                        (hdma_read_ext_bus) ? ext_bus_di :
                        cpu_do;

wire vram_wren = video_rd?1'b0:!vram_bank&&((hdma_rd&&isGBC)||cpu_wr_vram);
wire vram1_wren = video_rd?1'b0:vram_bank&&((hdma_rd&&isGBC)||cpu_wr_vram);

wire [15:0] hdma_target_addr;
wire [12:0] vram_addr = video_rd?video_addr:(hdma_rd&&isGBC)?hdma_target_addr[12:0]:(dma_rd&&dma_sel_vram)?dma_addr[12:0]:cpu_addr[12:0];

wire [7:0] Savestate_RAMReadData_VRAM0, Savestate_RAMReadData_VRAM1;

dpramV #(13) vram0 (
    .clock_a   (clk_sys  ),
    .ce_a      (ce_cpu),
    .address_a (vram_addr),
    .wren_a    (vram_wren),
    .data_a    (vram_di  ),
    .q_a       (vram_do  ),
    
    .clock_b   (clk_sys),
    .address_b (Savestate_RAMAddr[12:0]),
    .wren_b    (Savestate_RAMRWrEn[1] & !Savestate_RAMAddr[13]),
    .data_b    (Savestate_RAMWriteData),
    .q_b       (Savestate_RAMReadData_VRAM0)
);

//separate 8k for vbank1 for gbc because of BG reads
dpramV #(13) vram1 (
    .clock_a   (clk_sys   ),
    .ce_a      (ce_cpu),
    .address_a (vram_addr ),
    .wren_a    (vram1_wren),
    .data_a    (vram_di   ),
    .q_a       (vram1_do  ),
    
    .clock_b   (clk_sys),
    .address_b (Savestate_RAMAddr[12:0]),
    .wren_b    (Savestate_RAMRWrEn[1] & Savestate_RAMAddr[13]),
    .data_b    (Savestate_RAMWriteData),
    .q_b       (Savestate_RAMReadData_VRAM1)
);

assign Savestate_RAMReadData_VRAM = Savestate_RAMAddr[13] ? Savestate_RAMReadData_VRAM1 : Savestate_RAMReadData_VRAM0;

//GBC VRAM banking

assign SS_Top_BACK[22] = vram_bank;

always @(posedge clk_sys) begin
    if(reset_ss)
        vram_bank <= SS_Top[22]; // 1'd0;
    else if (ce_cpu) begin
        if((cpu_addr == 16'hff4f) && !cpu_wr_n_edge && isGBC)
            vram_bank <= cpu_do[0];
    end
end

// --------------------------------------------------------------------
// -------------------------- HDMA engine(GBC) ------------------------
// --------------------------------------------------------------------

hdma hdma(
    .reset            ( reset_ss         ),
    .clk                  ( clk_sys       ),
    .ce                ( ce_cpu         ),
    .speed               ( cpu_speed     ),
    
    // cpu register interface
    .sel_reg           ( sel_hdma      ),
    .addr                  ( cpu_addr[3:0] ),
    .wr                ( !cpu_wr_n_edge   ),
    .dout                  ( hdma_do       ),
    .din               ( cpu_do        ),
    
    .lcd_mode          ( lcd_mode_int     ),
    
    // dma connection
    .hdma_rd           ( hdma_rd          ),
    .hdma_active       ( hdma_active      ),
    .hdma_source_addr  ( hdma_source_addr ),
    .hdma_target_addr  ( hdma_target_addr ),
    
    .SaveStateBus_Din  (SaveStateBus_Din ), 
    .SaveStateBus_Adr  (SaveStateBus_Adr ),
    .SaveStateBus_wren (SaveStateBus_wren),
    .SaveStateBus_rst  (SaveStateBus_rst ),
    .SaveStateBus_Dout (SaveStateBus_wired_or[2])
);

// --------------------------------------------------------------------
// -------------------------- zero page ram ---------------------------
// --------------------------------------------------------------------

// 127 bytes internal zero page ram from $ff80 to $fffe
wire cpu_wr_zpram = sel_zpram && !cpu_wr_n;

dpramV #(
    .addr_width(7),
    .data_width(8)
    ) zpram (
    .clock_a   (clk_sys      ),
    .ce_a      (ce_cpu),
    .address_a (cpu_addr[6:0]),
    .wren_a    (cpu_wr_zpram ),
    .data_a    (cpu_do       ),
    .q_a       (zpram_do     ),
    
    .clock_b   (clk_sys),
    .address_b (Savestate_RAMAddr[6:0]),
    .wren_b    (Savestate_RAMRWrEn[3]),
    .data_b    (Savestate_RAMWriteData),
    .q_b       (Savestate_RAMReadData_ZRAM)
);

// --------------------------------------------------------------------
// -------------------------- 8k/32k(GBC) work ram  -------------------
// --------------------------------------------------------------------

// Separate WRAM bus on GBC.
assign hdma_read_wram_bus = (isGBC & hdma_rd & hdma_sel_wram);
wire dma_read_wram_bus = (dma_rd & dma_sel_wram);

wire wram_wren = ~isGBC ? (ext_bus_wram_sel & ext_bus_wr) :
                    (sel_wram & ~cpu_wr_n_edge & ~hdma_read_wram_bus & ~dma_read_wram_bus);

wire [12:0] wram_addr_i = ~isGBC ? ext_bus_addr[12:0] :
                            hdma_read_wram_bus ? hdma_source_addr[12:0] :
                                dma_read_wram_bus ? dma_addr[12:0] :
                                cpu_addr[12:0];

// 0 -> 1
wire [2:0] wram_bank_o = (!wram_bank ? 3'd1 : wram_bank);
wire [14:0] wram_addr = (wram_addr_i[12]) ? { wram_bank_o, wram_addr_i[11:0] }  // bank 1-7 $D000-DFFF
                                          : {        3'd0, wram_addr_i[11:0] }; // bank 0   $C000-CFFF
                              
dpramV #(15) wram ( 
    .clock_a   (clk_sys),
    .ce_a      (ce_cpu),
    .address_a (wram_addr),
    .wren_a    (wram_wren),
    .data_a    (cpu_do),
    .q_a       (wram_do),
    
    .clock_b   (clk_sys),
    .address_b (Savestate_RAMAddr[14:0]),
    .wren_b    (Savestate_RAMRWrEn[0]),
    .data_b    (Savestate_RAMWriteData),
    .q_b       (Savestate_RAMReadData_WRAM)
);

//GBC WRAM banking
assign SS_Top_BACK[2:0] = wram_bank;

always @(posedge clk_sys) begin
	if(reset_ss)
		wram_bank <= SS_Top[2:0]; // 3'd0;
	else if(ce_cpu && sel_wram_bank && !cpu_wr_n_edge) begin
		wram_bank <= cpu_do[2:0];
	end
end

// --------------------------------------------------------------------
// ------------------------ internal boot rom -------------------------
// --------------------------------------------------------------------

// writing 01(GB/SGB) or 11(GBC) to $ff50 disables the internal rom.
// SGB boots natively with isGBC=1, but its boot ROM must hand off with A=$01
// (the SGB identity the game checks), so accept FF50 bit0 as an unmap when
// isSGB -- matching the $01 unmap the pure-DMG path uses.

assign SS_Top_BACK[23] = boot_rom_enabled;

always @(posedge clk_sys) begin
    if(reset_ss)
        boot_rom_enabled <= SS_Top[23] & ~skip_boot_rom;
    else if (ce) begin
        if((cpu_addr == 16'hff50) && !cpu_wr_n_edge)
        begin
          if ((isGBC && cpu_do[7:0]==8'h11) || (!isGBC && cpu_do[0]) || (isSGB && cpu_do[0]))
                  boot_rom_enabled <= 1'b0;
        end
    end
end
            
// combine boot rom data with cartridge data

wire [15:0] boot_rom_addr = (isGBC && hdma_rd) ? hdma_source_addr : cpu_addr;

//  0- FF bootrom 1st part
//100-1FF Cart Header
//200-8FF bootrom 2nd part
assign sel_boot_rom_cgb = isGBC && (boot_rom_addr[15:8] >= 8'h02 && boot_rom_addr[15:8] <= 8'h08);
assign sel_boot_rom = boot_rom_enabled && (!boot_rom_addr[15:8] || sel_boot_rom_cgb) && ~megaduck;


// $000-8FF: GBC
// $900-9FF: DMG/SGB (SGB boot ROM sets A=$01 for DMG/SGB mode;
//           palette commands handled externally by sgb.v)
wire [11:0] boot_addr =
        isGBC ? boot_rom_addr[11:0] :
        { 4'h9, boot_rom_addr[7:0] };

wire boot_download = cgb_boot_download | dmg_boot_download;
wire [10:0] boot_wr_addr =
        dmg_boot_download ? {4'h9, ioctl_addr[7:1] } :
        ioctl_addr[11:1];

wire [7:0] boot_q;

dpram_difV #(12,8,11,16,"BootROMs/cgb_boot.hex") boot_rom (
    .clock (clk_sys),

    .address_a (boot_addr),
    .q_a (boot_q)
);

reg [7:0] boot_do_gba = 8'd0;

always @(boot_addr or boot_q) begin
    case (boot_addr)
        12'h0F2: boot_do_gba = 8'h00;
        12'h0F3: boot_do_gba = 8'h00;
        12'h0F5: boot_do_gba = 8'hCD;
        12'h0F6: boot_do_gba = 8'hD0;
        12'h0F7: boot_do_gba = 8'h05;
        12'h0F8: boot_do_gba = 8'hAF;
        12'h0F9: boot_do_gba = 8'hE0;
        12'h0FA: boot_do_gba = 8'h70;
        12'h0FB: boot_do_gba = 8'h04;
        12'h409: boot_do_gba = 8'h80;
        12'h40A: boot_do_gba = 8'hFF;
        default: boot_do_gba = boot_q;
    endcase
end

assign boot_do = (isGBC && boot_gba_en && real_cgb_boot) ? boot_do_gba : boot_q;
// --------------------------------------------------------------------
// ------------------ External bus (WRAM, Cartridge) ------------------
// --------------------------------------------------------------------

// Emulate cartridge open bus behavior.
// Some games read from Cart RAM while it is disabled or not present and
// expect to read the previous ROM byte instead of $FF otherwise they freeze.
// Ex. Tokyo Disneyland - Fantasy Tour (in Minnie's house) and
// Daiku no Gen-san - Kachikachi no Tonkachi ga Kachi (Stage 1-4)

reg [7:0] open_bus_data;
reg [2:0] open_bus_cnt;

assign SS_Top2_BACK[ 7: 0] = open_bus_data;
assign SS_Top2_BACK[10: 8] = open_bus_cnt;

always @(posedge clk_sys) begin
    if(reset_ss) begin
        open_bus_data <= SS_Top2[ 7: 0]; // 8'd0;
        open_bus_cnt  <= SS_Top2[10: 8]; // 3'd0;
    end else if (ce) begin
        open_bus_data <= ext_bus_di;
        if ( (~isGBC & ext_bus_wram_sel) | (cart_sel & cart_oe) ) begin
            open_bus_cnt <= 0;
        end else if (~&open_bus_cnt) begin
            open_bus_cnt <= open_bus_cnt + 1'b1;
            if (open_bus_cnt == 3'd4) open_bus_data <= 8'hFF; // Slow pull-up
        end
    end
end

assign cart_di = cpu_do;

assign hdma_read_ext_bus = (isGBC & hdma_rd & hdma_sel_ext_bus);
wire dma_read_ext_bus = (dma_rd & dma_sel_ext_bus);

assign nCS = ~( hdma_read_ext_bus ? hdma_sel_cram : dma_read_ext_bus ? dma_sel_ext_ram : (sel_cram | sel_wram) );

wire [15:0] ext_bus_i = hdma_read_ext_bus ? hdma_source_addr : dma_read_ext_bus ? dma_addr : cpu_addr;

// External A15 is not directly connected to internal A15. It does not go low when accessing the boot rom.
assign ext_bus_a15 = ext_bus_i[15] | sel_boot_rom;
assign ext_bus_addr = ext_bus_i[14:0];

assign ext_bus_rd = hdma_read_ext_bus | dma_read_ext_bus | (sel_ext_bus & ~cpu_rd_n);
assign ext_bus_wr = (sel_ext_bus & ~cpu_wr_n) & ~hdma_read_ext_bus & ~dma_read_ext_bus;

assign ext_bus_wram_sel = ~nCS &  ext_bus_addr[14];
assign ext_bus_cram_sel = ~nCS & ~ext_bus_addr[14];
assign ext_bus_rom_sel  = ~ext_bus_a15;

assign ext_bus_di = (~isGBC & ext_bus_wram_sel) ? wram_do :
                        (cart_sel & cart_oe) ? cart_do : cart_do;

assign cart_sel = ext_bus_rom_sel | ext_bus_cram_sel;
assign cart_rd = cart_sel & ext_bus_rd;
wire cart_wrold = cart_sel & ext_bus_wr;
assign cart_wr = cart_sel & WRITE_CYCLE &  ~hdma_read_ext_bus & ~dma_read_ext_bus;


assign DMA_on = hdma_read_ext_bus | dma_read_ext_bus;

// --------------------------------------------------------------------
// ------------------------ savestates -------------------------
// --------------------------------------------------------------------

wire savestate_savestate;
wire savestate_loadstate;
wire [31:0] savestate_address;
wire savestate_busy;
wire savestate_loaded;

assign SaveStateExt_Din  = SaveStateBus_Din;
assign SaveStateExt_Adr  = SaveStateBus_Adr;
assign SaveStateExt_wren = SaveStateBus_wren;
assign SaveStateExt_rst  = SaveStateBus_rst;
assign SaveStateExt_load = savestate_loaded;

assign Savestate_CRAMAddr      = Savestate_RAMAddr;    
assign Savestate_CRAMRWrEn     = Savestate_RAMRWrEn[4];
assign Savestate_CRAMWriteData = Savestate_RAMWriteData;

wire [63:0] SaveStateBus_Dout  = SaveStateBus_wired_or[0] | SaveStateBus_wired_or[1] | SaveStateBus_wired_or[2] | SaveStateBus_wired_or[3] |
                                 SaveStateBus_wired_or[4] | SaveStateBus_wired_or[5] | SaveStateBus_wired_or[6] | SaveStateBus_wired_or[7] |
                                 SaveStateExt_Dout;
 
wire sleep_rewind, sleep_savestates;

gb_savestates gb_savestates (
   .clk                    (clk_sys),
   .reset_in               (reset_r),
   .reset_out              (reset_ss),
   
   .load_done              (savestate_loaded),
   
   .increaseSSHeaderCount  (increaseSSHeaderCount),
   .save                   (savestate_savestate),
   .load                   (savestate_loadstate),
   .savestate_address      (savestate_address),
   .savestate_busy         (savestate_busy),      
   
   .cart_ram_size          (cart_ram_size),
   
   .lcd_vsync              (lcd_vsync_int),
   
   .BUS_Din                (SaveStateBus_Din), 
   .BUS_Adr                (SaveStateBus_Adr), 
   .BUS_wren               (SaveStateBus_wren), 
   .BUS_rst                (SaveStateBus_rst), 
   .BUS_Dout               (SaveStateBus_Dout),
      
   //.loading_savestate      (loading_savestate),
   //.saving_savestate       (saving_savestate),
   .sleep_savestate        (sleep_savestates),
   .clock_ena_in           (ce_2x),
   
// synthesis translate_off
   .Save_RAMAddr           (Savestate_RAMAddr),   
   .Save_RAMWrEn           (Savestate_RAMRWrEn),
   .Save_RAMWriteData      (Savestate_RAMWriteData),
   .Save_RAMReadData_WRAM  (Savestate_RAMReadData_WRAM),
   .Save_RAMReadData_VRAM  (Savestate_RAMReadData_VRAM),
   .Save_RAMReadData_ORAM  (Savestate_RAMReadData_ORAM),
   .Save_RAMReadData_ZRAM  (Savestate_RAMReadData_ZRAM),
   .Save_RAMReadData_CRAM  (Savestate_CRAMReadData),
// synthesis translate_on
            
   .bus_out_Din            (SAVE_out_Din),   
   .bus_out_Dout           (SAVE_out_Dout),  
   .bus_out_Adr            (SAVE_out_Adr),   
   .bus_out_rnw            (SAVE_out_rnw),   
   .bus_out_ena            (SAVE_out_ena),   
   .bus_out_be             (SAVE_out_be),   
   .bus_out_done           (SAVE_out_done)
);

gb_statemanager #(58720256, 33554432) gb_statemanager (
   .clk                 (clk_sys),
   .reset               (reset_r),

   .rewind_on           (rewind_on),    
   .rewind_active       (rewind_active),

   .savestate_number    (savestate_number),
   .save                (save_state),
   .load                (load_state),

   .sleep_rewind        (sleep_rewind),
   .vsync               (lcd_vsync_int),       

   .request_savestate   (savestate_savestate),
   .request_loadstate   (savestate_loadstate),
   .request_address     (savestate_address),  
   .request_busy        (savestate_busy)     
);

assign sleep_savestate = sleep_rewind | sleep_savestates;



// ============================================================================
// ==================== SGB engine (merged from sgb.v) ========================
// ============================================================================
// Super Game Boy packet parsing, palette/attribute injection and multitap
// joypad logic, inlined so SGB runs natively inside the core. It is gated by
// the static `isSGB` input (set once from the cartridge-header detect) -- no
// runtime reset and no isGBC toggling. Border rendering is intentionally NOT
// included (removed for area; the FPGA is utilisation-limited).
//
// Video tap: the frame scanner samples the MAIN-video taps (lcd_clkena_sgb,
// lcd_data_gb_sgb, lcd_mode_sgb, lcd_on_sgb) -- never the videoBypass-muxed
// display outputs -- so PAL_TRN/ATTR_TRN captures stay clean while the game's
// LCD is off (Pokemon Red/Blue, Donkey Kong '94 black-boot fix). The 2-bit DMG
// pixel index is lcd_data_gb_sgb[1:0]; palette colours are 15-bit RGB555 with
// no channel swap, matching the LCD pipeline.
//
// Internal declarations below replace the ports the standalone sgb.v had; the
// engine's other inputs map onto gb.v signals: reset->reset_ss, sgb_en->isSGB,
// lcd_clkena->lcd_clkena_sgb, lcd_data_gb->lcd_data_gb_sgb, lcd_data->lcd_data_core,
// lcd_mode->lcd_mode_sgb, lcd_on->lcd_on_sgb, pal_read_idx->lcd_data_gb,
// joy_do->sgb_joy_do. lcd_pix_x/lcd_pix_y/lcd_vsync/joy_p54 keep their names.
wire        tint = 1'b0;           // no tint (was an sgb.v input, tied off)
wire [14:0] sgb_pal_out;           // combinational SGB palette colour (overlay)
wire [3:0]  sgb_joy_do;            // SGB multitap joypad nibble
reg         sgb_pal_en;            // SGB palette overlay active
reg  [14:0] sgb_lcd_data;          // (retained, unused: registered SGB pixel)
reg         sgb_lcd_clkena;
reg  [1:0]  sgb_lcd_mode;
reg         sgb_lcd_on;
reg         sgb_lcd_freeze;
reg         sgb_lcd_vsync;

localparam CMD_PAL01    = 5'h00;
localparam CMD_PAL23    = 5'h01;
localparam CMD_PAL03    = 5'h02;
localparam CMD_PAL12    = 5'h03;
localparam CMD_ATTR_BLK = 5'h04;
localparam CMD_ATTR_LIN = 5'h05;
localparam CMD_ATTR_DIV = 5'h06;
localparam CMD_ATTR_CHR = 5'h07;
localparam CMD_PAL_SET  = 5'h0A;
localparam CMD_PAL_TRN  = 5'h0B;
localparam CMD_MLT_REQ  = 5'h11;
localparam CMD_CHR_TRN  = 5'h13;
localparam CMD_PCT_TRN  = 5'h14;
localparam CMD_ATTR_TRN = 5'h15;
localparam CMD_ATTR_SET = 5'h16;
localparam CMD_MASK_EN  = 5'h17;
localparam CMD_CART_1   = 5'h1E;
localparam CMD_CART_2   = 5'h1F;


wire p14 = joy_p54[0];
wire p15 = joy_p54[1];

reg old_p15, old_p14;
reg [7:0] data;
reg [3:0] byte_cnt;
reg [2:0] cnt, packet_cnt;
reg [2:0] length;
reg byte_done, packet_end;
reg [8:0] data_set_len, data_set_cnt;
reg [2:0] data_set_byte_cnt;
reg [4:0] cmd;

reg [1:0] mlt_ctrl;
reg trn_start, pal_set, attr_set;

reg [1:0] pal0123_no, pal0123_col_no;
reg [14:0] pal_color;
reg pal0123_wr;

reg [8:0] sys_pal_no[4];
reg [5:0] attr_file_no;
reg [1:0] mask_en;
reg cancel_mask;

reg [2:0] attr_blk_ctrl;
reg [5:0] attr_blk_pal;
reg [4:0] attr_blk_x1,attr_blk_x2,attr_blk_y1,attr_blk_y2;
reg attr_blk_set;

reg [7:0] attr_lin_data;
reg attr_lin_set;

reg [5:0] attr_div_pal;
reg attr_div_hv;
reg [4:0] attr_div_xy;
reg attr_div_set;

reg [7:0] attr_chr_data;
reg [4:0] attr_chr_data_x;
reg [8:0] attr_chr_data_offset, attr_chr_len;
reg attr_chr_set, attr_chr_dir, attr_chr_start;

always @(posedge clk_sys) begin
	 if (reset_ss) begin
		attr_file_no <= 0;
		packet_cnt <= 0;
		cnt <= 0;
		byte_cnt <= 0;
		byte_done <= 0;
		data_set_len <= 0;
		data_set_cnt <= 0;
		mask_en <= 0;
		packet_end <= 1'b1;
		mlt_ctrl <= 0;
	 end else if (ce) begin
		old_p15 <= p15;
		old_p14 <= p14;

		if (isSGB) begin
			// MiSTer edge-based packet detection
			if (~p15 & ~p14) begin
				{cnt, byte_cnt, packet_end} <= 0;
			end

			if (old_p15 & old_p14 & (p15 ^ p14)) begin
				if (~packet_end) begin
					data <= {~p15, data[7:1]};
					cnt <= cnt + 1'b1;
					if (&cnt) byte_done <= 1'b1;
				end
			end

			if ((old_p15 ^ p15) & (old_p15 ^ old_p14) & (p15 ^ p14)) begin
				packet_end <= 1'b1;
			end
		end

		trn_start <= 0;
		pal_set <= 0;
		attr_set <= 0;
		pal0123_wr <= 0;
		attr_blk_set <= 0;
		attr_lin_set <= 0;
		attr_div_set <= 0;
		attr_chr_set <= 0;

		if (pal_cancel_mask | attr_cancel_mask) mask_en <= 0;

		if (byte_done) begin
			byte_done <= 0;
			byte_cnt <= byte_cnt + 1'b1;

			if (!packet_cnt && !byte_cnt) begin
				{cmd,length} <= data;
				if (data[7:3] == CMD_CART_1 || data[7:3] == CMD_CART_2) begin
					length <= 1;
				end
			end

			case (cmd)
				CMD_MLT_REQ: begin
					if (byte_cnt == 5'd1) mlt_ctrl <= data[1:0];
				end
				CMD_CHR_TRN,
				CMD_PCT_TRN,
				CMD_PAL_TRN,
				CMD_ATTR_TRN: begin
					if (byte_cnt == 5'd1) trn_start <= 1'b1;
				end
				CMD_PAL01,
				CMD_PAL23,
				CMD_PAL03,
				CMD_PAL12: begin
					if (byte_cnt >= 4'd1 && byte_cnt <= 4'd14) begin
						if (byte_cnt[0]) pal_color[7:0] <= data;
						else begin
							pal_color[14:8] <= data[6:0];
							pal0123_wr <= 1'b1;
						end
					end

					case (byte_cnt)
						1:    pal0123_col_no <= 2'd0;
						3,9:  pal0123_col_no <= 2'd1;
						5,11: pal0123_col_no <= 2'd2;
						7,13: pal0123_col_no <= 2'd3;
					endcase

					case ({cmd,byte_cnt})
						{CMD_PAL01,4'd1},
						{CMD_PAL23,4'd1},
						{CMD_PAL03,4'd1},
						{CMD_PAL12,4'd1}: pal0123_no <= 2'd0;

						{CMD_PAL01,4'd9},
						{CMD_PAL12,4'd3}: pal0123_no <= 2'd1;

						{CMD_PAL12,4'd9},
						{CMD_PAL23,4'd3}: pal0123_no <= 2'd2;

						{CMD_PAL23,4'd9},
						{CMD_PAL03,4'd9}: pal0123_no <= 2'd3;
					endcase
				end
				CMD_PAL_SET:
					case (byte_cnt)
						1: sys_pal_no[0][7:0] <= data;
						2: sys_pal_no[0][8]   <= data[0];
						3: sys_pal_no[1][7:0] <= data;
						4: sys_pal_no[1][8]   <= data[0];
						5: sys_pal_no[2][7:0] <= data;
						6: sys_pal_no[2][8]   <= data[0];
						7: sys_pal_no[3][7:0] <= data;
						8: sys_pal_no[3][8]   <= data[0];
						9: begin
							attr_file_no <= data[5:0];
							cancel_mask <= data[6];
							if (data[7]) attr_set <= 1'b1;
							pal_set <= 1'b1;
						end
					endcase
				CMD_ATTR_SET: begin
					if (byte_cnt == 5'd1) begin
						attr_file_no <= data[5:0];
						cancel_mask <= data[6];
						attr_set <= 1'b1;
					end
				end
				CMD_ATTR_BLK: begin
					if (!packet_cnt && byte_cnt == 5'd1) begin
						data_set_len <= {4'd0,data[4:0]};
						data_set_cnt <= 0;
						data_set_byte_cnt <= 0;
					end

					if (|data_set_len && data_set_cnt < data_set_len) begin
						data_set_byte_cnt <= data_set_byte_cnt + 1'b1;

						case(data_set_byte_cnt)
							0: attr_blk_ctrl <= data[2:0];
							1: attr_blk_pal <= data[5:0];
							2: attr_blk_x1 <= data[4:0];
							3: attr_blk_y1 <= data[4:0];
							4: attr_blk_x2 <= data[4:0];
							5: begin
								attr_blk_y2 <= data[4:0];
								attr_blk_set <= 1'b1;
								data_set_byte_cnt <= 0;
								data_set_cnt <= data_set_cnt + 1'b1;
								if (data_set_cnt + 1'b1 == data_set_len) begin
									data_set_len <= 0;
								end
							end
						endcase
					end
				end
				CMD_ATTR_LIN: begin
					if (!packet_cnt && byte_cnt == 5'd1) begin
						data_set_len <= {2'd0,data[6:0]};
						data_set_cnt <= 0;
					end

					if (|data_set_len && data_set_cnt < data_set_len) begin
						attr_lin_data <= data;
						attr_lin_set <= 1'b1;
						data_set_cnt <= data_set_cnt + 1'b1;
						if (data_set_cnt + 1'b1 == data_set_len) begin
							data_set_len <= 0;
						end
					end
				end
				CMD_ATTR_DIV: begin
					case (byte_cnt)
						1: {attr_div_hv,attr_div_pal} <= data[6:0];
						2: begin
							attr_div_xy <= data[4:0];
							attr_div_set <= 1'b1;
						end
					endcase
				end
				CMD_ATTR_CHR: begin
					if (!packet_cnt) begin
						case (byte_cnt)
							1: attr_chr_data_x <= data[4:0];
							2: attr_chr_data_offset <= {data[4:0], 4'b0000} + {2'b00, data[4:0], 2'b00}; // 20*x = 16*x + 4*x, no DSP
							3: attr_chr_len[7:0] <= data;
							4: attr_chr_len[8] <= data[0];
							5: begin
								attr_chr_dir <= data[0];
								data_set_len <= attr_chr_len;
								data_set_cnt <= 0;
								attr_chr_start <= 1'b1;
							end
						endcase
					end

					if (|data_set_len && data_set_cnt < data_set_len) begin
						attr_chr_data <= data;
						attr_chr_set <= 1'b1;
						if (|data_set_cnt) attr_chr_start <= 0;
						data_set_cnt <= data_set_cnt + 9'd4;
						if (data_set_cnt + 9'd4 >= data_set_len) begin
							data_set_len <= 0;
						end
					end
				end
				CMD_MASK_EN: begin
					if (byte_cnt == 5'd1) mask_en <= data[1:0];
				end
			endcase

			if (&byte_cnt) begin
				packet_cnt <= packet_cnt + 1'b1;
				if (packet_cnt + 1'b1 >= length) begin
					packet_cnt <= 0;
					packet_end <= 1'b1;
					data_set_len <= 0;
				end
			end
		end
	end

end


// Joypad multiplexer (SGB multi-player protocol)
reg [1:0] joypad_id;

always @(posedge clk_sys) begin
	if (reset_ss) begin
		joypad_id <= 0;
	end else if (ce) begin
		joypad_id <= (joypad_id & mlt_ctrl);
		if (isSGB & ~old_p15 & p15) begin
			joypad_id <= (joypad_id + 1'b1);
		end
	end
end

// SGB joypad response. When both P14 and P15 are high the SGB multitap returns
// the player id (~{00,id}); this *changing* id as P15 toggles is how SGB games
// detect the SGB, so it must come from the engine's joypad_id state. For every
// other read we return joy_din -- the same decoded nibble the core uses for
// non-SGB games (built in emu_system_top from the button inputs).
//
// We deliberately do NOT use the original sgb.v joystick_0/joy_data path here:
// that 8-bit joystick_0 input crossed from the pclk button domain into the busy
// core and, at this device's utilisation limit, failed to route -- buttons read
// $FF (no controls) even though the packet-based palette path kept working (and
// the stuck sgb_detected also broke the EverDrive menu's joypad). joy_din is
// already in the core clock domain and is the proven working button path, so the
// SGB joypad now behaves exactly like non-SGB (single-player; only controller 0
// is wired on this hardware). The joystick_0 / joy_data / joystick mux logic is
// removed to free routing.
assign sgb_joy_do = (isSGB & p15 & p14) ? ~{2'b00, joypad_id} : joy_din;

wire lcd_off = !lcd_on_sgb || (lcd_mode_sgb == 2'd01);
reg old_lcd_off;



// lcd_clkena_sgb from gb.v is a 1-cycle pulse on ce edges, zeroed on non-ce
// edges. Capture and hold until next ce edge for the frame scanner.
reg lcd_clkena_captured;
always @(posedge clk_sys) begin
	if (lcd_clkena_sgb) lcd_clkena_captured <= 1'b1;
	else if (ce)   lcd_clkena_captured <= 1'b0;
end

// LCD frame scanner — tracks pixel position for tile-based palette assignment
// and collects pixel data for PAL_TRN / ATTR_TRN transfers.
reg trn_en, trn_wait, frame_end;
reg [7:0] pix_x, pix_y;
reg [8:0] tile_offset;
reg [6:0] trn_data_h, trn_data_l;

wire [8:0] tile_number = {tile_offset+pix_x[7:3]};
wire [13:0] pixel_wr_addr = {tile_number[7:0], pix_y[2:0], pix_x[2:0]};
wire [15:0] trn_data = {trn_data_h, lcd_data_gb_sgb[1], trn_data_l, lcd_data_gb_sgb[0]};

always @(posedge clk_sys) begin
	if (ce) begin
		frame_end <= 0;

		old_lcd_off <= lcd_off;
		if(~old_lcd_off & lcd_off) begin
			trn_en <= 0;
			pix_x <= 0;
			pix_y <= 0;
			tile_offset <= 0;
			frame_end <= 1'b1;
		end

		if(lcd_clkena_captured & ~lcd_off) begin
			pix_x <= pix_x + 1'b1;
			if (pix_x == 8'd159) begin
				pix_x <= 0;
				pix_y <= pix_y + 1'b1;
				if(&pix_y[2:0]) tile_offset <= tile_offset + 9'd20;
			end

			if (trn_en) begin
				trn_data_h <= {trn_data_h[5:0],lcd_data_gb_sgb[1]};
				trn_data_l <= {trn_data_l[5:0],lcd_data_gb_sgb[0]};
			end

			if (pix_x == 8'd159 && pix_y == 8'd103) begin // 256 tiles
				trn_en <= 0;
			end
		end

		if (trn_start) trn_wait <= 1'b1;

		if (old_lcd_off & ~lcd_off) begin
			trn_wait <= 0;
			if (trn_wait) begin
				trn_en <= 1'b1;
			end
		end
	end

end

wire trn_data_wr = (ce && lcd_clkena_captured && trn_en && &pix_x[2:0] && !tile_number[8]);

// System palette RAM (for PAL_TRN / PAL_SET)
(* ramstyle="block" *) reg [14:0] sys_pal_ram[2048];
// Attribute files RAM (for ATTR_TRN / ATTR_SET)
(* ramstyle="block" *) reg [15:0] attr_files_ram[2048];

always @(posedge clk_sys) begin
	if (trn_data_wr && cmd == CMD_PAL_TRN) begin
		sys_pal_ram[pixel_wr_addr[13:3]] <= trn_data[14:0];
	end

	if (trn_data_wr && cmd == CMD_ATTR_TRN && pixel_wr_addr[13:3] < 11'd2025) begin
		attr_files_ram[pixel_wr_addr[13:3]] <= {trn_data[7:0],trn_data[15:8]};
	end
end

// TEMP DEBUG: latches when the PAL_TRN frame-scanner capture actually writes a
// palette entry into sys_pal_ram. Used by the sgb_debug overlay to tell whether
// the PAL_TRN upload is populating the palette table (see sgb_pal_out below).
reg pal_trn_captured;
always @(posedge clk_sys) begin
	if (reset_ss) pal_trn_captured <= 1'b0;
	else if (trn_data_wr && cmd == CMD_PAL_TRN) pal_trn_captured <= 1'b1;
end

// Palette storage
reg [14:0] sys_pal_data, pal_wr_data;
reg [1:0] pal_wr_no, pal_wr_col_no;
reg [59:0] palette[4];
reg pal_set_wait, pal_set_busy, pal_wr, pal_cancel_mask, pal_clear;
reg [3:0] pal_set_cnt, pal_set_cnt_r;
reg output_sgb_pal;

always @(posedge clk_sys) begin
	if (reset_ss) begin
		output_sgb_pal <= 0;
		pal_set_busy <= 0;
		pal_set_wait <= 0;
		pal_clear <= 1'b1;
		pal_set_cnt <= 0;
	end else if (ce) begin

		pal_cancel_mask <= 0;
		pal_wr <= 0;

		if (pal_set) pal_set_wait <= 1'b1;

		// MiSTer-faithful: copy the selected system palettes at the next frame
		// boundary. pal_set_wait is sticky, so the copy is guaranteed to run on
		// the next frame_end (vblank). The previous same-cycle shortcut raced the
		// pal_set_busy/pal_set_cnt machinery and could strand a transition with
		// the palette un-copied and the mask un-cleared (black screen that did
		// not recover). One frame (~16ms) of palette latency is imperceptible.
		if (pal_set_wait & frame_end) begin
			pal_set_wait <= 0;
			pal_set_busy <= 1'b1;
			pal_set_cnt <= 0;
			pal_set_cnt_r <= 0;
		end

		sys_pal_data <= sys_pal_ram[{sys_pal_no[pal_set_cnt[3:2]], pal_set_cnt[1:0]}];

		if (pal_set_busy) begin
			pal_set_cnt <= pal_set_cnt + 1'b1;
			pal_set_cnt_r <= pal_set_cnt;

			if (&pal_set_cnt_r) begin
				pal_set_busy <= 0;
				output_sgb_pal <= 1'b1;
				if (cancel_mask) pal_cancel_mask <= 1'b1;
			end

			pal_wr <= 1'b1;
			pal_wr_data <= sys_pal_data;
			{pal_wr_no, pal_wr_col_no} <= pal_set_cnt_r;
		end

		if (pal0123_wr) begin
			output_sgb_pal <= 1'b1;
			pal_wr <= 1'b1;
			pal_wr_data <= pal_color;
			pal_wr_no <= pal0123_no;
			pal_wr_col_no <= pal0123_col_no;
		end

		if (pal_clear) begin
			pal_set_cnt <= pal_set_cnt + 1'b1;
			pal_wr <= 1'b1;
			pal_wr_data <= 0;
			{pal_wr_no, pal_wr_col_no} <= pal_set_cnt;
			if (&pal_set_cnt) pal_clear <= 0;
		end

		if (pal_wr) begin
			palette[pal_wr_no][pal_wr_col_no*15 +: 15] <= pal_wr_data;
		end

	end

end

// Attribute file storage — flat bit-vectors for atomic copy (matches MiSTer)
reg [15:0] attr_file_data;
reg [719:0] attr_file_temp, attr_file;
reg attr_file_ready;

reg attr_set_busy, attr_blk_busy, attr_lin_busy, attr_div_busy, attr_chr_busy;
reg [8:0] attr_set_cnt, attr_set_cnt_r;
reg [10:0] attr_file_ram_addr;
reg [5:0] attr_file_idx;
reg attr_cancel_mask;

reg [8:0] attr_tile_no, attr_tile_no_wr;
reg [4:0] attr_tile_cnt_x, attr_tile_cnt_y;
reg [1:0] attr_file_pal_wr;
reg attr_file_wr;

reg [8:0] attr_chr_pal_cnt;
reg [4:0] attr_chr_x;
reg [8:0] attr_chr_offset;

reg attr_clear;
always @(posedge clk_sys) begin
	if (reset_ss) begin
		attr_set_busy <= 0;
		attr_blk_busy <= 0;
		attr_lin_busy <= 0;
		attr_div_busy <= 0;
		attr_chr_busy <= 0;
		attr_clear <= 1'b1;
		attr_set_cnt <= 0;
		attr_file_ready <= 1;
	end else if (ce) begin

		attr_cancel_mask <= 0;
		attr_file_wr <= 0;

		if (attr_set) begin
			attr_set_busy <= 1'b1;
			attr_set_cnt <= 0;
			attr_file_ram_addr <= 0;
			attr_file_idx <= 0;
			attr_file_ready <= 0;
		end

		attr_file_data <= attr_files_ram[attr_file_ram_addr];

		if (attr_set_busy) begin
			if (attr_file_idx != attr_file_no) begin
				attr_file_idx <= attr_file_idx + 1'b1;
				attr_file_ram_addr <= attr_file_ram_addr + 11'd45;
			end else begin
				attr_set_cnt <= attr_set_cnt + 1'b1;
				attr_set_cnt_r <= attr_set_cnt;

				if (&attr_set_cnt[2:0]) attr_file_ram_addr <= attr_file_ram_addr + 1'b1;

				attr_file_pal_wr <= attr_file_data[~attr_set_cnt_r[2:0]*2 +: 2];
				attr_tile_no_wr <= attr_set_cnt_r;
				attr_file_wr <= 1'b1;
			end

			if (attr_file_idx == 6'd45 || attr_set_cnt_r == 9'd359) begin
				attr_set_busy <= 0;
				attr_file_ready <= 1;
				if (cancel_mask) attr_cancel_mask <= 1'b1;
			end
		end

		if (attr_blk_set) attr_blk_busy <= 1'b1;
		if (attr_lin_set) attr_lin_busy <= 1'b1;
		if (attr_div_set) attr_div_busy <= 1'b1;

		if (attr_blk_set | attr_lin_set | attr_div_set) begin
			attr_tile_no <= 0;
			attr_tile_cnt_x <= 0;
			attr_tile_cnt_y <= 0;
			attr_file_ready <= 0;
		end

		if (attr_blk_busy | attr_lin_busy | attr_div_busy) begin
			attr_tile_no <= attr_tile_no + 1'b1;
			attr_tile_cnt_x <= attr_tile_cnt_x + 1'b1;
			attr_tile_no_wr <= attr_tile_no;

			if(attr_tile_cnt_x == 5'd19) begin
				attr_tile_cnt_x <= 0;
				attr_tile_cnt_y <= attr_tile_cnt_y + 1'b1;
				if (attr_tile_cnt_y == 5'd17) begin
					attr_blk_busy <= 0;
					attr_lin_busy <= 0;
					attr_div_busy <= 0;
					if (!data_set_len | attr_div_busy) begin
						attr_file_ready <= 1;
					end
				end
			end
		end

		// ATTR_BLK
		if (attr_blk_busy) begin
			if (attr_tile_cnt_x > attr_blk_x1 && attr_tile_cnt_x < attr_blk_x2
				&& attr_tile_cnt_y > attr_blk_y1 && attr_tile_cnt_y < attr_blk_y2) begin
				if (attr_blk_ctrl[0]) begin
					attr_file_pal_wr <= attr_blk_pal[1:0];
					attr_file_wr <= 1'b1;
				end
			end else if (attr_tile_cnt_x < attr_blk_x1 || attr_tile_cnt_x > attr_blk_x2
				|| attr_tile_cnt_y < attr_blk_y1 || attr_tile_cnt_y > attr_blk_y2) begin
				if (attr_blk_ctrl[2]) begin
					attr_file_pal_wr <= attr_blk_pal[5:4];
					attr_file_wr <= 1'b1;
				end
			end else begin
				casez (attr_blk_ctrl)
					3'b001: begin
						attr_file_pal_wr <= attr_blk_pal[1:0];
						attr_file_wr <= 1'b1;
					end
					3'b100: begin
						attr_file_pal_wr <= attr_blk_pal[5:4];
						attr_file_wr <= 1'b1;
					end
					3'b?1?: begin
						attr_file_pal_wr <= attr_blk_pal[3:2];
						attr_file_wr <= 1'b1;
					end
				endcase
			end
		end

		// ATTR_LIN
		if (attr_lin_busy) begin
			if ( (attr_lin_data[7] && attr_tile_cnt_y == attr_lin_data[4:0])
				  || (~attr_lin_data[7] && attr_tile_cnt_x == attr_lin_data[4:0]) ) begin
				attr_file_wr <= 1'b1;
				attr_file_pal_wr <= attr_lin_data[6:5];
			end
		end

		// ATTR_DIV
		if (attr_div_busy) begin
			if ( (~attr_div_hv && attr_tile_cnt_x > attr_div_xy) || (attr_div_hv && attr_tile_cnt_y > attr_div_xy) ) begin
				attr_file_pal_wr <= attr_div_pal[1:0];
			end else if ( (~attr_div_hv && attr_tile_cnt_x < attr_div_xy) || (attr_div_hv && attr_tile_cnt_y < attr_div_xy) ) begin
				attr_file_pal_wr <= attr_div_pal[3:2];
			end else begin
				attr_file_pal_wr <= attr_div_pal[5:4];
			end
			attr_file_wr <= 1'b1;
		end

		// ATTR_CHR
		if (attr_chr_set) begin
			attr_chr_busy <= 1'b1;
			attr_file_ready <= 0;
			if (attr_chr_start) begin
				attr_chr_pal_cnt <= 0;
				attr_chr_x <= attr_chr_data_x;
				attr_chr_offset <= attr_chr_data_offset;
			end
		end

		if (attr_chr_busy) begin
			attr_chr_pal_cnt <= attr_chr_pal_cnt + 1'b1;
			if (&attr_chr_pal_cnt[1:0]) begin
				attr_chr_busy <= 0;
			end
			if (attr_chr_pal_cnt+1'b1 == attr_chr_len) begin
				attr_chr_busy <= 0;
				attr_file_ready <= 1;
			end

			if (attr_chr_dir) begin
				attr_chr_offset <= attr_chr_offset + 9'd20;
				if(attr_chr_offset == 9'd340) begin
					attr_chr_offset <= 0;
					attr_chr_x <= attr_chr_x + 1'b1;
					if (attr_chr_x == 5'd19) attr_chr_x <= 0;
				end
			end

			if (~attr_chr_dir) begin
				attr_chr_x <= attr_chr_x + 1'b1;
				if (attr_chr_x == 5'd19) begin
					attr_chr_x <= 0;
					attr_chr_offset <= attr_chr_offset + 9'd20;
					if (attr_chr_offset == 9'd340) attr_chr_offset <= 0;
				end
			end

			attr_tile_no_wr <= attr_chr_offset + attr_chr_x;
			case (attr_chr_pal_cnt[1:0])
				0: attr_file_pal_wr <= attr_chr_data[7:6];
				1: attr_file_pal_wr <= attr_chr_data[5:4];
				2: attr_file_pal_wr <= attr_chr_data[3:2];
				3: attr_file_pal_wr <= attr_chr_data[1:0];
			endcase
			attr_file_wr <= 1'b1;
		end

		if (attr_clear) begin
			attr_set_cnt <= attr_set_cnt + 1'b1;
			attr_file_pal_wr <= 0;
			attr_tile_no_wr <= attr_set_cnt;
			attr_file_wr <= 1'b1;
			if (attr_set_cnt == 9'd359) attr_clear <= 0;
		end

		if (attr_file_wr) begin
			attr_file_temp[attr_tile_no_wr*2 +: 2] <= attr_file_pal_wr;
		end

	end
end

// LCD palette output
reg [14:0] lcd_data_r;
reg [1:0]  lcd_data_gb_r;
reg [1:0]  pal_no;
reg lcd_clkena_r, lcd_on_r, lcd_vsync_r;
reg [1:0]  lcd_mode_r;
reg [1:0]  mask_en_r;

always @(posedge clk_sys) begin
	if (ce) begin

		if (lcd_off) begin
			mask_en_r <= mask_en;
			if (attr_file_ready) begin
				attr_file <= attr_file_temp;
			end
		end

		pal_no <= attr_file[tile_number*2 +: 2];
		lcd_data_r <= lcd_data_core;
		lcd_data_gb_r <= lcd_data_gb_sgb;
		lcd_clkena_r <= lcd_clkena_sgb;
		lcd_mode_r <= lcd_mode_sgb;
		lcd_on_r <= lcd_on_sgb;
		lcd_vsync_r <= lcd_vsync;

		if (~isSGB | ((~output_sgb_pal | tint | isGBC_game) & !mask_en_r) ) begin
			sgb_lcd_data <= lcd_data_r;
		end else if (mask_en_r == 2'd2) begin
			sgb_lcd_data <= 0;
		end else if (!lcd_data_gb_r || mask_en_r == 2'd3) begin
			sgb_lcd_data <= palette[0][14:0];
		end else begin
			sgb_lcd_data <= palette[pal_no][lcd_data_gb_r*15 +: 15];
		end

		sgb_lcd_clkena <= lcd_clkena_r;
		sgb_lcd_mode <= lcd_mode_r;
		sgb_lcd_on <= lcd_on_r;
		sgb_lcd_freeze <= isSGB && mask_en_r == 2'd1;
		sgb_pal_en <= isSGB & ( (output_sgb_pal & ~tint & ~isGBC_game) || |mask_en_r);
		sgb_lcd_vsync <= lcd_vsync_r;
	end
end

// External palette read port — combinational lookup.
// The palette select (attr_file tile) is derived from the video pixel
// counters lcd_pix_x/lcd_pix_y, which video.v captures in the same cycle as
// lcd_data_gb_sgb (lcd_data_gb). They therefore index the exact 8x8 screen tile
// the colour came from — no phase skew. (The free-running pix_x/tile_number
// lags the live pixel and is only correct for the PAL_TRN/ATTR_TRN frame
// scanner, where a constant offset is self-consistent.)
// No channel swap (SGB data is RGB555, matching LCD pipeline).
// ov_tile_number = (y/8)*20 + (x/8), computed as 16*row + 4*row + col (no DSP).
wire [8:0] ov_tile_number = {lcd_pix_y[7:3], 4'b0000} + {2'b00, lcd_pix_y[7:3], 2'b00} + {4'b0000, lcd_pix_x[7:3]};
wire [1:0] pal_no_comb = attr_file[ov_tile_number*2 +: 2];
wire [14:0] _pal_raw = (mask_en_r == 2'd2) ? 15'd0 :
                       (!lcd_data_gb || mask_en_r == 2'd3) ? palette[0][14:0] :
                       palette[pal_no_comb][lcd_data_gb*15 +: 15];
assign sgb_pal_out = _pal_raw;

endmodule
