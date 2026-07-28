// emu_system_top.v

module emu_system_top
(
    input               hclk,
    input               pclk,
    input               reset_n,
    input               POWER_GOOD,
    
    input               customPaletteEna,
    input [63:0]        paletteBGIn,
    input [63:0]        paletteOBJ0In,
    input [63:0]        paletteOBJ1In,
    input               paletteOff,
    input [1:0]         oc_lvl,      // 0=1x  1=2x  2=4x
    output              gbc_mode,
    output              isSGB_out,   // SGB game detected
    output [63:0]       gpd,
    
    input               BTN_NODIAGONAL,
    input               BTN_A,
    input               BTN_B,
    input               BTN_DPAD_DOWN,
    input               BTN_DPAD_LEFT,
    input               BTN_DPAD_RIGHT,
    input               BTN_DPAD_UP,
    input               BTN_MENU,
    input               BTN_SEL,
    input               BTN_START,
    input               MENU_CLOSED,
    output              boot_rom_enabled,
    output  [15:0]      CART_A,
    output              CART_CLK,
    output              CART_CS,
    inout   [7:0]       CART_D,
    output              CART_RD,
    inout               CART_RST,
    output              CART_WR,
    output              CART_DATA_DIR_E,

    input               IR_RX,
    output              IR_LED,

    inout               LINK_CLK,
    input               LINK_IN,
    output              LINK_OUT,

    output [15:0]       left,
    output [15:0]       right,

    output lcd_on_int,
    output lcd_off_overwrite,
    
    input               LCD_INIT_DONE,
    output              gb_lcd_clkena,
    output [14:0]       gb_lcd_data,
    output [1:0]        gb_lcd_mode,
    output              gb_lcd_on,
    output              gb_lcd_vsync,
    output              boot_done_out
    
);

    parameter SRSIZE = 15;

    reg [SRSIZE-1:0] BTN_DPAD_DOWN_sr;
    reg [SRSIZE-1:0] BTN_DPAD_UP_sr;
    reg [SRSIZE-1:0] BTN_DPAD_LEFT_sr;
    reg [SRSIZE-1:0] BTN_DPAD_RIGHT_sr;

    reg [SRSIZE-1:0] BTN_START_sr;
    reg [SRSIZE-1:0] BTN_SEL_sr;
    reg [SRSIZE-1:0] BTN_B_sr;
    reg [SRSIZE-1:0] BTN_A_sr;

    reg BTN_DPAD_DOWN_filtered;
    reg BTN_DPAD_UP_filtered;
    reg BTN_DPAD_LEFT_filtered;
    reg BTN_DPAD_RIGHT_filtered;
    reg BTN_START_filtered;
    reg BTN_SEL_filtered;
    reg BTN_B_filtered;
    reg BTN_A_filtered;
    
    reg BTN_DPAD_DOWN_filtered_dir;
    reg BTN_DPAD_UP_filtered_dir;
    reg BTN_DPAD_LEFT_filtered_dir;
    reg BTN_DPAD_RIGHT_filtered_dir;

    always@(posedge pclk)
    begin
        BTN_DPAD_DOWN_sr <= {BTN_DPAD_DOWN_sr[SRSIZE-2:0], BTN_DPAD_DOWN&~BTN_MENU&MENU_CLOSED};
        BTN_DPAD_UP_sr <= {BTN_DPAD_UP_sr[SRSIZE-2:0], BTN_DPAD_UP&~BTN_MENU&MENU_CLOSED};
        BTN_DPAD_LEFT_sr <= {BTN_DPAD_LEFT_sr[SRSIZE-2:0], BTN_DPAD_LEFT&~BTN_MENU&MENU_CLOSED};
        BTN_DPAD_RIGHT_sr <= {BTN_DPAD_RIGHT_sr[SRSIZE-2:0], BTN_DPAD_RIGHT&~BTN_MENU&MENU_CLOSED};
        BTN_START_sr <= {BTN_START_sr [SRSIZE-2:0], BTN_START&~BTN_MENU&MENU_CLOSED};
        BTN_SEL_sr <= {BTN_SEL_sr [SRSIZE-2:0], BTN_SEL&~BTN_MENU&MENU_CLOSED};
        BTN_A_sr <= {BTN_A_sr [SRSIZE-2:0], BTN_A&~BTN_MENU&MENU_CLOSED};
        BTN_B_sr <= {BTN_B_sr [SRSIZE-2:0], BTN_B&~BTN_MENU&MENU_CLOSED};

        BTN_DPAD_DOWN_filtered <= &BTN_DPAD_DOWN_sr[SRSIZE-1:1];
        BTN_DPAD_UP_filtered <= &BTN_DPAD_UP_sr[SRSIZE-1:1];
        BTN_DPAD_LEFT_filtered <= &BTN_DPAD_LEFT_sr[SRSIZE-1:1];
        BTN_DPAD_RIGHT_filtered <= &BTN_DPAD_RIGHT_sr[SRSIZE-1:1];
        BTN_START_filtered <= &BTN_START_sr[SRSIZE-1:1];
        BTN_SEL_filtered <= &BTN_SEL_sr[SRSIZE-1:1];
        BTN_A_filtered <= &BTN_A_sr[SRSIZE-1:1];
        BTN_B_filtered <= &BTN_B_sr[SRSIZE-1:1];
        
        if (BTN_NODIAGONAL) begin
            BTN_DPAD_DOWN_filtered_dir  <= BTN_DPAD_DOWN_filtered  & ~BTN_DPAD_UP_filtered & ~BTN_DPAD_LEFT_filtered_dir & ~BTN_DPAD_RIGHT_filtered_dir;
            BTN_DPAD_UP_filtered_dir    <= BTN_DPAD_UP_filtered    & ~BTN_DPAD_DOWN_filtered & ~BTN_DPAD_LEFT_filtered_dir & ~BTN_DPAD_RIGHT_filtered_dir;
            BTN_DPAD_LEFT_filtered_dir  <= BTN_DPAD_LEFT_filtered  & ~BTN_DPAD_RIGHT_filtered & ~BTN_DPAD_UP_filtered_dir & ~BTN_DPAD_DOWN_filtered_dir;
            BTN_DPAD_RIGHT_filtered_dir <= BTN_DPAD_RIGHT_filtered & ~BTN_DPAD_LEFT_filtered & ~BTN_DPAD_UP_filtered_dir & ~BTN_DPAD_DOWN_filtered_dir;
        end else begin
            BTN_DPAD_DOWN_filtered_dir  <= BTN_DPAD_DOWN_filtered  & ~BTN_DPAD_UP_filtered;
            BTN_DPAD_UP_filtered_dir    <= BTN_DPAD_UP_filtered    & ~BTN_DPAD_DOWN_filtered;
            BTN_DPAD_LEFT_filtered_dir  <= BTN_DPAD_LEFT_filtered  & ~BTN_DPAD_RIGHT_filtered;
            BTN_DPAD_RIGHT_filtered_dir <= BTN_DPAD_RIGHT_filtered & ~BTN_DPAD_LEFT_filtered;
        end
        
    end

    wire [3:0] btn_key = {
        BTN_DPAD_DOWN_filtered_dir, 
        BTN_DPAD_UP_filtered_dir,
        BTN_DPAD_LEFT_filtered_dir, 
        BTN_DPAD_RIGHT_filtered_dir
    }; 
     
    wire [3:0] dpad_key = {
        BTN_START_filtered,
        BTN_SEL_filtered,
        BTN_B_filtered,
        BTN_A_filtered
    };

    reg [3:0] joy_din;
    wire [1:0] joy_p54;
    // Non-SGB joypad decode. When an SGB game is active the SGB multitap,
    // merged into gb.v, overrides this internally (joy_nibble), so joy_din
    // is simply ignored by the core in that case.
    always@(posedge hclk)
    begin
        case(joy_p54)
            2'b00: joy_din <= (~btn_key) & (~dpad_key);
            2'b01: joy_din <= ~dpad_key;
            2'b10: joy_din <= ~btn_key;
            2'b11: joy_din <= 4'hF;
        endcase
    end

    wire wr;
    wire rd;
    wire [7:0] CART_DOUT;
    wire nCS;

    wire [7:0]  CART_DIN; 
    assign CART_DIN = CART_D;

    wire cpu_speed;
    wire cpu_halt;
    wire cpu_stop;
    
    wire [7:0] CART_DIN_r1;
    wire [15:0] a;
    wire [2:0] TSTATEo;
    wire TSTATE1, TSTATE2, TSTATE3, TSTATE4;
    wire sleep_savestate;
    wire ce_n;
    wire ce_4x;

    speedcontrol u_speedcontrol
    (
        .clk_sys     (hclk),
        .pause       (sleep_savestate),
        .speedup     (),
        .cart_act    (rd | wr),
        .save_act    (SAVE_out_ena),
        .DMA_on      (DMA_on),
        .ce          (ce),
        .ce_n        (ce_n),
        .ce_2x       (ce_2x),
        .ce_4x       (ce_4x),
        .refresh     (),
        .ff_on       ()
    );

    wire sel_cram = a[15:13] == 3'b101;           // 8k cart ram at $a000
    wire cart_oe = (rd & ~a[15]) | (sel_cram & rd);

    assign CART_RST = 1'bZ;

    reg gbreset;
    reg gbreset_ungated;
    reg CART_RST_r1;
    reg CART_RST_r2;
    reg [15:0] CART_RST_sr;   // debounce filter: /RST must be low 16 consecutive hclk (~1 us)
    reg ce_2x_r1;
    always@(posedge hclk)
        ce_2x_r1 <= ce_2x;

    always@(posedge hclk or negedge reset_n)
    begin
        if(~reset_n)
        begin
            gbreset <= 1'd1;
            gbreset_ungated <= 1'd1;
            CART_RST_sr <= 16'hFFFF;
        end
        else
        begin
            CART_RST_r1 <= CART_RST;
            CART_RST_r2 <= CART_RST_r1;
            CART_RST_sr <= {CART_RST_sr[14:0], CART_RST_r2};
            // Debounced: the FPGA never drives CART_RST (high-Z), so a sub-us
            // glitch/float on the cart's /RST line used to cause a spurious full
            // reboot. A real cart reset (EverDrive menu) is held for milliseconds,
            // so requiring 16 consecutive low samples rejects glitches only.
            gbreset_ungated <= ~LCD_INIT_DONE ? 1'b1 : (CART_RST_sr == 16'd0);
            if(~ce_2x_r1 & ce_2x & ce)
                gbreset <= gbreset_ungated;
                
            if (MENU_CLOSED & BTN_MENU & BTN_A & BTN_B & BTN_START & BTN_SEL) gbreset <= 1'd1;
            
            if (~POWER_GOOD) gbreset <= 1'b1;
        end
    end

    // ----------------------------------------------------------------
    // Native SGB detection: snoop the cart header during the single
    // CGB-mode boot and statically latch whether this is an SGB game.
    // There is NO reboot and NO runtime isGBC toggling -- the universal
    // boot ROM switches DMG/SGB games into DMG mode via FF4C (KEY0) and
    // pushes the SGB packet sequence itself (leaving A=$01), so the game
    // emits SGB commands in a single pass. The core only needs a static
    // enable for the SGB palette/multitap engine now merged into gb.v.
    //
    // sgb_detected deliberately PERSISTS across gbreset (it is cleared only
    // on reset_n, i.e. physical cart removal via CART_DET, PLL unlock or power
    // cycle): a save-state resume of an SGB game does not assert gbreset, so
    // the flag must stay set or the game would resume in SGB mode with the
    // engine disabled (isSGB=0) and hang.
    //
    // NOTE: this must NOT clear on CART_RST. An EverDrive opens its in-game
    // menu by asserting the cart /RST line, which (correctly) triggers a
    // gbreset and boots the OS ROM -- but the OS header is not SGB-flagged, so
    // clearing here left resumed SGB games with isSGB=0 and a disabled engine.
    // Cart removal is covered by CART_DET -> memrst -> reset_n, which does
    // clear the flag. While latched for a non-SGB cart the engine is simply
    // idle (no SGB packets -> overlay inactive) and the joypad still uses the
    // normal decode, so persistence is harmless.
    // ----------------------------------------------------------------
    reg sgb_detected;   // latched: 1 = SGB game (enables SGB palette/multitap)

    always @(posedge hclk or negedge reset_n) begin
        if (~reset_n)
            sgb_detected <= 1'b0;
        else if (isSGB_game && !isGBC_game)
            sgb_detected <= 1'b1;              // latch once the header is snooped
    end

    wire DMA_on;
    wire hdma_active;
    cart u_cart
    (
       .hclk            (hclk           ),
       .pclk            (pclk           ),
       .ce              (ce             ),
       .ce_2x           (ce_2x          ),
       .gbreset         (gbreset        ),
       .cpu_speed       (cpu_speed      ),
       .cpu_halt        (cpu_halt       ),
       .cpu_stop        (cpu_stop       ),
       .wr              (wr             ),
       .rd              (rd             ),
       .a               (a              ),
       .CART_DOUT       (CART_DOUT      ),
       .nCS             (nCS            ),
       .TSTATEo         (TSTATEo        ),
       .DMA_on          (DMA_on         ),
       .hdma_active     (hdma_active    ),
                                        
       .CART_A          (CART_A         ),
       .CART_CLK        (CART_CLK       ),
       .CART_CS         (CART_CS        ),
       .CART_D          (CART_D         ),
       .CART_RD         (CART_RD        ),
       .CART_WR         (CART_WR        ),
       .CART_DATA_DIR_E (CART_DATA_DIR_E),
       .CART_DIN_r1     (CART_DIN_r1    ),
       .cart_busy       (cart_busy      )
    );

    wire sc_int_clock2;
    wire serial_clk_in;
    wire serial_clk_out;
    wire serial_data_out;
    
    reg LINK_IN_r1;
    reg LINK_CLK_r1;
    reg serial_clk_out_r1;
    reg sc_int_clock2_r1;
    reg serial_data_out_r1;
    always@(posedge hclk)
    begin
        LINK_IN_r1         <= LINK_IN;
        LINK_CLK_r1        <= LINK_CLK;
        serial_clk_out_r1  <= serial_clk_out;
        sc_int_clock2_r1   <= sc_int_clock2;
        serial_data_out_r1 <= serial_data_out;
    end

    assign LINK_CLK = sc_int_clock2_r1 ? serial_clk_out_r1 : 1'bZ;
    assign serial_clk_in = LINK_CLK_r1;

    wire serial_data_in = LINK_IN_r1;
    assign LINK_OUT = serial_data_out_r1;

    // Declared early: speedcontrol.save_act and the ddram channel all
    // reference SAVE_out_ena, but speedcontrol is instantiated above the
    // gb module that drives it. Verilog allows forward use but we declare
    // it here for clarity and to avoid implicit-1-bit traps.
    wire [63:0] SAVE_out_Din  ;
    wire [63:0] SAVE_out_Dout ;
    wire [25:0] SAVE_out_Adr  ;
    wire SAVE_out_rnw  ;
    wire SAVE_out_ena  ;
    wire SAVE_out_done ;

    // cart_busy: high while the physical cartridge bus is mid-cycle.
    // Exposed up to top-level so the system monitor / save-state arbiter
    // can avoid kicking a save while the cart bus is in flight (which
    // would otherwise race against CPU reads on the Gowin BRAM path).
    wire cart_busy;

    reg ss_load = 1'b0;
    reg gbreset_1 = 1'b0;
    
    // synthesis translate_off
    always@(posedge hclk)
    begin
      gbreset_1 <= gbreset;
      ss_load   <= 1'b0;
      if (gbreset_1 && ~gbreset) ss_load <= 1'b1;
    end
    // synthesis translate_on

    wire [15:0] snd_l;  
    wire [15:0] snd_r;  

    // ----------------------------------------------------------------
    // ROM header detection: snoop CPU bus reads at key cart addresses
    // 0x0143 = CGB flag  (0x80=GBC compat, 0xC0=GBC only)
    // 0x0146 = SGB flag  (0x03=SGB enhanced)
    // 0x014B = old licensee code (0x33=new licensee, required for SGB)
    // ----------------------------------------------------------------
    reg [7:0] cart_cgb_flag    = 8'h00;
    reg [7:0] cart_sgb_flag    = 8'h00;
    reg [7:0] cart_old_lic     = 8'h00;
    reg       hdr_cgb_captured = 1'b0;
    reg       hdr_sgb_captured = 1'b0;
    reg       hdr_lic_captured = 1'b0;

    always @(posedge hclk) begin
        if (gbreset) begin
            hdr_cgb_captured <= 1'b0;
            hdr_sgb_captured <= 1'b0;
            hdr_lic_captured <= 1'b0;
        end else if (rd) begin
            case (a)
                16'h0143: begin cart_cgb_flag <= CART_DIN_r1; hdr_cgb_captured <= 1'b1; end
                16'h0146: begin cart_sgb_flag <= CART_DIN_r1; hdr_sgb_captured <= 1'b1; end
                16'h014B: begin cart_old_lic  <= CART_DIN_r1; hdr_lic_captured  <= 1'b1; end
                default:  ;
            endcase
        end
    end

    // isGBC_game: cart supports CGB features
    wire isGBC_game = hdr_cgb_captured ?
        (cart_cgb_flag == 8'h80 || cart_cgb_flag == 8'hC0) : 1'b1; // default GBC until detected
    // isSGB_game: cart has SGB enhancements (SGB flag=0x03 AND licensee=0x33).
    // Detected once during the single CGB-mode boot (the universal boot ROM
    // reads these header bytes) and statically latched into sgb_detected.
    wire isSGB_game = hdr_sgb_captured && hdr_lic_captured &&
                      (cart_sgb_flag == 8'h03) && (cart_old_lic == 8'h33);
    assign isSGB_out = sgb_detected;

    // ----------------------------------------------------------------
    // Sticky CGB-game detection: once a CGB-flagged cart ($0143=$80/$C0) has
    // booted, keep the core in CGB mode for the whole cart session. An
    // EverDrive opens its in-game menu via the cart /RST line and boots the
    // OS ROM, whose header is typically NOT CGB-flagged; FF4C (KEY0) is
    // write-once during boot, so without this latch a resumed GBC game would
    // run in DMG mode (HDMA/KEY1/SVBK dead) and crash. Cleared only on
    // reset_n (physical cart removal / PLL unlock / power), mirroring
    // sgb_detected. While no CGB cart has booted this session the flag is 0
    // and mode selection behaves exactly as before. DMG programs run fine
    // with CGB registers merely enabled (they never touch them).
    // ----------------------------------------------------------------
    reg cgb_game_detected = 1'b0;
    always @(posedge hclk or negedge reset_n) begin
        if (~reset_n)
            cgb_game_detected <= 1'b0;
        else if (hdr_cgb_captured && (cart_cgb_flag == 8'h80 || cart_cgb_flag == 8'hC0))
            cgb_game_detected <= 1'b1;
    end

    gb u_gb(
        .reset(gbreset),

        .clk_sys(hclk),
        .ce(ce),
        .ce_n(ce_n),
        .ce_2x(ce_2x),
        .ce_4x(ce_4x),
        .oc_lvl(oc_lvl),

        .isGBC(1'b1),                    // constant: CGB hardware; DMG/SGB mode set via FF4C KEY0
        .isSGB(sgb_detected),            // native SGB engine enable (static, from header snoop)
        .isGBC_game(isGBC_game & ~sgb_detected),
        .cgb_game_detected(cgb_game_detected),
        .real_cgb_boot(1'd0),
        .customPaletteEna(customPaletteEna),
        .paletteBGIn(paletteBGIn),
        .paletteOBJ0In(paletteOBJ0In),
        .paletteOBJ1In(paletteOBJ1In),

        .paletteOff(paletteOff),
        .gbc_mode(gbc_mode),
        .gpd(gpd),

        // cartridge interface
        // can adress up to 1MB ROM
        .ext_bus_addr(a[14:0]),
        .ext_bus_a15(a[15]),
        .cart_rd(rd),
        .cart_wr(wr),
        .cart_do(CART_DIN_r1),
        .cart_di(CART_DOUT),
        .cart_oe(cart_oe),
        .cart_busy(cart_busy),
        .TSTATEo(TSTATEo),
        .TSTATE1(TSTATE1),
        .TSTATE2(TSTATE2),
        .TSTATE3(TSTATE3),
        .TSTATE4(TSTATE4),
        // WRAM or Cart RAM CS
        .nCS(nCS),

        .cgb_boot_download(1'd0),
        .dmg_boot_download(1'd0),
        .ioctl_wr(1'd0),
        .ioctl_addr(25'd0),
        .ioctl_dout(16'd0),

        // Bootrom features
        .boot_rom_enabled(boot_rom_enabled),

        .boot_gba_en(1'd0),
        .fast_boot_en(1'd1),
        .skip_boot_rom(1'd0),  // reserved (tied 0): universal boot ROM must run (SGB packet TX + mode setup)
        // audio
        .audio_l(snd_l),
        .audio_r(snd_r),

        // Megaduck?
        .megaduck(1'd0),

        .IR_RX(IR_RX),
        .IR_LED(IR_LED),

        // lcd interface
        .lcd_clkena(gb_raw_lcd_clkena),
        .lcd_data(gb_raw_lcd_data),
        .lcd_data_gb(gb_raw_lcd_data_gb),
        .lcd_pix_x(gb_raw_lcd_pix_x),
        .lcd_pix_y(gb_raw_lcd_pix_y),
        .lcd_mode(gb_raw_lcd_mode),
        .lcd_on(gb_raw_lcd_on),
        .lcd_vsync(gb_raw_lcd_vsync),

        // SGB taps: main-video state only (no videoBypass), see gb.v
        .lcd_clkena_sgb(gb_sgb_lcd_clkena),
        .lcd_data_gb_sgb(gb_sgb_lcd_data_gb),
        .lcd_mode_sgb(gb_sgb_lcd_mode),
        .lcd_on_sgb(gb_sgb_lcd_on),

        .joy_p54(joy_p54),
        .joy_din(joy_din),

        .lcd_on_int(lcd_on_int),
        .lcd_off_overwrite(lcd_off_overwrite),

        .speed(cpu_speed),   //GBC
        .cpu_stop(cpu_stop),
        .cpu_halt(cpu_halt),
        .DMA_on(DMA_on),
        .hdma_active(hdma_active),
        .gg_reset(1'd0),
        .gg_en(1'd0),
        .gg_code(129'd0),
        .gg_available(),

        //serial port
        .sc_int_clock2(sc_int_clock2),
        .serial_clk_in(serial_clk_in),
        .serial_clk_out(serial_clk_out),
        .serial_data_in(serial_data_in),
        .serial_data_out(serial_data_out),

        // savestates
        .increaseSSHeaderCount(1'd0),
        .cart_ram_size(8'd0),
        .save_state(1'd0),
        .load_state(ss_load),
        .savestate_number(2'd0),
        .sleep_savestate(sleep_savestate),

        .SaveStateExt_Din(), 
        .SaveStateExt_Adr(), 
        .SaveStateExt_wren(),
        .SaveStateExt_rst(), 
        .SaveStateExt_Dout(64'd0),
        .SaveStateExt_load(),

        .Savestate_CRAMAddr(),     
        .Savestate_CRAMRWrEn(),    
        .Savestate_CRAMWriteData(),
        .Savestate_CRAMReadData(8'd0),

        .SAVE_out_Din  (SAVE_out_Din ),    // data read from savestate
        .SAVE_out_Dout (SAVE_out_Dout),  // data written to savestate
        .SAVE_out_Adr  (SAVE_out_Adr ),    // all addresses are DWORD addresses!
        .SAVE_out_rnw  (SAVE_out_rnw ),     // read = 1, write = 0
        .SAVE_out_ena  (SAVE_out_ena ),     // one cycle high for each action
        .SAVE_out_done (SAVE_out_done),    // should be one cycle high when write is done or read value is valid

        .rewind_on(1'd0),
        .rewind_active(1'd0)
    );

    // ----------------------------------------------------------------
    // Internal GB LCD raw wires (before SGB palette processing)
    // ----------------------------------------------------------------
    wire        gb_raw_lcd_clkena;
    wire [14:0] gb_raw_lcd_data;
    wire [1:0]  gb_raw_lcd_data_gb;  // 2-bit DMG pixel indices for SGB
    wire [7:0]  gb_raw_lcd_pix_x;    // screen x of the lcd_data_gb pixel (in lockstep)
    wire [7:0]  gb_raw_lcd_pix_y;    // screen y of the lcd_data_gb pixel (in lockstep)
    wire [1:0]  gb_raw_lcd_mode;
    wire        gb_raw_lcd_on;
    wire        gb_raw_lcd_vsync;

    // Main-video LCD taps for the SGB module (never videoBypass outputs)
    wire        gb_sgb_lcd_clkena;
    wire [1:0]  gb_sgb_lcd_data_gb;
    wire [1:0]  gb_sgb_lcd_mode;
    wire        gb_sgb_lcd_on;

    // The SGB palette/multitap engine is now merged into gb.v (u_gb). It
    // applies the SGB palette overlay to gb.v's lcd_data output internally,
    // gated by the static isSGB enable, so there is no external SGB module,
    // no overlay mux here, and no per-frame SGB pipeline delay to match.

    // Hold the LCD *pixels* black from power-on until the game's first real
    // picture, so the boot ROM frame, any SGB init leftover white, AND
    // any solid-colour init screen all stay hidden -- e.g. Pokemon's Init turns
    // the LCD on to a cleared-VRAM white screen (BGP=0) before LoadSGB and the
    // intro, and a blank-counter heuristic wakes on that white, producing
    // white->intro-black->title-white. The backlight itself is on from panel
    // init (see top.v), so this black is a lit black and the game wakes with no
    // backlight pop. "Real picture" is detected as a *non-uniform* frame: a
    // cleared-VRAM / BGP=0 / videoBypass-off frame is uniform (every pixel
    // identical) whereas actual content (a logo, sprites, the Pokemon shooting
    // star) is not, so we release on the first few non-uniform frames after the
    // boot ROM unmaps. A long timeout (gated on lcd_on_int) is the safety net
    // for the rare game whose first screen is a single solid colour. A new boot
    // (boot_rom_enabled rising = power-on or cart change) re-arms; there is NO
    // memrst reset, so a cart-detect/PLL glitch during play cannot disturb the
    // picture. SGB packet capture is unaffected -- the SGB engine (merged into
    // gb.v) reads the un-overridden gb_raw / SGB taps, not gb_lcd_data.
    reg        boot_done;
    reg        prev_boot_rom_enabled;
    reg [14:0] frame_ref;
    reg        frame_started;
    reg        frame_nonuniform;
    reg [1:0]  nonuniform_frames;
    reg [6:0]  frames_since_unmap;
    reg        gb_vsync_d;
    always @(posedge hclk) begin
        gb_vsync_d            <= gb_raw_lcd_vsync;
        prev_boot_rom_enabled <= boot_rom_enabled;
        if (gbreset_ungated || gbreset || (~prev_boot_rom_enabled && boot_rom_enabled)) begin
            // Re-arm on any reset (power-on, cart load, soft reset)
            // so the reset's own video noise (e.g. the EverDrive menu->ROM load
            // showing the videoBypass fill / greenish reset garbage) is held
            // black too, not just the boot-ROM window.
            boot_done          <= 1'b0;
            nonuniform_frames  <= 2'd0;
            frames_since_unmap <= 7'd0;
            frame_started      <= 1'b0;
            frame_nonuniform   <= 1'b0;
        end else if (~boot_done) begin
            if (gb_raw_lcd_vsync & ~gb_vsync_d) begin
                // Tally the just-finished frame (read before resetting it).
                if (~boot_rom_enabled) begin
                    if (lcd_on_int && frame_nonuniform) begin
                        if (nonuniform_frames < 2'd2)
                            nonuniform_frames <= nonuniform_frames + 2'd1;
                    end else
                        nonuniform_frames <= 2'd0;
                    if (frames_since_unmap < 7'd63)
                        frames_since_unmap <= frames_since_unmap + 7'd1;
                    if (nonuniform_frames >= 2'd2
                            || (frames_since_unmap >= 7'd60 && lcd_on_int))
                        boot_done <= 1'b1;
                end
                frame_started    <= 1'b0;
                frame_nonuniform <= 1'b0;
            end
            if (gb_raw_lcd_clkena) begin
                if (!frame_started) begin
                    frame_ref     <= gb_raw_lcd_data;
                    frame_started <= 1'b1;
                end else if (gb_raw_lcd_data != frame_ref)
                    frame_nonuniform <= 1'b1;
            end
        end
    end

    assign gb_lcd_clkena = gb_raw_lcd_clkena;
    // Drive black until boot_done (boot ROM frame + any init blank stay hidden).
    // gb_raw_lcd_data is gb.v's lcd_data output, which already carries the SGB
    // palette overlay (applied inside gb.v when isSGB & sgb_pal_en).
    assign gb_lcd_data   = boot_done ? gb_raw_lcd_data : 15'h0000;
    assign gb_lcd_mode   = gb_raw_lcd_mode;
    assign gb_lcd_on     = gb_raw_lcd_on;
    assign gb_lcd_vsync  = gb_raw_lcd_vsync;
    assign boot_done_out = boot_done;
    
    audio_filter u_audio_filter
    (
       .reset    (~reset_n),
       .clk      (hclk),

       .core_l   (snd_l),
       .core_r   (snd_r),

       .filter_l (left),
       .filter_r (right)
    );
    
// synthesis translate_off
   wire DDRAM_CLK            ;
   wire DDRAM_BUSY           ;
   wire [7:0]  DDRAM_BURSTCNT;
   wire [28:0] DDRAM_ADDR    ;
   wire [63:0] DDRAM_DOUT    ;
   wire DDRAM_DOUT_READY     ;
   wire DDRAM_RD             ;
   wire [63:0] DDRAM_DIN     ;   
   wire [7:0]  DDRAM_BE      ;   
   wire DDRAM_WE             ;

   wire [27:1] ch1_addr;         
   wire [63:0] ch1_dout;         
   wire [63:0] ch1_din ;         
   wire [7:0]  ch1_be  ;         
   wire ch1_req        ;  
   wire ch1_rnw        ;  
   wire ch1_ready      ;  
    
   ddram iddram
   (
      .DDRAM_CLK        (hclk),      
      .DDRAM_BUSY       (DDRAM_BUSY),      
      .DDRAM_BURSTCNT   (DDRAM_BURSTCNT),  
      .DDRAM_ADDR       (DDRAM_ADDR),      
      .DDRAM_DOUT       (DDRAM_DOUT),      
      .DDRAM_DOUT_READY (DDRAM_DOUT_READY),
      .DDRAM_RD         (DDRAM_RD),        
      .DDRAM_DIN        (DDRAM_DIN),       
      .DDRAM_BE         (DDRAM_BE),        
      .DDRAM_WE         (DDRAM_WE),                
                
      .ch1_addr         (ch1_addr),        
      .ch1_dout         (ch1_dout),        
      .ch1_din          (ch1_din),  
      .ch1_be           (ch1_be),      
      .ch1_req          (ch1_req),         
      .ch1_rnw          (ch1_rnw),         
      .ch1_ready        (ch1_ready)
   );
   
   assign ch1_addr      = { SAVE_out_Adr[25:0], 1'b0 };
   assign ch1_din       = SAVE_out_Din;
   assign ch1_req       = SAVE_out_ena;
   assign ch1_rnw       = SAVE_out_rnw;
   assign ch1_be        = 8'hFF; // only required for increaseSSHeaderCount
   assign SAVE_out_Dout = ch1_dout;
   assign SAVE_out_done = ch1_ready;
   
   ddrram_model iddrram_model
   (
      .DDRAM_CLK        (hclk),      
      .DDRAM_BUSY       (DDRAM_BUSY),      
      .DDRAM_BURSTCNT   (DDRAM_BURSTCNT),  
      .DDRAM_ADDR       (DDRAM_ADDR),      
      .DDRAM_DOUT       (DDRAM_DOUT),      
      .DDRAM_DOUT_READY (DDRAM_DOUT_READY),
      .DDRAM_RD         (DDRAM_RD),        
      .DDRAM_DIN        (DDRAM_DIN),       
      .DDRAM_BE         (DDRAM_BE),        
      .DDRAM_WE         (DDRAM_WE)       
   );
// synthesis translate_on
    
endmodule
