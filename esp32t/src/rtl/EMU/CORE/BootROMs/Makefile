PLATFORM := $(shell uname -s)
ifneq ($(findstring MINGW,$(PLATFORM)),)
PLATFORM := windows32
USE_WINDRES := true
endif

ifneq ($(findstring MSYS,$(PLATFORM)),)
PLATFORM := windows32
endif

SRC := ./src
OBJ := ./obj
BIN := ./bin

ifeq ($(PLATFORM),windows32)
_ := $(shell chcp 65001)
EXESUFFIX:=.exe
NATIVE_CC = clang -IWindows -Wno-deprecated-declarations --target=i386-pc-windows
# To force use of the Unix version instead of the Windows version
MKDIR := $(shell which mkdir)
NULL := NUL
else
EXESUFFIX:=
NATIVE_CC := cc
MKDIR := mkdir
NULL := /dev/null
endif

IMG_COMPRESS := $(OBJ)/logo-compress$(EXESUFFIX)

.PHONY : default
default: clean prepare

.PHONY : bootroms
bootroms: $(BIN)/cgb_boot.bin vmem

# Binary compiler
$(BIN)/%.bin: $(SRC)/%.asm $(OBJ)/CGB_logo.rle
	-@$(MKDIR) -p $(dir $@)
	rgbasm -i $(OBJ) -i $(SRC) -o $@.tmp $<
	rgblink -o $@.tmp2 $@.tmp
	dd if=$@.tmp2 of=$@ count=1 bs=$(if $(findstring mgb,$@)$(findstring dmg,$@)$(findstring sgb,$@),256,2304) 2> $(NULL)
	@rm $@.tmp $@.tmp2

# CGB logo compression
$(OBJ)/%.rle: $(OBJ)/%.1bpp $(IMG_COMPRESS)
	-@$(MKDIR) -p $(dir $@)
	$(realpath $(IMG_COMPRESS)) < $< > $@

# Make CGB logo
$(OBJ)/%.1bpp: %.png
	-@$(MKDIR) -p $(dir $@)
	rgbgfx -d 1 -L 0,0:16,3 -Z -o $@ $<

$(OBJ)/%$(EXESUFFIX): $(SRC)/%.c
	-@$(MKDIR) -p $(dir $@)
	$(NATIVE_CC) -std=c99 -Wall -Werror $< -o $@

.PHONY : clean
clean:
	rm -rf $(SRC)
	rm -rf $(OBJ)
	rm -rf $(BIN)
	rm -f *.mif
	rm -f CGB_logo.png

.PHONY : prepare
prepare:
	cp -R ../../../../Gameboy_MiSTer/BootROMs/src/ $(SRC)
	cp -f ../../../../Gameboy_MiSTer/BootROMs/CGB_logo.png CGB_logo.png
	cp -f cgb_boot.asm $(SRC)/cgb_boot.asm

.PHONY : mif
mif:
	srec_cat \
	$(BIN)/cgb_boot.bin -Binary \
	-Output_Block_Size 16 -o cgb_boot.mif -mif -CRLF

# The universal cgb_boot.asm detects CGB/DMG/SGB from the cartridge header and
# boots natively in a single pass, so the standalone SGB boot ROM is deprecated
# and no longer built (its packet sequence now lives in cgb_boot.asm).

# Memory layout: the universal boot ROM occupies $000-$8FF (2304 bytes). The
# old DMG/SGB boot regions ($900+) are unused now that everything is served
# from the CGB region (the core runs with isGBC=1 and switches mode via FF4C).
.PHONY : vmem
vmem: $(BIN)/cgb_boot.bin
	hexdump -v -e '1/1 "%02X\n"' $(BIN)/cgb_boot.bin > cgb_boot.mif.vmem

