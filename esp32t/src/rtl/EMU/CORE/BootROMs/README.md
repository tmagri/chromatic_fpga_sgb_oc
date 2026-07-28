# Building the BootROM

1. Install RGBDS v0.6 (see https://rgbds.gbdev.io/)

2. Ensure that RGBDS is on your path.

3. Run `make && make bootroms && make clean`

Note: The BootROM is restricted to $900 (2304 bytes) to limit impact on
BlockRAM; the assembler `FAIL`s if `BootEnd` exceeds it.

## Universal boot ROM (native SGB)

`cgb_boot.asm` is the single, universal boot ROM. It runs in CGB mode, reads
the cartridge header ($0143 CGB flag, $0146 SGB flag, $014B licensee) and boots
each cart type natively in one pass:

- **CGB** games boot in CGB mode (`A=$11`).
- **DMG** games are switched into DMG mode via FF4C (KEY0) with DMG palettes.
- **SGB** games are switched into DMG mode, push the SGB packet sequence
  (`SGBBoot`), then hand off with `A=$01` so the game emits its SGB palette
  commands (colourised by the SGB engine merged into `gb.v`).

The standalone `sgb_boot.asm` / `sgb2_boot.asm` are **deprecated** and no
longer built; the old SGB double-boot (reset + re-boot in DMG mode) has been
removed. The core runs with `isGBC=1` constantly, so only the CGB region
($000-$8FF) of `cgb_boot.mif.vmem` is used.
