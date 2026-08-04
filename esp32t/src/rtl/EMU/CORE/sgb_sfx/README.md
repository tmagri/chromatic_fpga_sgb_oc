# SGB built-in SFX bank (rendered from the real SGB BIOS)

This folder captures the tooling and rendered assets used to reproduce the
**Super Game Boy BIOS's built-in sound-effect bank** (the sounds the SGB plays
for the SGB `SOUND` packet command, index-based) so they can be played back on
this core without the original SNES SPC700 + DSP silicon.

The effects are rendered by **executing the real SGB BIOS APU code** (the
N-SPC driver at `$0400` in the SGB BIOS) in a cycle-accurate SPC700 emulator,
fed into a faithful SNES DSP emulation. The result is captured to WAV.

## What is in here

| File | Purpose |
|---|---|
| `render.py` | Main driver: boots the BIOS APU image, triggers each SFX index via the SGB `SOUND` packet bytes, captures DSP output to WAV. |
| `spc700.py` | Cycle-counted SPC700 CPU core (Python). |
| `snesdsp.py` | SNES DSP emulation, a faithful port of the [ares](https://github.com/ares-emulator/ares) emulator's DSP (`brr.cpp`, `gaussian.cpp`, `envelope.cpp`, `counter.cpp`, `voice.cpp`, `memory.cpp`). |
| `gauss_table.py` | The 512-entry SNES DSP Gaussian interpolation table (extracted from `SGB_MiSTer/rtl/DSP_PKG.vhd`, identical to the SNES DSP ROM table). Makes the tools self-contained. |
| `wav2brr.py` | WAV -> BRR encoder (for converting rendered audio to BRR if a BRR playback path is wanted). |
| `spc2wav.py` | SPC -> WAV helper (uses the external `spct`/libgme player) -- used for cross-checking against an independent DSP implementation. |
| `wav/` | The 73 rendered effects: `A01..A30` (SFX-A) and `B01..B19` (SFX-B), 32 kHz 16-bit stereo. |

## Requirements

- **`SGB1.sfc`** -- the copyrighted SGB BIOS. Provide your own dump. Set the
  path with the `SGB_BIOS` env var, or place it next to `render.py`.
- Python 3 (standard library only).

## Regenerating the bank

```sh
export SGB_BIOS=/path/to/SGB1.sfc      # your SGB BIOS dump
export SGB_OUT=./wav                    # output dir (default: ./wav)
python3 render.py all                   # render all 73 effects
python3 render.py a                     # SFX-A only
python3 render.py b                     # SFX-B only
```

Each effect is rendered by booting the BIOS APU image, then pulsing the SGB
`SOUND` packet bytes on the APU I/O ports (`$F4..$F7`), and capturing the DSP
output at full master volume.

## How the rendering was validated

1. **BRR decode** was validated against the DKGB ground-truth WAV that ships
   with the DKGB disassembly
   (`DKGBDisasm/audio/sfx/sgb_pauline_help_scream.wav` next to the `.brr`).
   Correlation 1.0000, mean abs diff ~7.5/32768.
2. The **DSP** is a faithful port of the ares emulator's DSP. An earlier
   in-house DSP (wrong BRR filter coefficients + wrong decode pacing) produced
   garbled "burning" audio; the ares port fixed it. The applause effects
   (`B01..B03`) were A/B confirmed against a real-SGB reference recording.
3. Output was cross-checked against `spct`/libgme (an independent SNES DSP)
   via `spc2wav.py`.

## Effect index reference

Effect indices follow the Pan Docs / SGB BIOS `SOUND` command tables.
SFX-A (byte 1) and SFX-B (byte 2) are separate banks. Byte 3 carries pitch /
volume for A and B; byte 4 is unused (0). The rendered WAVs use the BIOS's
recommended pitch for each effect. See the Pan Docs "Sound Effect A/B Tables"
for the human-readable names.

## Notes

- Effects were rendered at full master volume (the BIOS's slow MVOL ramp is
  overridden) so short effects are captured consistently.
- The rendered WAVs are the *source* assets. For FPGA playback they are
  converted to the playback format used by the core (see the playback engine
  RTL); the WAVs themselves are kept here as the reference.
