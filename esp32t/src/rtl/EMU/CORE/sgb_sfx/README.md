# SGB built-in SFX bank (ROM-native APU sound image + BRR HLE player)

This folder captures the tooling used to reproduce the **Super Game Boy
BIOS's built-in sound-effect bank** (all 73 effects the SGB plays for the
SGB `SOUND` packet command) on this core -- **without** the original SNES
SPC700 + DSP silicon and **without** embedding the copyrighted BIOS.

## The pipeline (current)

The FPGA plays the effects straight from the SGB ROM's **original BRR sample
data**:

1. `extract_apu.py` opens your own `SGB1.sfc` dump (`SGB_BIOS` env var or
   `sgb_sfx/SGB1.sfc`; the ROM itself is never copied into this repo),
   extracts the APU **sound image** only -- the 256 B sample directory
   ($4B00) plus the verbatim BRR region ($4DB0, 41,280 B) -- and traces all
   73 effects through the real BIOS APU code to pick each effect's dominant
   sample, its **piecewise-constant pitch timeline** (the driver's pitch
   writes, run-length-encoded and reduced) and its **amplitude envelope**
   (the dominant voice's DSP envelope, ENVX, decimated to 1 ms steps).
2. It emits `build/sgb_sfx_bank.bin` (~62 KB): header + 73 x 16-byte
   records + the verbatim BRR region (+ a small constructed extension for
   the horse gallop) + the pitch/amplitude envelope segment table. The MCU
   (`chromatic_mcu`, `sfx` partition @0x110000) copies it to PSRAM
   0x100000 at boot.
3. The FPGA's `sgb_sfx_play.v` (see one directory up) is a **BRR HLE
   player**: on a trigger it fetches the effect's record, streams the BRR
   bytes back out of PSRAM and decodes them to PCM in the audio domain
   (decode datapath bit-exact with `sgb_snd.v` / `build_bank.py::dec_sample`).
   Effects with an envelope replay it as a **segmented tick+amp envelope**
   at decoded-block (16-sample) granularity: the tick divider reloads at
   each traced pitch change (varispeed pitch) and the decoded output is
   scaled by the traced amplitude (predictor history stays unscaled, so
   filter behaviour is unaffected). No SPC700, no Gaussian resampling, no
   echo -- one dominant voice per effect, loop region honored. This is the
   "closest enough" fidelity contract; see the player's header comment for
   the scope.

This supersedes the earlier PCM flow (`render.py` -> WAV -> uncompressed PCM
bank for the 8 effects KDL2 uses, `build_bank*.py`); those tools are kept as
the validation reference, not the ship path.

## Bank format (record v6)

```
0x000..0x013  header: 'SFXB', u16 version(6), u16 count(73),
                      u32 table_off(0x100), u32 brr_off(0x800), u32 seg_off(0xC950)
0x100..       table:  73 x 16-byte little-endian records (8 x u16)
0x800..       BRR:    verbatim copy of the ROM's $4DB0 record (41,280 B),
                      followed by the constructed gallop extension (8,208 B)
0xC950..      segs:   envelope segment table (2033 entries x 6 B)
```

Record (16 B, 8 x u16 little-endian): `brr_off` (byte offset of the first
BRR block, multiple of 9), `loop_off` (loop start; 0xFFFF = none), `tick`
(segment-0 hClk divider reload = round(16777216 / R_eff) - 1), `blocks`
(first-pass blocks incl. terminal), `flags` (bit0 = loop region valid,
bit1 = loop forever, bit2 = env-loop, bits 15:8 = initial amplitude 0..255
applied from the first sample), `count` (total BLOCKS to emit for looped
one-shots), `seg_off` (entry index of this effect in the segment table),
`seg_count` (number of envelope entries; 0 = fixed pitch+amp).

`flags` bit2 (env-loop, bank v6.1): the envelope is a single cycle that
restarts from entry 0 after the final segment completes (only meaningful
with bit1 loop-forever). The builder appends a **wrap-around entry** whose
`next_tick`/`next_amp` are segment 0's values, so the player's normal
boundary reload re-arms the cycle with no extra state. Used for the looping
ambiences whose loudness/pitch sweep must repeat: B0B (wave swell with its
silent break) and B06 (storm thunder's pitch sweep).

Segment entry (6 B): `u16 blocks16` = length of the CURRENT segment in
16-sample blocks, `u16 next_tick` = divider reload of the NEXT segment,
`u16 next_amp` (low byte) = amplitude of the NEXT segment. Entry `e`
carries segment `e`'s length and segment `e+1`'s tick/amp; segment 0's
tick/amp are the record's `tick` / `flags[15:8]` and the final segment has
no entry (holds its tick/amp to the end), so the envelope cannot desync
from the block stream. Amplitudes are quantized to a global step (16 for
this bank) so consecutive equal (tick,amp) pairs merge away.

Effect index mapping (see `gb.v` CMD_SOUND): SFX-A num 0x01..0x30 -> index
num-1 (0..47); SFX-B num 0x01..0x19 -> index 47+num (48..72).

## What is in here

| File | Purpose |
|---|---|
| `extract_apu.py` | **The builder.** Extracts the APU sound image, traces all 73 effects, builds `build/` artifacts + reference decode streams, runs self-checks. |
| `overrides.json` | Manual corrections per effect (`srcn`/`tick`/`flags`/`count`/`env_loop`, plus special builders `gallop_ms` for the horse and `repeat_env` (+`repeat_gap_ms`) which turns the wave's traced swell into a looping env-loop cycle). |
| `tb_sfx_trig.v` | iverilog TB replicating gb.v's SOUND trigger logic (7-bit index map, stop semantics, reset). |
| `tb_sgb_sfx_play.v` | iverilog TB: feeds `build/sim_bank.hex` through an arbiter model and compares every effect's output bit-exactly against `build/ref_XXX.hex` (ticks patched to 3 for sim speed; envelope sequencing still exercised at real block boundaries). |
| `tb_sgb_sfx_env.v` | iverilog TB: plays the envelope effects (A01/A02/A03) at REAL tick rates and checks, sample by sample, that the in-force tick AND amplitude match the bank's piecewise envelope (envelope-timing regression). |
| `tb_sfx_race.v` | iverilog TB: phase-sweeps a stop command into burst completions to prove the supersede path never loses an `xRamDone` pulse (writer-deadlock regression); also covers the segment-table fetch abort. |
| `debug/pitch_timeline.py` | Standalone time-ordered pitch tracer that sizes the segment table (runs per effect, reduced count) used to dimension the RTL segment store. |
| `tb_check.py` | Legacy checker for the old PCM bank captures (superseded; kept with the PCM flow). |
| `render.py` | Legacy/reference: boots the BIOS APU image and renders each effect to WAV via the cycle-accurate SPC700 + DSP emulation. |
| `spc700.py` | Cycle-counted SPC700 CPU core (Python), used by render/trace. |
| `snesdsp.py` | SNES DSP emulation, a faithful port of the [ares](https://github.com/ares-emulator/ares) emulator's DSP. |
| `gauss_table.py` | The 512-entry SNES DSP Gaussian table (self-contained copy). |
| `DSP_PKG.vhd` | Copy of the SGB_MiSTer core's DSP package (open source); gauss-table fallback source. |
| `build_bank.py` | Legacy PCM-flow bank builder; **`dec_sample` is the authoritative BRR decode reference** the RTL and refs transcribe. |
| `build_bank_pcm.py`, `build_bank_uncompressed.py` | Legacy PCM-flow builders (superseded). |
| `wav2brr.py` | WAV -> BRR encoder (legacy reference flow). |
| `spc2wav.py` | SPC -> WAV helper for cross-checking against an independent DSP. |
| `debug/` | Investigation/verification scripts: ROM record probes, driver voice tracer (`trace_sfx.py`), KON start-address check, live-DSP vs `decode_stream` bit-exact check. |
| `build/` | Generated: `sgb_sfx_bank.bin` (+ `.vh`/`.json`), `sim_bank.hex`, per-effect `ref_XXX.hex` decode streams, `trace.json`. |
| `wav/` | The 73 rendered effects (validation reference only). |

## Self-contained layout / copyright

Everything the tools need lives in this folder -- no paths outside it -- with
one deliberate exception:

- **`SGB1.sfc`** -- the copyrighted SGB BIOS. It is *not* distributed here
  and the scripts never copy it in. Provide your own dump: place it as
  `sgb_sfx/SGB1.sfc`, or set the `SGB_BIOS` env var to its path.

All scripts resolve defaults relative to their own location, so the folder
can be moved or checked out anywhere and still works.

## Requirements

- Python 3 (standard library only).
- iverilog for the testbenches.

## Regenerating the bank

```sh
export SGB_BIOS=/path/to/SGB1.sfc        # your own SGB BIOS dump
python3 extract_apu.py                   # rebuilds build/ (bank + refs + manifest)
```

Optional: `render.py all` re-renders the WAV references (slow; only needed
when validating the emulation itself).

## Verification

1. **Self-checks** in `extract_apu.py`: directory entries inside the region,
   invalid-shift scan, decode-vs-WAV correlation per effect.
2. **Trigger TB**:
   `iverilog -g2001 -o /tmp/tb_trig.out sgb_sfx/tb_sfx_trig.v && vvp /tmp/tb_trig.out`
3. **Player TB** (from `esp32t/src/rtl/EMU/CORE`):
   `iverilog -g2001 -o /tmp/tb_play.out sgb_sfx/tb_sgb_sfx_play.v sgb_sfx_play.v && vvp /tmp/tb_play.out`
   -- plays all 73 effects through a stall/bubble-injected arbiter model and
   compares bit-exactly against the reference decodes (the TB patches each
   record's `tick` and every segment entry's next-tick to 3 for sim speed;
   ticks only pace the output, and the envelope sequencer still steps its
   real block boundaries). Includes an envelope->envelope retrigger case.
4. **Envelope-timing TB**:
   `iverilog -g2001 -o /tmp/tb_env.out sgb_sfx/tb_sgb_sfx_env.v sgb_sfx_play.v && vvp /tmp/tb_env.out`
   -- plays A01/A02/A03 at REAL tick rates and verifies the in-force tick
   and amplitude match the bank's piecewise envelope for every emitted
   sample.
5. **Race TB**:
   `iverilog -g2001 -o /tmp/tb_race.out sgb_sfx/tb_sfx_race.v sgb_sfx_play.v && vvp /tmp/tb_race.out`
   -- phase-sweeps stops into burst completions (supersede-vs-xRamDone race),
   including stops landing mid-way through the segment-table fetch.
6. **Hardware**: the 8 KDL2-regression effects (A1F/A26/A30/B01/B04/B07/
   B08/B0B) plus the full bank via SGB test packets.

## Notes

- BRR offsets are multiples of 9 (odd/even both occur); the player fetches
  even-aligned and skips the pad bytes -- exercised by the TB.
- Predictor history is zeroed at effect start and carries across the loop
  jump (matches ares/bsnes).
- Loop-forever effects (SFX-B ambiences) run until a SOUND bit-7 stop or
  core reset; `gb.v`'s reset branch re-asserts the stop for the engine.
- Envelopes (bank v6) cover the effects whose dominant voice changes pitch
  and/or loudness over time (coins, melodies, slides, swells). Loop-forever
  ambiences and genuinely flat effects stay fixed (2033 segment entries
  total across the bank). The pitch envelope only changes the emission
  rate; the amplitude envelope scales each decoded sample by
  `(pcm * amp) >> 8` (predictor history unscaled). The bit-exact refs
  include both. Retriggering refills the segment store before the echo; an
  interrupted effect can transiently read the new store for the sub-100 us
  before it restarts (inaudible by design).
- A few effects use `overrides.json` beyond the traced dominant voice:
  B0F (horse) replays a constructed three-beat gallop extension appended to
  the BRR region; B0B (wave) and B06 (storm) use the env-loop flag so their
  traced swell / thunder-sweep envelope cycles forever over the looping BRR
  (matching the original console, which sustains both until a stop packet);
  B10 (warning) uses the sample's natural loop (a steady 830 Hz tone)
  instead of a full-track loop.
