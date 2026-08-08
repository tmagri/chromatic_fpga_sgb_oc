#!/usr/bin/env python3
# Builds a small synthetic N-SPC song (raw binary source) plus its song.json
# config, so the compiler can be exercised end-to-end without a ROM. The song
# is a 2-voice arpeggio + bass loop (~3.5 s) using one looped BRR sample.
import json, os, struct

_here = os.path.dirname(os.path.abspath(__file__))

A_BLOCKLIST = 0x2000     # gft entry points here
A_BLOCKHDR  = 0x2010     # 8 x u16 channel pointers
A_PATCHES   = 0x2100     # instrument patch table (6 B/patch)
A_CH0       = 0x2200     # voice 0 channel stream
A_CH1       = 0x2240     # voice 1 channel stream
A_SAMPLEDIR = 0x2400     # DSP sample directory (4 B/entry)
A_BRR       = 0x2800     # BRR blocks

ram = bytearray(0x3000)

def w16(a, v):
    ram[a] = v & 0xFF
    ram[a + 1] = (v >> 8) & 0xFF

# ---- song table (gft): one song -> block list ----------------------------
w16(0x1F00, A_BLOCKLIST)
# block list: one block header word, then end-of-song
w16(A_BLOCKLIST, A_BLOCKHDR)
w16(A_BLOCKLIST + 2, 0x0000)
# block header: channels 0 and 1 active
w16(A_BLOCKHDR + 0, A_CH0)
w16(A_BLOCKHDR + 2, A_CH1)

# ---- patches: patch 0 -> SRCN 0, pitch mult 1.0 ---------------------------
ram[A_PATCHES + 0] = 0          # srcn
ram[A_PATCHES + 4] = 0x01       # mult integer
ram[A_PATCHES + 5] = 0x00       # mult fraction

# ---- channel 0: arpeggio --------------------------------------------------
p = A_CH0
ram[p] = 0xE0; ram[p + 1] = 0; p += 2      # sno 0
ram[p] = 0xE1; ram[p + 1] = 0x0A; p += 2   # pan center
for note in (0x3C, 0x40, 0x43, 0x47, 0x43, 0x40):
    ram[p] = 0x08                          # duration 8 steps
    ram[p + 1] = 0x7B                      # gate ~full, vel index 11
    ram[p + 2] = 0x80 | note               # note on
    p += 3
ram[p] = 0x00                              # segment end

# ---- channel 1: bass ------------------------------------------------------
p = A_CH1
ram[p] = 0xE0; ram[p + 1] = 0; p += 2
ram[p] = 0xE1; ram[p + 1] = 0x06; p += 2   # slight left
for note in (0x24, 0x2B):
    ram[p] = 0x12                          # duration 18 steps
    ram[p + 1] = 0x65                      # vel index 5
    ram[p + 2] = 0x80 | note               # note on
    p += 3
ram[p] = 0x00

# ---- sample directory: SRCN 0 -> looped chain of 4 blocks -----------------
w16(A_SAMPLEDIR + 0, A_BRR)                # sample start
w16(A_SAMPLEDIR + 2, A_BRR + 9)            # loop point (block 1)

# ---- BRR: 4 blocks, filter 0, decaying amplitude, block 3 = terminal ------
for i in range(4):
    hdr = (4 << 4) | (0 << 2) | (1 if i == 3 else 0)   # shift 4, filter 0
    ram[A_BRR + 9 * i] = hdr
    amp = max(1, 6 - i)
    for j in range(8):
        s = amp if (j % 2 == 0) else -amp
        ram[A_BRR + 9 * i + 1 + j] = s & 0xFF

open(os.path.join(_here, 'synthetic_song.bin'), 'wb').write(ram)

cfg = {
    'name': 'synthetic-placeholder',
    'source': os.path.join(_here, 'synthetic_song.bin'),
    'song_table': 0x1F00,
    'song': 0,
    'patches': A_PATCHES,
    'sample_dir': A_SAMPLEDIR,
    'voice_map': [0, 1],
    'default_tempo': 0xE0,
    'loop': True,
}
json.dump(cfg, open(os.path.join(_here, 'song.json'), 'w'), indent=1)
print('wrote synthetic_song.bin + song.json')
