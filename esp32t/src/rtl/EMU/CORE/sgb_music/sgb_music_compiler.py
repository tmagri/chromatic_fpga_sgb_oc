#!/usr/bin/env python3
# sgb_music_compiler.py -- Generic N-SPC (Kankichi-kun) -> FPGA tracker compiler.
#
# Compiles an N-SPC song (sequence bytecode + instrument patches + BRR sample
# directory) into the proprietary "MSGB" tracker bank consumed by the
# sgb_music.v micro-sequencer (see that file for the hardware side).
#
# The N-SPC bytecode semantics implemented here were decoded against the
# Star Fox N-SPC driver disassembly (SGSOUND0.asm / KAN.asm / patches.asm),
# which is the reference ("Rosetta Stone") for every Nintendo N-SPC title:
#
#   byte 0x00          end of channel data in the current block
#   byte 0x01..0x7F    duration (in steps); may be followed by a
#                      velocity/gate byte (<0x80): hi nybble -> gate%
#                      table index, lo nybble -> velocity table index
#   byte 0x80..0xC9    note on (note = byte & 0x7F), 0xCA = tie, 0xCB = rest
#   byte 0xCC..0xDF    percussion: set sample from drum table, play note 0x30
#   byte 0xE0..0xFA    special vc; argument lengths from the spfp table:
#                        E0 sno(instrument,1) E1 pan(1) E2 pam(2) E3 vib(3)
#                        E4 vof(0) E5 mv1(1) E6 mv2(2) E7 tp1(1) E8 tp2(2)
#                        E9 ktp(1) EA ptp(1) EB tre(3) EC tof(0) ED pv1(1)
#                        EE pv2(2) EF pat(3, subroutine) F0 vch(1) F1 swk(3)
#                        F2 sws(3) F3 sof(0) F4 tun(1) F5 ecv(3) F6 eof(0)
#                        F7 edl(3) F8 ev2(3) F9 pitch-slide(3) FA wav(1)
#
#   Song layout (gft): the song table holds u16 pointers to a BLOCK list.
#   A block list is a sequence of u16 words:
#     hi != 0   -> pointer to a 16-byte header: 8 x u16 channel start ptrs
#     0x00xx    -> xx = repeat count; the NEXT word is the loop-back address
#     0x0000    -> end of song
#
# Timing model: the SPC700 driver advances one sequencer step every
# 4096/tempo ms (16 ms frames, u8 accumulator += tempo). Everything is
# resolved to a fixed-rate tick grid offline, so the FPGA player needs no
# tempo arithmetic beyond one reloadable divider.
#
# Fidelity scope (matches the hardware player's gate budget):
#   * notes, ties, rests, durations, per-note gate% + velocity  -> exact
#   * instrument (SRCN + pitch multiplier), channel volume/pan  -> exact
#   * tempo set / tempo fade                                    -> quantized
#   * block repeats / subroutines (EF pat)                      -> unrolled
#   * vibrato / tremolo / pitch slides / echo / ADSR            -> dropped
#     (optional --pitch-fx approximates slides/vibrato with PITCH events)
#
# Sources: an .spc dump (standard 66048-byte SPC700 snapshot) or a raw
# binary + explicit addresses (config JSON). DK94 itself keeps its music on
# the GB-side APU engine; SGB-native N-SPC material (SGB BIOS programs or
# other Nintendo N-SPC titles) arrives as .spc dumps, hence both paths.
#
# Usage:
#   python3 sgb_music_compiler.py --cfg song.json [--out build/]
#
# Config JSON keys:
#   source         path to .spc file or raw binary
#   song_table     addr of gft (song pointer table) in APU space
#   song           song index into the table (default 0)
#   patches        addr of the instrument patch table (6 bytes / patch)
#   patch_count    number of patches (default 32)
#   drum_table     optional: list of SRCNs for vc 0xCC..0xDF, or addr of a
#                  22-byte SRCN table in APU space
#   voice_map      N-SPC channel -> hardware voice (default [0,1,2,3]);
#                  channels mapped to -1 are dropped
#   default_tempo  tempo assumed before any E7 vcmd (default 0xE0)
#   loop           true: emit a LOOP opcode back to stream start (default)
#   max_seconds    hard cap on compiled length (default 240)
#   name           free-form song name for the manifest

import argparse, json, math, os, struct, sys

_here = os.path.dirname(os.path.abspath(__file__))

TICK_RATE   = 1000          # FPGA sequencer tick rate (Hz), see sgb_music.v
MAX_VOICES  = 4
BANK_MAGIC  = b'MSGB'
BANK_VER    = 1
TABLE_OFF   = 0x0100        # song table
SAMPLE_OFF  = 0x0200        # sample directory (64 x 8 bytes)
DATA_OFF    = 0x0800        # stream + BRR region start (16-byte aligned)

# N-SPC driver tables (SGSOUND0.asm PROG_CODE_00)
GATE = [0x32, 0x65, 0x7F, 0x98, 0xB2, 0xCB, 0xE5, 0xFC]   # dur % values
VEL  = [0x19, 0x32, 0x4C, 0x65, 0x72, 0x7F, 0x8C, 0x98,
        0xA5, 0xB2, 0xBF, 0xCB, 0xD8, 0xE5, 0xF2, 0xFC]   # velocity values
SPFP = [1,1,2,3,0,1,2,1, 2,1,1,3,0,1,2,3, 1,3,3,0,1,3,0,3, 3,3,1]  # E0..FA

VC_NOTE_END = 0xCA          # tie  (xxx)
VC_REST     = 0xCB          # rest (yyy)
VC_DRUM0    = 0xCC          # first percussion vcmd
VC_SNO      = 0xE0          # set instrument
VC_PAN      = 0xE1
VC_TP1      = 0xE7          # tempo set
VC_PV1      = 0xED          # channel volume
VC_PAT      = 0xEF          # subroutine call
VC_WAV      = 0xFA          # percussion patch base

# ---------------------------------------------------------------------------
# source loading
# ---------------------------------------------------------------------------

def load_source(path):
    """Return a 64 KB APU address-space image."""
    data = open(path, 'rb').read()
    if path.lower().endswith('.spc') or len(data) == 66048:
        assert len(data) >= 0x10100, "short .spc file"
        # SPC700 dump: 0x100 header, 0x10000 RAM, DSP regs
        return bytearray(data[0x100:0x10100])
    if len(data) <= 0x10000:
        # raw blob: caller gives absolute APU addresses, image is 1:1
        img = bytearray(0x10000)
        img[:len(data)] = data
        return img
    raise SystemExit(f"unrecognized source {path} ({len(data)} bytes)")

def u16(ram, a):
    return ram[a] | ram[a + 1] << 8

# ---------------------------------------------------------------------------
# N-SPC song parsing -> per-channel event lists on the tick grid
# ---------------------------------------------------------------------------

class Channel:
    def __init__(self, idx):
        self.idx = idx
        self.events = []        # (tick, kind, ...)
        self.instr = 0
        self.vol_l = 0x7F       # after pan/vol bake
        self.vol_r = 0x7F

class Song:
    def __init__(self, ram, cfg):
        self.ram = ram
        self.cfg = cfg
        self.tempo = cfg.get('default_tempo', 0xE0)
        self.tick = 0.0                     # fractional tick position
        self.channels = [Channel(i) for i in range(8)]
        self.chan_time = [0.0] * 8          # next-free tick per channel
        self.warnings = []

    # -- timing -------------------------------------------------------------
    def step_ticks(self):
        # one sequencer step = 4096/tempo ms -> TICK_RATE*4.096/tempo ticks
        return max(1.0, TICK_RATE * 4.096 / self.tempo)

    # -- instruments / drums ------------------------------------------------
    def patch(self, n):
        base = self.cfg.get('patches')
        if base is None:
            return (n & 0x3F, 0x100)        # no table: SRCN passthrough, 1.0x
        a = base + 6 * n
        srcn = self.ram[a]
        # patches.asm: pitch mult = integer byte, then fractional 256ths
        mult = (self.ram[a + 4] << 8) | self.ram[a + 5]
        return (srcn, mult or 0x100)

    def drum_srcn(self, d):
        t = self.cfg.get('drum_table')
        if isinstance(t, list):
            return t[d] if d < len(t) else 0
        if isinstance(t, int):
            return self.ram[t + d]
        return 0                             # no drum mapping -> silence

    # -- vcmd effects we keep ------------------------------------------------
    def set_pan(self, ch, pan):
        # N-SPC pan 0..20 (0x0A = center). Bake into L/R attenuation.
        pan = min(20, max(0, pan))
        ch.vol_r = (pan * 0x7F) // 20
        ch.vol_l = ((20 - pan) * 0x7F) // 20
        ch.events.append((self.emit_tick(ch), 3, ch.vol_l, ch.vol_r))

    def emit_tick(self, ch):
        return int(round(self.chan_time[ch.idx]))

    # -- channel stream decode ----------------------------------------------
    def decode_channel(self, ch, addr, depth=0):
        ram = self.ram
        pc = addr
        dur = None                  # pending duration (steps)
        gate_pc = 255               # pending gate % (0xFC = full)
        vel = 0xFC                  # velocity
        t_end = self.t_end
        while True:
            if self.chan_time[ch.idx] >= t_end:
                return
            b = ram[pc]; pc += 1
            if b == 0x00:                       # segment end (block logic)
                return
            if b < 0x80:                        # duration
                dur = b
                nb = ram[pc]
                if nb < 0x80:                   # gate/velocity byte
                    pc += 1
                    gate_pc = GATE[nb >> 4]
                    vel = VEL[nb & 0xF]
                continue
            if b < VC_NOTE_END:                 # note on
                self.note(ch, b & 0x7F, dur, gate_pc, vel)
                continue
            if b == VC_NOTE_END:                # tie: extend previous note
                self.tie(ch, dur, gate_pc)
                continue
            if b == VC_REST:                    # rest
                self.rest(ch, dur)
                continue
            if b < VC_SNO:                      # percussion 0xCC..0xDF
                srcn = self.drum_srcn(b - VC_DRUM0)
                self.note(ch, 0x30, dur, gate_pc, vel, force_srcn=srcn,
                          drum=True)
                continue
            if b == VC_SNO:                     # instrument
                ch.instr = ram[pc]; pc += 1
                ch.events.append((self.emit_tick(ch), 1, ch.instr))
                continue
            if b == VC_PAN:
                self.set_pan(ch, ram[pc]); pc += 1
                continue
            if b == VC_TP1:                     # tempo
                self.tempo = ram[pc] or 1; pc += 1
                ch.events.append((self.emit_tick(ch), 4,
                                  max(1, round(self.step_ticks()))))
                continue
            if b == VC_PV1:                     # channel volume
                v = ram[pc]; pc += 1
                # bake into both sides keeping the pan ratio
                tot = ch.vol_l + ch.vol_r or 1
                ch.vol_l = (v * ch.vol_l) // tot if tot else v // 2
                ch.vol_r = v - ch.vol_l
                ch.events.append((self.emit_tick(ch), 3, ch.vol_l, ch.vol_r))
                continue
            if b == VC_PAT:                     # subroutine call
                sub = ram[pc] | ram[pc + 1] << 8
                cnt = ram[pc + 2]; pc += 3
                if depth >= 3:
                    self.warnings.append(f"subroutine nesting too deep @{pc-3:#x}")
                    continue
                for _ in range(cnt or 1):
                    ret = self.decode_channel(ch, sub, depth + 1)
                continue
            if 0xE0 <= b <= 0xFA:               # skip other specials
                pc += SPFP[b - 0xE0]
                continue
            self.warnings.append(f"unknown vcmd ${b:02X} @{pc-1:#x}")
            return                              # stream desynced; stop channel

    # -- note emission --------------------------------------------------------
    def note(self, ch, note, dur, gate_pc, vel, force_srcn=None, drum=False):
        if dur is None:
            dur = 1
        len_t = dur * self.step_ticks()
        t0 = self.chan_time[ch.idx]
        gate_t = max(1.0, len_t * gate_pc / 256.0)
        v = self.channels[ch.idx]
        srcn, mult = self.patch(v.instr) if force_srcn is None \
            else (force_srcn, 0x100)
        v.events.append((int(round(t0)), 0, note & 0x7F,
                         int(round(gate_t)), vel, srcn, mult,
                         1 if drum else 0))
        self.chan_time[ch.idx] = t0 + len_t

    def tie(self, ch, dur, gate_pc):
        if dur is None:
            return
        # extend the last note's sounding length (re-emit with longer len)
        v = self.channels[ch.idx]
        len_t = dur * self.step_ticks()
        last = None
        for i in range(len(v.events) - 1, -1, -1):
            if v.events[i][1] == 0:
                last = i; break
        if last is None:
            self.rest(ch, dur)
            return
        ev = list(v.events[last])
        ev[3] += int(round(len_t * gate_pc / 256.0))
        v.events[last] = tuple(ev)
        self.chan_time[ch.idx] += len_t

    def rest(self, ch, dur):
        if dur is None:
            return
        self.chan_time[ch.idx] += dur * self.step_ticks()

# ---------------------------------------------------------------------------
# block list / song walk
# ---------------------------------------------------------------------------

def compile_song(ram, cfg):
    song_no = cfg.get('song', 0)
    gft = cfg['song_table']
    blk_ptr = u16(ram, gft + 2 * song_no)
    if blk_ptr == 0:
        raise SystemExit(f"song {song_no}: empty gft entry")
    voice_map = cfg.get('voice_map', [0, 1, 2, 3])
    assert len(voice_map) <= MAX_VOICES and all(-1 <= v < 8 for v in voice_map)
    max_ticks = TICK_RATE * cfg.get('max_seconds', 240)

    song = Song(ram, cfg)
    song.t_end = max_ticks
    active = []                                # channels used by current block
    pc = blk_ptr
    blc = 0                                    # block repeat counter
    guard = 0
    while guard < 4096:
        guard += 1
        w = u16(ram, pc); pc += 2
        hi = w >> 8
        if hi != 0:                            # block: 8 channel pointers
            ptrs = [u16(ram, w + 2 * i) for i in range(8)]
            active = []
            for ch_i, p in enumerate(ptrs):
                if p == 0:
                    continue
                if ch_i >= len(voice_map) or voice_map[ch_i] == -1:
                    continue
                # N-SPC channels map to hardware voices via voice_map;
                # decode into the channel slot for the voice index
                slot = voice_map[ch_i]
                ch = song.channels[slot]
                song.chan_time[slot] = min(song.chan_time[c2]
                                           for c2 in range(8)
                                           if c2 == slot)
                active.append((slot, p))
            base_time = min((song.chan_time[s] for s, _ in active), default=0)
            for s, _ in active:
                song.chan_time[s] = base_time   # block sync
            for s, p in active:
                song.decode_channel(song.channels[s], p)
            # re-sync to the slowest channel at block end
            end_time = max((song.chan_time[s] for s, _ in active), default=0)
            for s, _ in active:
                song.chan_time[s] = end_time
            continue
        lo = w & 0xFF
        if lo == 0:                            # song end
            break
        # repeat: [00 cnt][addr]
        if blc == 0 or blc > 128:
            blc = lo
        else:
            blc -= 1
        addr = u16(ram, pc); pc += 2
        if blc != 0:
            pc = addr                          # loop back
        continue
    return song

# ---------------------------------------------------------------------------
# event merge -> linear tracker stream
# ---------------------------------------------------------------------------

def build_stream(song, cfg, voice_map):
    """Merge per-voice event lists into the single linear byte stream."""
    evs = []
    for v in range(MAX_VOICES):
        for ev in song.channels[v].events:
            evs.append((ev[0], v, ev[1:]))
    evs.sort(key=lambda e: (e[0], e[1], 0 if e[2][0] in (1, 3, 4) else 1))
    stream = bytearray()
    tick = 0
    def wait(delta):
        nonlocal tick
        while delta > 0:
            n = min(delta, 127)
            stream.append(n - 1)               # 0x00..0x7E = WAIT n
            delta -= n
    for t, v, body in evs:
        if t > tick:
            wait(t - tick)
            tick = t
        kind = body[0]
        if kind == 0:                          # NOTE_ON
            _, note, gate, vel, srcn, mult, drum = body
            stream += bytes([0x80 | v, note & 0x7F])
            stream += struct.pack('<H', min(0xFFFF, gate))
            stream += bytes([vel, srcn, mult & 0xFF, mult >> 8])
        elif kind == 1:                        # INSTR
            stream += bytes([0x90 | v, body[1]])
        elif kind == 3:                        # PANVOL
            stream += bytes([0xA0 | v, body[1], body[2]])
        elif kind == 4:                        # TEMPO (ticks per step)
            stream += bytes([0xF1]) + struct.pack('<H', body[1])
    if cfg.get('loop', True):
        stream += bytes([0xF0]) + b'\x00\x00\x00'   # LOOP -> stream start
    else:
        stream += bytes([0xFF])                # STOP
    return bytes(stream)

# ---------------------------------------------------------------------------
# sample directory / BRR collection
# ---------------------------------------------------------------------------

def collect_samples(ram, cfg, stream_refs):
    """Walk the APU sample directory; keep only samples referenced by the
    compiled song (plus their full block chains). Returns (dir records,
    brr region bytes, srcn->record map)."""
    dir_addr = cfg.get('sample_dir')
    if dir_addr is None:
        # .spc convention: DSP reg 0x5D (DIR) << 8
        dir_addr = cfg.get('dsp_dir', 0) << 8 if cfg.get('dsp_dir') else 0
    records = {}
    region = bytearray()
    pos_of = {}
    for srcn in sorted(stream_refs):
        ent = dir_addr + 4 * srcn
        st = u16(ram, ent)
        lp = u16(ram, ent + 2)
        if st == 0 or st == 0xFFFF:
            continue
        # walk the block chain to find the terminal block
        a = st; n = 0
        while n < 8192:
            h = ram[a]
            n += 1
            if h & 1:
                end = a + 9
                break
            a += 9
        else:
            raise SystemExit(f"SRCN ${srcn:02X}: no terminal BRR block")
        has_loop = lp != 0 and lp != 0xFFFF and st <= lp < end
        nblocks = (end - st) // 9
        off = len(region)
        # re-pack with a 10-byte stride (9-byte block + 1 pad): the PSRAM
        # arbiter serves 16-bit words, so every block must start at an even
        # address. The hardware player steps +10 per block and the fetch of
        # exactly one padded block is 5 words.
        for b_i in range(nblocks):
            region += ram[st + 9 * b_i: st + 9 * b_i + 9] + b'\x00'
        loop_off = ((lp - st) // 9) * 10 if has_loop else 0xFFFFFFFF
        records[srcn] = (off, loop_off, 1 if has_loop else 0)
        pos_of[srcn] = off
    return records, bytes(region)

def stream_srcn_refs(song):
    refs = set()
    for ch in song.channels:
        for ev in ch.events:
            if ev[1] == 0:                     # NOTE_ON carries srcn
                refs.add(ev[6] & 0x3F)
    return refs

# ---------------------------------------------------------------------------
# bank assembly
# ---------------------------------------------------------------------------

def emit_bank(stream, records, region, cfg, outdir):
    os.makedirs(outdir, exist_ok=True)
    bank = bytearray()
    bank += BANK_MAGIC + struct.pack('<HHIII', BANK_VER, 1,
                                     TABLE_OFF, SAMPLE_OFF, DATA_OFF)
    bank += b'\x00' * (TABLE_OFF - len(bank))
    # song table: 16 bytes / entry: u32 stream_off, u32 stream_len,
    # u32 brr_off (all relative to DATA_OFF), u32 reserved.
    # The hardware (sgb_music.v) reads this entry at song start, so a bank
    # is fully self-describing -- the RTL needs no per-bank parameters.
    bank += struct.pack('<IIII', 0, len(stream), 0, 0)
    bank += b'\x00' * (SAMPLE_OFF - len(bank))
    # sample directory: 64 x 8 bytes: u32 brr_off, u32 loop_off (rel region)
    for s in range(64):
        if s in records:
            off, loop, flags = records[s]
            bank += struct.pack('<II', off, loop | (0x80000000 * flags))
        else:
            bank += struct.pack('<II', 0xFFFFFFFF, 0xFFFFFFFF)
    bank += b'\x00' * (DATA_OFF - len(bank))
    stream_off = len(bank) - DATA_OFF
    bank += stream
    bank += b'\x00' * ((16 - len(bank) % 16) % 16)   # 16-align the region
    brr_off = len(bank) - DATA_OFF
    bank += region
    # pad to 16 bytes: block fetches are 10 bytes (9-byte BRR block + one
    # trailing byte for chain peeking), so the last block must have at
    # least one readable byte behind it in PSRAM
    bank += b'\x00' * ((16 - len(bank) % 16) % 16 or 16)
    # patch back the real offsets
    struct.pack_into('<IIII', bank, TABLE_OFF, stream_off, len(stream),
                     brr_off, 0)
    open(os.path.join(outdir, 'sgb_music_bank.bin'), 'wb').write(bytes(bank))

    dur_s = 0  # informational; the stream is tick-based
    vh = (f"// Auto-generated by sgb_music_compiler.py -- N-SPC -> MSGB bank.\n"
          f"// song: {cfg.get('name', '?')}\n"
          f"// layout: header@0x000, song table@{TABLE_OFF:#x}\n"
          f"//   (16B/entry: stream_off, stream_len, brr_off, rsvd -- rel "
          f"data@{DATA_OFF:#x}),\n"
          f"//   sample dir@{SAMPLE_OFF:#x} (64 x 8B: u32 brr_off rel BRR "
          f"region,\n"
          f"//   u32 loop_off rel sample start | 0x80000000 if looped)\n"
          f"// stream @{DATA_OFF + stream_off:#x} ({len(stream)} bytes), "
          f"BRR region @{DATA_OFF + brr_off:#x} ({len(region)} bytes)\n"
          f"`define SGB_MUSIC_BANK_BYTES {len(bank)}\n"
          f"`define SGB_MUSIC_DATA_OFF {DATA_OFF}\n"
          f"`define SGB_MUSIC_STREAM_OFF {DATA_OFF + stream_off}\n"
          f"`define SGB_MUSIC_BRR_OFF {DATA_OFF + brr_off}\n"
          f"`define SGB_MUSIC_TICK_RATE {TICK_RATE}\n")
    open(os.path.join(outdir, 'sgb_music_bank.vh'), 'w').write(vh)

    man = {'name': cfg.get('name'), 'tick_rate': TICK_RATE,
           'stream_bytes': len(stream), 'stream_off': DATA_OFF + stream_off,
           'brr_off': DATA_OFF + brr_off, 'brr_bytes': len(region),
           'samples': {f"${s:02X}": {'off': o, 'loop': l & 0x7FFFFFFF,
                                      'looped': bool(l >> 31)}
                       for s, (o, l, _) in records.items()},
           'bank_bytes': len(bank)}
    json.dump(man, open(os.path.join(outdir, 'sgb_music_bank.json'), 'w'),
              indent=1)
    # sim hex (16-bit LE words) for the RTL testbench
    with open(os.path.join(outdir, 'sim_bank.hex'), 'w') as f:
        pad = bytes(bank) + (b'\x00' if len(bank) & 1 else b'')
        for i in range(0, len(pad), 2):
            f.write("%04x\n" % (pad[i] | (pad[i + 1] << 8)))
    return bytes(bank), stream_off, brr_off

# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cfg', required=True, help='song config JSON')
    ap.add_argument('--out', default=os.path.join(_here, 'build'))
    a = ap.parse_args()
    cfg = json.load(open(a.cfg))
    ram = load_source(cfg['source'])
    song = compile_song(ram, cfg)
    refs = stream_srcn_refs(song)
    records, region = collect_samples(ram, cfg, refs)
    stream = build_stream(song, cfg, cfg.get('voice_map', [0, 1, 2, 3]))
    bank, soff, boff = emit_bank(stream, records, region, cfg, a.out)
    ticks = max((ev[0] for ch in song.channels for ev in ch.events),
                default=0)
    print(f"song '{cfg.get('name', '?')}': {len(stream)} stream bytes, "
          f"{len(refs)} samples, {len(region)} BRR bytes, "
          f"~{ticks / TICK_RATE:.1f}s, bank {len(bank)} bytes "
          f"({len(bank) / 1024:.1f} KB)")
    for w in song.warnings[:16]:
        print("warn:", w)

if __name__ == '__main__':
    main()
