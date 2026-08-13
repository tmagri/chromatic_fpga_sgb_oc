#!/usr/bin/env python3
# Build the ROM-native SGB SFX bank (APU sound image + effect table).
#
# Instead of pre-rendering the 73 built-in SFX to PCM (build_bank_pcm.py,
# 304 KB for 8 effects), this ships the SGB BIOS's OWN sample data:
#
#   * the APU sound image is extracted from SGB1.sfc (LoROM file offset
#     0x30000 holds the APU boot stream): the sample directory record
#     ($4B00, 256 B) plus the BRR sample record ($4DB0, 41280 B), copied
#     VERBATIM. The SPC700 driver code ($0400) is NOT included -- the FPGA
#     never executes it (HLE player, see sgb_sfx_play.v).
#   * a per-effect playback table is derived by booting the real BIOS APU
#     image in the verified SPC700/DSP emulation (render.Rig) and tracing,
#     for every SFX index, which sample (SRCN) dominates the sound, at what
#     DSP pitch the driver plays it, and how long a one-shot runs.
#
# Bank layout (build/sgb_sfx_bank.bin, loaded by the MCU at PSRAM 0x100000):
#   0x000  header   'SFXB', u16 version(5), u16 count, u32 table_off,
#                    u32 brr_off, u32 seg_off
#   0x100  table    73 x 16-byte little-endian records (record v5, 8 x u16),
#                    index = effect: A01..A30 -> 0..47, B01..B19 -> 48..72
#                     u16 brr_off    byte offset of first BRR block in region
#                     u16 loop_off   byte offset of loop block; 0xFFFF = none
#                     u16 tick       segment-0 hClk divider reload
#                     u16 blocks     first-pass block count (incl. terminal)
#                     u16 flags      bit0 = loop region valid,
#                                    bit1 = loop forever
#                     u16 count      total BLOCKS to emit (0 = natural end /
#                                    loop-forever); 16-sample granularity
#                     u16 seg_off    entry index into the segment region
#                     u16 seg_count  envelope entries (0 = fixed pitch)
#   0x800  region   verbatim $4DB0 BRR record (byte offsets above are into it)
#   seg_off region  pitch envelope: seg_count x (u16 blocks16 of the current
#                    segment, u16 tick of the NEXT segment). The first
#                    segment's tick is the record's tick field; the final
#                    segment has no entry and holds its tick to the end.
#
# The pitch envelope comes from re-tracing each effect through the real BIOS
# APU code and run-length-encoding the dominant voice's DSP pitch writes over
# time (see debug/pitch_timeline.py for the standalone investigation tool).
# One-shots get the full contour; loop-forever SFX-B ambiences keep a single
# pitch. The sample SEQUENCE the player emits is unchanged by the envelope --
# only the per-sample rate varies -- so the bit-exact ref streams below stay
# valid; the envelope is verified separately in simulation.
#
# The tool also emits, under build/:
#   sgb_sfx_bank.vh / .json   documentation + manifest
#   sim_bank.hex              16-bit words for the RTL testbench
#   ref_XXX.hex               per-effect bit-exact reference decode (the exact
#                             sample sequence the FPGA player must emit)
#   trace.json                raw per-effect trace stats (reuse with --reuse)
#
# Fidelity scope (see plan / README): single dominant sample per effect,
# traced piecewise-constant pitch envelope (bank v5; no Gaussian, no ADSR,
# no echo), nearest-sample rate conversion, BRR loop points and predictor
# history carry taken from the ROM data.
#
# Usage:
#   python3 extract_apu.py                 # full run (trace + build)
#   python3 extract_apu.py --reuse         # rebuild from saved trace.json
#   python3 extract_apu.py --only A1F,B0B  # trace a subset (dev iteration)

import struct, json, os, argparse, math

import render                      # Rig, parse_stream, attrs, name/pitch tables
from build_bank import dec_sample  # verified ares-exact BRR nibble decode

_here = os.path.dirname(os.path.abspath(__file__))
DIR_ADDR = 0x4B00                 # sample directory record (APU addr)
BRR_ADDR = 0x4DB0                 # BRR sample data record (APU addr)
TABLE_OFF = 0x100
BRR_OFF = 0x800
HCLK = 16777216                   # FPGA audio clock (Hz)
DSP_RATE = 32000                  # SNES DSP output rate (Hz)
PITCH_1X = 0x1000                 # SNES DSP pitch register = 1.0x

def load_image():
    """Return (ram, entry, records) of the SGB1 APU image."""
    rom = open(render.BIOS, 'rb').read()
    recs, entry = render.parse_stream(rom, 0x30000)
    ram = bytearray(0x10000)
    for dst, data in recs:
        ram[dst:dst + len(data)] = data
    return ram, entry, recs

# ---------------------------------------------------------------------------
# sample directory / BRR region parsing
# ---------------------------------------------------------------------------

def parse_samples(ram, recs):
    """Walk the sample directory. Returns (region_bytes, samples) where
    samples[srcn] = {start, loop, end, has_loop, blocks, brr_off, loop_off}
    with offsets relative to the BRR region."""
    dir_data = next(data for dst, data in recs if dst == DIR_ADDR)
    brr_data = next(data for dst, data in recs if dst == BRR_ADDR)
    assert len(dir_data) == 256
    region_end = BRR_ADDR + len(brr_data)
    samples = {}
    for s in range(64):
        st = dir_data[s*4] | dir_data[s*4+1] << 8
        lp = dir_data[s*4+2] | dir_data[s*4+3] << 8
        if st in (0, 0xFFFF):
            continue
        assert BRR_ADDR <= st < region_end, f"SRCN ${s:02X} start ${st:04X} outside region"
        # walk blocks from start to the terminal (end-flag) block
        a = st; blocks = 0
        while True:
            assert a + 9 <= region_end, f"SRCN ${s:02X} walk ran past the region"
            h = ram[a]
            blocks += 1
            assert blocks < 8000, f"SRCN ${s:02X} no terminal block"
            assert (h >> 4) <= 15
            if h & 1:
                has_loop = bool(h & 2)
                end = a + 9
                break
            a += 9
        if has_loop:
            assert st <= lp < end, f"SRCN ${s:02X} loop ${lp:04X} not inside sample"
        if lp in (0, 0xFFFF):
            lp = None
        samples[s] = {
            'srcn': s, 'start': st, 'loop': lp, 'end': end,
            'has_loop': has_loop, 'blocks': blocks,
            'brr_off': st - BRR_ADDR,
            'loop_off': (lp - BRR_ADDR) if (has_loop and lp is not None) else None,
            'bytes': end - st,
        }
    return bytes(brr_data), samples

# ---------------------------------------------------------------------------
# driver trace: boot the real APU image, trigger each effect, watch the DSP
# ---------------------------------------------------------------------------

def run_snap(rig, ms, stats, audio, snap=None):
    """Like Rig.run_and_capture(full_vol=True) but also snapshots the DSP
    voices once per output sample (32 kHz) to accumulate per-SRCN energy and
    pitch statistics. If `snap` is a list, the per-sample list of audible
    (srcn, pitch14, envelope) voices is appended too -- the time-ordered
    record the pitch-envelope builder reduces into constant-pitch runs."""
    dsp, cpu = rig.dsp, rig.cpu
    target = cpu.cycles + int(ms * 1024)
    orig_w = dsp.write
    dsp.mvol = [0x7F, 0x7F]
    def w(a, v):
        if a in (0x0C, 0x1C):      # block the driver's MVOL ramp
            return
        orig_w(a, v)
    dsp.write = w
    try:
        while cpu.cycles < target and not cpu.halted:
            before = cpu.t2_div
            cpu.step()
            if cpu.t2_div < before:   # wrapped => one DSP output sample
                audio.append((dsp.out_l, dsp.out_r))
                vs = []
                for v in dsp.voices:
                    # audible contribution only (past the 5-sample KON delay)
                    if v.keyonDelay == 0 and v.envelope > 0:
                        e = v.envelope
                        if v._noise:
                            stats['noise'] += e
                            continue
                        s = v.source & 0xFF
                        p = v.pitch & 0x3FFF
                        stats['energy'][s] = stats['energy'].get(s, 0) + e
                        stats['ticks'][s] = stats['ticks'].get(s, 0) + 1
                        pe = stats['pitch'].setdefault(s, {})
                        pe[p] = pe.get(p, 0) + e
                        vs.append((s, p, e))
                if snap is not None:
                    snap.append(vs)
    finally:
        dsp.write = orig_w
    if cpu.halted:
        raise RuntimeError(f"CPU halted at ${cpu.pc:04X}")

def trace_effect(rig, side, idx, pitch, max_s=8.0, tail_s=0.25):
    """Trigger one effect exactly like render.render_effect and return
    (stats, audio, snap). `snap` is the time-ordered per-output-sample voice
    list from the trigger onward (the pitch-envelope source)."""
    stats = {'energy': {}, 'ticks': {}, 'pitch': {}, 'noise': 0}
    audio = []
    snap = []
    b1 = idx if side == 'A' else 0
    b2 = idx if side == 'B' else 0
    pa = pitch if side == 'A' else 0
    pb = pitch if side == 'B' else 0
    attr = render.attrs(pa, 0, pb, 0)
    # retrigger: clear the index first
    rig.set_ports(b1=0, b2=0, b3=attr)
    run_snap(rig, 12, stats, audio)
    rig.set_ports(b1=b1, b2=b2, b3=attr)
    run_snap(rig, 6, stats, audio, snap)
    silence_ms = 0; heard = False; total = 0
    cap = int(max_s * 1000)
    hit_cap = True                      # still sounding when the trace gave up
    while total < cap:
        n0 = len(audio)
        run_snap(rig, 20, stats, audio, snap)
        peak = max((abs(l) + abs(r) for l, r in audio[n0:]), default=0)
        if peak > 256:
            heard = True; silence_ms = 0
        else:
            silence_ms += 20
        total += 20
        if heard and silence_ms >= int(tail_s * 1000):
            hit_cap = False             # driver envelope decayed to silence
            break
        if not heard and total >= 1500:
            hit_cap = False
            break
    # stop packet (ends sustained SFX-B loops)
    rig.set_ports(b1=0x80 if b1 else 0, b2=0x80 if b2 else 0)
    run_snap(rig, 30, stats, audio, snap)
    return stats, audio, hit_cap, snap

def rle_runs(snap, dom):
    """Run-length-encode the per-sample timeline of voice `dom` into
    [[pitch, n32_samples, sum_envelope], ...]; samples where the voice is
    silent split runs."""
    runs = []
    for vs in snap:
        hit = next((t for t in vs if t[0] == dom), None)
        if hit is not None:
            p, e = hit[1], hit[2]
            if runs and runs[-1][0] == p:
                runs[-1][1] += 1
                runs[-1][2] += e
            else:
                runs.append([p, 1, e])
        elif runs:
            runs.append([None, 1, 0])            # gap marker
    return [r for r in runs if r[0] is not None]

def reduce_runs(runs, min_n32=32):
    """Size-friendly run reduction: absorb runs shorter than min_n32 (1 ms)
    into the left neighbor, then merge adjacent runs whose pitches differ by
    less than 1% (an inaudible step)."""
    red = []
    for r in runs:
        if red and r[1] < min_n32:
            red[-1][1] += r[1]
            red[-1][2] += r[2]
        else:
            red.append(list(r))
    out = []
    for r in red:
        if out and abs(r[0] - out[-1][0]) * 100 < max(r[0], out[-1][0], 1):
            out[-1][1] += r[1]
            out[-1][2] += r[2]
        else:
            out.append(list(r))
    return out

def trace_all(only=None, wavdir=None):
    rig = render.Rig()
    rig.run_ms(400)                       # boot the N-SPC driver
    print(f"driver booted (entry=${rig.cpu.pc:04X}, cycles={rig.cpu.cycles})")
    out = {}
    sets = [('A', render.A_NAMES, render.A_PITCH),
            ('B', render.B_NAMES, render.B_PITCH)]
    for side, names, pitches in sets:
        for idx in sorted(names):
            key = f"{side}{idx:02X}"
            if only and key.lower() not in only:
                continue
            cap = 5.0 if (side == 'B' and idx in render.B_LOOP) else 6.0
            stats, audio, hit_cap, snap = trace_effect(rig, side, idx,
                                                       pitches[idx], max_s=cap)
            # time-ordered pitch runs of the energy-dominant voice (the
            # pitch-envelope source; see build_segments)
            runs = []
            if stats['energy']:
                dom = max(stats['energy'], key=stats['energy'].get)
                runs = reduce_runs(rle_runs(snap, dom))
            # JSON-serializable stats (dict keys -> str). The raw audio is not
            # kept; the trimmed audible duration is what classification needs.
            out[key] = {
                'name': render.A_NAMES.get(idx) if side == 'A' else render.B_NAMES.get(idx),
                'energy': {str(k): v for k, v in stats['energy'].items()},
                'ticks':  {str(k): v for k, v in stats['ticks'].items()},
                'pitch':  {str(k): {str(p): e for p, e in v.items()}
                           for k, v in stats['pitch'].items()},
                'noise': stats['noise'],
                'audio_samples': len(audio),
                'duration_s': audible_duration(audio),
                'hit_cap': hit_cap,
                'runs': runs,
            }
            top = sorted(stats['energy'].items(), key=lambda kv: -kv[1])[:3]
            tops = ", ".join(f"SRCN ${s:02X}" for s, _ in top) or "silent"
            print(f"{key} {out[key]['name']:<20} {tops} "
                  f"({len(audio)/32000:.2f}s, noise {stats['noise']}"
                  f"{', sustained' if hit_cap else ''})")
            # re-render the reference WAV with the (fixed) emulator so wav/
            # matches what the verified DSP actually produces
            if wavdir and heard_any(audio):
                os.makedirs(wavdir, exist_ok=True)
                nm = out[key]['name'].replace(' ', '')
                render.save_wav(os.path.join(wavdir, f"{key}_{nm}.wav"), audio)
    return out

def heard_any(audio, thresh=256):
    return any(abs(l) + abs(r) > thresh for l, r in audio)

# ---------------------------------------------------------------------------
# pitch envelope: timeline runs -> bank v5 segment entries
# ---------------------------------------------------------------------------
#
# The player is varispeed: with pitch P it emits one decoded sample every
# tick = round(HCLK / (DSP_RATE*P/4096)) - 1 hClk cycles, i.e. at
# R = DSP_RATE*P/4096 samples/s. A timeline run of n32 DSP-output samples at
# pitch P lasts n32/DSP_RATE seconds, in which the player emits
# M = n32 * P / 4096 samples -- so run i maps to
#     blocks16_i = M_i / 16        (the record's 16-sample block granularity)
#     tick_i     = round(HCLK / R_i) - 1
# The segment table stores, per transition, the CURRENT segment's blocks16
# and the NEXT segment's tick; the first segment's tick is the record's tick
# field and the final segment has no entry (it holds its tick until the
# effect's own end, so the envelope can never desync from the block stream).

def build_segments(runs, total_blocks):
    """Reduce timeline runs into v5 segment data spanning `total_blocks`
    16-sample blocks. Returns (entries, segments, tick0):
      entries  list of (blocks16, next_tick) bank entries ([] = no envelope)
      segments full per-segment [(blocks16, tick)] list (documentation/checks)
      tick0    the first segment's tick (None if runs are unusable)"""
    # pitch<=0 = driver paused the voice: donate the time to the left run
    cleaned = []
    for P, n, e in runs:
        if P <= 0:
            if cleaned:
                cleaned[-1][1] += n
            continue
        cleaned.append([P, n, e])
    if not cleaned:
        return [], [], None
    conv = []
    for P, n, e in cleaned:
        blk_f = n * P / PITCH_1X / 16.0
        tick = max(1, round(HCLK * PITCH_1X / (DSP_RATE * P)) - 1)
        conv.append([blk_f, tick])
    # sub-half-block dust (<8 emitted samples) merges left
    merged = []
    for blk_f, tick in conv:
        if blk_f < 0.5:
            if merged:
                merged[-1][0] += blk_f
            continue
        merged.append([blk_f, tick])
    if not merged:
        return [], [], None
    # block boundaries (clamped to total; the last run absorbs the remainder
    # or whatever is left when the timeline over-runs the effect)
    bounds, cum_f = [], 0.0
    for blk_f, _ in merged:
        cum_f += blk_f
        bounds.append(min(total_blocks, int(round(cum_f))))
    bounds[-1] = total_blocks
    segs, prev = [], 0
    for b, (_, tick) in zip(bounds, merged):
        if b > prev:
            segs.append([b - prev, tick])
            prev = b
    if prev < total_blocks:
        segs.append([total_blocks - prev, merged[-1][1]])
    if len(segs) < 2:
        # everything collapsed to one pitch: no envelope, but report the pitch
        return [], segs, segs[0][1] if segs else merged[0][1]
    entries = [(blocks, segs[i + 1][1])
               for i, (blocks, _) in enumerate(segs[:-1])]
    return entries, segs, segs[0][1]

def run_total_blocks(runs):
    """The looped-one-shot duration implied by the timeline (in 16-sample
    blocks): sum of n32*P/4096 over the runs."""
    tot = 0.0
    for P, n, e in runs:
        if P > 0:
            tot += n * P / PITCH_1X
    return max(1, int(round(tot / 16.0)))

# ---------------------------------------------------------------------------
# classification -> per-effect records
# ---------------------------------------------------------------------------

def weighted_median(pe):
    """Energy-weighted median of a {pitch: energy} dict."""
    items = sorted((int(p), e) for p, e in pe.items())
    tot = sum(e for _, e in items)
    if tot <= 0:
        return items[0][0] if items else 0
    acc = 0
    for p, e in items:
        acc += e
        if acc * 2 >= tot:
            return p
    return items[-1][0]

def audible_duration(audio, thresh=256):
    """Seconds up to the last sample above threshold (drops the silent tail
    the 250 ms gap detector appends)."""
    last = 0
    for i, (l, r) in enumerate(audio):
        if abs(l) + abs(r) > thresh:
            last = i
    return (last + 1) / DSP_RATE

def classify(trace, samples, overrides):
    """Build the 73 effect records (record v5). Returns
    (records, manifest, seg_entries) where records is index-ordered
    (A01..A30, B01..B19), each entry is
    (brr_off, loop_off, tick, blocks, flags, count, seg_off, seg_count)
    with seg_off a placeholder assigned by emit(), and seg_entries[i] is the
    effect's list of (blocks16, next_tick) segment-table entries."""
    records = []
    manifest = {}
    seg_entries = []
    sets = [('A', render.A_NAMES, render.A_PITCH),
            ('B', render.B_NAMES, render.B_PITCH)]
    for side, names, pitches in sets:
        for idx in sorted(names):
            key = f"{side}{idx:02X}"
            tr = trace.get(key)
            rec = {'effect': key, 'name': names[idx], 'side': side,
                   'attr_pitch': pitches[idx]}
            brr_off = loop_off = tick = blocks = flags = count = None
            entries, segs = [], []
            if tr and tr['energy']:
                # dominant sample = largest envelope-energy share
                srcn = max((int(k) for k in tr['energy']),
                           key=lambda s: tr['energy'][str(s)])
                tot = sum(tr['energy'].values()) + tr['noise']
                share = tr['energy'][str(srcn)] / tot if tot else 0.0
                pe = tr['pitch'].get(str(srcn), {})
                P = weighted_median(pe) if pe else 0
                P0 = min((int(p) for p in pe), default=0)  # lowest pitch seen
                R = DSP_RATE * P / PITCH_1X
                tick = max(1, round(HCLK / R) - 1) if R > 0 else 0
                smp = samples.get(srcn)
                dur = tr.get('duration_s', 0.0)
                # hit_cap = still sounding when the trace window ended. For
                # looped samples that is the ONLY reliable sustain signal:
                # the driver's envelope decides when the sound stops (BRR
                # keeps looping underneath while it fades). E.g. B0B Wave
                # loops forever in BRR but its envelope dies by ~2.5 s, so
                # the trace ends early and it becomes a count-based one-shot.
                # Traces without hit_cap data (old trace.json) fall back to
                # the B-loop-set heuristic.
                FORCE_FULL_LOOP = {'B0B', 'B10', 'B19'}
                is_force = (key in FORCE_FULL_LOOP)
                sustained = tr.get('hit_cap',
                                   side == 'B' and idx in render.B_LOOP)
                if is_force: sustained = True
                runs = tr.get('runs', [])
                if smp is None:
                    rec['error'] = f"SRCN ${srcn:02X} has no directory entry"
                elif P == 0 and not runs:
                    rec['error'] = f"traced pitch is 0 (SRCN ${srcn:02X})"
                elif not smp['has_loop'] and not is_force:
                    flags, count = 0, 0             # natural BRR end flag stops it
                elif sustained and side == 'B':
                    flags, count = 3, 0             # loop region until SOUND stop
                else:
                    flags = 1                       # looped sample, one-shot effect
                    # count in BLOCKS (16-sample units; <=0.5 ms granularity,
                    # inaudible) so it fits the record's u16 field. When a
                    # pitch timeline exists it defines the duration exactly
                    # (the old formula used the median rate).
                    if runs:
                        count = run_total_blocks(runs)
                    else:
                        count = max(1, math.ceil(dur * R / 16))
                rec['sustained'] = bool(sustained)
                if smp is not None and 'error' not in rec:
                    brr_off = smp['brr_off']
                    if is_force:
                        loop_off = brr_off
                    else:
                        loop_off = smp['loop_off'] if smp['loop_off'] is not None else 0xFFFF
                    blocks = smp['blocks']          # first-pass blocks incl. terminal
                # ---- pitch envelope (record v5) ----
                # Loop-forever ambiences keep a single (median) pitch: their
                # timelines are jitter around a drone and the envelope would
                # have to be loop-aligned; one-shots get the full contour.
                if runs and flags in (0, 1) and 'error' not in rec:
                    total = run_total_blocks(runs) if flags == 1 else blocks
                    entries, segs, tick0 = build_segments(runs, total)
                    if entries and tick0:
                        tick = tick0                # record tick = segment-0 tick
                        if flags == 1:
                            count = total           # envelope-spans duration
                rec.update({'srcn': srcn, 'energy_share': round(share, 3),
                            'noise_share': round(tr['noise'] / tot, 3) if tot else 0.0,
                            'pitch': P, 'pitch_min': P0,
                            'rate_hz': round(R, 1), 'tick': tick,
                            'start_addr': smp['start'] if smp else None,
                            'loop_addr': smp['start'] if (smp and is_force) else (smp['loop'] if smp else None),
                            'duration_s': round(dur, 3),
                            'blocks': smp['blocks'] if smp else None,
                            'segments': segs})
            else:
                rec['error'] = 'silent in trace'
            # manual overrides (srcn re-pick / tick / count / flags / noseg)
            ov = overrides.get(key, {})
            if ov:
                srcn = ov.get('srcn', rec.get('srcn'))
                smp = samples.get(srcn) if srcn is not None else None
                if smp is not None:
                    brr_off = smp['brr_off']
                    loop_off = smp['loop_off'] if smp['loop_off'] is not None else 0xFFFF
                    blocks = smp['blocks']
                    rec['srcn'] = srcn
                    rec['start_addr'] = smp['start']
                    rec['loop_addr'] = smp['start'] if is_force else smp['loop']
                    rec['blocks'] = smp['blocks']
                    rec.pop('error', None)
                if 'tick' in ov:
                    tick = ov['tick']; rec['tick'] = tick
                    entries = []                    # manual tick = fixed pitch
                if 'count' in ov: count = ov['count']
                if 'flags' in ov: flags = ov['flags']
                if ov.get('noseg'): entries = []
                rec['overridden'] = sorted(ov)
            if brr_off is None:                     # invalid/silent effect
                records.append((0xFFFF, 0xFFFF, 0, 0, 0, 0, 0, 0))
            else:
                # record v5: 8 x u16 LE (brr_off, loop_off (0xFFFF=none),
                # tick, blocks, flags, count, seg_off, seg_count)
                records.append((brr_off,
                                loop_off if loop_off is not None else 0xFFFF,
                                tick or 0, blocks or 0, flags or 0, count or 0,
                                0, len(entries)))
            seg_entries.append(entries)
            rec['seg_count'] = len(entries)
            rec['record'] = len(records) - 1
            manifest[key] = rec
    return records, manifest, seg_entries

# ---------------------------------------------------------------------------
# reference decode: the exact sample sequence the FPGA player must emit
# ---------------------------------------------------------------------------

def decode_stream(region, smp, flags, count):
    """bsnes/ares-exact decode (build_bank.dec_sample) of one effect's stream.
    History starts at zero and CARRIES across the loop jump (real DSP does not
    reset it). `count` is in BLOCKS (record v4). Loop-forever effects: first
    pass + one loop pass. One-shots: until the end flag, or `count` blocks
    for looped samples played once."""
    out = []
    looped_forever = bool(flags & 2)
    has_loop = bool(flags & 1) and smp['loop_off'] is not None
    limit = None
    if has_loop and not looped_forever:
        limit = count * 16                          # count is in blocks
    elif has_loop and looped_forever:
        first = (smp['end'] - smp['start']) // 9          # blocks in first pass
        loop_blocks = (smp['end'] - smp['loop']) // 9
        limit = (first + loop_blocks) * 16
    p1 = p2 = 0
    pos = smp['brr_off']
    region_n = len(region)
    while limit is None or len(out) < limit:
        if pos + 9 > region_n:
            raise RuntimeError("BRR walk left the region")
        h = region[pos]
        shift = h >> 4; filt = (h >> 2) & 3; fl = h & 3
        for i in range(8):
            b = region[pos + 1 + i]
            for nib in (b >> 4, b & 0xF):
                d = dec_sample(nib, shift, filt, p1, p2)
                out.append(d)
                p2 = p1; p1 = d
                if limit is not None and len(out) >= limit:
                    return out
        if fl & 1:
            if (fl & 2) and has_loop:
                pos = smp['loop_off']      # predictor history carries over
            else:
                return out
        else:
            pos += 9
    return out

# ---------------------------------------------------------------------------
# bank assembly
# ---------------------------------------------------------------------------

def emit(records, manifest, region, seg_entries, outdir):
    """Assemble the v5 bank: header, 73 x 16B records (8 x u16), verbatim BRR
    region, then the pitch-envelope segment region (u16 blocks16 + u16
    next_tick per entry). Record seg_off is an ENTRY index into the region."""
    os.makedirs(outdir, exist_ok=True)
    seg_region = BRR_OFF + len(region)       # segment region right after BRR
    total_entries = sum(len(e) for e in seg_entries)
    assert total_entries * 4 + seg_region <= 0x10000, "bank exceeds u16 offsets"
    bank = bytearray()
    bank += b'SFXB' + struct.pack('<HHIII', 5, len(records),
                                  TABLE_OFF, BRR_OFF, seg_region)
    bank += b'\x00' * (TABLE_OFF - len(bank))
    entry_pos = 0
    for i, rec in enumerate(records):
        brr_off, loop_off, tick, blocks, flags, count = rec[:6]
        assert len(seg_entries[i]) == rec[7]
        assert 0 <= brr_off <= 0xFFFF and 0 <= loop_off <= 0xFFFF
        bank += struct.pack('<HHHHHHHH', brr_off, loop_off, tick, blocks,
                            flags, count, entry_pos, rec[7])
        entry_pos += rec[7]
    bank += b'\x00' * (BRR_OFF - len(bank))
    bank += region
    bank += b'\x00' * (seg_region - len(bank))
    for entries in seg_entries:
        for blocks16, next_tick in entries:
            assert 1 <= blocks16 <= 0xFFFF and 1 <= next_tick <= 0xFFFF
            bank += struct.pack('<HH', blocks16, next_tick)
    open(os.path.join(outdir, 'sgb_sfx_bank.bin'), 'wb').write(bytes(bank))

    # VH documentation header
    vh = (f"// Auto-generated by extract_apu.py -- ROM-native SGB SFX bank (v5).\n"
          f"// APU sound image: sample directory + verbatim BRR from SGB1.sfc,\n"
          f"// plus per-effect pitch-envelope segments (bank v5).\n"
          f"// Layout: header@0x000, table@{TABLE_OFF:#x} (73 x 16B LE records:\n"
          f"//   u16 brr_off, u16 loop_off (0xFFFF=none), u16 tick (segment-0\n"
          f"//   tick), u16 blocks (first-pass blocks incl. terminal), u16 flags\n"
          f"//   (bit0 loop-region valid, bit1 loop-forever), u16 count\n"
          f"//   (total BLOCKS to emit; 0 = natural end / loop-forever),\n"
          f"//   u16 seg_off (entry index into the segment region),\n"
          f"//   u16 seg_count (envelope entries; 0 = fixed pitch)),\n"
          f"//   BRR region@{BRR_OFF:#x} (record offsets are into the region),\n"
          f"//   segment region@{seg_region:#x}: seg_count x (u16 blocks16 of the\n"
          f"//   current segment, u16 tick of the NEXT segment); the final\n"
          f"//   segment has no entry and holds its tick to the effect's end.\n"
          f"`define SGB_SFX_COUNT {len(records)}\n"
          f"`define SGB_SFX_TABLE_OFF {TABLE_OFF}\n"
          f"`define SGB_SFX_BRR_OFF {BRR_OFF}\n"
          f"`define SGB_SFX_SEG_OFF {seg_region}\n"
          f"`define SGB_SFX_SEG_ENTRIES {total_entries}\n"
          f"`define SGB_SFX_BANK_BYTES {len(bank)}\n")
    for key in sorted(manifest, key=lambda k: manifest[k]['record']):
        m = manifest[key]
        r = records[m['record']]
        extra = m.get('error', f"SRCN ${m.get('srcn', 0):02X} P={m.get('pitch')} "
                               f"tick={r[2]} blocks={r[3]} flags={r[4]} "
                               f"count={r[5]} segs={r[7]}")
        vh += f"// record {m['record']:2d}: {key} {m['name']:<20} {extra}\n"
    open(os.path.join(outdir, 'sgb_sfx_bank.vh'), 'w').write(vh)

    # JSON manifest (strip the raw audio)
    man = {k: {f: v for f, v in m.items() if f != 'audio'}
           for k, m in manifest.items()}
    json.dump(man, open(os.path.join(outdir, 'sgb_sfx_bank.json'), 'w'), indent=1)

    # sim bank for the RTL testbench (16-bit words, little-endian)
    with open(os.path.join(outdir, 'sim_bank.hex'), 'w') as f:
        for i in range(0, len(bank), 2):
            f.write("%04x\n" % (bank[i] | (bank[i+1] << 8)))
    return bytes(bank)

def emit_refs(records, manifest, region, samples, outdir):
    """Per-effect reference decodes for bit-exact RTL comparison."""
    refs = {}
    for key in sorted(manifest, key=lambda k: manifest[k]['record']):
        m = manifest[key]
        rec = records[m['record']]
        if rec[0] == 0xFFFF or m.get('error'):
            continue
        smp = samples[m['srcn']]
        s = decode_stream(region, smp, rec[4], rec[5])
        with open(os.path.join(outdir, f"ref_{key}.hex"), 'w') as f:
            for x in s:
                f.write("%04x\n" % (x & 0xFFFF))
        refs[key] = len(s)
    return refs

# ---------------------------------------------------------------------------
# self-check: correlate the reference decode against the rendered WAV
# ---------------------------------------------------------------------------

def resample_lin(sig, src_rate, dst_rate):
    out = []
    p = 0.0
    ratio = src_rate / dst_rate
    n = len(sig)
    while int(p) < n - 1:
        i = int(p); f = p - i
        out.append(sig[i] * (1 - f) + sig[i+1] * f)
        p += ratio
    return out

def norm_corr(a, b, max_lag):
    n = min(len(a), len(b))
    if n < 64:
        return 0.0
    a = a[:n]; b = b[:n]
    ma = sum(a) / n; mb = sum(b) / n
    a = [x - ma for x in a]; b = [x - mb for x in b]
    sa = math.sqrt(sum(x*x for x in a)); sb = math.sqrt(sum(x*x for x in b))
    if sa == 0 or sb == 0:
        return 0.0
    best = -1.0
    step = max(1, n // 2000)
    for lag in range(0, max_lag, step):
        for sgn in (1, -1) if lag else (1,):
            if sgn > 0:
                acc = sum(a[i] * b[i+lag] for i in range(n - lag))
            else:
                acc = sum(a[i] * b[i-lag] for i in range(lag, n))
            c = acc / (sa * sb)
            if c > best:
                best = c
    return best

def synth_envelope(ref, segs):
    """Time-domain reconstruction of the player's output: resample each
    segment's slice of the reference stream at that segment's rate and
    concatenate (32 kHz). For single-pitch effects this equals the plain
    fixed-rate resample."""
    out = []
    pos = 0
    for blocks16, tick in segs:
        n = min(blocks16 * 16, len(ref) - pos)
        if n <= 0:
            break
        R = HCLK / (tick + 1)
        out += resample_lin([float(x) for x in ref[pos:pos + n]], R, DSP_RATE)
        pos += n
    # tail (if the ref is longer than the envelope span)
    if pos < len(ref):
        R = HCLK / (segs[-1][1] + 1)
        out += resample_lin([float(x) for x in ref[pos:]], R, DSP_RATE)
    return out

def check_against_wav(records, manifest, region, samples, wavdir):
    """The rendered WAV is the DSP's Gaussian output at 32 kHz; the reference
    decode is the raw BRR sample stream at the effect rate. Resample both to
    4 kHz and report the best-lag normalized correlation. `fixed` uses one
    rate for the whole effect (the v4 behavior); `env` reconstructs the v5
    per-segment timing -- for pitch-modulated effects env is the meaningful
    number. Single-sample, non-sweep effects should be ~1.0; multi-voice
    effects legitimately score lower (logged, not failed)."""
    import wave as wavemod
    print(f"\n{'fx':<5} {'fixed':>6} {'env':>6}  {'len':>6}  note")
    results = {}
    for key in sorted(manifest, key=lambda k: manifest[k]['record']):
        m = manifest[key]
        rec = records[m['record']]
        if rec[0] == 0xFFFF or m.get('error'):
            continue
        path = None
        for fn in os.listdir(wavdir):
            if fn.startswith(key) and fn.endswith('.wav'):
                path = os.path.join(wavdir, fn); break
        if path is None:
            print(f"{key:<5}   --   no rendered WAV"); continue
        smp = samples[m['srcn']]
        ref = decode_stream(region, smp, rec[4], rec[5])
        w = wavemod.open(path)
        nfr = w.getnframes(); ch = w.getnchannels()
        raw = struct.unpack('<%dh' % (nfr * ch), w.readframes(nfr))
        w.close()
        mono = ([(raw[i] + raw[i+1]) / 2 for i in range(0, len(raw), 2)]
                if ch == 2 else list(raw))
        R = HCLK / (rec[2] + 1)
        a = resample_lin([float(x) for x in ref], R, 4000)[:4000]
        b = resample_lin(mono, DSP_RATE, 4000)[:4000]
        c_fixed = norm_corr(a, b, 800)
        segs = m.get('segments') or []
        if len(segs) >= 2:
            e = synth_envelope(ref, segs)
            c_env = norm_corr(resample_lin(e, DSP_RATE, 4000)[:4000], b, 800)
        else:
            c_env = c_fixed
        results[key] = {'fixed': round(c_fixed, 3), 'env': round(c_env, 3)}
        note = ''
        if m.get('energy_share', 1.0) < 0.85:
            note = 'multi-sample effect'
        elif m.get('noise_share', 0) > 0.05:
            note = 'noise component'
        print(f"{key:<5} {c_fixed:6.3f} {c_env:6.3f}  {len(ref):6d}  {note}")
    return results

# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(_here, 'build'))
    ap.add_argument('--wav', default=os.path.join(_here, 'wav'))
    ap.add_argument('--only', default='', help='comma list, e.g. A1F,B0B')
    ap.add_argument('--reuse', action='store_true',
                    help='reuse build/trace.json instead of re-emulating')
    a = ap.parse_args()
    only = set(s.strip().lower() for s in a.only.split(',') if s.strip()) or None

    ram, entry, recs = load_image()
    print(f"APU image: entry=${entry:04X}, {len(recs)} records")
    for dst, data in recs:
        print(f"  ${dst:04X} len={len(data)}")
    region, samples = parse_samples(ram, recs)
    print(f"BRR region: {len(region)} bytes, {len(samples)} directory entries, "
          f"{sum(s['has_loop'] for s in samples.values())} looped samples")
    # invalid-shift scan (sgb_sfx_play.v must implement the shift>12 rule)
    bad = []
    for s in samples.values():
        pos = s['brr_off']
        while True:
            h = region[pos]; shift = h >> 4
            if shift > 12:
                bad.append((s['srcn'], pos, h))
            if h & 1:
                break
            pos += 9
    for srcn, pos, h in bad:
        print(f"NOTE: invalid shift block: SRCN ${srcn:02X} region+{pos:#x} hdr=${h:02X} "
              f"(player implements the (n>>>3)<<<12 rule)")

    overrides = {}
    ovpath = os.path.join(_here, 'overrides.json')
    if os.path.exists(ovpath):
        overrides = json.load(open(ovpath))

    tpath = os.path.join(a.out, 'trace.json')
    if a.reuse:
        trace = json.load(open(tpath))
        if only:
            trace = {k: v for k, v in trace.items() if k.lower() in only}
        print(f"reused trace ({len(trace)} effects)")
    else:
        trace = trace_all(only, wavdir=a.wav)
        os.makedirs(a.out, exist_ok=True)
        slim = {k: {f: v for f, v in t.items() if f != 'audio'}
                for k, t in trace.items()}
        json.dump(slim, open(tpath, 'w'), indent=1)
        print(f"trace saved to {tpath}")

    records, manifest, seg_entries = classify(trace, samples, overrides)
    bank = emit(records, manifest, region, seg_entries, a.out)
    refs = emit_refs(records, manifest, region, samples, a.out)
    n_ok = sum(1 for r in records if r[0] != 0xFFFF)
    n_loop = sum(1 for r in records if r[4] & 2)
    n_env = sum(1 for r in records if r[7] > 0)
    n_seg = sum(len(e) for e in seg_entries)
    seg_region = BRR_OFF + len(region)
    print(f"\nbank: {len(bank)} bytes ({len(bank)/1024:.1f} KB) | "
          f"{n_ok}/{len(records)} effects mapped | {n_loop} loop-forever | "
          f"{n_env} envelope effects / {n_seg} segments | "
          f"BRR @{BRR_OFF:#x}, segments @{seg_region:#x}")
    errs = {k: m['error'] for k, m in manifest.items() if m.get('error')}
    if errs:
        print("UNMAPPED:")
        for k, e in sorted(errs.items()):
            print(f"  {k}: {e}")
    check_against_wav(records, manifest, region, samples, a.wav)

if __name__ == '__main__':
    main()
