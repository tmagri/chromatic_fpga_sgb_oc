#!/usr/bin/env python3
# Time-ordered pitch timelines for the dominant voice of each SGB SFX.
#
# Purpose: size + feed the bank-v5 pitch-segment feature. trace.json (built by
# extract_apu.py) only keeps energy-weighted pitch HISTOGRAMS per SRCN -- the
# time ordering of the driver's pitch writes is discarded. The piecewise
# pitch sequencer needs exactly that ordering: this script re-traces every
# effect through the real BIOS APU image and records, once per 32 kHz DSP
# output sample, which voice dominates (largest envelope, noise excluded) and
# its current 14-bit pitch. The per-effect result is run-length-encoded into
# constant-pitch runs: [(pitch, n_samples_at_32k, sum_envelope), ...].
#
# Output: build/pitch_timeline.json  {effect: {srcn, hit_cap, runs: [[P, n32, e]]}}
# plus a sizing report (runs per effect, total, distribution) used to choose
# the segment-table depth and the RTL segment store.
#
# The ROM is only READ via render.BIOS (SGB_BIOS env var or sgb_sfx/SGB1.sfc);
# nothing is copied, per the repo copyright rule.
#
# Usage:
#   SGB_BIOS=/path/to/SGB1.sfc python3 debug/pitch_timeline.py
#   SGB_BIOS=... python3 debug/pitch_timeline.py --only a01,a02,a03

import os, sys, json, argparse

_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_here))

import render
import extract_apu as ex      # reuse its rig boot path indirectly via render


def snap_timeline(rig, ms, snaps, energy, audio):
    """Advance the rig for `ms` milliseconds of APU time, appending, per
    32 kHz DSP output sample, the list of (srcn, pitch14, envelope) of every
    audible (KON'd, nonzero-envelope, non-noise) voice. Recording all voices
    (not just the per-sample max) matters: the energy-dominant voice of an
    effect is not always the loudest voice in every sample. Blocks the
    driver's MVOL ramp like extract_apu.run_snap so traces run at full
    master volume."""
    dsp, cpu = rig.dsp, rig.cpu
    target = cpu.cycles + int(ms * 1024)
    orig_w = dsp.write
    dsp.mvol = [0x7F, 0x7F]
    def w(a, v):
        if a in (0x0C, 0x1C):
            return
        orig_w(a, v)
    dsp.write = w
    try:
        while cpu.cycles < target and not cpu.halted:
            before = cpu.t2_div
            cpu.step()
            if cpu.t2_div < before:          # wrapped => one DSP output sample
                audio.append((dsp.out_l, dsp.out_r))
                vs = []
                for v in dsp.voices:
                    if v.keyonDelay == 0 and v.envelope > 0 and not v._noise:
                        s = v.source & 0xFF
                        e = v.envelope
                        energy[s] = energy.get(s, 0) + e
                        vs.append((s, v.pitch & 0x3FFF, e))
                snaps.append(vs)
    finally:
        dsp.write = orig_w
    if cpu.halted:
        raise RuntimeError(f"CPU halted at ${cpu.pc:04X}")


def trace(rig, side, idx, pitch, max_s=8.0, tail_s=0.25):
    """Trigger one effect (same port sequence as extract_apu.trace_effect) and
    return (snaps, energy, hit_cap) with snaps starting at the trigger."""
    snaps, energy, audio = [], {}, []
    b1 = idx if side == 'A' else 0
    b2 = idx if side == 'B' else 0
    pa = pitch if side == 'A' else 0
    pb = pitch if side == 'B' else 0
    attr = render.attrs(pa, 0, pb, 0)
    rig.set_ports(b1=0, b2=0, b3=attr)
    snap_timeline(rig, 12, [], energy, [])   # retrigger quiet; snaps discarded
    rig.set_ports(b1=b1, b2=b2, b3=attr)
    silence_ms, heard, total, hit_cap = 0, False, 0, True
    cap = int(max_s * 1000)
    while total < cap:
        n0 = len(snaps)
        snap_timeline(rig, 20, snaps, energy, audio)
        peak = max((abs(l) + abs(r) for l, r in audio[n0:]), default=0)
        if peak > 256:
            heard, silence_ms = True, 0
        else:
            silence_ms += 20
        total += 20
        if heard and silence_ms >= int(tail_s * 1000):
            hit_cap = False
            break
        if not heard and total >= 1500:
            hit_cap = False
            break
    rig.set_ports(b1=0x80 if b1 else 0, b2=0x80 if b2 else 0)
    snap_timeline(rig, 30, snaps, energy, [])  # stop packet tail
    return snaps, energy, hit_cap


def rle(snaps, dom):
    """Run-length-encode the dominant voice's pitch into
    [[pitch, n32_samples, sum_envelope], ...]. Samples where the dominant
    voice is silent terminate runs (gap markers are dropped)."""
    runs = []
    for vs in snaps:
        hit = next((t for t in vs if t[0] == dom), None)
        if hit is not None:
            p, e = hit[1], hit[2]
            if runs and runs[-1][0] == p:
                runs[-1][1] += 1
                runs[-1][2] += e
            else:
                runs.append([p, 1, e])
        else:
            if runs:
                runs.append([None, 1, 0])      # gap marker
    return [r for r in runs if r[0] is not None]


def reduce_runs(runs, min_n32=32):
    """Builder-style reduction for sizing: absorb runs shorter than min_n32
    (1 ms) into their left neighbor, then merge adjacent runs whose pitches
    differ by < 1%. Returns the reduced run count."""
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default=None, help='comma list e.g. a01,b0b')
    args = ap.parse_args()
    only = {s.strip().lower() for s in args.only.split(',')} if args.only else None

    rig = render.Rig()
    rig.run_ms(400)
    print(f"driver booted (entry=${rig.cpu.pc:04X})")

    out = {}
    sets = [('A', render.A_NAMES, render.A_PITCH),
            ('B', render.B_NAMES, render.B_PITCH)]
    for side, names, pitches in sets:
        for idx in sorted(names):
            key = f"{side}{idx:02X}"
            if only and key.lower() not in only:
                continue
            cap = 5.0 if (side == 'B' and idx in render.B_LOOP) else 6.0
            snaps, energy, hit_cap = trace(rig, side, idx, pitches[idx], max_s=cap)
            if not energy:
                print(f"{key}: silent")
                continue
            dom = max(energy, key=energy.get)
            runs = rle(snaps, dom)
            out[key] = {
                'srcn': dom,
                'hit_cap': hit_cap,
                'runs': runs,
            }
            nrun = len(runs)
            red = reduce_runs(runs)
            pitches_seen = sorted({r[0] for r in runs})
            pr = f"({min(pitches_seen)}..{max(pitches_seen)})" if pitches_seen else "(none)"
            print(f"{key} SRCN ${dom:02X}: {nrun:3d} runs -> {len(red):3d} reduced, "
                  f"{len(pitches_seen):3d} pitches {pr}"
                  f"{', sustained' if hit_cap else ''}")

    bdir = os.path.join(os.path.dirname(_here), 'build')
    os.makedirs(bdir, exist_ok=True)
    path = os.path.join(bdir, 'pitch_timeline.json')
    with open(path, 'w') as f:
        json.dump(out, f)
    print(f"wrote {path}")

    # ---- sizing report ----
    print("\n== sizing ==")
    total_raw = sum(len(e['runs']) for e in out.values())
    total_red = 0
    for key, e in sorted(out.items()):
        red = reduce_runs(e['runs'])
        total_red += len(red)
        if len(red) > 240:
            print(f"  OVER BUDGET after reduce: {key} = {len(red)}")
    print(f"total runs over {len(out)} effects: raw={total_raw}, reduced={total_red}")
    longest = sorted(
        ((k, len(reduce_runs(e['runs']))) for k, e in out.items()),
        key=lambda kv: -kv[1])[:10]
    print("worst (reduced):", ", ".join(f"{k}={n}" for k, n in longest))


if __name__ == '__main__':
    main()
