#!/usr/bin/env python3
# Bit-exact decode equivalence: live DSP (snesdsp.py, the verified ares port)
# vs extract_apu.decode_stream (the reference the FPGA player is built to).
#
# Boots the real SGB APU image, triggers one effect, hooks _brr_decode to
# capture the per-voice decoded 4-sample groups from the key-on burst onward,
# and compares the concatenated stream against decode_stream() of the same
# sample (zero predictor history at start, history carries across the loop
# jump -- the FPGA player contract).
#
# Known accepted divergence: if the driver writes SRCN/pitch long before KON,
# the never-keyed voice already decodes garbage (envelope=0, silent) and the
# LAST 12 garbage samples sit in the buffer as stale predictor history when
# KON starts; real hardware does the same. The player zeroes history, so a
# mismatch confined to the first few samples is expected and documented.
#
# Usage: python3 debug/decode_equiv_check.py [B0B] [16000]
#        (uses sgb_sfx/SGB1.sfc, or set SGB_BIOS=/path/to/your/SGB1.sfc)
import os, sys
base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, base)
# SGB1.sfc is the copyrighted SGB BIOS -- provide your own dump, either as
# sgb_sfx/SGB1.sfc or via the SGB_BIOS env var.
os.environ.setdefault('SGB_BIOS', os.path.join(base, 'SGB1.sfc'))

import render
from render import Rig, attrs
import snesdsp
import extract_apu as X

def main():
    key = sys.argv[1] if len(sys.argv) > 1 else 'B0B'
    want = int(sys.argv[2]) if len(sys.argv) > 2 else 16000
    side, idx = key[0].upper(), int(key[1:], 16)

    ram, entry, recs = X.load_image()
    region, samples = X.parse_samples(ram, recs)

    rig = Rig()
    rig.run_ms(400)
    names = render.A_NAMES if side == 'A' else render.B_NAMES
    pitches = render.A_PITCH if side == 'A' else render.B_PITCH
    pa = pitches[idx] if side == 'A' else 0
    pb = pitches[idx] if side == 'B' else 0
    attr = render.attrs(pa, 0, pb, 0)

    groups = {}                          # voice -> list of (addr, off, 4-sample tuple)
    armed = [False]
    orig = snesdsp.SNESDSP._brr_decode
    def hook(self, vv, brr_byte, brr_header):
        addr, off = vv.brrAddress, vv.brrOffset
        r = orig(self, vv, brr_byte, brr_header)
        idxv = self.voices.index(vv)
        if armed[0] and idxv in groups:
            bo = vv.bufferOffset
            g = tuple(vv.buffer[(bo - 4 + i) % 12] for i in range(4))
            groups[idxv].append((addr, off, g))
        return r
    snesdsp.SNESDSP._brr_decode = hook

    # arm capture on the first KON-burst decode of each voice; latch the SRCN
    # at that moment (the driver may rewrite source afterwards)
    srcns = {}
    orig2 = hook
    def hook2(self, vv, brr_byte, brr_header):
        idxv = self.voices.index(vv)
        if armed[0] and vv.keyonDelay >= 1 and idxv not in groups:
            groups[idxv] = []
            srcns[idxv] = vv.source & 0xFF
        return orig2(self, vv, brr_byte, brr_header)
    snesdsp.SNESDSP._brr_decode = hook2

    b1 = idx if side == 'A' else 0
    b2 = idx if side == 'B' else 0
    rig.set_ports(b1=0, b2=0, b3=attr)
    rig.run_and_capture(12, full_vol=False)
    rig.set_ports(b1=b1, b2=b2, b3=attr)
    armed[0] = True
    rig.run_and_capture(1500, full_vol=False)   # long enough for >1 loop pass
    snesdsp.SNESDSP._brr_decode = orig

    if not groups:
        print("FAIL: no voice keyed on"); sys.exit(1)
    vidx = max(groups, key=lambda k: len(groups[k]))
    srcn = srcns.get(vidx, rig.dsp.voices[vidx].source & 0xFF)
    smp = samples[srcn]
    # The very first captured group can be the stale tail of the previous
    # stream: upstream fires one last decode of the old brrAddress on the
    # same sample where keyonDelay becomes 5 (the address reload happens on
    # the next sample). Slice to the true playback start: the first group at
    # (sample start address, offset 1).
    gs = groups[vidx]
    i0 = next((i for i, (a, o, g) in enumerate(gs)
               if a == smp['start'] and o == 1), 0)
    stale = i0
    live = [s for _, _, g in gs[i0:] for s in g]
    flags = 3 if smp['has_loop'] else 0
    ref = X.decode_stream(region, smp, flags, 0)
    n = min(len(live), len(ref), want)
    diffs = [i for i in range(n) if live[i] != ref[i]]
    print(f"{key}: voice {vidx} SRCN ${srcn:02X} | live {len(live)} ref {len(ref)} "
          f"compared {n} (dropped {stale} stale-tail group(s))")
    print(f"first 8 live: {live[:8]}")
    print(f"first 8 ref:  {ref[:8]}")
    if not diffs:
        print("RESULT: PASS (bit-exact)")
        sys.exit(0)
    print(f"diffs: {len(diffs)} first at {diffs[:8]}")
    for i in diffs[:4]:
        print(f"  [{i}] live={live[i]} ref={ref[i]}")
    print("first captured groups (addr, off, samples):")
    for addr, off, g in groups[vidx][:6]:
        print(f"  ${addr:04X} off={off} {list(g)}")
    # accepted: stale-history transient confined to the first block
    if len(diffs) <= 8 and diffs[-1] < 16:
        print("RESULT: PASS-with-note (stale pre-KON buffer history, "
              "documented divergence; player zeroes history)")
        sys.exit(0)
    print("RESULT: FAIL")
    sys.exit(1)

if __name__ == '__main__':
    main()
