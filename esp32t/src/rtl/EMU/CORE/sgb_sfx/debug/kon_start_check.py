#!/usr/bin/env python3
# KON start-address check (the experiment that found the snesdsp.py port bug).
#
# Upstream ares (sfc/dsp/voice.cpp) selects the sample-directory word by key
# state:
#     n16 address = brr._address;
#     if(!v.keyonDelay) address += 2;     // running  -> 2nd word (loop ptr)
#     brr._nextAddress = word at address; // keying   -> 1st word (start addr)
# so a keyed-on voice begins decoding at the directory entry's FIRST word;
# the second word is only used when a block with the end+loop flags is
# reached. An earlier revision of snesdsp.py latched the second word
# unconditionally, so playback wrongly began at the loop pointer (for B0B:
# $D96F instead of $D94B).
#
# This script re-runs the experiment: it traces the first BRR decode address
# of the voice the SGB driver keys on for an effect and asserts it equals the
# FIRST directory word. Passes with the fixed snesdsp.py.
#
# Usage: python3 debug/kon_start_check.py        (uses sgb_sfx/SGB1.sfc,
#        or set SGB_BIOS=/path/to/your/SGB1.sfc)
import os, sys
base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, base)
# SGB1.sfc is the copyrighted SGB BIOS -- provide your own dump, either as
# sgb_sfx/SGB1.sfc or via the SGB_BIOS env var.
os.environ.setdefault('SGB_BIOS', os.path.join(base, 'SGB1.sfc'))

import render
from render import Rig, attrs
import snesdsp

def dir_entry(ram4b00, srcn):
    s = ram4b00[4*srcn] | ram4b00[4*srcn+1] << 8
    l = ram4b00[4*srcn+2] | ram4b00[4*srcn+3] << 8
    return s, l

def main():
    rig = Rig()
    rig.run_ms(400)                      # boot the SGB APU driver
    dsp = rig.dsp
    # directory record from the ROM image ($4B00, 256 B)
    dir_ram = dsp.ram[0x4B00:0x4C00]

    firsts = {}                          # voice -> first KON-burst decode addr
    armed = [False]
    orig_dec = snesdsp.SNESDSP._brr_decode
    def hook(self, vv, brr_byte, brr_header):
        # The SGB driver writes SRCN/pitch before KON, so a never-keyed voice
        # already "runs" (gaussianOffset accumulates) and silently decodes
        # garbage at its reset brrAddress ($0000) with envelope=0 — upstream
        # ares does the same. The real playback start is the first decode of
        # the key-on burst (keyonDelay still counting down, >= 1 here since
        # it is read after the decrement).
        if armed[0] and vv.keyonDelay >= 1:
            idx = self.voices.index(vv)
            if idx not in firsts:
                firsts[idx] = (vv.brrAddress, vv.source)
        return orig_dec(self, vv, brr_byte, brr_header)
    snesdsp.SNESDSP._brr_decode = hook

    # trigger B0B Wave (SFX-B, looped sample, SRCN $3D)
    rig.set_ports(b1=0, b2=0, b3=attrs(0, 0, 0x0B, 0))
    rig.run_and_capture(12, full_vol=False)   # retrigger clear
    rig.set_ports(b1=0, b2=0x0B, b3=attrs(0, 0, 0x0B, 0))
    armed[0] = True
    rig.run_and_capture(120, full_vol=False)
    snesdsp.SNESDSP._brr_decode = orig_dec

    fails = 0
    for idx, (addr, srcn) in sorted(firsts.items()):
        start, loop = dir_entry(dir_ram, srcn)
        ok = (addr == start)
        print(f"voice {idx}: SRCN ${srcn:02X} first decode ${addr:04X} | "
              f"dir start=${start:04X} loop=${loop:04X} -> "
              f"{'OK (starts at first word)' if ok else 'FAIL (not start word)'}")
        if not ok:
            fails += 1
    if not firsts:
        print("FAIL: no voice decoded after trigger")
        fails += 1
    print("RESULT:", "FAIL" if fails else "PASS")
    sys.exit(1 if fails else 0)

if __name__ == '__main__':
    main()
