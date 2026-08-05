#!/usr/bin/env python3
# tb_check_snd.py -- verify the sgb_snd (DKGB Pauline-help scream) capture.
#
# Three-way check:
#   1. RTL capture (tb_sgb_snd.raw) vs a software BRR decode that mirrors the
#      RTL shift-add expressions EXACTLY (all four filters). Must be
#      sample-exact.
#   2. Same reference vs the rendered WAV from DKGBDisasm (11025 Hz mono).
#   3. Report what the OLD bug (f2/f3 decoded as f0) did, so the audible
#      improvement is quantified.
#
# Usage: run the testbench first (it writes tb_sgb_snd.raw), then
#   python3 tb_check_snd.py
import struct, wave, os, math

HERE = os.path.dirname(os.path.abspath(__file__))
BRR  = os.path.join(HERE, 'sgb_pauline_help_scream.brr')
WAV  = os.path.join(HERE, 'sgb_pauline_help_scream.wav')
RAW  = os.path.join(HERE, 'tb_sgb_snd.raw')

def sar(v, n):   # arithmetic shift right on ints (Python >> is arithmetic)
    return v >> n

def decode(brr, f2f3_as_f0=False, lo_first=False):
    """Bit-exact mirror of sgb_snd.v's datapath: HIGH-nibble-first and the
    bsnes SPC DSP filter arithmetic (20-bit adders, 16-bit saturation)."""
    out = []
    prev1 = prev2 = 0
    for blk in range(len(brr) // 9):
        h = brr[blk*9]
        shift = h >> 4
        filt  = (h >> 2) & 3
        end   = h & 1
        if f2f3_as_f0 and filt >= 2:
            filt = 0
        for i in range(16):
            byte = brr[blk*9 + 1 + (i >> 1)]
            nib  = (byte & 0xF) if ((i & 1) ^ lo_first) else (byte >> 4)
            snib = nib - 16 if nib >= 8 else nib
            shifted = snib << shift
            p1, p2h = prev1, prev2 >> 1
            if   filt == 1:
                fcon = sar(p1,1) + sar(-p1,5)
            elif filt == 2:
                fcon = p1 - p2h + sar(p2h,4) + sar(-(p1+p1+p1),6)
            elif filt == 3:
                fcon = p1 - p2h + sar((p2h<<1)+p2h,4) + sar(-13*p1,7)
            else:
                fcon = 0
            acc = shifted + fcon
            v = 32767 if acc > 32767 else (-32768 if acc < -32768 else acc)
            out.append(v)
            prev2, prev1 = prev1, v
        if end:
            break
    return out

def corr(a, b):
    n = min(len(a), len(b))
    if n == 0: return 0.0
    a = a[:n]; b = b[:n]
    ma = sum(a)/n; mb = sum(b)/n
    num = sum((x-ma)*(y-mb) for x,y in zip(a,b))
    da = math.sqrt(sum((x-ma)**2 for x in a))
    db = math.sqrt(sum((y-mb)**2 for y in b))
    return num/(da*db) if da > 0 and db > 0 else 0.0

def main():
    brr = open(BRR, 'rb').read()
    ref = decode(brr)
    # what the old build produced: low-nibble-first AND f2/f3 decoded as f0
    old = decode(brr, f2f3_as_f0=True, lo_first=True)

    w = wave.open(WAV)
    wav = list(struct.unpack('<%dh' % w.getnframes(), w.readframes(w.getnframes())))
    w.close()

    fails = 0
    cap = list(struct.unpack('<%dh' % (os.path.getsize(RAW)//2), open(RAW,'rb').read()))

    print('capture: %d samples | reference decode: %d | wav: %d'
          % (len(cap), len(ref), len(wav)))

    # 1. RTL vs reference decode -- must be sample-exact
    n = min(len(cap), len(ref))
    diffs = [(i, cap[i], ref[i]) for i in range(n) if cap[i] != ref[i]]
    exact = not diffs and len(cap) == len(ref)
    print('rtl-vs-reference: %s' % ('EXACT' if exact else 'MISMATCH'))
    if not exact:
        fails += 1
        print('  cap=%d ref=%d diffs=%d' % (len(cap), len(ref), len(diffs)))
        for i, c, r in diffs[:8]:
            print('  [%5d] cap=%6d ref=%6d' % (i, c, r))

    # 2. reference vs rendered WAV -- informational. The WAV's renderer is of
    # unknown provenance (its envelope matches exactly but it is not
    # bit-exact against ANY hardware-faithful decode); the spec is the
    # hardware-verified bsnes semantics, which the reference implements.
    n = min(len(ref), len(wav))
    c = corr(ref, wav)
    print('reference-vs-wav: corr=%.4f (informational; envelope RMS ratio %.4f)'
          % (c, math.sqrt(sum(x*x for x in ref)/len(ref)) /
                math.sqrt(sum(x*x for x in wav)/len(wav))))
    if c < 0.60:
        fails += 1
        print('  FAIL: wav correlation implausibly low')

    # 3. quantify what the old build got wrong
    od = sum(1 for i in range(len(ref)) if ref[i] != old[i])
    err = [ref[i]-old[i] for i in range(len(ref))]
    rms = math.sqrt(sum(e*e for e in err)/len(err))
    print('old build (lo-first + f2/f3 as f0): %d/%d samples wrong, '
          'rms error %.0f (%.1f%% of full scale)'
          % (od, len(ref), rms, rms/32768*100))

    print('RESULT:', 'FAIL (%d)' % fails if fails else 'PASS')

if __name__ == '__main__':
    main()
