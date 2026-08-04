#!/usr/bin/env python3
# tb_check.py -- verify the sgb_sfx_play testbench captures.
#
# For each of the 7 bank effects:
#   1. REFERENCE decode: software BRR decode of the bank7 bytes, using the
#      exact ares/SNES-DSP algorithm (same as snesdsp.py _brr_decode). The
#      RTL implements the same algorithm, so this must match the testbench
#      capture SAMPLE-EXACTLY (any byte-alignment or nibble-order bug in the
#      RTL breaks this immediately).
#   2. WAV sanity: correlate the first decoded pass against the rendered
#      WAV (mono, 16 kHz) the bank was encoded from. Lossy BRR, so this is
#      a correlation check, not exact.
#
# Usage: run the testbench first (it writes bank7/tb_fx{0..6}.raw), then
#   python3 tb_check.py
import struct, wave, os, math

HERE = os.path.dirname(os.path.abspath(__file__))
BANK = os.path.join(HERE, 'bank7')
WAV  = os.path.join(HERE, 'wav')

MANIFEST = [
    # (fx idx, wav stem, offset, bytes, loop_off)  -- mirrors bank7 json/RTL
    (0, 'A1F_SwordSwing',     0,     6381,  None),
    (1, 'A26_PictureFloats',  6384,  7641,  None),
    (2, 'A30_SmallLaser',     14032, 4941,  None),
    (3, 'B01_ApplauseSmall',  18976, 5175,  837),
    (4, 'B04_Wind',           24160, 10530, 4302),
    (5, 'B07_StormThunder',   34704, 4059,  1017),
    (6, 'B08_LightningB',     38768, 23796, 22482),
    (7, 'B0B_Wave',           62576, 25101, None),
]

def sclamp16(v):
    return -32768 if v < -32768 else (32767 if v > 32767 else v)

def decode_stream(brr, loop_off, count):
    """ares-exact BRR decode. p1/p2 are the last two *full-scale* outputs
    (the stored, post <<1 values); the filter sees p2 pre-halved."""
    out = []
    p1 = 0
    p2f = 0
    nb = len(brr) // 9
    bi = 0
    while len(out) < count:
        if bi >= nb:
            if loop_off is None:
                break
            bi = loop_off // 9
            continue
        h = brr[bi*9]
        shift = h >> 4
        filt = (h >> 2) & 3
        done = False
        for k in range(8):
            if done:
                break
            by = brr[bi*9 + 1 + k]
            for sh in (4, 0):                     # high nibble first
                n = (by >> sh) & 0xF
                s = n - 16 if n & 8 else n
                s = (s << shift) >> 1 if shift <= 12 else (s & ~0x7FF)
                p2h = p2f >> 1
                if filt == 1:
                    s += (p1 >> 1) + ((-p1) >> 5)
                elif filt == 2:
                    s += p1 - p2h + (p2h >> 4) + ((p1 * -3) >> 6)
                elif filt == 3:
                    s += p1 - p2h + ((p1 * -13) >> 7) + ((p2h * 3) >> 4)
                s = sclamp16(s)
                s = (s << 1) & 0xFFFF             # clamp-then-double, wrap
                if s >= 0x8000:
                    s -= 0x10000
                out.append(s)
                p2f, p1 = p1, s
                if len(out) >= count:
                    done = True
                    break
        bi += 1
    return out[:count]

def load_wav_mono_16k(path):
    w = wave.open(path)
    n = w.getnframes(); ch = w.getnchannels(); rate = w.getframerate()
    smp = struct.unpack('<%dh' % (n*ch), w.readframes(n))
    w.close()
    mono = [(smp[i] + smp[i+1]) >> 1 for i in range(0, len(smp), 2)] if ch == 2 \
           else list(smp)
    assert rate == 32000, path
    return mono[::2]                               # 32k -> 16k (build_bank decimation)

def corr(a, b):
    n = min(len(a), len(b))
    if n == 0:
        return 0.0
    a = a[:n]; b = b[:n]
    ma = sum(a)/n; mb = sum(b)/n
    num = sum((x-ma)*(y-mb) for x, y in zip(a, b))
    da = math.sqrt(sum((x-ma)**2 for x in a))
    db = math.sqrt(sum((y-mb)**2 for y in b))
    return num/(da*db) if da > 0 and db > 0 else 0.0

def main():
    bank = open(os.path.join(BANK, 'sgb_sfx_bank.bin'), 'rb').read()
    fails = 0
    for idx, stem, off, ln, loop in MANIFEST:
        raw_path = os.path.join(BANK, 'tb_fx%d.raw' % idx)
        if not os.path.exists(raw_path):
            print('fx%d %-20s SKIP (no %s)' % (idx, stem, raw_path))
            continue
        cap = list(struct.unpack('<%dh' % (os.path.getsize(raw_path)//2),
                                 open(raw_path, 'rb').read()))
        brr = bank[off:off+ln]
        ref = decode_stream(brr, loop, len(cap) if loop is not None else (ln//9)*16)

        # 1. sample-exact vs reference decode of the same bank bytes
        n = min(len(cap), len(ref))
        diffs = [(i, cap[i], ref[i]) for i in range(n) if cap[i] != ref[i]]
        exact = (n == len(cap) == len(ref)) and not diffs
        if loop is None and len(cap) != len(ref):
            exact = False
        if exact:
            print('fx%d %-20s EXACT  %6d samples' % (idx, stem, n))
        else:
            fails += 1
            print('fx%d %-20s MISMATCH cap=%d ref=%d diffs=%d' %
                  (idx, stem, len(cap), len(ref), len(diffs)))
            for i, c, r in diffs[:8]:
                print('    [%6d] cap=%6d ref=%6d' % (i, c, r))

        # 2. correlation vs the rendered WAV the bank came from
        wavp = None
        for fn in os.listdir(WAV):
            if fn.startswith(stem):
                wavp = os.path.join(WAV, fn)
                break
        if wavp:
            w = load_wav_mono_16k(wavp)
            firstpass = decode_stream(brr, None, (ln//9)*16)
            m = min(len(w), len(firstpass))
            c_enc = corr(firstpass[:m], w[:m])     # encoder quality
            c_rtl = corr(cap[:m], w[:m])           # full path
            print('       corr(enc,wav)=%.4f  corr(rtl,wav)=%.4f' % (c_enc, c_rtl))
            if c_rtl < 0.98:
                fails += 1
                print('       FAIL: rtl-vs-wav correlation too low')
    # interrupt tests: capture after the re-trigger must bit-match the new fx.
    # A few old-stream tail samples leak in before the flush lands (the
    # retrigger CDC latency), so find the alignment offset of the new stream
    # inside the capture, then require an exact match from there.
    for raw, src in (('tb_fxR1.raw', 2), ('tb_fxR2.raw', 1)):
        rawp = os.path.join(BANK, raw)
        if not os.path.exists(rawp):
            continue
        cap = list(struct.unpack('<%dh' % (os.path.getsize(rawp)//2),
                                 open(rawp, 'rb').read()))
        idx, stem, off, ln, loop = MANIFEST[src]
        ref = decode_stream(bank[off:off+ln], None, (ln//9)*16)
        # search small start offsets for the best exact-match alignment
        best_k, best_m = 0, -1
        for k in range(0, 65):
            m = 0
            for i in range(0, min(len(ref), len(cap)-k), 7):   # stride for speed
                if cap[k+i] == ref[i]:
                    m += 1
            if m > best_m:
                best_m, best_k = m, k
        k = best_k
        n = min(len(cap)-k, len(ref))
        diffs = [i for i in range(n) if cap[k+i] != ref[i]]
        if not diffs and n >= len(ref) - 4:
            print('%-14s after retrigger        EXACT  %6d samples (fx%d, align +%d)' %
                  (raw, n, src, k))
        else:
            fails += 1
            print('%-14s after retrigger        MISMATCH cap=%d ref=%d align=%d diffs=%d' %
                  (raw, len(cap), len(ref), k, len(diffs)))
            for i in diffs[:8]:
                print('    [%6d] cap=%6d ref=%6d' % (i, cap[k+i], ref[i]))
    print('RESULT:', 'FAIL (%d)' % fails if fails else 'PASS')

if __name__ == '__main__':
    main()
