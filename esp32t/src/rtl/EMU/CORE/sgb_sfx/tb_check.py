#!/usr/bin/env python3
# tb_check.py -- verify the sgb_sfx_play (PCM) testbench captures.
#
# The bank is UNCOMPRESSED 16-bit mono PCM, so the reference is simply the
# bank's own PCM bytes. For each effect we expand the bank stream (following
# the loop point for looping effects) and compare SAMPLE-EXACTLY against the
# testbench capture. Any byte-alignment / word-order bug in the RTL breaks
# this immediately. We also correlate against the rendered source WAV.
#
# Usage: run the testbench first (it writes bank_pcm/tb_fx{...}.raw), then
#   python3 tb_check.py
import struct, wave, os, math, json

HERE = os.path.dirname(os.path.abspath(__file__))
BANK = os.path.join(HERE, 'bank_pcm')
WAV  = os.path.join(HERE, 'wav')

def load_bank():
    bank = open(os.path.join(BANK, 'sgb_sfx_bank.bin'), 'rb').read()
    man = json.load(open(os.path.join(BANK, 'sgb_sfx_bank.json')))
    # order records by 'record' and build (stem, offset, bytes, loop) list
    recs = sorted(man.items(), key=lambda kv: kv[1]['record'])
    table = []
    for stem, v in recs:
        table.append((stem[:3], v['offset'], v['bytes'], v['loop_offset']))
    return bank, table

def expand(bank, off, nbytes, loop_off, count):
    """Return the first `count` samples of the effect as a list of ints,
    following the loop point once the first pass is exhausted."""
    base = off
    # samples are 16-bit little-endian
    def s16(i):  # sample index -> int
        return struct.unpack_from('<h', bank, base + i*2)[0]
    total = nbytes // 2
    loop_smp = (loop_off // 2) if loop_off is not None else None
    out = []
    i = 0
    while len(out) < count:
        if i >= total:
            if loop_smp is None:
                break
            i = loop_smp
            continue
        out.append(s16(i))
        i += 1
    return out

def load_wav_mono_16k(stem):
    for fn in os.listdir(WAV):
        if fn.startswith(stem) and fn.endswith('.wav'):
            w = wave.open(os.path.join(WAV, fn))
            n = w.getnframes(); ch = w.getnchannels(); rate = w.getframerate()
            smp = struct.unpack('<%dh' % (n*ch), w.readframes(n)); w.close()
            mono = [(smp[i]+smp[i+1]) >> 1 for i in range(0, len(smp), 2)] if ch == 2 else list(smp)
            if rate != 16000:
                ratio = rate/16000; o=[]; p=0.0
                while int(p) < len(mono)-1:
                    k=int(p); f=p-k; o.append(int(mono[k]*(1-f)+mono[k+1]*f)); p+=ratio
                mono=o
            return mono
    return None

def corr(a, b):
    n = min(len(a), len(b))
    if n == 0: return 0.0
    a=a[:n]; b=b[:n]
    ma=sum(a)/n; mb=sum(b)/n
    num=sum((x-ma)*(y-mb) for x,y in zip(a,b))
    da=math.sqrt(sum((x-ma)**2 for x in a)); db=math.sqrt(sum((y-mb)**2 for y in b))
    return num/(da*db) if da>0 and db>0 else 0.0

# testbench capture files: tb_fx{idx}.raw for idx 0..7, retrigger R1/R2
def main():
    bank, table = load_bank()
    fails = 0
    # map testbench capture files to table entries by index order
    for idx, (stem, off, nbytes, loop_off) in enumerate(table):
        raw = os.path.join(BANK, 'tb_fx%d.raw' % idx)
        if not os.path.exists(raw):
            continue
        cap = list(struct.unpack('<%dh' % (os.path.getsize(raw)//2), open(raw,'rb').read()))
        ref = expand(bank, off, nbytes, loop_off, len(cap))
        n = min(len(cap), len(ref))
        diffs = [(i, cap[i], ref[i]) for i in range(n) if cap[i] != ref[i]]
        exact = (not diffs) and n == len(cap)
        tag = 'EXACT ' if exact else 'MISMATCH'
        print('fx%d %-16s %-8s %6d samples' % (idx, stem, tag, n))
        if not exact:
            fails += 1
            print('    cap=%d ref=%d diffs=%d' % (len(cap), len(ref), len(diffs)))
            for i,c,r in diffs[:8]:
                print('    [%6d] cap=%6d ref=%6d' % (i,c,r))
        # WAV correlation sanity (against the rendered source). Compare only
        # the first pass: for looping effects the capture continues into the
        # loop region, which the WAV render does not follow sample-for-sample.
        mono = load_wav_mono_16k(stem)
        if mono:
            fp = nbytes // 2
            c = corr(cap[:fp], mono)
            print('       corr(rtl,wav firstpass)=%.4f' % c)
            if c < 0.90:
                fails += 1
                print('       FAIL: rtl-vs-wav correlation too low')
    print('RESULT:', 'FAIL (%d)' % fails if fails else 'PASS')

if __name__ == '__main__':
    main()
