# Encode rendered SFX WAVs to SNES BRR blocks for the FPGA playback engine.
# Greedy per-block encoder: tries all (shift, filter) combos with predictive
# re-synthesis and keeps the minimum-error choice. Output: raw .brr stream
# (9-byte blocks, end/loop flags) + a JSON manifest.
import wave, struct, json, sys, os

def load_wav_mono(path, target_rate=None):
    w = wave.open(path)
    n = w.getnframes(); ch = w.getnchannels(); sw = w.getsampwidth(); rate = w.getframerate()
    raw = w.readframes(n)
    assert sw == 2
    smp = struct.unpack('<%dh' % (n*ch), raw)
    if ch == 2:
        mono = [(smp[i] + smp[i+1]) >> 1 for i in range(0, len(smp), 2)]
    else:
        mono = list(smp)
    if target_rate and target_rate != rate:
        # naive linear resample
        ratio = rate / target_rate
        out = []
        pos = 0.0
        while int(pos) < len(mono) - 1:
            i = int(pos); f = pos - i
            out.append(int(mono[i]*(1-f) + mono[i+1]*f))
            pos += ratio
        mono = out
    w.close()
    return mono

def clamp16(v):
    return max(-32768, min(32767, v))

def try_encode_block(s16, shift, filt, p1, p2):
    """encode 16 samples; return (nibbles bytes, decoded samples, error, new p1,p2)"""
    out = []
    err = 0
    s1, s2 = p1, p2
    data = bytearray(8)
    for i in range(16):
        s = s16[i]
        # prediction
        if filt == 1:
            pred = (s1 >> 1) + ((-s1) >> 5)
        elif filt == 2:
            pred = s1 + ((-(s1 + (s1 >> 1))) >> 5) - (s2 >> 1) + (s2 >> 5)
        elif filt == 3:
            pred = s1 + ((-(s1 + (s1<<1) + (s1<<2) + (s1<<3))) >> 6) \
                   - (s2 >> 1) + ((s2 + (s2 >> 1)) >> 4)
        else:
            pred = 0
        r = s - pred
        # nibble = round(r / 2^shift)
        if shift == 0:
            n = r
        else:
            n = (r + (1 << (shift-1))) >> shift
        n = max(-8, min(7, n))
        # decode back
        ds = (n << shift) + pred
        ds = clamp16(ds)
        d = ds - s
        err += d*d
        out.append(n)
        s2, s1 = s1, ds
    for i in range(8):
        data[i] = ((out[2*i] & 0xF) << 4) | (out[2*i+1] & 0xF)
    return bytes(data), err, s1, s2

def encode_brr(samples, loop=False, rate_note=None):
    blocks = []
    p1 = p2 = 0
    n = len(samples)
    pos = 0
    blk_idx = 0
    while pos < n:
        chunk = samples[pos:pos+16]
        if len(chunk) < 16:
            chunk = chunk + [0]*(16-len(chunk))
        best = None
        for filt in range(4):
            for shift in range(13):
                data, err, np1, np2 = try_encode_block(chunk, shift, filt, p1, p2)
                if best is None or err < best[0]:
                    best = (err, shift, filt, data, np1, np2)
        err, shift, filt, data, p1, p2 = best
        pos += 16
        blk_idx += 1
        blocks.append((shift, filt, data, pos >= n))
    out = bytearray()
    nblk = len(blocks)
    for i, (shift, filt, data, is_last) in enumerate(blocks):
        end = 1 if is_last else 0
        lp = 1 if (loop and not is_last) or (loop and is_last) else 0
        # loop flag set on all blocks when looping; end flag only on last
        hdr = (shift << 4) | (filt << 2) | (lp << 1) | end
        out.append(hdr)
        out += data
    return bytes(out)

def main():
    # Defaults live inside this folder so the tree is self-contained:
    # encode the rendered WAVs in ./wav into ./brr (legacy reference flow;
    # the ROM-native bank in build/ takes the original BRR bytes verbatim
    # instead of re-encoding).
    _here = os.path.dirname(os.path.abspath(__file__))
    default_src = os.path.join(_here, 'wav')
    default_dst = os.path.join(_here, 'brr')
    src = sys.argv[1] if len(sys.argv) > 1 else default_src
    dst = sys.argv[2] if len(sys.argv) > 2 else default_dst
    rate = int(sys.argv[3]) if len(sys.argv) > 3 else 16000
    os.makedirs(dst, exist_ok=True)
    manifest = {}
    for fn in sorted(os.listdir(src)):
        if not fn.endswith('.wav'): continue
        mono = load_wav_mono(os.path.join(src, fn), rate)
        if max(abs(s) for s in mono) < 64:
            print(f"skip (silent): {fn}")
            continue
        loop = fn[0] == 'B'
        brr = encode_brr(mono, loop=loop)
        out_fn = fn[:-4] + '.brr'
        open(os.path.join(dst, out_fn), 'wb').write(brr)
        manifest[out_fn] = {
            'samples': len(mono), 'rate': rate, 'brr_bytes': len(brr),
            'loop': loop, 'duration_ms': int(len(mono)*1000/rate)
        }
        print(f"{fn} -> {out_fn}: {len(mono)} smp @ {rate} -> {len(brr)} B ({len(brr)*2//1024} kB/10)")
    json.dump(manifest, open(os.path.join(dst, 'manifest.json'), 'w'), indent=1)
    tot = sum(m['brr_bytes'] for m in manifest.values())
    print(f"TOTAL: {len(manifest)} effects, {tot} bytes ({tot/1024:.1f} KB)")

if __name__ == '__main__':
    main()
