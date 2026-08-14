#!/usr/bin/env python3
# Build the SGB built-in SFX bank for FPGA playback.
#
# Reads the rendered WAVs in ./wav (see render.py / README.md), downmixes to
# mono, resamples to the playback rate, detects loop regions for the looping
# B-side effects, BRR-encodes each, and packs everything into a single bank
# file plus an index table for the FPGA playback engine.
#
# Bank layout (sgb_sfx_bank.bin):
#   [ BRR streams, each 16-byte aligned ][ index table ]
# Index table record (16 bytes, little-endian), one per effect, in A01..A30,
# B01..B19 order (use the JSON for the idx -> record mapping):
#   u32 offset        byte offset of the BRR stream within the bank
#   u32 length        BRR byte count (multiple of 9)
#   u32 loop_offset   byte offset (within the stream) of the loop-start block,
#                     0xFFFFFFFF = one-shot (no loop)
#   u32 sample_count  decoded sample count (informational)
#
# Usage:
#   python3 build_bank.py [--rate 16000] [--wav ./wav] [--out ./bank]

import wave, struct, json, os, argparse

# Sustained-loop SFX-B effects whose rendered capture holds only one pass.
# Force a full-track loop (loop point at byte 0).
FORCE_FULL_LOOP = {'b0f', 'b10', 'b19'}

def clamp16(v): return max(-32768, min(32767, v))

def dec_sample(n, shift, filt, p1, p2):
    """Exact SNES-DSP (ares) decode of one nibble; p1/p2 are the previous two
    FULL-SCALE outputs. The encoder MUST re-synthesize with this or the bank
    decodes to something unlike the source (the old half-scale predictor
    formulas measured 2x too little feedback and decorrelated the output).
    n may be signed (-8..7) or an unsigned 4-bit nibble (0..15)."""
    n &= 0xF
    t = n - 16 if n & 8 else n
    t = (t << shift) >> 1 if shift <= 12 else (t & ~0x7FF)
    p2h = p2 >> 1
    if filt==1:   t += (p1>>1)+((-p1)>>5)
    elif filt==2: t += p1-p2h+(p2h>>4)+((p1*-3)>>6)
    elif filt==3: t += p1-p2h+((p1*-13)>>7)+((p2h*3)>>4)
    t = clamp16(t)
    t = (t << 1) & 0xFFFF
    return t - 0x10000 if t >= 0x8000 else t

def _try_block(s16, shift, filt, p1, p2):
    out=[]; err=0; s1,s2=p1,p2; data=bytearray(8)
    for i in range(16):
        s=s16[i]
        pred=dec_sample(0, shift, filt, s1, s2)   # history-only contribution
        r=s-pred
        n=r if shift==0 else (r+(1<<(shift-1)))>>shift
        n=max(-8,min(7,n))
        ds=dec_sample(n, shift, filt, s1, s2)
        d=ds-s; err+=d*d; out.append(n); s2,s1=s1,ds
    for i in range(8): data[i]=((out[2*i]&0xF)<<4)|(out[2*i+1]&0xF)
    return bytes(data), err, s1, s2

def encode_brr(samples, loop_start_smp=None):
    """BRR-encode samples. If loop_start_smp is set, blocks from that point are
    flagged as the loop region and the final block is end+loop."""
    blocks=[]; p1=p2=0; pos=0
    loop_blk=(loop_start_smp//16) if loop_start_smp is not None else None
    while pos < len(samples):
        chunk=samples[pos:pos+16]
        if len(chunk)<16: chunk=chunk+[0]*(16-len(chunk))
        best=None
        for filt in range(4):
            for shift in range(13):
                d,e,np1,np2=_try_block(chunk,shift,filt,p1,p2)
                if best is None or e<best[0]: best=(e,shift,filt,d,np1,np2)
        _,shift,filt,data,p1,p2=best
        blocks.append((shift,filt,data)); pos+=16
    out=bytearray(); n=len(blocks)
    for i,(shift,filt,data) in enumerate(blocks):
        is_last=(i==n-1); end=1 if is_last else 0
        lp=1 if (loop_blk is not None and i>=loop_blk) else 0
        if loop_blk is not None and is_last: end,lp=1,1
        out.append((shift<<4)|(filt<<2)|(lp<<1)|end); out+=data
    return bytes(out)

def load_wav_mono(path, target_rate):
    w=wave.open(path); n=w.getnframes(); ch=w.getnchannels(); rate=w.getframerate()
    smp=struct.unpack('<%dh'%(n*ch), w.readframes(n)); w.close()
    mono=[(smp[i]+smp[i+1])>>1 for i in range(0,len(smp),2)] if ch==2 else list(smp)
    if target_rate!=rate:
        ratio=rate/target_rate; out=[]; p=0.0
        while int(p)<len(mono)-1:
            i=int(p); f=p-i; out.append(int(mono[i]*(1-f)+mono[i+1]*f)); p+=ratio
        mono=out
    return mono

def detect_loop(mono, rate):
    """Return (loop_start, loop_len, keep) in samples, or None. Finds the
    steady-state onset, then the dominant period by autocorrelation, and keeps
    attack + one loop period."""
    n=len(mono)
    if n < rate: return None
    blk=16; nb=n//blk
    rms=[0]*nb
    for b in range(nb):
        seg=mono[b*blk:(b+1)*blk]
        rms[b]=int((sum(x*x for x in seg)/blk)**0.5)
    tail=rms[nb//2:]; tmean=sum(tail)/len(tail)
    if tmean<4: return None
    start_blk=0
    for b in range(nb):
        if all(abs(x-tmean)<=0.35*tmean+8 for x in rms[b:min(b+8,nb)]):
            start_blk=b; break
    s0=start_blk*blk
    seg=mono[s0:n]
    if len(seg)<rate//2: return None
    mean=sum(seg)/len(seg); seg=[x-mean for x in seg]
    maxlag=min(len(seg)//2, rate*2)
    def corr(lag):
        acc=0; nn=len(seg)-lag
        for i in range(0,nn,4): acc+=seg[i]*seg[i+lag]
        return acc
    cands=[]; lag=rate//50
    while lag<maxlag: cands.append((corr(lag),lag)); lag+=32
    if not cands: return None
    cands.sort(reverse=True); _,bl=cands[0]
    bestv,bestl=corr(bl),bl
    for lag in range(max(1,bl-16),bl+17):
        v=corr(lag)
        if v>bestv: bestv,bestl=v,lag
    loop_start=s0; loop_len=bestl; keep=loop_start+loop_len
    if keep>n: keep=n; loop_len=keep-loop_start
    if loop_len<rate//8: return None
    return loop_start, loop_len, keep

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--rate', type=int, default=16000)
    base=os.path.dirname(os.path.abspath(__file__))
    ap.add_argument('--wav', default=os.path.join(base,'wav'))
    ap.add_argument('--out', default=os.path.join(base,'bank'))
    ap.add_argument('--only', default='', help='comma-separated effect stems to include, e.g. A0C,B04')
    a=ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    only=set(s.strip().lower() for s in a.only.split(',') if s.strip())
    ALIGN=16
    bank=bytearray(); table=[]; man={}
    for fn in sorted(os.listdir(a.wav)):
        if not fn.endswith('.wav') or fn[0] not in 'AB': continue
        stem=fn[:3].lower()
        if only and stem not in only: continue
        idx=int(fn[1:3],16); side=fn[0]
        mono=load_wav_mono(os.path.join(a.wav,fn), a.rate)
        if max(abs(x) for x in mono)<64:
            print(f"skip silent {fn}"); continue
        loop_start=None; keep=len(mono)
        if side=='B':
            det=detect_loop(mono,a.rate)
            if det: loop_start,loop_len,keep=det
            if stem in FORCE_FULL_LOOP: loop_start=0; keep=len(mono)
        enc=encode_brr(mono[:keep], loop_start_smp=loop_start)
        pad=(ALIGN-(len(bank)%ALIGN))%ALIGN
        bank+=b'\x00'*pad
        off=len(bank)
        bank+=enc
        loop_off=(loop_start//16)*9 if loop_start is not None else 0xFFFFFFFF
        table.append((fn,idx,side,off,len(enc),loop_off,keep))
        man[fn]={'record':len(table)-1,'offset':off,'bytes':len(enc),
                 'loop_offset':loop_off if loop_off!=0xFFFFFFFF else None,
                 'samples':keep,'rate':a.rate}
        ls=f"loop@{loop_off}" if loop_off!=0xFFFFFFFF else "one-shot"
        print(f"{fn}: {keep} smp@{a.rate} -> {len(enc)}B {ls}")
    # index table (16-byte records)
    pad=(ALIGN-(len(bank)%ALIGN))%ALIGN
    bank+=b'\x00'*pad
    tbl_off=len(bank)
    for (fn,idx,side,off,ln,loop_off,keep) in table:
        bank+=struct.pack('<IIII', off, ln, loop_off, keep)
    open(os.path.join(a.out,'sgb_sfx_bank.bin'),'wb').write(bytes(bank))
    hdr=(f"// Auto-generated by build_bank.py. SGB built-in SFX bank index table.\n"
         f"// Bank file: sgb_sfx_bank.bin. Record = 16 bytes: "
         f"{{u32 offset,u32 length,u32 loop_offset,u32 sample_count}}.\n"
         f"// loop_offset = byte offset of loop-start block within the stream, "
         f"0xFFFFFFFF = one-shot.\n"
         f"`define SGB_SFX_COUNT {len(table)}\n"
         f"`define SGB_SFX_TABLE_OFF {tbl_off}\n"
         f"`define SGB_SFX_BANK_BYTES {len(bank)}\n")
    for i,(fn,idx,side,off,ln,loop_off,keep) in enumerate(table):
        hdr+=f"// record {i}: {fn} offset={off} bytes={ln} loop={loop_off:#x} samples={keep}\n"
    open(os.path.join(a.out,'sgb_sfx_bank.vh'),'w').write(hdr)
    json.dump(man, open(os.path.join(a.out,'sgb_sfx_bank.json'),'w'), indent=1)
    tot=sum(t[4] for t in table)
    print(f"\nTOTAL {len(table)} effects | bank {len(bank)} B ({len(bank)/1024:.0f} KB) "
          f"| BRR {tot} B ({tot/1024:.0f} KB) | table @{tbl_off}")

if __name__=='__main__':
    main()
