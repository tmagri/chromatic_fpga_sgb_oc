# SNES DSP emulation -- faithful port of ares-emulator/ares/sfc/dsp
# (brr.cpp, gaussian.cpp, envelope.cpp, counter.cpp, voice.cpp, memory.cpp).
# Reference: https://github.com/ares-emulator/ares  (sfc/dsp)
GTBL = []
def _load_gtbl():
    if GTBL: return
    # Self-contained gauss table (extracted from SGB_MiSTer/rtl/DSP_PKG.vhd).
    try:
        from gauss_table import GAUSS
        assert len(GAUSS) == 512
        GTBL.extend(GAUSS)
        return
    except ImportError:
        pass
    import re
    txt = open('/Users/troymagri/Desktop/SGB_MiSTer/rtl/DSP_PKG.vhd').read()
    m = re.search(r'GTBL: GaussTbl_t := \((.*?)\);', txt, re.S)
    vals = re.findall(r'x"([0-9A-Fa-f]{3})"', m.group(1))
    assert len(vals) == 512, len(vals)
    GTBL.extend(int(v,16) for v in vals)

EM_RELEASE, EM_ATTACK, EM_DECAY, EM_SUSTAIN = 0,1,2,3

def sclamp16(v):
    return max(-32768, min(32767, v))
def s8(v):
    v &= 0xFF
    return v - 0x100 if v >= 0x80 else v

# ares counter.cpp
COUNTER_RATE = [0,2048,1536,1280,1024,768,640,512,384,320,256,192,160,128,96,80,
                64,48,40,32,24,20,16,12,10,8,6,5,4,3,2,1]
COUNTER_OFFSET = [0,0,1040,536,0,1040,536,0,1040,536,0,1040,536,0,1040,536,
                  0,1040,536,0,1040,536,0,1040,536,0,1040,536,0,1040,0,0]

class Voice:
    def __init__(self, idx):
        self.index = idx << 4
        self.buffer = [0]*12
        self.bufferOffset = 0
        self.gaussianOffset = 0
        self.keyonDelay = 0
        self.brrAddress = 0
        self.brrOffset = 1
        self.nextAddress = 0
        self.envelope = 0
        self._envelope = 0
        self.envelopeMode = EM_RELEASE
        self.envx = 0
        self.pitch = 0
        self.source = 0
        self.volume = [0,0]
        self.adsr0 = 0        # reg 0x05
        self.adsr1 = 0        # reg 0x06
        self.gain = 0         # reg 0x07
        self.keyon = False; self._keylatch = False; self._keyon = False
        self.keyoff = False; self._keyoff = False
        self.modulate = False; self._modulate = False
        self.noise = False; self._noise = False
        self.echo = False; self._echo = False
        self._end = False; self._looped = False

class SNESDSP:
    def __init__(self, ram):
        _load_gtbl()
        self.ram = ram
        self.regs = [0]*128
        self.voices = [Voice(i) for i in range(8)]
        self.cpu = None
        self.out_l = 0; self.out_r = 0
        # main volume / echo / noise state
        self.mvol = [0,0]; self.evol = [0,0]
        self.mute = False; self.mvol_reset = False
        self.noise_freq = 0; self.noise_lfsr = 0x4000
        self.echo_readonly=False; self.echo_page=0; self.echo_delay=0; self.echo_feedback=0
        self.echo_fir=[0]*8
        self.clock_counter = 0
        self.clock_sample = False
        self.brr_bank = 0
        self._brr_header = 0
        self.echo_buf = [0]*65536  # simplified echo ring
        self.echo_pos = 0
        self.fir_hist = [[0,0] for _ in range(8)]

    # ---- register interface (memory.cpp) ----
    def read(self, a):
        a &= 0x7F
        if a == 0x7C:
            v = 0
            for i,vv in enumerate(self.voices):
                if vv._end: v |= 1<<i
            return v
        if a & 0x0F == 0x08:
            return self.voices[a>>4].envx & 0x7F
        return self.regs[a]

    def write(self, a, v):
        a &= 0x7F; v &= 0xFF
        self.regs[a] = v
        if a == 0x0C: self.mvol[0]=v
        elif a == 0x1C: self.mvol[1]=v
        elif a == 0x2C: self.evol[0]=v
        elif a == 0x3C: self.evol[1]=v
        elif a == 0x4C:
            for n in range(8):
                self.voices[n].keyon = bool(v & (1<<n))
                self.voices[n]._keylatch = bool(v & (1<<n))
        elif a == 0x5C:
            for n in range(8): self.voices[n].keyoff = bool(v & (1<<n))
        elif a == 0x6C:
            self.noise_freq = v & 0x1F
            self.echo_readonly = bool(v & 0x20)
            self.mute = bool(v & 0x40)
            self.mvol_reset = bool(v & 0x80)
        elif a == 0x7C:
            for n in range(8): self.voices[n]._end=False
            self.regs[0x7C]=0
        elif a == 0x0D: self.echo_feedback=v
        elif a == 0x2D:
            for n in range(8): self.voices[n].modulate=bool(v&(1<<n))
            self.voices[0].modulate=False
        elif a == 0x3D:
            for n in range(8): self.voices[n].noise=bool(v&(1<<n))
        elif a == 0x4D:
            for n in range(8): self.voices[n].echo=bool(v&(1<<n))
        elif a == 0x5D: self.brr_bank=v
        elif a == 0x6D: self.echo_page=v
        elif a == 0x7D: self.echo_delay=v&0x0F
        n = (a>>4)&7
        lo4 = a & 0x0F
        if lo4==0x0: self.voices[n].volume[0]=v
        elif lo4==0x1: self.voices[n].volume[1]=v
        elif lo4==0x2: self.voices[n].pitch = (self.voices[n].pitch & ~0xFF) | v
        elif lo4==0x3: self.voices[n].pitch = (self.voices[n].pitch & 0xFF) | ((v&0x3F)<<8)
        elif lo4==0x4: self.voices[n].source=v
        elif lo4==0x5: self.voices[n].adsr0=v
        elif lo4==0x6: self.voices[n].adsr1=v
        elif lo4==0x7: self.voices[n].gain=v
        elif lo4==0xF: self.echo_fir[n]=v

    def vreg(self,i,r): return self.regs[(i<<4)|r]

    # ---- counter (counter.cpp) ----
    def _counter_tick(self):
        if self.clock_counter == 0: self.clock_counter = 30720
        self.clock_counter -= 1
    def _counter_poll(self, rate):
        if rate == 0: return False
        return (self.clock_counter + COUNTER_OFFSET[rate]) % COUNTER_RATE[rate] == 0

    # ---- dir entry ----
    def _dir(self, srcn):
        base = (self.brr_bank << 8) + (srcn << 2)
        ram=self.ram
        return (ram[base&0xFFFF] | (ram[(base+1)&0xFFFF]<<8)), \
               (ram[(base+2)&0xFFFF] | (ram[(base+3)&0xFFFF]<<8))

    # ---- brr decode (brr.cpp) ----
    def _brr_decode(self, vv, brr_byte, brr_header):
        ram = self.ram
        nybbles = ((brr_byte << 8) | ram[(vv.brrAddress + vv.brrOffset + 1) & 0xFFFF]) & 0xFFFF
        filt = (brr_header >> 2) & 3
        scale = (brr_header >> 4) & 0xF
        for _ in range(4):
            # top 4 bits = current nybble, sign-extended
            top = (nybbles >> 12) & 0xF
            s = top - 16 if top & 8 else top
            nybbles = (nybbles << 4) & 0xFFFF
            if scale <= 12:
                s = (s << scale) >> 1
            else:
                s = s & ~0x7FF
            off = vv.bufferOffset - 1
            if off < 0: off = 11
            p1 = vv.buffer[off]
            off -= 1
            if off < 0: off = 11
            p2 = vv.buffer[off] >> 1
            if filt == 1:
                s += (p1 >> 1) + ((-p1) >> 5)
            elif filt == 2:
                s += p1 - p2 + (p2 >> 4) + ((p1 * -3) >> 6)
            elif filt == 3:
                s += p1 - p2 + ((p1 * -13) >> 7) + ((p2 * 3) >> 4)
            s = sclamp16(s)
            s = (s << 1) & 0xFFFF
            if s >= 0x8000: s -= 0x10000
            vv.buffer[vv.bufferOffset] = s
            vv.bufferOffset += 1
            if vv.bufferOffset >= 12: vv.bufferOffset = 0

    # ---- gaussian (gaussian.cpp) ----
    def _gauss(self, vv):
        offset = (vv.gaussianOffset >> 4) & 0xFF
        g = GTBL
        base = (vv.bufferOffset + (vv.gaussianOffset >> 12)) % 12
        buf = vv.buffer
        o  = g[255-offset]*buf[base] >> 11
        base += 1
        if base>=12: base=0
        o += g[511-offset]*buf[base] >> 11
        base += 1
        if base>=12: base=0
        o += g[256+offset]*buf[base] >> 11
        o = sclamp16(o) & 0xFFFF
        if o>=0x8000: o-=0x10000
        base += 1
        if base>=12: base=0
        o += g[offset]*buf[base] >> 11
        return sclamp16(o) & ~1

    # ---- envelope (envelope.cpp) ----
    def _envelope_run(self, vv, latch_adsr0):
        env = vv.envelope
        if vv.envelopeMode == EM_RELEASE:
            env -= 0x8
            if env < 0: env = 0
            vv.envelope = env
            return
        envelopeData = vv.adsr1
        if latch_adsr0 & 0x80:   # ADSR
            if vv.envelopeMode >= EM_DECAY:
                env -= 1
                env -= env >> 8
                rate = envelopeData & 0x1F
                if vv.envelopeMode == EM_DECAY:
                    rate = ((latch_adsr0 >> 4) & 7) * 2 + 16
            else:  # attack
                rate = (latch_adsr0 & 0xF) * 2 + 1
                env += 0x20 if rate < 31 else 0x400
        else:  # GAIN
            envelopeData = vv.gain
            mode = envelopeData >> 5
            if mode < 4:
                env = envelopeData << 4
                rate = 31
            else:
                rate = envelopeData & 0x1F
                if mode == 4:
                    env -= 0x20
                elif mode < 6:
                    env -= 1
                    env -= env >> 8
                else:
                    env += 0x20
                    if mode > 6 and (vv._envelope & 0xFFFF) >= 0x600:
                        env += 0x8 - 0x20
        if (env >> 8) == (envelopeData >> 5) and vv.envelopeMode == EM_DECAY:
            vv.envelopeMode = EM_SUSTAIN
        vv._envelope = env
        if env > 0x7FF or env < 0:
            env = 0 if env < 0 else 0x7FF
            if vv.envelopeMode == EM_ATTACK:
                vv.envelopeMode = EM_DECAY
        if self._counter_poll(rate):
            vv.envelope = env

    # ---- per sample (dsp.cpp main + voice.cpp) ----
    def sample(self):
        # misc29/30
        self.clock_sample = not self.clock_sample
        if self.clock_sample:
            for v in self.voices:
                v._keylatch = v._keylatch and not v._keyon
        if self.clock_sample:
            for v in self.voices:
                v._keyon = v._keylatch
                v._keyoff = v.keyoff
        self._counter_tick()
        if self._counter_poll(self.noise_freq):
            lfsr = self.noise_lfsr
            feedback = ((lfsr << 13) ^ (lfsr << 14)) & 0xFFFF
            self.noise_lfsr = (feedback & 0x4000) | (lfsr >> 1)

        mix = [0,0]; echo = [0,0]
        for v in self.voices:
            # voice1/voice2: source dir, latch adsr0 + pitch
            brr_src_addr, brr_next = self._dir(v.source)
            v.nextAddress = brr_next
            latch_adsr0 = v.adsr0
            pitch = v.pitch & 0x3FFF
            # current brr byte + header (voice3b)
            brr_byte = self.ram[(v.brrAddress + v.brrOffset) & 0xFFFF]
            brr_header = self.ram[v.brrAddress & 0xFFFF]
            # pitch modulation
            # (skip: needs previous voice output latch; PMON rarely used)
            # KON handling (voice3c)
            if v.keyonDelay:
                if v.keyonDelay == 5:
                    v.brrAddress = v.nextAddress
                    v.brrOffset = 1
                    v.bufferOffset = 0
                    brr_header = 0
                v.envelope = 0; v._envelope = 0
                v.gaussianOffset = 0
                v.keyonDelay -= 1
                if v.keyonDelay & 3: v.gaussianOffset = 0x4000
                pitch = 0
            # gaussian interpolation
            output = self._gauss(v)
            if v._noise:
                output = (self.noise_lfsr << 1) & 0xFFFF
                if output >= 0x8000: output -= 0x10000
            latch_output = (output * v.envelope >> 11) & ~1
            latch_output = sclamp16(latch_output)
            v.envx = v.envelope >> 4
            # immediate silence: end-of-sample (end, not loop) or reset
            if self.mvol_reset or (brr_header & 0x03) == 1:
                v.envelopeMode = EM_RELEASE
                v.envelope = 0
            if self.clock_sample:
                if v._keyoff: v.envelopeMode = EM_RELEASE
                if v._keyon:
                    v.keyonDelay = 5
                    v.envelopeMode = EM_ATTACK
            if not v.keyonDelay:
                self._envelope_run(v, latch_adsr0)
            # voice4: decode + advance pitch
            v._looped = False
            if v.gaussianOffset >= 0x4000:
                self._brr_decode(v, brr_byte, brr_header)
                v.brrOffset += 2
                if v.brrOffset >= 9:
                    v.brrAddress = (v.brrAddress + 9) & 0xFFFF
                    if brr_header & 1:
                        v.brrAddress = v.nextAddress
                        v._looped = True
                    v.brrOffset = 1
            v.gaussianOffset = (v.gaussianOffset & 0x3FFF) + pitch
            if v.gaussianOffset > 0x7FFF: v.gaussianOffset = 0x7FFF
            # voice output L/R (voice4/voice5)
            amp_l = (latch_output * s8(v.volume[0])) >> 7
            mix[0] = sclamp16(mix[0] + amp_l)
            if v._echo: echo[0] = sclamp16(echo[0] + amp_l)
            amp_r = (latch_output * s8(v.volume[1])) >> 7
            mix[1] = sclamp16(mix[1] + amp_r)
            if v._echo: echo[1] = sclamp16(echo[1] + amp_r)
            # ENDX
            v._end = v._end or v._looped
            if v.keyonDelay == 5: v._end = False

        # echo FIR (simplified) -- read echo ring, apply FIR, add with EVOL
        fir_out = [0,0]
        if self.echo_delay:
            # push current echo input into history
            self.fir_hist.append([echo[0], echo[1]])
            self.fir_hist = self.fir_hist[-8:]
            for k in range(8):
                c = s8(self.echo_fir[k])
                fir_out[0] += (self.fir_hist[k][0] * c) >> 6
                fir_out[1] += (self.fir_hist[k][1] * c) >> 6
            fir_out[0]=sclamp16(fir_out[0]); fir_out[1]=sclamp16(fir_out[1])
        out_l = sclamp16(((mix[0]*s8(self.mvol[0]))>>7) + ((fir_out[0]*s8(self.evol[0]))>>7))
        out_r = sclamp16(((mix[1]*s8(self.mvol[1]))>>7) + ((fir_out[1]*s8(self.evol[1]))>>7))
        if self.mute: out_l = out_r = 0
        self.out_l = out_l; self.out_r = out_r
        return out_l, out_r
