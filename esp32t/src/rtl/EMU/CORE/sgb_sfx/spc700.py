# Functional SPC700 CPU core (cycle-counted, no IPL ROM).
# Semantics per https://wiki.superfamicom.org/spc700-reference

MASKS12 = [0xFFF,0x7FF,0x7FF,0x7FF,0x3FF,0x3FF,0x3FF,0x1FF,0x1FF,0x1FF,
           0x0FF,0x0FF,0x0FF,0x07F,0x07F,0x07F,0x03F,0x03F,0x03F,0x01F,
           0x01F,0x01F,0x00F,0x00F,0x00F,0x007,0x007,0x007,0x003,0x003,
           0x001,0x000]

class SPC700:
    def __init__(self, mem, dsp=None):
        self.mem = mem
        self.dsp = dsp
        self.a = self.x = self.y = 0
        self.sp = 0xEF
        self.pc = 0
        self.psw = 0
        self.cycles = 0
        self.ports_cpu = [0,0,0,0]
        self.ports_ext = [0,0,0,0]
        self.timers_en = [False]*3
        self.timers_target = [0xFF,0xFF,0xFF]
        self.timers_cnt = [0,0,0]
        self.timers_out = [0,0,0]
        self.t0_div = 0; self.t2_div = 0
        self.gcnt_by1 = 0; self.gcnt_by3 = 0x410; self.gcnt_by5 = 0x218
        self.dsp_addr = 0
        self.halted = False

    # ---------- flags ----------
    @property
    def C(self): return self.psw & 1
    def setNZ(self, v):
        v &= 0xFF
        self.psw = (self.psw & 0x7D) | (v & 0x80) | (2 if v == 0 else 0)
        return v
    def dpb(self): return 0x100 if (self.psw & 0x20) else 0

    # ---------- io ----------
    def rd(self, a):
        a &= 0xFFFF
        if 0xF0 <= a <= 0xFF: return self.io_r(a)
        return self.mem[a]
    def wr(self, a, v):
        a &= 0xFFFF; v &= 0xFF
        if 0xF0 <= a <= 0xFF: self.io_w(a, v); return
        self.mem[a] = v
    def io_r(self, a):
        if a == 0xF2: return self.dsp_addr
        if a == 0xF3: return self.dsp.read(self.dsp_addr) if self.dsp else 0
        if 0xF4 <= a <= 0xF7: return self.ports_ext[a-0xF4]
        if a == 0xF8: return self.ports_ext[0]
        if a == 0xF9: return self.ports_ext[1]
        if a == 0xFD: v = self.timers_out[0]; self.timers_out[0] = 0; return v
        if a == 0xFE: v = self.timers_out[1]; self.timers_out[1] = 0; return v
        if a == 0xFF: v = self.timers_out[2]; self.timers_out[2] = 0; return v
        return 0
    def io_w(self, a, v):
        if a == 0xF1:
            for i in range(3):
                if v & (1<<i):
                    if not self.timers_en[i]:
                        self.timers_cnt[i] = 0; self.timers_out[i] = 0
                    self.timers_en[i] = True
                else: self.timers_en[i] = False
            if v & 0x10: self.ports_ext[0] = self.ports_ext[1] = 0
            if v & 0x20: self.ports_ext[2] = self.ports_ext[3] = 0
        elif a == 0xF2: self.dsp_addr = v & 0x7F
        elif a == 0xF3:
            if self.dsp: self.dsp.write(self.dsp_addr, v)
        elif 0xF4 <= a <= 0xF7: self.ports_cpu[a-0xF4] = v
        elif a == 0xF8: self.ports_cpu[0] = v
        elif a == 0xF9: self.ports_cpu[1] = v
        elif a == 0xFA: self.timers_target[0] = v
        elif a == 0xFB: self.timers_target[1] = v
        elif a == 0xFC: self.timers_target[2] = v

    def tick(self, nc):
        # DSP-side 32 kHz step happens every 32 cpu cycles
        self.t2_div += nc
        while self.t2_div >= 32:
            self.t2_div -= 32
            if self.dsp: self.dsp.sample()
            # envelope-rate counters advance once per DSP sample
            self.gcnt_by1 = (self.gcnt_by1 + 1) & 0xFFF
            if (self.gcnt_by3 & 3) == 0: self.gcnt_by3 = (self.gcnt_by3 & 0xFFC) | 2
            else: self.gcnt_by3 = (self.gcnt_by3 + 1) & 0xFFF
            if (self.gcnt_by5 & 7) == 0: self.gcnt_by5 = (self.gcnt_by5 & 0xFF8) | 4
            else: self.gcnt_by5 = (self.gcnt_by5 + 1) & 0xFFF
            if self.timers_en[2]:
                self.timers_cnt[2] += 1
                if self.timers_cnt[2] > self.timers_target[2]:
                    self.timers_cnt[2] = 0
                    self.timers_out[2] = (self.timers_out[2]+1) & 0xF
        self.t0_div += nc
        while self.t0_div >= 128:
            self.t0_div -= 128
            for i in (0,1):
                if self.timers_en[i]:
                    self.timers_cnt[i] += 1
                    if self.timers_cnt[i] > self.timers_target[i]:
                        self.timers_cnt[i] = 0
                        self.timers_out[i] = (self.timers_out[i]+1) & 0xF

    def gcnt_trigger(self, rate):
        if rate == 0: return False
        if rate == 31: return True
        m = MASKS12[rate]
        grp = rate % 3
        if grp == 1: c = self.gcnt_by1
        elif grp == 2: c = self.gcnt_by3
        else: c = self.gcnt_by5
        return (c & m) == 0

    # ---------- stack / fetch ----------
    def push(self, v): self.mem[0x100+self.sp] = v & 0xFF; self.sp = (self.sp-1)&0xFF
    def pop(self): self.sp = (self.sp+1)&0xFF; return self.mem[0x100+self.sp]
    def f8(self): v = self.mem[self.pc]; self.pc = (self.pc+1)&0xFFFF; return v
    def f16(self): lo = self.f8(); hi = self.f8(); return lo|(hi<<8)
    def rel(self):
        d = self.f8(); d = d-256 if d > 127 else d
        return (self.pc + d) & 0xFFFF

    # ---------- alu ----------
    def adc(self, a, b):
        c = self.psw & 1
        r = a + b + c
        p = self.psw & 0x3C
        if r > 0xFF: p |= 1
        if (~(a^b) & (a^r) & 0x80): p |= 0x40
        rr = r & 0xFF
        p |= (rr & 0x80) | (2 if rr == 0 else 0)
        self.psw = p; return rr
    def sbc(self, a, b):
        c = self.psw & 1
        r = a - b - (1-c)
        p = self.psw & 0x3C
        if r >= 0: p |= 1
        if ((a^b) & (a^r) & 0x80): p |= 0x40
        rr = r & 0xFF
        p |= (rr & 0x80) | (2 if rr == 0 else 0)
        self.psw = p; return rr
    def cmp(self, a, b):
        r = (a - b) & 0xFF
        p = self.psw & 0x7C
        if a >= b: p |= 1
        p |= (r & 0x80) | (2 if r == 0 else 0)
        self.psw = p
    def branch(self, cond, cyc_base=2):
        t = self.rel()
        if cond: self.pc = t; return 2
        return 0

    def step(self):
        mem = self.mem
        RD, WR = self.rd, self.wr
        op = mem[self.pc]; self.pc = (self.pc+1)&0xFFFF
        cyc = 2

        # --- branches/implied first (hot in idle loops) ---
        if op == 0xF0: cyc += self.branch(self.psw & 2)
        elif op == 0xD0: cyc += self.branch(not (self.psw & 2))
        elif op == 0xB0: cyc += self.branch(self.psw & 1)
        elif op == 0x90: cyc += self.branch(not (self.psw & 1))
        elif op == 0x70: cyc += self.branch(self.psw & 0x40)
        elif op == 0x50: cyc += self.branch(not (self.psw & 0x40))
        elif op == 0x30: cyc += self.branch(self.psw & 0x80)
        elif op == 0x10: cyc += self.branch(not (self.psw & 0x80))
        elif op == 0x2F: self.pc = self.rel(); cyc = 4
        elif op == 0x00: pass
        elif op == 0x6F: self.pc = self.pop() | (self.pop()<<8); cyc = 5
        elif op == 0x7F: self.psw = self.pop(); self.pc = self.pop() | (self.pop()<<8); cyc = 6
        elif op == 0xEF: self.halted = True   # SLEEP
        elif op == 0xFF: self.halted = True   # STOP
        # TCALL n
        elif op & 0x0F == 0x01:
            n = op >> 4
            vec = 0xFFDE - 2*n
            self.push(self.pc>>8); self.push(self.pc&0xFF)
            self.pc = mem[vec] | (mem[vec+1]<<8); cyc = 8
        elif op == 0x0F:  # BRK
            self.push(self.pc>>8); self.push(self.pc&0xFF); self.push(self.psw)
            self.psw |= 8
            self.pc = mem[0xFFDE] | (mem[0xFFDF]<<8); cyc = 8
        elif op == 0x3F:  # CALL
            d = self.f16()
            self.push(self.pc>>8); self.push(self.pc&0xFF)
            self.pc = d; cyc = 8
        elif op == 0x4F:  # PCALL
            u = self.f8()
            self.push(self.pc>>8); self.push(self.pc&0xFF)
            self.pc = 0xFF00 | u; cyc = 6
        elif op == 0x5F:  # JMP abs
            self.pc = self.f16(); cyc = 3
        elif op == 0x1F:  # JMP [abs+X]
            d = (self.f16() + self.x) & 0xFFFF
            self.pc = RD(d) | (RD((d+1)&0xFFFF)<<8); cyc = 6
        # BBS/BBC
        elif op & 0x0F == 0x03:
            dp = self.f8(); t = self.rel(); bit = (op>>5)&7
            v = RD(self.dpb() | dp)
            take = bool(v & (1<<bit)) ^ bool(op & 0x10)
            if take: self.pc = t
            cyc = 5
        elif op & 0x0F == 0x02:  # SET1 / CLR1
            dp = self.f8(); bit = (op>>5)&7
            a = self.dpb() | dp
            if op & 0x10: WR(a, RD(a) & ~(1<<bit))
            else: WR(a, RD(a) | (1<<bit))
            cyc = 4
        # ---- MOV ----
        elif op == 0xE8: self.a = self.f8(); self.setNZ(self.a)
        elif op == 0xCD: self.x = self.f8(); self.setNZ(self.x)
        elif op == 0x8D: self.y = self.f8(); self.setNZ(self.y)
        elif op == 0xE4: self.a = RD(self.dpb()|self.f8()); self.setNZ(self.a); cyc=3
        elif op == 0xF4: self.a = RD(((self.dpb()|self.f8())+self.x)&0xFFFF); self.setNZ(self.a); cyc=4
        elif op == 0xE5: self.a = RD(self.f16()); self.setNZ(self.a); cyc=4
        elif op == 0xF5: self.a = RD((self.f16()+self.x)&0xFFFF); self.setNZ(self.a); cyc=5
        elif op == 0xF6: self.a = RD((self.f16()+self.y)&0xFFFF); self.setNZ(self.a); cyc=5
        elif op == 0xE6: self.a = RD(self.x); self.setNZ(self.a); cyc=3
        elif op == 0xBF: self.a = RD(self.x); self.x=(self.x+1)&0xFF; self.setNZ(self.a); cyc=4
        elif op == 0xE7:
            dp=self.f8(); p0=self.dpb()|dp; ptr=RD(p0)|(RD((p0+1)&0xFFFF)<<8)
            self.a = RD((ptr+self.x)&0xFFFF); self.setNZ(self.a); cyc=6
        elif op == 0xF7:
            dp=self.f8(); p0=self.dpb()|dp; ptr=RD(p0)|(RD((p0+1)&0xFFFF)<<8)
            self.a = RD((ptr+self.y)&0xFFFF); self.setNZ(self.a); cyc=6
        elif op == 0xE9: self.x = RD(self.f16()); self.setNZ(self.x); cyc=4
        elif op == 0xF8: self.x = RD(self.dpb()|self.f8()); self.setNZ(self.x); cyc=3
        elif op == 0xF9: self.x = RD(((self.dpb()|self.f8())+self.y)&0xFFFF); self.setNZ(self.x); cyc=4
        elif op == 0xEB: self.y = RD(self.dpb()|self.f8()); self.setNZ(self.y); cyc=3
        elif op == 0xFB: self.y = RD(((self.dpb()|self.f8())+self.x)&0xFFFF); self.setNZ(self.y); cyc=4
        elif op == 0xEC: self.y = RD(self.f16()); self.setNZ(self.y); cyc=4
        elif op == 0xC4: WR(self.dpb()|self.f8(), self.a); cyc=4
        elif op == 0xD4: WR(((self.dpb()|self.f8())+self.x)&0xFFFF, self.a); cyc=5
        elif op == 0xC5: WR(self.f16(), self.a); cyc=5
        elif op == 0xD5: WR((self.f16()+self.x)&0xFFFF, self.a); cyc=6
        elif op == 0xD6: WR((self.f16()+self.y)&0xFFFF, self.a); cyc=6
        elif op == 0xC6: WR(self.x, self.a); cyc=4
        elif op == 0xAF: WR(self.x, self.a); self.x=(self.x+1)&0xFF; cyc=4
        elif op == 0xC7:
            dp=self.f8(); p0=self.dpb()|dp; ptr=RD(p0)|(RD((p0+1)&0xFFFF)<<8)
            WR((ptr+self.x)&0xFFFF, self.a); cyc=7
        elif op == 0xD7:
            dp=self.f8(); p0=self.dpb()|dp; ptr=RD(p0)|(RD((p0+1)&0xFFFF)<<8)
            WR((ptr+self.y)&0xFFFF, self.a); cyc=7
        elif op == 0xC9: WR(self.f16(), self.x); cyc=5
        elif op == 0xD8: WR(self.dpb()|self.f8(), self.x); cyc=4
        elif op == 0xD9: WR(((self.dpb()|self.f8())+self.y)&0xFFFF, self.x); cyc=5
        elif op == 0xCC: WR(self.f16(), self.y); cyc=5
        elif op == 0xCB: WR(self.dpb()|self.f8(), self.y); cyc=4
        elif op == 0xDB: WR(((self.dpb()|self.f8())+self.x)&0xFFFF, self.y); cyc=5
        elif op == 0xFA:
            s = self.f8(); d = self.f8()
            WR(self.dpb()|d, RD(self.dpb()|s)); cyc=5
        elif op == 0x8F:
            imm = self.f8(); dp = self.f8()
            WR(self.dpb()|dp, imm); cyc=5
        elif op == 0x7D: self.a = self.x; self.setNZ(self.a)
        elif op == 0xDD: self.a = self.y; self.setNZ(self.a)
        elif op == 0x5D: self.x = self.a; self.setNZ(self.x)
        elif op == 0xFD: self.y = self.a; self.setNZ(self.y)
        elif op == 0x9D: self.x = self.sp; self.setNZ(self.x)
        elif op == 0xBD: self.sp = self.x
        elif op == 0x9F: self.a = ((self.a>>4)|((self.a<<4)&0xFF)); self.setNZ(self.a); cyc=5
        # ---- MOVW / word ops ----
        elif op == 0xBA:
            dp = self.f8(); p0 = self.dpb()|dp
            self.a = RD(p0); self.y = RD((p0+1)&0xFFFF)
            self.psw = (self.psw & 0x7D) | (self.y & 0x80) | (2 if (self.a|(self.y<<8))==0 else 0)
            cyc=5
        elif op == 0xDA:
            dp = self.f8(); p0 = self.dpb()|dp
            WR(p0, self.a); WR((p0+1)&0xFFFF, self.y); cyc=5
        elif op == 0x3A:
            dp = self.f8(); p0 = self.dpb()|dp
            w = ((RD(p0) | (RD((p0+1)&0xFFFF)<<8)) + 1) & 0xFFFF
            WR(p0, w&0xFF); WR((p0+1)&0xFFFF, w>>8); cyc=6
        elif op == 0x1A:
            dp = self.f8(); p0 = self.dpb()|dp
            w = (RD(p0) | (RD((p0+1)&0xFFFF)<<8)) - 1
            w &= 0xFFFF
            WR(p0, w&0xFF); WR((p0+1)&0xFFFF, w>>8); cyc=6
        elif op == 0x7A:
            dp = self.f8(); p0 = self.dpb()|dp
            w = RD(p0) | (RD((p0+1)&0xFFFF)<<8)
            ya = self.a | (self.y<<8)
            r = ya + w
            p = self.psw & 0x3C
            if r > 0xFFFF: p |= 1
            if (~(ya^w) & (ya^r) & 0x8000): p |= 0x40
            rr = r & 0xFFFF
            p |= ((rr>>8) & 0x80) | (2 if rr == 0 else 0)
            self.psw = p
            self.a = rr & 0xFF; self.y = rr >> 8
            cyc=5
        elif op == 0x9A:
            dp = self.f8(); p0 = self.dpb()|dp
            w = RD(p0) | (RD((p0+1)&0xFFFF)<<8)
            ya = self.a | (self.y<<8)
            r = ya - w
            p = self.psw & 0x3C
            if r >= 0: p |= 1
            if ((ya^w) & (ya^r) & 0x8000): p |= 0x40
            rr = r & 0xFFFF
            p |= ((rr>>8) & 0x80) | (2 if rr == 0 else 0)
            self.psw = p
            self.a = rr & 0xFF; self.y = rr >> 8
            cyc=5
        elif op == 0x5A:
            dp = self.f8(); p0 = self.dpb()|dp
            w = RD(p0) | (RD((p0+1)&0xFFFF)<<8)
            ya = self.a | (self.y<<8)
            r = (ya - w) & 0xFFFF
            p = self.psw & 0x7C
            if ya >= w: p |= 1
            p |= ((r>>8) & 0x80) | (2 if r == 0 else 0)
            self.psw = p
            cyc=4
        # ---- 8-bit ALU: A, operand ----
        elif op == 0x08: self.a |= self.f8(); self.setNZ(self.a)
        elif op == 0x28: self.a &= self.f8(); self.setNZ(self.a)
        elif op == 0x48: self.a ^= self.f8(); self.setNZ(self.a)
        elif op == 0x68: self.cmp(self.a, self.f8())
        elif op == 0x88: self.a = self.adc(self.a, self.f8())
        elif op == 0xA8: self.a = self.sbc(self.a, self.f8())
        elif op == 0xC8: self.cmp(self.x, self.f8())
        elif op == 0xAD: self.cmp(self.y, self.f8())
        # ---- dp / abs / indexed ALU ----
        elif op == 0x3E: self.cmp(self.x, RD(self.dpb()|self.f8())); cyc=3
        elif op == 0x1E: self.cmp(self.x, RD(self.f16())); cyc=4
        elif op == 0x7E: self.cmp(self.y, RD(self.dpb()|self.f8())); cyc=3
        elif op == 0x5E: self.cmp(self.y, RD(self.f16())); cyc=4
        elif op & 0x0F in (0x04,0x05,0x06,0x07,0x14,0x15,0x16,0x17,
                           0x24,0x25,0x26,0x27,0x34,0x35,0x36,0x37,
                           0x44,0x45,0x46,0x47,0x54,0x55,0x56,0x57,
                           0x64,0x65,0x66,0x67,0x74,0x75,0x76,0x77,
                           0x84,0x85,0x86,0x87,0x94,0x95,0x96,0x97,
                           0xA4,0xA5,0xA6,0xA7,0xB4,0xB5,0xB6,0xB7):
            kind = op >> 4
            sub = op & 0x0F
            odd = bool((op >> 4) & 1)   # odd rows use indexed addressing modes
            if sub == 0x04:
                a = self.dpb()|self.f8()
                if odd: a = (a + self.x) & 0xFFFF
                cyc = 4 if odd else 3
            elif sub == 0x05:
                a = self.f16()
                if odd: a = (a + self.x) & 0xFFFF
                cyc = 5 if odd else 4
            elif sub == 0x06:
                if odd: a = (self.f16() + self.y) & 0xFFFF; cyc = 5
                else: a = self.x; cyc = 3
            else:  # 0x07
                dp=self.f8(); p0=self.dpb()|dp; ptr=RD(p0)|(RD((p0+1)&0xFFFF)<<8)
                a=(ptr+(self.y if odd else self.x))&0xFFFF; cyc=6
            v = RD(a)
            if kind in (0x0,0x1): self.a |= v; self.setNZ(self.a)
            elif kind in (0x2,0x3): self.a &= v; self.setNZ(self.a)
            elif kind in (0x4,0x5): self.a ^= v; self.setNZ(self.a)
            elif kind in (0x6,0x7): self.cmp(self.a, v)
            elif kind in (0x8,0x9): self.a = self.adc(self.a, v)
            else: self.a = self.sbc(self.a, v)
        # ---- mem,mem & (X),(Y) ----
        elif op in (0x09,0x29,0x49,0x69,0x89,0xA9):
            s = self.f8(); d = self.f8()
            sv = RD(self.dpb()|s)
            if op == 0x09: r = RD(self.dpb()|d) | sv
            elif op == 0x29: r = RD(self.dpb()|d) & sv
            elif op == 0x49: r = RD(self.dpb()|d) ^ sv
            elif op == 0x69: self.cmp(RD(self.dpb()|d), sv); r = None
            elif op == 0x89: r = self.adc(RD(self.dpb()|d), sv)
            else: r = self.sbc(RD(self.dpb()|d), sv)
            if r is not None:
                WR(self.dpb()|d, r)
                self.psw = (self.psw & 0x7D) | (r & 0x80) | (2 if r == 0 else 0)
            cyc=6
        elif op in (0x19,0x39,0x59,0x79,0x99,0xB9):
            xv = RD(self.x)
            yv = RD(self.y)
            if op == 0x19: r = xv | yv
            elif op == 0x39: r = xv & yv
            elif op == 0x59: r = xv ^ yv
            elif op == 0x79: self.cmp(xv, yv); r = None
            elif op == 0x99: r = self.adc(xv, yv)
            else: r = self.sbc(xv, yv)
            if r is not None:
                WR(self.x, r)
                self.psw = (self.psw & 0x7D) | (r & 0x80) | (2 if r == 0 else 0)
            cyc=5
        # ---- dp,#imm ----
        elif op in (0x18,0x38,0x58,0x78,0x98,0xB8):
            imm = self.f8(); dp = self.f8()
            dv = RD(self.dpb()|dp)
            if op == 0x18: r = dv | imm
            elif op == 0x38: r = dv & imm
            elif op == 0x58: r = dv ^ imm
            elif op == 0x78: self.cmp(dv, imm); r = None
            elif op == 0x98: r = self.adc(dv, imm)
            else: r = self.sbc(dv, imm)
            if r is not None:
                WR(self.dpb()|dp, r)
                self.psw = (self.psw & 0x7D) | (r & 0x80) | (2 if r == 0 else 0)
            cyc=5
        # ---- shifts ----
        elif op == 0x1C: self.psw=(self.psw&~1)|(self.a>>7); self.a=(self.a<<1)&0xFF; self.setNZ(self.a)
        elif op == 0x5C: self.psw=(self.psw&~1)|(self.a&1); self.a>>=1; self.setNZ(self.a)
        elif op == 0x3C:
            c=self.psw&1; self.psw=(self.psw&~1)|(self.a>>7); self.a=((self.a<<1)|c)&0xFF; self.setNZ(self.a)
        elif op == 0x7C:
            c=self.psw&1; self.psw=(self.psw&~1)|(self.a&1); self.a=(c<<7)|(self.a>>1); self.setNZ(self.a)
        elif op in (0x0B,0x0C,0x1B,0x2B,0x2C,0x3B,0x4B,0x4C,0x5B,0x6B,0x6C,0x7B):
            if op == 0x0C or op == 0x2C or op == 0x4C or op == 0x6C: a = self.f16(); cyc=5
            else: a = ((self.dpb()|self.f8()) + (self.x if op in (0x1B,0x3B,0x5B,0x7B) else 0))&0xFFFF; cyc=4
            v = RD(a)
            if op in (0x0B,0x0C): self.psw=(self.psw&~1)|(v>>7); v=(v<<1)&0xFF
            elif op in (0x2B,0x2C):
                c=self.psw&1; self.psw=(self.psw&~1)|(v>>7); v=((v<<1)|c)&0xFF
            elif op in (0x4B,0x4C): self.psw=(self.psw&~1)|(v&1); v>>=1
            else:
                c=self.psw&1; self.psw=(self.psw&~1)|(v&1); v=(c<<7)|(v>>1)
            self.setNZ(v); WR(a, v)
        # ---- inc/dec ----
        elif op == 0xBC: self.a=(self.a+1)&0xFF; self.setNZ(self.a)
        elif op == 0x3D: self.x=(self.x+1)&0xFF; self.setNZ(self.x)
        elif op == 0xFC: self.y=(self.y+1)&0xFF; self.setNZ(self.y)
        elif op == 0x9C: self.a=(self.a-1)&0xFF; self.setNZ(self.a)
        elif op == 0x1D: self.x=(self.x-1)&0xFF; self.setNZ(self.x)
        elif op == 0xDC: self.y=(self.y-1)&0xFF; self.setNZ(self.y)
        elif op in (0xAB,0x8B,0xBB,0x9B,0xAC,0x8C):
            if op == 0xAC or op == 0x8C: a = self.f16(); cyc=5
            else: a = ((self.dpb()|self.f8()) + (self.x if op in (0xBB,0x9B) else 0))&0xFFFF; cyc=4
            v = RD(a)
            v = (v+1)&0xFF if op in (0xAB,0xBB,0xAC) else (v-1)&0xFF
            self.setNZ(v); WR(a, v)
        # ---- CBNE / DBNZ ----
        elif op == 0x2E:
            dp = self.f8(); t = self.rel()
            if RD(self.dpb()|dp) != self.a: self.pc = t; cyc = 7
            else: cyc = 5
        elif op == 0xDE:
            dp = self.f8(); t = self.rel()
            if RD(((self.dpb()|dp)+self.x)&0xFFFF) != self.a: self.pc = t; cyc = 8
            else: cyc = 6
        elif op == 0x6E:
            dp = self.f8(); t = self.rel()
            a = self.dpb()|dp
            v = (RD(a)-1)&0xFF; WR(a, v)
            if v: self.pc = t; cyc = 7
            else: cyc = 5
        elif op == 0xFE:
            t = self.rel()
            self.y = (self.y-1)&0xFF
            if self.y: self.pc = t; cyc = 6
            else: cyc = 4
        # ---- push/pop ----
        elif op == 0x0D: self.push(self.psw); cyc=4
        elif op == 0x2D: self.push(self.a); cyc=4
        elif op == 0x4D: self.push(self.x); cyc=4
        elif op == 0x6D: self.push(self.y); cyc=4
        elif op == 0x8E: self.psw = self.pop(); cyc=4
        elif op == 0xAE: self.a = self.pop(); cyc=4
        elif op == 0xCE: self.x = self.pop(); cyc=4
        elif op == 0xEE: self.y = self.pop(); cyc=4
        # ---- multiply/divide ----
        elif op == 0xCF:
            r = self.y * self.a
            self.a = r & 0xFF; self.y = (r>>8) & 0xFF
            ya = self.a | (self.y<<8)
            self.psw = (self.psw & 0x7D) | (self.y & 0x80) | (2 if ya == 0 else 0)
            cyc=9
        elif op == 0x9E:
            ya = self.a | (self.y<<8)
            if self.x == 0:
                self.psw |= 0x40; cyc=12
            else:
                self.psw = self.psw & ~0x40
                if self.y >= self.x: self.psw |= 0x40
                q = ya // self.x; r = ya % self.x
                self.a = q & 0xFF; self.y = r & 0xFF
                self.setNZ(self.a)
                cyc=12
        # ---- TSET1 / TCLR1 ----
        elif op == 0x0E or op == 0x4E:
            a = self.f16(); v = RD(a)
            r = self.a & v
            self.setNZ(r)
            if op == 0x0E: WR(a, v | self.a)
            else: WR(a, v & ~self.a)
            cyc=6
        # ---- DAA / DAS ----
        elif op == 0xDF:
            if (self.a & 0x0F) > 9 or (self.psw & 0x08) or False:
                pass
            # approximate DAA/DAS (not used by this driver)
            self.setNZ(self.a)
        elif op == 0xBE:
            self.setNZ(self.a)
        # ---- carry-bit memory ops ----
        elif op in (0x0A,0x2A,0x4A,0x6A,0x8A,0xAA,0xCA,0xEA):
            lo = self.f8(); hi = self.f8()
            v = lo | (hi<<8)
            bit = (v >> 13) & 7
            a = v & 0x1FFF
            m = RD(a)
            b = (m >> bit) & 1
            c = self.psw & 1
            if op == 0x0A: nb = c | b
            elif op == 0x2A: nb = c & b
            elif op == 0x4A: nb = c ^ b
            elif op == 0x6A: nb = c & (1-b)
            elif op == 0x8A: nb = c ^ b
            elif op == 0xAA: nb = b        # MOV C,bit
            elif op == 0xCA:              # MOV bit,C
                if c: WR(a, m | (1<<bit))
                else: WR(a, m & ~(1<<bit))
                nb = c
            else:                          # NOT1
                WR(a, m ^ (1<<bit)); nb = c
            self.psw = (self.psw & ~1) | (nb & 1)
            cyc = 4 if op in (0xCA,0xEA) else 3
        elif op == 0xED: self.psw ^= 1    # NOTC
        elif op == 0x60: self.psw &= ~1   # CLRC
        elif op == 0x80: self.psw |= 1    # SETC
        elif op == 0x20: self.psw &= ~0x20  # CLRP
        elif op == 0x40: self.psw |= 0x20   # SETP
        elif op == 0xE0: self.psw &= ~0x40  # CLRV
        elif op == 0xA0: self.psw |= 8    # EI
        elif op == 0xC0: self.psw &= ~8   # DI
        else:
            self.halted = True
            print(f"SPC700: unhandled opcode ${op:02X} at ${self.pc-1:04X}")
        self.cycles += cyc
        self.tick(cyc)
        return cyc
