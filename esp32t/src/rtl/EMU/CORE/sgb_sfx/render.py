# Render the SGB BIOS built-in SFX bank to WAV by executing the real
# N-SPC driver (SPC700) with the real SNES DSP algorithm.
import struct, wave, sys, os
from spc700 import SPC700
from snesdsp import SNESDSP

# Paths (override with env SGB_BIOS / SGB_OUT, or edit here).
# SGB1.sfc is the copyrighted SGB BIOS -- provide your own dump.
_here = os.path.dirname(os.path.abspath(__file__))
BIOS = os.environ.get('SGB_BIOS', os.path.join(_here, 'SGB1.sfc'))
OUT  = os.environ.get('SGB_OUT',  os.path.join(_here, 'wav'))
os.makedirs(OUT, exist_ok=True)

def parse_stream(rom, base):
    recs = []
    o = base
    while True:
        ln = rom[o] | rom[o+1]<<8
        dst = rom[o+2] | rom[o+3]<<8
        o += 4
        if ln == 0:
            return recs, dst
        recs.append((dst, rom[o:o+ln]))
        o += ln

def load_apu():
    rom = open(BIOS,'rb').read()
    # bank 6 (LoROM file offset 0x30000) holds the APU image stream
    recs, entry = parse_stream(rom, 0x30000)
    ram = bytearray(0x10000)
    for dst, data in recs:
        ram[dst:dst+len(data)] = data
    return ram, entry

class Rig:
    def __init__(self):
        self.ram, entry = load_apu()
        self.dsp = SNESDSP(self.ram)
        self.cpu = SPC700(self.ram, self.dsp)
        self.dsp.cpu = self.cpu
        self.cpu.pc = entry
        self.audio = []
    def run_cycles(self, n):
        cpu = self.cpu
        end = cpu.cycles + n
        while cpu.cycles < end and not cpu.halted:
            cpu.step()
        if cpu.halted:
            raise RuntimeError(f"CPU halted at ${cpu.pc:04X}")
    def run_ms(self, ms):
        self.run_cycles(int(ms * 1024))
    def run_and_capture(self, ms, full_vol=True):
        # capture dsp output while running for ms milliseconds of SPC time
        target = self.cpu.cycles + int(ms*1024)
        out = self.audio
        dsp = self.dsp
        cpu = self.cpu
        orig_w = dsp.write
        if full_vol:
            # BIOS ramps MVOL slowly from ~0x1A; pin it at full so every
            # effect renders at full volume regardless of its duration.
            dsp.mvol = [0x7F, 0x7F]
            def w(a, v):
                if a in (0x0C, 0x1C):   # block the driver's MVOL ramp writes
                    return
                orig_w(a, v)
            dsp.write = w
        try:
            while cpu.cycles < target and not cpu.halted:
                before = cpu.t2_div
                cpu.step()
                if cpu.t2_div < before:  # wrapped => a DSP sample happened
                    out.append((dsp.out_l, dsp.out_r))
        finally:
            dsp.write = orig_w
        if cpu.halted:
            raise RuntimeError(f"CPU halted at ${cpu.pc:04X}")
    def set_ports(self, b4=0, b1=0, b2=0, b3=0):
        # byte1(SFX A)->$F5, byte2(SFX B)->$F6, byte3(attrs)->$F7, byte4->$F4
        self.cpu.ports_ext[0] = b4 & 0xFF
        self.cpu.ports_ext[1] = b1 & 0xFF
        self.cpu.ports_ext[2] = b2 & 0xFF
        self.cpu.ports_ext[3] = b3 & 0xFF

def attrs(pitch_a=0, vol_a=0, pitch_b=0, vol_b=0):
    return (pitch_a & 3) | ((vol_a & 3) << 2) | ((pitch_b & 3) << 4) | ((vol_b & 3) << 6)

def render_effect(rig, name, b1=0, b2=0, pitch_a=0, pitch_b=0,
                  max_s=8.0, tail_s=0.25, cap_s=None):
    rig.audio = []
    # retrigger: clear index first
    rig.set_ports(b1=0, b2=0, b3=attrs(pitch_a, 0, pitch_b, 0))
    rig.run_and_capture(12)
    rig.set_ports(b1=b1, b2=b2, b3=attrs(pitch_a, 0, pitch_b, 0))
    rig.run_and_capture(6)
    silence_ms = 0
    heard = False
    cap = int((cap_s or max_s) * 1000)
    total = 0
    while total < cap:
        n0 = len(rig.audio)
        rig.run_and_capture(20)
        seg = rig.audio[n0:]
        peak = max((abs(l)+abs(r) for l,r in seg), default=0)
        if peak > 256:
            heard = True
            silence_ms = 0
        else:
            silence_ms += 20
        total += 20
        if heard and silence_ms >= int(tail_s*1000):
            break
        if not heard and total >= 1500:
            break
    if not heard:
        return False, 0
    # stop
    rig.set_ports(b1=0x80 if b1 else 0, b2=0x80 if b2 else 0)
    rig.run_and_capture(30)
    return True, len(rig.audio)

def save_wav(path, samples, rate=32000):
    w = wave.open(path, 'w')
    w.setnchannels(2); w.setsampwidth(2); w.setframerate(rate)
    frames = bytearray()
    for l, r in samples:
        frames += struct.pack('<hh', max(-32768,min(32767,l)), max(-32768,min(32767,r)))
    w.writeframes(bytes(frames)); w.close()

# Official SGB BIOS effect names (Pan Docs)
A_NAMES = {
0x01:'Nintendo',0x02:'GameOver',0x03:'Drop',0x04:'OkA',0x05:'OkB',
0x06:'SelectA',0x07:'SelectB',0x08:'SelectC',0x09:'MistakeBuzzer',
0x0A:'CatchItem',0x0B:'GateSqueak',0x0C:'ExplosionSmall',0x0D:'ExplosionMedium',
0x0E:'ExplosionLarge',0x0F:'AttackedA',0x10:'AttackedB',0x11:'HitPunchA',
0x12:'HitPunchB',0x13:'BreathInAir',0x14:'RocketProjectileA',0x15:'RocketProjectileB',
0x16:'EscapingBubble',0x17:'Jump',0x18:'FastJump',0x19:'JetTakeoff',
0x1A:'JetLanding',0x1B:'CupBreaking',0x1C:'GlassBreaking',0x1D:'LevelUp',
0x1E:'InsertAir',0x1F:'SwordSwing',0x20:'WaterFalling',0x21:'Fire',
0x22:'WallCollapsing',0x23:'Cancel',0x24:'Walking',0x25:'BlockingStrike',
0x26:'PictureFloats',0x27:'FadeIn',0x28:'FadeOut',0x29:'WindowOpen',
0x2A:'WindowClose',0x2B:'BigLaser',0x2C:'StoneGate',0x2D:'Teleportation',
0x2E:'LightningA',0x2F:'EarthquakeA',0x30:'SmallLaser'}
A_PITCH = {0x01:3,0x02:3,0x03:3,0x04:3,0x05:3,0x06:3,0x07:3,0x08:2,0x09:2,
0x0A:2,0x0B:2,0x0C:1,0x0D:1,0x0E:1,0x0F:3,0x10:3,0x11:0,0x12:0,0x13:3,
0x14:3,0x15:3,0x16:2,0x17:3,0x18:3,0x19:0,0x1A:0,0x1B:2,0x1C:1,0x1D:2,
0x1E:1,0x1F:1,0x20:2,0x21:1,0x22:1,0x23:1,0x24:1,0x25:1,0x26:3,0x27:0,
0x28:0,0x29:1,0x2A:0,0x2B:3,0x2C:0,0x2D:3,0x2E:0,0x2F:0,0x30:2}
B_NAMES = {
0x01:'ApplauseSmall',0x02:'ApplauseMedium',0x03:'ApplauseLarge',0x04:'Wind',
0x05:'Rain',0x06:'Storm',0x07:'StormThunder',0x08:'LightningB',0x09:'EarthquakeB',
0x0A:'Avalanche',0x0B:'Wave',0x0C:'River',0x0D:'Waterfall',0x0E:'SmallCharRunning',
0x0F:'HorseRunning',0x10:'WarningSound',0x11:'ApproachingCar',0x12:'JetFlying',
0x13:'UfoFlying',0x14:'ElectromagneticWaves',0x15:'ScoreUp',0x16:'FireB',
0x17:'CameraShutter',0x18:'WriteFormanto',0x19:'ShowUpTitle'}
B_PITCH = {0x01:2,0x02:2,0x03:2,0x04:1,0x05:1,0x06:1,0x07:2,0x08:0,0x09:0,
0x0A:0,0x0B:0,0x0C:3,0x0D:2,0x0E:3,0x0F:3,0x10:1,0x11:0,0x12:1,0x13:2,
0x14:0,0x15:3,0x16:2,0x17:3,0x18:0,0x19:0}
# B effects are sustained loops; render a fixed window
B_LOOP = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,
          0x0E,0x0F,0x11,0x12,0x13,0x14,0x17,0x18,0x19}

def main():
    rig = Rig()
    print(f"entry=${rig.cpu.pc:04X}, booting driver...")
    rig.run_ms(400)
    print(f"booted; cycles={rig.cpu.cycles}")

    which = sys.argv[1] if len(sys.argv) > 1 else 'all'
    if which in ('all','a'):
        for idx in sorted(A_NAMES):
            ok, n = render_effect(rig, f"A{idx:02X}", b1=idx, pitch_a=A_PITCH[idx],
                                  max_s=6.0)
            if ok:
                save_wav(f"{OUT}/A{idx:02X}_{A_NAMES[idx]}.wav", rig.audio)
                print(f"A ${idx:02X} {A_NAMES[idx]:<20} OK {n/32000:.2f}s")
            else:
                print(f"A ${idx:02X} {A_NAMES[idx]:<20} SILENT")
    if which in ('all','b'):
        for idx in sorted(B_NAMES):
            cap = 5.0 if idx in B_LOOP else 6.0
            ok, n = render_effect(rig, f"B{idx:02X}", b2=idx, pitch_b=B_PITCH[idx],
                                  max_s=6.0, cap_s=cap)
            if ok:
                save_wav(f"{OUT}/B{idx:02X}_{B_NAMES[idx]}.wav", rig.audio)
                print(f"B ${idx:02X} {B_NAMES[idx]:<20} OK {n/32000:.2f}s")
            else:
                print(f"B ${idx:02X} {B_NAMES[idx]:<20} SILENT")

if __name__ == '__main__':
    main()
