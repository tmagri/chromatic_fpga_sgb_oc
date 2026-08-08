# Execute .spc dumps (APU RAM + DSP regs + CPU regs) in the SPC700+DSP
# harness and record the audio output to WAV.
import os, sys, struct, wave
from spc700 import SPC700
from snesdsp import SNESDSP

def load_spc(path):
    d = open(path, 'rb').read()
    assert d[:0x20].startswith(b'SNES-SPC700'), 'not an spc file'
    pc  = d[0x25] | d[0x26]<<8
    a, x, y, psw, sp = d[0x27], d[0x28], d[0x29], d[0x2A], d[0x2B]
    ram = bytearray(d[0x100:0x10100])
    dsp = list(d[0x10100:0x10180])
    return pc, a, x, y, psw, sp, ram, dsp

def render(path, out, seconds=45.0):
    pc, a, x, y, psw, sp, ram, dspregs = load_spc(path)
    dsp = SNESDSP(ram)
    dsp.regs[:] = dspregs
    cpu = SPC700(ram, dsp)
    dsp.cpu = cpu
    cpu.pc, cpu.a, cpu.x, cpu.y, cpu.psw, cpu.sp = pc, a, x, y, psw, sp
    audio = []
    orig = dsp.sample
    def smp():
        r = orig()
        audio.append(r)
        return r
    dsp.sample = smp
    target_cycles = int(seconds * 1024 * 1000)
    while cpu.cycles < target_cycles and not cpu.halted:
        cpu.step()
    w = wave.open(out, 'w')
    w.setnchannels(2); w.setsampwidth(2); w.setframerate(32000)
    frames = bytearray()
    for l, r in audio:
        frames += struct.pack('<hh', max(-32768,min(32767,l)), max(-32768,min(32767,r)))
    w.writeframes(bytes(frames)); w.close()
    print(f"{os.path.basename(path)} -> {os.path.basename(out)}: {len(audio)/32000:.1f}s, halted={cpu.halted}")

if __name__ == '__main__':
    # Defaults live inside this folder so the tree is self-contained.
    # .spc dumps are copyrighted SGB material -- provide your own in ./spc.
    _here = os.path.dirname(os.path.abspath(__file__))
    default_src = os.path.join(_here, 'spc')
    default_dst = os.path.join(_here, 'spc', 'wav')

    src = sys.argv[1] if len(sys.argv) > 1 else default_src
    dst = sys.argv[2] if len(sys.argv) > 2 else default_dst
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 45.0
    
    os.makedirs(dst, exist_ok=True)
    for fn in sorted(os.listdir(src)):
        if fn.lower().endswith('.spc'):
            render(os.path.join(src, fn), os.path.join(dst, fn[:-4] + '.wav'), secs)