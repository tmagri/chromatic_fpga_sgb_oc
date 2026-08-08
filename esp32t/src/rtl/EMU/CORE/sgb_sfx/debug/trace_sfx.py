import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# SGB1.sfc is the copyrighted SGB BIOS -- provide your own dump, either as
# sgb_sfx/SGB1.sfc or via the SGB_BIOS env var.
os.environ.setdefault('SGB_BIOS', os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'SGB1.sfc'))
import render
from render import Rig, attrs

rig = Rig()
rig.run_ms(400)  # boot driver

# intercept DSP writes
events = []
orig_write = rig.dsp.write
def traced(a, v):
    events.append((rig.cpu.cycles, a, v))
    orig_write(a, v)
rig.dsp.write = traced

def trigger(b1=0, b2=0, pa=0, pb=0, ms=1500):
    events.clear()
    rig.set_ports(b1=0,b2=0,b3=attrs(pa,0,pb,0)); rig.run_and_capture(12, full_vol=False)
    rig.set_ports(b1=b1,b2=b2,b3=attrs(pa,0,pb,0)); rig.run_and_capture(ms, full_vol=False)
    # summarize
    kons = []; srcns = {}; pitches = {}; adsrs={}
    t0 = events[0][0] if events else 0
    for cyc,a,v in events:
        lo = a & 0x0F; vo = a >> 4
        if a == 0x4C and v: kons.append((cyc-t0, v))
        elif lo == 0x4: srcns.setdefault(vo, []).append((cyc-t0, v))
        elif lo == 0x2: pitches.setdefault(vo, []).append((cyc-t0, v))
        elif lo == 0x5: adsrs.setdefault(vo, []).append((cyc-t0, v))
    print(f"  KON events: {len(kons)} -> {[hex(v) for _,v in kons[:12]]}")
    for vo in sorted(srcns):
        ss = srcns[vo]
        uniq = sorted(set(v for _,v in ss))
        print(f"  voice{vo}: SRCNs written={uniq} (n={len(ss)}) pitch-writes={len(pitches.get(vo,[]))}")

tests = [
    ("A04 OkA",        dict(b1=0x04, pa=3)),
    ("A0E ExplLarge",  dict(b1=0x0E, pa=1)),
    ("A17 Jump",       dict(b1=0x17, pa=3)),
    ("A1F SwordSwing", dict(b1=0x1F, pa=1)),
    ("B01 ApplauseSm", dict(b2=0x01, pb=2)),
    ("B04 Wind",       dict(b2=0x04, pb=1)),
]
for name, kw in tests:
    print(f"== {name}")
    trigger(**kw, ms=1200)
