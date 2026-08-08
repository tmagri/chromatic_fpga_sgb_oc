import struct, os
# SGB1.sfc is the copyrighted SGB BIOS -- provide your own dump, either as
# sgb_sfx/SGB1.sfc or via the SGB_BIOS env var.
_here = os.path.dirname(os.path.abspath(__file__))
BIOS = os.environ.get('SGB_BIOS', os.path.join(os.path.dirname(_here), 'SGB1.sfc'))
rom=open(BIOS,'rb').read()
def parse_stream(rom, base):
    recs=[]; o=base
    while True:
        ln=rom[o]|rom[o+1]<<8; dst=rom[o+2]|rom[o+3]<<8; o+=4
        if ln==0: return recs, dst, o
        recs.append((dst,o,rom[o:o+ln])); o+=ln
recs, entry, end = parse_stream(rom, 0x30000)
ram=bytearray(0x10000)
for dst,foff,data in recs: ram[dst:dst+len(data)]=data

def hexdump(addr, n, label):
    print(f"=== {label} @ ${addr:04X} ({n} bytes) ===")
    for i in range(0, n, 16):
        chunk=ram[addr+i:addr+i+16]
        hx=' '.join(f'{b:02X}' for b in chunk)
        print(f"  ${addr+i:04X}: {hx}")

# small records
for dst,foff,data in recs:
    if len(data) in (378,24):
        hexdump(dst, min(len(data),160), f"small record dst=${dst:04X} len={len(data)}")
