import struct, os
# SGB1.sfc is the copyrighted SGB BIOS -- provide your own dump, either as
# sgb_sfx/SGB1.sfc or via the SGB_BIOS env var.
_here = os.path.dirname(os.path.abspath(__file__))
BIOS = os.environ.get('SGB_BIOS', os.path.join(os.path.dirname(_here), 'SGB1.sfc'))
rom=open(BIOS,'rb').read()
print("ROM size", len(rom), hex(len(rom)))

def parse_stream(rom, base):
    recs=[]; o=base
    while True:
        ln=rom[o]|rom[o+1]<<8
        dst=rom[o+2]|rom[o+3]<<8
        o+=4
        if ln==0:
            return recs, dst, o
        recs.append((dst,o,rom[o:o+ln]))
        o+=ln

recs, entry, end = parse_stream(rom, 0x30000)
print(f"APU stream at file 0x30000: entry=${entry:04X}, stream ends at file 0x{end:X}")
print(f"num records = {len(recs)}")
total=0
for dst,foff,data in recs:
    total+=len(data)
    print(f"  dst=${dst:04X} len={len(data):6d} (0x{len(data):04X}) file_off=0x{foff:05X}..0x{foff+len(data):05X}")
print("total APU bytes", total, hex(total))
print("APU stream file span: 0x30000 .. 0x%X (%d bytes)"%(end, end-0x30000))
