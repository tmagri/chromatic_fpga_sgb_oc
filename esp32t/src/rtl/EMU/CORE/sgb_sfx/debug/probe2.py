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

# Sample directory at $4B00: 64 entries of (start,loop) 16-bit LE
print("=== SAMPLE DIRECTORY @ $4B00 ===")
for i in range(64):
    s = ram[0x4B00+4*i] | ram[0x4B00+4*i+1]<<8
    l = ram[0x4B00+4*i+2] | ram[0x4B00+4*i+3]<<8
    if s!=0 or l!=0:
        print(f"  dir[{i:2d}] start=${s:04X} loop=${l:04X}")
