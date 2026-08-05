#!/usr/bin/env python3
import os
import json
import struct
import argparse

def main():
    ap = argparse.ArgumentParser(description="Pack existing BRR files into a bank.")
    ap.add_argument('--brr', default='brr', help="Directory containing .brr files and manifest.json")
    ap.add_argument('--out', default='bankUncompressed', help="Output directory")
    ap.add_argument('--only', default='', help='comma-separated effect stems to include, e.g. A0C,B04')
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    only = set(s.strip().lower() for s in a.only.split(',') if s.strip())

    manifest_path = os.path.join(a.brr, 'manifest.json')

    if not os.path.exists(manifest_path):
        print(f"Manifest not found at {manifest_path}")
        return

    with open(manifest_path, 'r') as f:
        manifest = json.load(f)

    ALIGN = 16
    bank = bytearray()
    table = []
    man_out = {}

    # Sort files to maintain the A01..A30, B01..B19 ordering
    for fn in sorted(manifest.keys()):
        if not fn.endswith('.brr'):
            continue

        stem = fn[:3].lower()
        if only and stem not in only:
            continue

        brr_path = os.path.join(a.brr, fn)
        if not os.path.exists(brr_path):
            print(f"Warning: {brr_path} not found.")
            continue

        with open(brr_path, 'rb') as f:
            brr_data = f.read()

        meta = manifest[fn]
        samples = meta['samples']
        is_loop = meta['loop']

        # 16-byte align each BRR stream
        pad = (ALIGN - (len(bank) % ALIGN)) % ALIGN
        bank += b'\x00' * pad

        off = len(bank)
        bank += brr_data

        # wav2brr.py sets the loop flag across the whole file when looping, so offset is 0.
        loop_off = 0 if is_loop else 0xFFFFFFFF

        idx_str = fn[1:3]
        idx = int(idx_str, 16) if idx_str.isalnum() else 0
        side = fn[0]

        table.append((fn, idx, side, off, len(brr_data), loop_off, samples))
        man_out[fn] = {
            'record': len(table) - 1,
            'offset': off,
            'bytes': len(brr_data),
            'loop_offset': loop_off if loop_off != 0xFFFFFFFF else None,
            'samples': samples,
            'rate': meta['rate']
        }
        
        ls = f"loop@{loop_off}" if loop_off != 0xFFFFFFFF else "one-shot"
        print(f"{fn}: {samples} smp@{meta['rate']} -> {len(brr_data)}B {ls}")

    # Build the index table (16-byte records)
    pad = (ALIGN - (len(bank) % ALIGN)) % ALIGN
    bank += b'\x00' * pad
    tbl_off = len(bank)

    for (fn, idx, side, off, ln, loop_off, keep) in table:
        bank += struct.pack('<IIII', off, ln, loop_off, keep)

    # Write the binary bank
    with open(os.path.join(a.out, 'sgb_sfx_bank.bin'), 'wb') as f:
        f.write(bank)

    # Write the new JSON manifest
    with open(os.path.join(a.out, 'sgb_sfx_bank.json'), 'w') as f:
        json.dump(man_out, f, indent=1)

    # Write the Verilog Header (.vh)
    vh_lines = [
        "// Auto-generated from pre-compressed BRR files.",
        "// Bank file: sgb_sfx_bank.bin. Record = 16 bytes: {u32 offset,u32 length,u32 loop_offset,u32 sample_count}.",
        "// loop_offset = byte offset of loop-start block within the stream, 0xFFFFFFFF = one-shot.",
        f"`define SGB_SFX_COUNT {len(table)}",
        f"`define SGB_SFX_TABLE_OFF {tbl_off}",
        f"`define SGB_SFX_BANK_BYTES {len(bank)}"
    ]
    for i, (fn, idx, side, off, ln, loop_off, keep) in enumerate(table):
        loop_str = f"0x{loop_off:x}" if loop_off != 0xFFFFFFFF else "0xffffffff"
        vh_lines.append(f"// record {i}: {fn} offset={off} bytes={ln} loop={loop_str} samples={keep}")

    with open(os.path.join(a.out, 'sgb_sfx_bank.vh'), 'w') as f:
        f.write('\n'.join(vh_lines) + '\n')

    tot = sum(t[4] for t in table)
    print(f"\nTOTAL {len(table)} effects | bank {len(bank)} B ({len(bank)/1024:.0f} KB) | BRR {tot} B ({tot/1024:.0f} KB) | table @{tbl_off}")

if __name__ == '__main__':
    main()