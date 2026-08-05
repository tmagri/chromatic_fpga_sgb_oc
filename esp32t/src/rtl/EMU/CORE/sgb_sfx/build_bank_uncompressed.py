#!/usr/bin/env python3
import os
import json
import struct
import argparse
import wave

def main():
    ap = argparse.ArgumentParser(description="Pack existing WAV files into an uncompressed stereo bank.")
    ap.add_argument('--wav', default='wav', help="Directory containing .wav files")
    ap.add_argument('--out', default='bankUncompressed', help="Output directory")
    ap.add_argument('--only', default='', help='comma-separated effect stems to include, e.g. A0C,B04')
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    only = set(s.strip().lower() for s in a.only.split(',') if s.strip())

    ALIGN = 16
    bank = bytearray()
    table = []
    man_out = {}

    if not os.path.exists(a.wav):
        print(f"WAV directory not found at {a.wav}")
        return

    # Sort files to maintain the A01..A30, B01..B19 ordering
    for fn in sorted(os.listdir(a.wav)):
        if not fn.lower().endswith('.wav'):
            continue

        stem = fn[:3].lower()
        if only and stem not in only:
            continue

        wav_path = os.path.join(a.wav, fn)
        
        with wave.open(wav_path, 'rb') as w:
            n_channels = w.getnchannels()
            samp_width = w.getsampwidth()
            rate = w.getframerate()
            n_frames = w.getnframes()
            wav_data = w.readframes(n_frames)

        # Ensure we are dealing with stereo 16-bit PCM
        if n_channels != 2 or samp_width != 2:
            print(f"Warning: {fn} is not 16-bit stereo. Channels: {n_channels}, Width: {samp_width}")

        samples = n_frames
        # B-side effects are sustained loops
        is_loop = fn.upper().startswith('B')

        # 16-byte align each WAV stream
        pad = (ALIGN - (len(bank) % ALIGN)) % ALIGN
        bank += b'\x00' * pad

        off = len(bank)
        bank += wav_data

        # Offset is 0 for looped files, 0xFFFFFFFF for one-shots
        loop_off = 0 if is_loop else 0xFFFFFFFF

        idx_str = fn[1:3]
        idx = int(idx_str, 16) if idx_str.isalnum() else 0
        side = fn[0].upper()

        table.append((fn, idx, side, off, len(wav_data), loop_off, samples))
        man_out[fn] = {
            'record': len(table) - 1,
            'offset': off,
            'bytes': len(wav_data),
            'loop_offset': loop_off if loop_off != 0xFFFFFFFF else None,
            'samples': samples,
            'rate': rate
        }
        
        ls = f"loop@{loop_off}" if loop_off != 0xFFFFFFFF else "one-shot"
        print(f"{fn}: {samples} smp@{rate} -> {len(wav_data)}B {ls}")

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
        "// Auto-generated from uncompressed stereo WAV files.",
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
    print(f"\nTOTAL {len(table)} effects | bank {len(bank)} B ({len(bank)/1024:.0f} KB) | WAV Data {tot} B ({tot/1024:.0f} KB) | table @{tbl_off}")

if __name__ == '__main__':
    main()