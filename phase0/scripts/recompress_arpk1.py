#!/usr/bin/env python3
"""ARPK1 重压缩：读现有 ARPK1 → 逐块 zstd-decompress → zstd level N 重压 → 重写。

保留 288B 定长头原样（source_desc 等），仅替换压缩块与块索引、回填 SHA-256。
块索引 <QI：offset(8B LE) + size(4B LE)，共 n_blocks*12B，位于 HEADER_SIZE+32。
SHA-256 覆盖 idx_start..文件尾（与 builtin.rs writer 规范一致）。

用法: python recompress_arpk1.py <in.arpk1> <out.arpk1> [--zstd-level 19]
"""
import argparse
import hashlib
import struct
import sys
from concurrent.futures import ThreadPoolExecutor

import pyzstd

HEADER_SIZE = 288
SHA_SIZE = 32
IDX_ENTRY = 12  # <QI


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--zstd-level", type=int, default=19)
    args = ap.parse_args()

    data = open(args.src, "rb").read()
    assert data[:8] == b"ARPACK1\0", "not an ARPK1 file"
    idx_start = HEADER_SIZE + SHA_SIZE
    # 块数 = blocks_x * blocks_y（头部 offset 72/76；见 builtin.rs writer 规范）
    blocks_x = struct.unpack("<I", data[72:76])[0]
    blocks_y = struct.unpack("<I", data[76:80])[0]
    n_blocks = blocks_x * blocks_y
    idx_bytes = n_blocks * IDX_ENTRY
    header = data[:HEADER_SIZE]
    entries = [
        struct.unpack("<QI", data[idx_start + i * IDX_ENTRY : idx_start + (i + 1) * IDX_ENTRY])
        for i in range(n_blocks)
    ]
    print(f"[info] {args.src}: {blocks_x}x{blocks_y} = {n_blocks} blocks, {len(data)/1e9:.3f} GB", flush=True)

    # 逐块解压 → 重压（并行压缩，顺序写）
    off = idx_start + idx_bytes
    new_entries = []
    with open(args.dst, "wb") as f:
        f.write(header)
        f.write(b"\0" * SHA_SIZE)
        f.write(b"\0" * idx_bytes)
        CHUNK = 64
        for i0 in range(0, n_blocks, CHUNK):
            chunk = []
            for (o, l) in entries[i0 : i0 + CHUNK]:
                raw = data[o : o + l]
                try:
                    chunk.append(pyzstd.decompress(raw))
                except Exception:
                    chunk.append(raw)  # 解压失败按原样保留（不损坏）
            with ThreadPoolExecutor(max_workers=8) as ex:
                comps = list(ex.map(lambda b: pyzstd.compress(b, args.zstd_level), chunk))
            for k, comp in enumerate(comps):
                new_entries.append((off, len(comp)))
                f.write(comp)
                off += len(comp)
            print(f"[progress] {min(i0+CHUNK, n_blocks)}/{n_blocks}", flush=True)

    # 回填索引
    with open(args.dst, "r+b") as fw:
        for k, (o, l) in enumerate(new_entries):
            fw.seek(idx_start + k * IDX_ENTRY)
            fw.write(struct.pack("<QI", o, l))
        fw.flush()
    # 回填 SHA
    with open(args.dst, "rb") as rf:
        rf.seek(idx_start)
        digest = hashlib.sha256(rf.read()).digest()
    with open(args.dst, "r+b") as fw:
        fw.seek(HEADER_SIZE)
        fw.write(digest)
        fw.flush()

    old = len(data)
    new = off
    print(f"[ok] {args.dst}: {new/1e9:.3f} GB (was {old/1e9:.3f} GB, ratio {old/max(new,1):.2f}x)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
