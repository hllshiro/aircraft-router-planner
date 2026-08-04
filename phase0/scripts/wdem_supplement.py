#!/usr/bin/env python3
"""World_DEM_L11 补充验证：北京区域采样 + 压缩率小样本估算。"""
import json

import numpy as np
import tifffile

PATH = r"D:\workspace\code_engineer\3rd_party\World_DEM_L11\World_DEM_L11.tif"
TILE = 256

tf = tifffile.TiffFile(PATH)
page = tf.pages[0]
offsets = page.dataoffsets
counts = page.databytecounts
shape = page.shape
rows, cols = shape[0], shape[1]
tiles_per_row = cols // TILE


def read_tile(fh, idx):
    fh.seek(offsets[idx])
    raw = fh.read(counts[idx])
    return np.frombuffer(raw, dtype=np.float32).reshape(TILE, TILE)


def read_region(fh, r0, c0, h, w):
    """按 tile 读任意矩形区域（浮点）。"""
    rt0, rt1 = r0 // TILE, (r0 + h - 1) // TILE
    ct0, ct1 = c0 // TILE, (c0 + w - 1) // TILE
    block = np.zeros((h, w), dtype=np.float32)
    for rt in range(rt0, rt1 + 1):
        for ct in range(ct0, ct1 + 1):
            t = read_tile(fh, rt * tiles_per_row + ct)
            tr, tc = rt * TILE, ct * TILE
            dr0 = max(r0, tr) - r0
            dc0 = max(c0, tc) - c0
            dr1 = min(r0 + h, tr + TILE) - r0
            dc1 = min(c0 + w, tc + TILE) - c0
            block[dr0:dr1, dc0:dc1] = t[dr0 + r0 - tr : dr1 + r0 - tr, dc0 + c0 - tc : dc1 + c0 - tc]
    return block


def lonlat_to_rc(lon, lat):
    """GeoTransform=(-180, px, 0, 90, 0, -px) → 像素行列。"""
    px = 0.0054931640625
    c = int((lon - (-180.0)) / px)
    r = int((90.0 - lat) / px)
    return r, c


with open(PATH, "rb") as fh:
    # 北京 116.4E, 39.9N
    r_bj, c_bj = lonlat_to_rc(116.4, 39.9)
    bj = read_region(fh, r_bj - 128, c_bj - 128, 256, 256)
    fin = np.isfinite(bj)
    print(f"[beijing] row={r_bj} col={c_bj} (lat~39.9N lon~116.4E)")
    print(
        f"  finite={fin.sum()}/{bj.size} ({fin.sum()/bj.size*100:.2f}%) "
        f"min={bj[fin].min():.1f} max={bj[fin].max():.1f} mean={bj[fin].mean():.1f}"
        f" nan={int((~fin).sum())}"
    )

    # 压缩率小样本：5 个代表性 tile（海洋/陆地/北京/南极/北极）
    import pyzstd

    samples = {
        "ocean_north_atlantic": read_tile(fh, 50 * tiles_per_row + 100),
        "land_asia": read_tile(fh, 60 * tiles_per_row + 200),
        "beijing": read_tile(fh, (r_bj // TILE) * tiles_per_row + (c_bj // TILE)),
        "antarctica": read_tile(fh, 127 * tiles_per_row + 150),
        "arctic": read_tile(fh, 0 * tiles_per_row + 150),
    }
    print("\n[compression] zstd-19 小样本 (256x256 float32 = 262144B):")
    ratios = []
    for name, arr in samples.items():
        raw = arr.tobytes()
        comp = pyzstd.compress(raw, 19)
        ratio = len(raw) / len(comp)
        ratios.append(ratio)
        print(f"  {name:>22}: {len(raw)} -> {len(comp)}B ({ratio:.2f}x)")
    mean_ratio = float(np.mean(ratios))
    est_global = 8_590_459_424 / mean_ratio
    print(
        f"\n[estimate] 样本均压缩率 {mean_ratio:.2f}x → 全球 8.59GB 压缩后约 "
        f"{est_global/1e9:.2f} GB（对照 ≤800MB 目标）"
    )
    print(f"  原始文件 {8_590_459_424/1e9:.2f} GB（不含 .ovr 2.34GB）")
