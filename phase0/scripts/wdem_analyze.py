#!/usr/bin/env python3
"""World_DEM_L11 真实分辨率/像素类型/空洞率/值域分析（正式数据验证流程）。

读取 tif 头部（尺寸/类型/压缩/GeoTransform）+ 分区域采样统计。
不整读 8.59GB 文件，仅按需读子块。
"""
import json
import sys

import numpy as np
import tifffile

PATH = r"D:\workspace\code_engineer\3rd_party\World_DEM_L11\World_DEM_L11.tif"


def main() -> None:
    tf = tifffile.TiffFile(PATH)
    page = tf.pages[0]
    shape = page.shape
    dtype = page.dtype
    print(f"[header] shape={shape} dtype={dtype}")
    print(f"[header] compression={page.compression} planar={page.planarconfig}")

    # GeoTransform（TIFF GeoTIFF 标签）
    px_scale = page.tags.get(33550)  # ModelPixelScaleTag
    tiepoint = page.tags.get(33922)  # ModelTiepointTag
    gt = None
    if px_scale is not None and tiepoint is not None:
        scale = px_scale.value
        tie = tiepoint.value
        # tie: [i, j, k, x, y, z]
        gt = (tie[3], scale[0], 0.0, tie[4], 0.0, -scale[1])
        print(f"[header] ModelPixelScale={scale} (deg/px)")
        print(f"[header] ModelTiepoint={tie}")
        print(f"[header] GeoTransform={gt}")
        px_deg = scale[0]
        px_arcsec = px_deg * 3600.0
        px_m_equator = px_deg * 111_320.0
        print(
            f"[resolution] {px_deg:.9f} deg/px = {px_arcsec:.3f} arcsec ≈ {px_m_equator:.2f} m (equator)"
        )
    else:
        print("[header] no GeoTIFF tags (px_scale/tiepoint missing)")

    # NoData tag
    nd = page.tags.get(42113)  # GDAL_NODATA
    print(f"[header] GDAL_NODATA={nd.value if nd is not None else None}")

    # 分区域采样统计（8 个区域，512x512 每块；按 tile 手动读取，不整读 8.59GB）
    # tile 布局：256x256 float32 = 262144B；序号 = row_tile * tiles_per_row + col_tile
    import struct

    tile_size = 256
    with open(PATH, "rb") as fh:
        offsets = page.dataoffsets
        counts = page.databytecounts

        rows, cols = shape[0], shape[1]
        tiles_per_row = cols // tile_size

        def read_tile(tile_idx):
            off = offsets[tile_idx]
            cnt = counts[tile_idx]
            fh.seek(off)
            raw = fh.read(cnt)
            return np.frombuffer(raw, dtype=np.float32).reshape(tile_size, tile_size)

        regions = [
            (0, 0),
            (0, cols // 2),
            (0, cols - 512),
            (rows // 2, 0),
            (rows // 2, cols // 2),
            (rows - 512, 0),
            (rows - 512, cols // 2),
            (rows - 512, cols - 512),
        ]
        print("\n[regions] 8 x 512x512 samples:")
        all_stats = []
        for (r0, c0) in regions:
            r0 = min(max(r0, 0), rows - 512)
            c0 = min(max(c0, 0), cols - 512)
            # 覆盖区域的 tile 范围
            rt0, rt1 = r0 // tile_size, (r0 + 512 - 1) // tile_size
            ct0, ct1 = c0 // tile_size, (c0 + 512 - 1) // tile_size
            tiles = {}
            for rt in range(rt0, rt1 + 1):
                for ct in range(ct0, ct1 + 1):
                    tiles[(rt, ct)] = read_tile(rt * tiles_per_row + ct)
            block = np.zeros((512, 512), dtype=np.float32)
            for rt in range(rt0, rt1 + 1):
                for ct in range(ct0, ct1 + 1):
                    tr = rt * tile_size
                    tc = ct * tile_size
                    dr0 = max(r0, tr) - r0
                    dc0 = max(c0, tc) - c0
                    dr1 = min(r0 + 512, tr + tile_size) - r0
                    dc1 = min(c0 + 512, tc + tile_size) - c0
                    src = tiles[(rt, ct)]
                    block[dr0:dr1, dc0:dc1] = src[
                        dr0 + r0 - tr : dr1 + r0 - tr, dc0 + c0 - tc : dc1 + c0 - tc
                    ]
            block = block.astype(np.float64)
            finite = np.isfinite(block)
            n_finite = int(finite.sum())
            if n_finite == 0:
                stats = {
                    "region": [r0, c0],
                    "n": 512 * 512,
                    "n_finite": 0,
                    "zero_pct": float((block == 0).sum() / block.size * 100.0),
                    "min": None,
                    "max": None,
                    "mean": None,
                }
            else:
                z = block[finite]
                zero_pct = float((z == 0).sum() / z.size * 100.0)
                stats = {
                    "region": [r0, c0],
                    "n": 512 * 512,
                    "n_finite": n_finite,
                    "zero_pct": zero_pct,
                    "min": float(z.min()),
                    "max": float(z.max()),
                    "mean": float(z.mean()),
                }
            all_stats.append(stats)
            print(
                f"  r{r0:>6} c{c0:>6}: finite={n_finite:>7}/{512*512} "
                f"zero={stats['zero_pct']:.2f}% min={stats['min']} "
                f"max={stats['max']} mean={stats['mean']}"
            )

    total = sum(s["n"] for s in all_stats)
    nfinite = sum(s["n_finite"] for s in all_stats)
    zeros = sum(s["zero_pct"] * s["n"] / 100.0 for s in all_stats)
    print(
        f"\n[summary] finite={nfinite}/{total} ({nfinite/total*100:.2f}%) "
        f"zero(==0)={zeros/total*100:.2f}% (含 NoData 或真实海平面)"
    )

    out = {
        "path": PATH,
        "shape": list(shape),
        "dtype": str(dtype),
        "compression": str(page.compression),
        "geotransform": gt,
        "px_deg": px_deg if gt else None,
        "px_arcsec": px_arcsec if gt else None,
        "px_m_equator": px_m_equator if gt else None,
        "nodata_tag": nd.value if nd is not None else None,
        "samples": all_stats,
    }
    with open(r"D:\workspace\code_engineer\coding_projects\AircraftRouterPlanner\phase0\wdem_analyze_out.json", "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("\n[out] phase0/wdem_analyze_out.json written")


if __name__ == "__main__":
    sys.exit(main())
