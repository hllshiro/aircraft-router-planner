#!/usr/bin/env python3
"""GMTED2010.jp2 验证工具（复用完整验证流程）。

用法:
  python gmted_verify.py <jp2_path> <opj_decompress路径或SDK环境已配置> [--no-zstd]

纯 Python 头部解析（JP2 box + codestream SIZ/COD）+ OpenJPEG 区域解码验证：
  - 分辨率/覆盖/位深/无损有损（头部）
  - 北京/青藏/海洋三点区域解码（值域/偏置验证）
  - 全图 r4 降采样统计（空洞率/值域/海洋占比）
  - zstd-19 压缩率样本外推（ARPK1 体积估算）
"""
import os
import subprocess
import sys

import numpy as np
import pyzstd
import tifffile

TMP = os.path.join(os.path.dirname(__file__), "..", "data", "gmted_verify_tmp")
OS = os.name


def parse_header(path):
    """零依赖头部解析：JP2 boxes + SIZ + COD。"""
    import struct

    with open(path, "rb") as f:
        head = f.read(12)
        assert head[4:8] == b"jP  ", "not JP2"
        f.seek(12)
        boxes = {}
        while True:
            pos = f.tell()
            hdr = f.read(8)
            if len(hdr) < 8:
                break
            (size,) = struct.unpack(">I", hdr[:4])
            btype = hdr[4:8]
            if size == 0:
                boxes[btype] = (pos, 0, 8)
                break
            boxes[btype] = (pos, size, 8)
            f.seek(pos + size)
        # jp2h -> ihdr
        jp2h_pos = boxes[b"jp2h"][0]
        f.seek(jp2h_pos + 8)
        sub = f.read(boxes[b"jp2h"][1] - 8)
        off = 0
        while off + 8 <= len(sub):
            (size,) = struct.unpack(">I", sub[off : off + 4])
            btype = sub[off + 4 : off + 8]
            if btype == b"ihdr":
                ihdr = sub[off + 8 : off + size]
            off += size
        height, width, nc, bpc = struct.unpack(">IHHB", ihdr[:11])
        # codestream SIZ/COD
        cs_pos = boxes[b"jp2c"][0] + 8
        f.seek(cs_pos)
        cs = f.read(8192)
        i = 2
        info = {"siz": None, "cod": None}
        while i + 4 < len(cs) and cs[i] == 0xFF:
            m = cs[i + 1]
            if m == 0xD9:
                break
            if m in (0x93, 0x4F):
                i += 2
                continue
            (L,) = struct.unpack(">H", cs[i + 2 : i + 4])
            seg = cs[i + 4 : i + 4 + L - 2]
            if m == 0x51:
                info["siz"] = {
                    "Xsiz": struct.unpack(">I", seg[2:6])[0],
                    "Ysiz": struct.unpack(">I", seg[6:10])[0],
                    "XTsiz": struct.unpack(">I", seg[18:22])[0],
                    "YTsiz": struct.unpack(">I", seg[22:26])[0],
                    "prec": (seg[36] & 0x7F) + 1,
                    "signed": bool(seg[36] & 0x80),
                }
            elif m == 0x52:
                info["cod"] = {"transform": (seg[0] >> 3) & 1, "layers": struct.unpack(">H", seg[2:4])[0]}
            if info["cod"]:
                break
            i += 2 + L - 2
    return {
        "width": width,
        "height": height,
        "ihdr_bpc": bpc,
        "siz": info["siz"],
        "cod": info["cod"],
        "arcsec": 360 * 3600 / width,
        "m_eq": 360 * 3600 / width * 111320 / 3600,
    }


def run_opj(jp2, out_tif, region=None, reduce=None, opj="opj_decompress", extra_env=None):
    cmd = [opj, "-i", jp2, "-o", out_tif, "-threads", "ALL_CPUS"]
    if region:
        cmd += ["-d", ",".join(map(str, region))]
    if reduce:
        cmd += ["-r", str(reduce)]
    env = dict(os.environ)
    if extra_env:
        env.update(extra_env)
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise RuntimeError(f"opj_decompress failed: {r.stderr[-500:]}")
    return out_tif


def lonlat_to_rc(lon, lat):
    px = 0.00208333333333334
    c = int(round((lon + 180.00013888888893) / px))
    r = int(round((83.9998611111112 - lat) / px))
    return r, c


def main() -> None:
    jp2 = sys.argv[1] if len(sys.argv) > 1 else r"D:\workspace\code_engineer\3rd_party\GMTED2010\GMTED2010.jp2"
    opj = sys.argv[2] if len(sys.argv) > 2 else "opj_decompress"
    do_zstd = "--no-zstd" not in sys.argv
    os.makedirs(TMP, exist_ok=True)

    h = parse_header(jp2)
    print("== 头部解析 ==")
    print(f"  尺寸: {h['width']}x{h['height']}  分辨率: {h['arcsec']:.3f} 弧秒 ≈ {h['m_eq']:.1f}m(赤道)")
    print(f"  IHDR bpc={h['ihdr_bpc']} (生成器标记，非真值)  SIZ: {h['siz']}")
    print(f"  COD: transform_bit={h['cod']['transform']} -> "
          f"{'5/3 无损' if h['cod']['transform'] else '9/7 有损'}  层数={h['cod']['layers']}")

    # 三点区域验证（北京/青藏/太平洋海洋）
    print("\n== 区域解码（OpenJPEG）==")
    points = {"bj": (116.4, 39.9), "tibet": (90.0, 30.0), "ocean": (-170.0, 0.0)}
    results = {}
    for name, (lon, lat) in points.items():
        r0, c0 = lonlat_to_rc(lon, lat)
        region = (c0 - 256, r0 - 256, c0 + 256, r0 + 256)
        tif = os.path.join(TMP, f"{name}.tif")
        run_opj(jp2, tif, region=region, opj=opj)
        a = tifffile.imread(tif)
        hh = a.astype(np.int32) - 32768
        print(f"  {name:6} ({lon},{lat}): raw {a.min()}..{a.max()} -> 高程 {hh.min()}..{hh.max()}m "
              f"finite={np.isfinite(a).mean():.2f}")
        results[name] = a

    # 全图 r4 统计
    print("\n== 全图 r4 降采样统计 ==")
    r4 = os.path.join(TMP, "global_r4.tif")
    run_opj(jp2, r4, reduce=4, opj=opj)
    a = tifffile.imread(r4)
    hh = a.astype(np.int32) - 32768
    ocean = (a == 32768)
    print(f"  r4 尺寸 {a.shape}  高程 {hh.min()}..{hh.max()}m")
    print(f"  海洋(0m)占比 {ocean.mean()*100:.2f}%  陆地占比 {(~ocean).mean()*100:.2f}%")
    for v in [0, 1, 65535, 65534]:
        if (a == v).sum():
            print(f"  ! 异常值 {v}: {(a == v).sum()} px")
    print(f"  负高程占比 {(hh<0).mean()*100:.4f}% (min {hh.min()})")

    # zstd 体积估算
    if do_zstd:
        print("\n== zstd-19 体积估算（ARPK1 转存参考）==")
        ratios = {}
        for name in ["bj", "tibet", "ocean"]:
            arr = results[name]
            comp = pyzstd.compress(arr.astype(np.uint16).tobytes(), 19)
            ratios[name] = arr.nbytes / len(comp)
            print(f"  {name:6}: {ratios[name]:.2f}x")
        raw = h["width"] * h["height"] * 2
        frac_land = (~ocean).mean()
        land_r = ratios.get("bj", 2.5)
        ocean_r = ratios.get("ocean", 100)
        est = raw * (frac_land / land_r + (1 - frac_land) / ocean_r)
        print(f"  全图 raw {raw/1e9:.2f}GB -> zstd-19 ≈ {est/1e9:.2f}GB "
              f"(陆地 {frac_land*100:.1f}% @ {land_r:.2f}x + 海洋 @ {ocean_r:.1f}x)")


if __name__ == "__main__":
    sys.exit(main())
