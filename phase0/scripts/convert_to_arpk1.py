#!/usr/bin/env python3
"""DEM → ARPK1 转换工具（开发期；发布版仅消费 ARPK1）。

用法:
  python convert_to_arpk1.py <input> <output.arpk1> [--zstd-level N] [--no-data VAL]
      <input> 支持:
        - GeoTIFF (tifffile 读取, tile/strip 兼容)
        - JP2     (需 opj_decompress 在 PATH 或 SDKShell 环境, 1024x1024 tiled)

输出 ARPK1 与 cli/src/terrain/builtin.rs 的 writer 规范一致:
  magic "ARPACK1\0" + 288B 定长头 + SHA-256 + 块索引 + zstd 压缩块
  块 = 256x256 i16, 行内差分(wrapping_sub), zstd 压缩, 越界填充 no_data
"""
import hashlib
import os
import struct
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pyzstd
import tifffile

MAGIC = b"ARPACK1\0"
FORMAT_VERSION = 1
HEADER_SIZE = 288
BLOCK = 256
COMPRESSION_ZSTD = 1


def _put_u32(b: bytearray, i: int, v: int):
    b[i : i + 4] = struct.pack("<I", v)


def _put_f64(b: bytearray, i: int, v: float):
    b[i : i + 8] = struct.pack("<d", v)


def _diff_encode(block_i16: np.ndarray) -> bytes:
    """行内差分编码 + 返回 LE 字节。"""
    flat = block_i16.reshape(-1).astype(np.int16)
    diff = np.empty_like(flat)
    diff[0] = flat[0]
    diff[1:] = (flat[1:].astype(np.int32) - flat[:-1].astype(np.int32)).astype(np.int16)
    return diff.tobytes()


class GeoTiffReader:
    """逐 256 行读取 GeoTIFF（tile/strip 兼容），按 256x256 块产出 i16。

    统一约定：输入栅格北朝上（像素 (0,0) = 西北角，tie 在左上）；ARPK1 要求
    origin_lat = 最小纬度（南），行序 lat 递增。故内部做行翻转。
    """

    def __init__(self, path: str, no_data: int):
        self.tf = tifffile.TiffFile(path)
        self.page = self.tf.pages[0]
        self.rows, self.cols = self.page.shape
        gt = self.page.tags.get(33550).value
        tie = self.page.tags.get(33922).value
        self.origin_lon = tie[3]
        cell_lon = gt[0]
        cell_lat = gt[1]
        if cell_lat < 0:
            cell_lat = -cell_lat
        self.cell_lon = abs(cell_lon)
        self.cell_lat = cell_lat
        # 北朝上：像素 (0,0) 是最大纬度 → origin_lat = lat0 - (rows-1)*cell_lat
        self.origin_lat = tie[4] - (self.rows - 1) * cell_lat
        self.no_data = no_data
        self._fh = open(path, "rb")
        self._offsets = self.page.dataoffsets
        self._counts = self.page.databytecounts

    def close(self):
        self._fh.close()
        self.tf.close()

    def read_rows(self, r0: int, nrows: int) -> np.ndarray:
        """读取 ARPK1 行 [r0:r0+nrows]（lat 递增）的 nrows x cols float32；越界填 NaN。

        源栅格北朝上，故源行 = rows-1-(r0+nrows-1) 起，再上下翻转。
        """
        h = min(nrows, self.rows - r0)
        src_last = self.rows - 1 - r0  # 源栅格最后一行（南端）
        out = np.zeros((nrows, self.cols), dtype=np.float32)
        for k in range(h):
            src_r = src_last - k
            idx = src_r // self.page.rowsperstrip
            self._fh.seek(self._offsets[idx])
            raw = self._fh.read(self._counts[idx])
            line = np.frombuffer(raw, dtype=np.float32).reshape(-1, self.cols)
            out[k] = line[src_r % self.page.rowsperstrip]
        out[h:] = np.nan
        return out


class Jp2Reader:
    """opj_decompress 分 tile 解码 JP2（1024x1024 tiled，int16 有符号）。"""

    def __init__(self, path: str, no_data: int, tmpdir: str, opj: str = "opj_decompress"):
        self._opj = opj
        # 头部解析（尺寸/原点/像元）
        import struct as st

        with open(path, "rb") as f:
            f.seek(12)
            boxes = {}
            while True:
                pos = f.tell()
                hdr = f.read(8)
                if len(hdr) < 8:
                    break
                (size,) = st.unpack(">I", hdr[:4])
                btype = hdr[4:8]
                if size == 0:
                    boxes[btype] = (pos, 0)
                    break
                boxes[btype] = (pos, size)
                f.seek(pos + size)
            # jp2h -> ihdr
            jp2h_pos = boxes[b"jp2h"][0]
            f.seek(jp2h_pos + 8)
            sub = f.read(boxes[b"jp2h"][1] - 8)
            off = 0
            while off + 8 <= len(sub):
                (size,) = st.unpack(">I", sub[off : off + 4])
                if sub[off + 4 : off + 8] == b"ihdr":
                    ihdr = sub[off + 8 : off + size]
                off += size
            self.height, self.width = st.unpack(">II", ihdr[:8])
        # GeoJP2 uuid 内嵌 TIFF 地理信息（同 GeoTIFF 惯例：北朝上，tie 在左上角）
        from _geojp2_geo import geo_from_uuid

        self.origin_lon, lat_top, self.cell_lon, self.cell_lat = geo_from_uuid(path)
        self.origin_lat = lat_top - (self.height - 1) * self.cell_lat
        self.rows = self.height  # 兼容 write_arpk1（rows/cols）
        self.cols = self.width
        self.no_data = no_data
        self.path = path
        self.tmpdir = tmpdir
        self._row_cache = None  # (ty, np.ndarray int16 (tile_h, width))

    def read_rows(self, r0: int, nrows: int) -> np.ndarray:
        """读取 ARPK1 行 [r0:r0+nrows]（lat 递增）的 nrows x width int16；越界填 NaN。

        源栅格北朝上：ARPK1 行 r ↔ 源行 rows-1-r。按源 tile 行整体解码并缓存
        （一次 1024 行全宽），供相邻 read_rows 复用。
        """
        out = np.zeros((nrows, self.width), dtype=np.float32)
        out[:] = np.nan
        src_r0 = self.height - 1 - (r0 + nrows - 1)
        src_r1 = self.height - 1 - r0
        t0 = src_r0 // 1024
        t1 = src_r1 // 1024
        for ty in range(t0, t1 + 1):
            row = self._load_row(ty)  # int16 (tile_h, width)，tile_h<=1024
            tr0 = max(src_r0, ty * 1024)
            tr1 = min(src_r1 + 1, (ty + 1) * 1024)
            if tr1 <= tr0:
                continue
            seg = row[tr0 - ty * 1024 : tr1 - ty * 1024, :].astype(np.float32)
            ar0 = self.height - 1 - (tr1 - 1)
            ar1 = self.height - 1 - tr0
            out[ar0 - r0 : ar1 - r0 + 1, :] = seg[::-1, :]
        return out

    def close(self):
        self._row_cache = None
        self.path = None

    def _xtiles(self):
        return (self.width + 1023) // 1024

    def _load_row(self, ty: int):
        if self._row_cache is not None and self._row_cache[0] == ty:
            return self._row_cache[1]
        xt = self._xtiles()
        tile_h = min(1024, self.height - ty * 1024)
        row = np.zeros((tile_h, self.width), dtype=np.int16)
        # 并行解码该 tile 行全部 tile
        with ThreadPoolExecutor(max_workers=8) as ex:
            futures = {
                ex.submit(self._decode_tile, ty, tx): tx for tx in range(xt)
            }
            for fut in futures:
                tx = futures[fut]
                a = fut.result()
                w = min(1024, self.width - tx * 1024)
                row[:, tx * 1024 : tx * 1024 + w] = a[:, :w]
        self._row_cache = (ty, row)
        return row

    def _decode_tile(self, ty: int, tx: int):
        tile_no = ty * self._xtiles() + tx
        out = os.path.join(self.tmpdir, f"t{tile_no}.tif")
        r = subprocess.run(
            [self._opj, "-i", self.path, "-o", out, "-t", str(tile_no), "-threads", "ALL_CPUS"],
            capture_output=True, text=True,
        )
        if r.returncode != 0:
            raise RuntimeError(f"opj_decompress tile {tile_no} failed: {r.stderr[-300:]}")
        a = tifffile.imread(out)
        a = (a.astype(np.int32) - 32768).astype(np.int16)  # openjpeg 无符号偏置还原
        h = min(1024, self.height - ty * 1024)
        w = min(1024, self.width - tx * 1024)
        return a[:h, :w]


def write_arpk1(reader, out_path: str, zstd_level: int, source_desc: str, vertical_datum_egm96: bool):
    rows, cols = reader.rows, reader.cols
    blocks_x = (rows + BLOCK - 1) // BLOCK
    blocks_y = (cols + BLOCK - 1) // BLOCK
    n_blocks = blocks_x * blocks_y
    idx_start = HEADER_SIZE + 32
    idx_bytes = n_blocks * 12

    out = bytearray(idx_start + idx_bytes)
    out[0:8] = MAGIC
    _put_u32(out, 8, FORMAT_VERSION)
    _put_u32(out, 12, 1)  # data_version
    _put_u32(out, 16, rows)
    _put_u32(out, 20, cols)
    _put_f64(out, 24, reader.origin_lon)
    _put_f64(out, 32, reader.origin_lat)
    _put_f64(out, 40, reader.cell_lon)
    _put_f64(out, 48, reader.cell_lat)
    _put_f64(out, 56, 1.0)  # z_resolution_m（垂直精度元数据）
    out[64] = 0 if not vertical_datum_egm96 else 1  # 0=ellipsoid, 1=EGM96
    out[65] = 0  # equiangular
    out[66] = COMPRESSION_ZSTD
    out[67] = 0
    _put_u32(out, 68, BLOCK)
    _put_u32(out, 72, blocks_x)
    _put_u32(out, 76, blocks_y)
    out[80:82] = struct.pack("<h", reader.no_data)
    src = source_desc.encode()[:174]
    out[82 : 82 + len(src)] = src

    off = idx_start + idx_bytes
    idx_entries = []
    with open(out_path, "wb") as f:
        f.write(out)  # 头 + 索引占位（索引稍后回填）
        for bx in range(blocks_x):
            r0 = bx * BLOCK
            if r0 >= rows:
                # 块全越界（rows 非 256 倍数时最后一行块不存在此分支）
                pass
            nrows = min(BLOCK, rows - r0)
            rows_data = reader.read_rows(r0, nrows)
            # 内层按 32 块一批：并行压缩（pyzstd 释放 GIL），顺序写出
            by = 0
            while by < blocks_y:
                chunk = []
                for by2 in range(by, min(by + 32, blocks_y)):
                    c0 = by2 * BLOCK
                    ncols = min(BLOCK, cols - c0)
                    block = np.full((BLOCK, BLOCK), reader.no_data, dtype=np.float32)
                    block[:nrows, :ncols] = rows_data[:nrows, c0 : c0 + ncols]
                    i16 = np.where(np.isfinite(block), np.clip(np.round(block), -32768, 32767), reader.no_data).astype(np.int16)
                    chunk.append(_diff_encode(i16))
                with ThreadPoolExecutor(max_workers=8) as ex:
                    comps = list(ex.map(lambda p: pyzstd.compress(p, zstd_level), chunk))
                for k, comp in enumerate(comps):
                    idx_entries.append((off, len(comp)))
                    f.write(comp)
                    off += len(comp)
                by += 32
    # 回填块索引（r+b，先定位再写；避免在缓冲读模式混写）
    with open(out_path, "r+b") as fw:
        for k, (o, l) in enumerate(idx_entries):
            fw.seek(idx_start + k * 12)
            fw.write(struct.pack("<QI", o, l))
        fw.flush()
    # 回填 SHA-256（数据部分 = idx_start..文件尾）
    with open(out_path, "rb") as rf:
        rf.seek(idx_start)
        data_part = rf.read()
    digest = hashlib.sha256(data_part).digest()
    with open(out_path, "r+b") as fw:
        fw.seek(HEADER_SIZE)
        fw.write(digest)
    print(f"[ok] {out_path}: {rows}x{cols} -> {off/1e9:.3f} GB")


def main() -> None:
    src = sys.argv[1]
    dst = sys.argv[2]
    zstd_level = 19
    if "--zstd-level" in sys.argv:
        zstd_level = int(sys.argv[sys.argv.index("--zstd-level") + 1])
    no_data = -32768
    if "--no-data" in sys.argv:
        no_data = int(sys.argv[sys.argv.index("--no-data") + 1])
    opj = "opj_decompress"
    if "--opj" in sys.argv:
        opj = sys.argv[sys.argv.index("--opj") + 1]

    if src.lower().endswith((".jp2", ".j2k")):
        with tempfile.TemporaryDirectory() as td:
            reader = Jp2Reader(src, no_data, td, opj=opj)
            write_arpk1(reader, dst, zstd_level, "GMTED2010 7.5arcsec median (USGS, JP2 lossy)", True)
            reader.close()
    else:
        reader = GeoTiffReader(src, no_data)
        write_arpk1(reader, dst, zstd_level, os.path.basename(src), True)
        reader.close()


if __name__ == "__main__":
    sys.exit(main())
