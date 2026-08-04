import numpy as np
import sys

sys.path.insert(0, r"D:\workspace\code_engineer\coding_projects\AircraftRouterPlanner\phase0\scripts")
from convert_to_arpk1 import Jp2Reader

OPJ = r"D:\workspace\code_engineer\3rd_party\gdal-3-12-1-mapserver-8-6-0\bin\opj_decompress.exe"
JP2 = r"D:\workspace\code_engineer\3rd_party\GMTED2010\GMTED2010.jp2"
import tempfile

with tempfile.TemporaryDirectory() as td:
    r = Jp2Reader(JP2, -32768, td, opj=OPJ)
    print("rows,cols:", r.height, r.width)
    print("origin_lon, origin_lat:", r.origin_lon, r.origin_lat)
    print("cell:", r.cell_lon, r.cell_lat)
    # 最南段（ARPK1 r0=0 ↔ 源 56°S 海洋）
    a = r.read_rows(0, 256)
    print("south block: finite%", np.isfinite(a).mean(), "min/max:", np.nanmin(a), np.nanmax(a))
    # 最北段（ARPK1 r0=height-256 ↔ 源 84°N 北极）
    a = r.read_rows(r.height - 256, 256)
    print("north block: finite%", np.isfinite(a).mean(), "min/max:", np.nanmin(a), np.nanmax(a))
    # 北京附近（lat 39.9 → ARPK1 r ≈ 46032）
    r0 = int((39.9 - r.origin_lat) / r.cell_lat) - 128
    a = r.read_rows(r0, 256)
    # 北京列：lon 116.4 → col
    c = int((116.4 - r.origin_lon) / r.cell_lon)
    seg = a[:, c - 32 : c + 32]
    print(f"beijing row r0={r0} col={c}: finite%", np.isfinite(seg).mean(), "min/max:", np.nanmin(seg), np.nanmax(seg))
    print("beijing center val:", a[128, c])
    # 青藏高原（lat 35, lon 90）
    r0 = int((35.0 - r.origin_lat) / r.cell_lat)
    c = int((90.0 - r.origin_lon) / r.cell_lon)
    print("tibet val:", r.read_rows(r0, 1)[0, c])
    r.close()
