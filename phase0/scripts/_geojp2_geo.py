#!/usr/bin/env python3
"""从 JP2 的 GeoJP2 uuid box（内嵌 TIFF IFD）解析地理信息。

返回 (origin_lon, origin_lat, cell_lon_deg, cell_lat_deg)。
"""
import struct


def geo_from_uuid(path: str):
    with open(path, "rb") as f:
        f.seek(12)
        uuid_pos = None
        while True:
            pos = f.tell()
            hdr = f.read(8)
            if len(hdr) < 8:
                break
            (size,) = struct.unpack(">I", hdr[:4])
            btype = hdr[4:8]
            if size == 0:
                break
            if btype == b"uuid":
                uuid_pos = pos
                break
            f.seek(pos + size)
        if uuid_pos is None:
            raise RuntimeError("no uuid box (GeoJP2)")
        f.seek(uuid_pos)
        (size,) = struct.unpack(">I", f.read(4))
        payload = f.read(size - 8)[16:]  # 跳过 usertype

    # 定位内嵌 TIFF 头（部分文件带 4 字节前缀，如 GMTED2010 的 d5a6ce03）
    le = payload.find(b"II*\x00")
    be = payload.find(b"MM\x00*")
    if le >= 0 and (be < 0 or le < be):
        endian = "<"
        tiff_off = le
    elif be >= 0:
        endian = ">"
        tiff_off = be
    else:
        raise RuntimeError("GeoJP2 payload contains no TIFF header")
    (ifd_off,) = struct.unpack(endian + "I", payload[tiff_off + 4 : tiff_off + 8])
    (count,) = struct.unpack(endian + "H", payload[tiff_off + ifd_off : tiff_off + ifd_off + 2])

    def read_double(off):
        return struct.unpack(endian + "d", payload[tiff_off + off : tiff_off + off + 8])[0]

    scale = None
    tie = None
    for i in range(count):
        e = payload[tiff_off + ifd_off + 2 + i * 12 : tiff_off + ifd_off + 14 + i * 12]
        tag, typ, cnt = struct.unpack(endian + "HHI", e[:8])
        raw = e[8:12]
        if typ == 12:  # DOUBLE
            off = struct.unpack(endian + "I", raw[:4])[0]
            if tag == 33550:  # ModelPixelScale
                scale = (read_double(off), read_double(off + 8))
            elif tag == 33922:  # ModelTiepoint
                tie = (
                    read_double(off + 0),
                    read_double(off + 8),
                    read_double(off + 16),
                    read_double(off + 24),
                    read_double(off + 32),
                    read_double(off + 40),
                )
    if scale is None or tie is None:
        raise RuntimeError("GeoJP2 missing ModelPixelScale/ModelTiepoint")
    # tie = (r=0, c=0, z=0, lon, lat, z) → 原点像素 (0,0) 对应 (lon, lat)
    origin_lon = tie[3]
    origin_lat = tie[4]
    cell_lon = scale[0]
    cell_lat = scale[1]
    return origin_lon, origin_lat, cell_lon, cell_lat
