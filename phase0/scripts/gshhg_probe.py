#!/usr/bin/env python3
"""GSHHG GMT 二进制格式探针（2.0+，big-endian，micro-degrees）。

记录格式（README.TXT 226-249 行确认）：
  header 44B (11 × int32 BE): id, n, flag, west, east, south, north,
                              area, area_full, container, ancestor
  flag: level=flag&255 (1 land / 2 lake / 3 island_in_lake / 4 pond;
        wdb: 1 ocean / 2 lake ...), version=(flag>>8)&255,
        greenwich=(flag>>16)&1, source=(flag>>24)&1, river=(flag>>25)&1
  然后 n × (int32 x, int32 y) = micro-degrees lon/lat
用法: python gshhg_probe.py <file.b>
"""
import struct
import sys

HEADER = struct.Struct(">iiiiiiiiiii")  # id n flag west east south north area area_full container ancestor
POINT = struct.Struct(">ii")

LEVEL_MASK = 0xFF
SCALE = 1e-6


def probe(path: str):
    with open(path, "rb") as f:
        buf = f.read()
    total = len(buf)
    off = 0
    n_records = 0
    n_points = 0
    level_count = {}
    src_wvs = 0
    cross_gw = 0
    cross_am_flag = 0
    river = 0
    versions = set()
    bad_coord = 0
    bad_n = 0
    lon_min, lon_max = 1e9, -1e9
    lat_min, lat_max = 1e9, -1e9
    while off + HEADER.size <= total:
        (rid, n, flag, west, east, south, north,
         area, area_full, container, ancestor) = HEADER.unpack_from(buf, off)
        off += HEADER.size
        n_records += 1
        if rid != n_records - 1:
            if n_records <= 3:
                print(f"[WARN] record {n_records}: id={rid} 不连续（wdb 国界等文件存在 id 空洞，属数据源特性）")
        level = flag & LEVEL_MASK
        level_count[level] = level_count.get(level, 0) + 1
        versions.add((flag >> 8) & 255)
        if flag & (1 << 16):
            cross_gw += 1
        if flag & (1 << 24):
            src_wvs += 1
        if flag & (1 << 25):
            river += 1
        if not (0 < n <= 100_000_000):
            bad_n += 1
            print(f"[ERR] record {n_records}: n={n}")
            return False
        n_points += n
        pts_bytes = n * POINT.size
        if off + pts_bytes > total:
            print(f"[ERR] record {n_records}: n={n} 坐标越界 (剩余 {total - off}B < {pts_bytes}B)")
            return False
        # 坐标表示：GSHHG 多边形 x 坐标可能用 [-180,180) 或 [0,360) 两种惯例
        # （跨格林尼治多边形用 0-360，但 flag bit16 与坐标表示不完全一致，
        #  故用坐标本身判定：合法范围 [-180, 360]，归一化到 [-180,180) 统计）
        step = max(1, n // 500)
        for i in range(0, n, step):
            x, y = POINT.unpack_from(buf, off + i * POINT.size)
            lon, lat = x * SCALE, y * SCALE
            if not (-180.0 <= lon <= 360.0) or not (-90.0 <= lat <= 90.0):
                bad_coord += 1
                if bad_coord <= 3:
                    print(f"[ERR] record {n_records} point {i}: lon={lon} lat={lat} (x={x} y={y})")
            lon_n = lon if lon <= 180.0 else lon - 360.0  # 归一化统计
            lon_min, lon_max = min(lon_min, lon_n), max(lon_max, lon_n)
            lat_min, lat_max = min(lat_min, lat), max(lat_max, lat)
        off += pts_bytes
    print(f"文件: {path} ({total} B)")
    print(f"记录数: {n_records}, 坐标点总数: {n_points}, 平均 {n_points / max(1, n_records):.1f} 点/多边形")
    print(f"flag version byte (release): {sorted(versions)}")
    print(f"level 分布 (1=land/ocean, 2=lake, 3=island_in_lake, 4=pond): {dict(sorted(level_count.items()))}")
    print(f"greenwich 跨: {cross_gw}, WVS source: {src_wvs}, river 标记: {river}")
    print(f"坐标范围: lon [{lon_min:.6f}, {lon_max:.6f}], lat [{lat_min:.6f}, {lat_max:.6f}]")
    print(f"异常: bad_n={bad_n}, bad_coord={bad_coord}, 剩余字节: {total - off}")
    ok = n_records > 0 and bad_n == 0 and bad_coord == 0
    print("== 解析结果:", "VALID" if ok else "INVALID")
    return ok


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python gshhg_probe.py <file.b>")
        sys.exit(2)
    sys.exit(0 if probe(sys.argv[1]) else 1)
