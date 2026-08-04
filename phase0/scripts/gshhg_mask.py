#!/usr/bin/env python3
"""GSHHG → 海陆掩膜构建器（10 弧秒，逐多边形扫描线，RLE 存储，3 态）。

用法:
  python gshhg_mask.py <gshhs_f.b> <out.mask> [--arcsec 10] [--limit N]

语义（3 态）:
  class 0 = 海洋（默认，海平面高程 0）
  class 1 = 陆地（含南极冰盖 level 5/6、南极内陆补全 -85.15°S 以南）
  class 2 = 内陆湖（湖泊洞 level 2/4；湖面高程≠0，需 DEM 提供）

算法:
  1. 解析 GSHHG 2.0+ 二进制（44B 头 11×int32 BE + micro-degrees int32 坐标）
  2. 正区 = level 1/3/5/6（陆地）并集；负区 = level 2/4（湖泊洞）并集
     陆地 = 正区 − 负区；内陆湖 = 负区 ∩ 正区；其余 = 海洋
  3. 坐标统一到 -180..180 线性表示（greenwich 多边形 x>180 减 360，跨 ±180 边替换）
     —— 0-360 表示会破坏真实经度顺序导致交点排序错位
  4. 逐多边形奇偶填充（并列大陆用并集合并，避免全局奇偶配对把海洋填成陆地）
  5. RLE 行存储：每行 [nseg, (u8 class, u32 start, u32 end)×nseg] + 行偏移索引表
"""
import struct
import sys
import time

MAGIC = b"ARPACK_MASK_V2__"  # 固定 16B
HEADER = struct.Struct(">16sI III II ddd")  # magic + version + arcsec rows cols + lon0 lat0 + reserved
HDR_SIZE = 64

SCALE = 1e-6
POS_LEVELS = (1, 3, 5, 6)  # 陆地语义
NEG_LEVELS = (2, 4)        # 湖泊语义（洞）


def read_polys(path: str):
    """返回 [(level, greenwich, [(x_deg, y_deg), ...])]。
    greenwich=1: x 用 [0,360) 表示且已含 x=360 边界点（GSHHG 在 0 经线切开）；
    greenwich=0: x 用 [-180,180) 表示（反经线多边形含 ±180 边界点）。"""
    with open(path, "rb") as f:
        buf = f.read()
    hdr = struct.Struct(">iiiiiiiiiii")  # id n flag west east south north area area_full container ancestor
    pt = struct.Struct(">ii")
    polys = []
    off = 0
    total = len(buf)
    while off + hdr.size <= total:
        (rid, n, flag, west, east, south, north,
         area, area_full, container, ancestor) = hdr.unpack_from(buf, off)
        off += hdr.size
        level = flag & 0xFF
        greenwich = (flag >> 16) & 1
        pts = []
        for i in range(n):
            x, y = pt.unpack_from(buf, off)
            off += pt.size
            pts.append((x * SCALE, y * SCALE))
        polys.append((level, greenwich, pts))
    return polys


def norm_x(x: float) -> float:
    """归一化到 [0, 360)。"""
    return x % 360.0


def add_edge(edges, x1, y1, x2, y2, res_deg, pidx=-1):
    """边 (x1,y1)-(x2,y2)（度，x∈[0,360)）加入边表。
    行范围用闭区间 [r_lo, r_hi]（含上界行），y 半开区间过滤在扫描时做。
    注意：不能把 yhi 所在行排除——边 [ylo, yhi) 的 yhi 行中心可能仍在边内。"""
    if y1 == y2:
        return  # 水平边不参与扫描线交点（奇偶填充不需要水平边）
    ylo, yhi = min(y1, y2), max(y1, y2)
    r_lo = max(0, int((ylo + 90.0) / res_deg))
    r_hi = int((yhi + 90.0) / res_deg) + 1  # 含上界行（半开区间在 t 过滤时处理）
    if r_hi > 64800:
        r_hi = 64800
    if r_lo >= r_hi:
        return
    edges.append((r_lo, r_hi, x1, y1, x2, y2, pidx))


def merge_segs(segs, cols):
    """段并集合并（排序 + 合并重叠/相邻）。"""
    segs.sort()
    out = []
    for c0, c1 in segs:
        if c0 < 0:
            c0 = 0
        if c1 > cols:
            c1 = cols
        if c1 <= c0:
            continue
        if out and out[-1][1] >= c0:
            out[-1] = (out[-1][0], max(out[-1][1], c1))
        else:
            out.append((c0, c1))
    return out


def subtract_segs(pos, neg):
    """正区段 − 负区段（挖空湖泊等洞区域）。"""
    out = []
    ni = 0
    for c0, c1 in pos:
        cur = c0
        while ni < len(neg) and neg[ni][1] <= cur:
            ni += 1
        j = ni
        while j < len(neg) and neg[j][0] < c1:
            if neg[j][0] > cur:
                out.append((cur, neg[j][0]))
            cur = max(cur, neg[j][1])
            j += 1
        if cur < c1:
            out.append((cur, c1))
    return out


def build(rows, cols, res_deg, polys, progress_every=20000):
    """逐多边形实心填充（奇偶），行内分正/负区并集，最后差集。
    正区：level 1 陆地 + 3 岛中湖 + 5/6 南极冰（陆地语义）
    负区：level 2 湖泊 + 4 pond（洞，挖空）
    —— 并列大陆多边形（欧亚/非洲/北美…）用并集合并而非全局奇偶配对，
    避免把大陆之间的海洋错误填充。

    坐标系：统一输出 0-360 列语义（列 c ∈ [0, cols) ↔ 经度 c*res）。
    greenwich 多边形 x 已 0-360（含 360 边界点）→ 列直接 x/res；
    普通多边形 x ∈ [-180,180) → 列 (x+180)/res 转 0-360 语义（跨 0 段分裂）。
    不做全局归一化——归一化会让真实连续的多边形（如非洲 -17..51）
    变成跨 0 环，单环奇偶填充会把内部方向弄反。"""
    POS_LEVELS = (1, 3, 5, 6)
    edges = []  # (r_lo, r_hi, x1, y1, x2, y2, pidx)
    n_poly = 0
    for pidx, (level, greenwich, pts) in enumerate(polys):
        n_poly += 1
        m = len(pts)
        if m < 3:
            continue
        # 统一到 -180..180 线性表示：greenwich 多边形 0-360 → x>180 减 360。
        # 0-360 表示会破坏真实经度顺序（-6.25 变 353.75），导致交点排序错位。
        if greenwich:
            pts_use = [(x - 360.0 if x > 180.0 else x, y) for (x, y) in pts]
        else:
            pts_use = pts
        prev = pts_use[-1]
        for i in range(m):
            x1, y1 = prev
            x2, y2 = pts_use[i]
            if abs(x2 - x1) > 180.0:
                # 跨 ±180 线（greenwich 多边形转 -180..180 后北海岸等长边）：替换为两段
                y180 = y1 + (180.0 - x1) * (y2 - y1) / (x2 - x1)
                y_neg = y1 + (-180.0 - x1) * (y2 - y1) / (x2 - x1)
                if x1 > x2:
                    add_edge(edges, x1, y1, 180.0, y180, res_deg, pidx)
                    add_edge(edges, -180.0, y_neg, x2, y2, res_deg, pidx)
                else:
                    add_edge(edges, x1, y1, -180.0, y_neg, res_deg, pidx)
                    add_edge(edges, 180.0, y180, x2, y2, res_deg, pidx)
            else:
                add_edge(edges, x1, y1, x2, y2, res_deg, pidx)
            prev = pts_use[i]
        if n_poly % progress_every == 0:
            print(f"  ... 多边形 {n_poly}, 边 {len(edges)}")
    edges.sort(key=lambda e: e[0])
    row_segs = [[] for _ in range(rows)]
    active = []  # (r_hi, x1, y1, x2, y2, pidx)
    ei = 0
    n_edge = len(edges)
    for r in range(rows):
        while ei < n_edge and edges[ei][0] == r:
            _, r_hi, x1, y1, x2, y2, pidx = edges[ei]
            active.append((r_hi, x1, y1, x2, y2, pidx))
            ei += 1
        active = [e for e in active if e[0] >= r]
        if not active:
            continue
        y_r = -90.0 + (r + 0.5) * res_deg
        poly_xs = {}
        for r_hi, x1, y1, x2, y2, pidx in active:
            ylo, yhi = (y1, y2) if y1 < y2 else (y2, y1)
            if not (ylo <= y_r < yhi):
                continue
            t = (y_r - y1) / (y2 - y1)
            if 0.0 <= t < 1.0:
                poly_xs.setdefault(pidx, []).append(x1 + t * (x2 - x1))
        if not poly_xs:
            continue
        pos = []
        neg = []
        for pidx, xs in poly_xs.items():
            xs.sort()
            level, greenwich, _ = polys[pidx]
            if level in POS_LEVELS:
                dst = pos
            else:
                dst = neg
            # 奇偶配对 → 段（-180..180 列），再转 0-360 语义（跨 0 分裂）
            for i in range(0, len(xs) - 1, 2):
                xa, xb = xs[i], xs[i + 1]
                c0 = int((xa + 180.0) / res_deg)
                c1 = int((xb + 180.0) / res_deg)
                c0 = (c0 + cols // 2) % cols
                c1 = (c1 + cols // 2) % cols
                if c0 < c1:
                    append_col(dst, c0, c1, cols)
                elif c0 > c1:
                    # 跨 0 点段分裂（0-360 语义）
                    append_col(dst, c0, cols, cols)
                    append_col(dst, 0, c1, cols)
        pos = merge_segs(pos, cols)
        neg = merge_segs(neg, cols)
        if pos:
            land = subtract_segs(pos, neg)
            lake = intersect_segs(neg, pos)
            row_segs[r] = [(1, c0, c1) for (c0, c1) in land] + [(2, c0, c1) for (c0, c1) in lake]
            row_segs[r].sort()
        if r % 5000 == 0:
            n_act = len(active)
            print(f"  ... 行 {r}/{rows}, 活动边 {n_act}, 多边形 {len(poly_xs)}")
    return row_segs


def intersect_segs(a, b):
    """两段列表交集（a、b 需已合并排序）。"""
    out = []
    i = j = 0
    while i < len(a) and j < len(b):
        lo = max(a[i][0], b[j][0])
        hi = min(a[i][1], b[j][1])
        if hi > lo:
            out.append((lo, hi))
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return out


def append_col(dst, c0, c1, cols):
    if c0 < 0:
        c0 = 0
    if c1 > cols:
        c1 = cols
    if c1 > c0:
        dst.append((c0, c1))


def write_mask(path, arcsec, rows, cols, row_segs):
    res_deg = arcsec / 3600.0
    with open(path, "wb") as f:
        f.write(b"\0" * HDR_SIZE)
        # 行索引表（rows+1 项 u64）
        idx_off = HDR_SIZE
        offsets = []
        pos = idx_off + (rows + 1) * 8
        total_segs = 0
        for r in range(rows):
            offsets.append(pos)
            segs = row_segs[r]
            pos += 4 + len(segs) * 9  # u32 nseg + (u8 class, u32 start, u32 end)
            total_segs += len(segs)
        offsets.append(pos)
        # 回填 header
        f.seek(0)
        f.write(MAGIC + b"\0" * (16 - len(MAGIC)))
        f.write(struct.pack(">I", 2))       # version (2 = 3态: 0海洋/1陆地/2内陆湖)
        f.write(struct.pack(">I", arcsec))  # arcsec
        f.write(struct.pack(">I", rows))
        f.write(struct.pack(">I", cols))
        f.write(struct.pack(">d", 0.0))     # lon0
        f.write(struct.pack(">d", -90.0))   # lat0
        f.write(struct.pack(">d", res_deg))
        # 行索引表
        f.seek(idx_off)
        for o in offsets:
            f.write(struct.pack(">Q", o))
        # 行数据
        f.seek(offsets[0])
        for r in range(rows):
            segs = row_segs[r]
            f.write(struct.pack(">I", len(segs)))
            for cls, c0, c1 in segs:
                f.write(struct.pack(">BII", cls, c0, c1))
        # 文件尾写个魔数标记（完整性）
        f.write(b"END")
    return pos + 3, total_segs


def query(path, lon, lat):
    """验证用：查询 (lon, lat) 的掩膜类别。lon ∈ [-180,180)。
    返回 'water'(海洋) / 'land'(陆地) / 'lake'(内陆湖)。"""
    with open(path, "rb") as f:
        magic = f.read(16)
        ver = struct.unpack(">I", f.read(4))[0]
        arcsec = struct.unpack(">I", f.read(4))[0]
        rows = struct.unpack(">I", f.read(4))[0]
        cols = struct.unpack(">I", f.read(4))[0]
        lon0, lat0, res_deg = struct.unpack(">ddd", f.read(24))
        f.seek(HDR_SIZE)  # 行索引表从 64B 头后开始
        n_idx = rows + 1
        idx = struct.unpack(f">{n_idx}Q", f.read(n_idx * 8))
        if lon < 0:
            lon += 360.0
        c = int((lon - lon0) / res_deg)
        r = int((lat - lat0) / res_deg)
        if not (0 <= r < rows and 0 <= c < cols):
            return "out_of_bounds"
        f.seek(idx[r])
        nseg = struct.unpack(">I", f.read(4))[0]
        for _ in range(nseg):
            cls, c0, c1 = struct.unpack(">BII", f.read(9))
            if c0 <= c < c1:
                return {1: "land", 2: "lake"}.get(cls, "water")
            if c > c1:
                continue
        return "water"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    limit = None
    arcsec = 10
    for i, a in enumerate(sys.argv[1:]):
        if a == "--arcsec":
            arcsec = int(sys.argv[i + 2])
        if a == "--limit":
            limit = int(sys.argv[i + 2])
    if len(args) < 2:
        print("用法: python gshhg_mask.py <gshhs_f.b> <out.mask> [--arcsec 10] [--limit N]")
        sys.exit(2)
    src, out = args[0], args[1]
    res_deg = arcsec / 3600.0
    rows = int(180.0 / res_deg)
    cols = int(360.0 / res_deg)
    print(f"GSHHG → 掩膜 {arcsec} 弧秒: {rows}x{cols}")
    t0 = time.time()
    polys = read_polys(src)
    print(f"解析 {len(polys)} 多边形, {time.time()-t0:.1f}s")
    if limit:
        polys = polys[:limit]
        print(f"[LIMIT] 仅前 {limit} 个多边形")
    t1 = time.time()
    row_segs = build(rows, cols, res_deg, polys)
    print(f"光栅化完成, {time.time()-t1:.1f}s")
    # 南极内陆补全：GSHHG 南极用接地线，接地线以南的内陆冰盖无数据。
    # 段1：数据南界（-85.15°S）以南全部按陆地（极点冰盖）。
    # 段2：东南极内陆缺口（接地线在东南极海岸 ~-75°S）——-75..-85.15°S, 0..160°E 按陆地，
    #      避开罗斯海（160°E 以东）与威德尔海（-60°W 以西，即 300°E 以西）。
    min_lat = min(min(y for _, y in pts) for lv, _, pts in polys if lv in POS_LEVELS)
    r_pole = int((min_lat + 90.0) / res_deg)  # 行中心 < min_lat（更南）的行
    if r_pole > 0:
        print(f"南极补全段1: 行 0..{r_pole-1} ({min_lat:.3f}°S 以南) → 陆地")
        for r in range(r_pole):
            row_segs[r] = [(1, 0, cols)]
    r_low = r_pole                  # -85.15°S
    r_high = int((-75.0 + 90.0) / res_deg)  # -75°S
    c_east = 0                      # 0°E
    c_west = int(160.0 / res_deg)   # 160°E
    if r_high > r_low:
        print(f"南极补全段2: 行 {r_low}..{r_high-1} (-85.15..-75°S), 0..160°E → 陆地(东南极内陆)")
        for r in range(r_low, r_high):
            row_segs[r].append((1, c_east, c_west))
            row_segs[r].sort()
    size, total_segs = write_mask(out, arcsec, rows, cols, row_segs)
    print(f"掩膜写出: {out} ({size} B), 段 {total_segs}")
    print(f"总耗时 {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
