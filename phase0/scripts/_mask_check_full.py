import sys
import struct
sys.path.insert(0, r'phase0\scripts')
from gshhg_mask import query

m = sys.argv[1] if len(sys.argv) > 1 else r'phase0\data\mask_10as.mask'


def load_index(path):
    with open(path, 'rb') as f:
        f.read(16)
        ver, arcsec, rows, cols = struct.unpack('>IIII', f.read(16))
        f.read(24)
        f.seek(64)
        idx = struct.unpack(f'>{rows+1}Q', f.read((rows + 1) * 8))
    return ver, arcsec, rows, cols, idx


def land_ratio(path):
    ver, arcsec, rows, cols, idx = load_index(path)
    land_cells = lake_cells = 0
    with open(path, 'rb') as f:
        for r in range(rows):
            f.seek(idx[r])
            nseg = struct.unpack('>I', f.read(4))[0]
            for _ in range(nseg):
                cls, c0, c1 = struct.unpack('>BII', f.read(9))
                if cls == 1:
                    land_cells += c1 - c0
                elif cls == 2:
                    lake_cells += c1 - c0
    total = rows * cols
    return land_cells / total, lake_cells / total


tests = [
    # (名称, lon, lat, 期望)
    ('北京(116.4,39.9)', 116.4, 39.9, 'land'),
    ('上海(121.5,31.2)', 121.5, 31.2, 'land'),
    ('太平洋中部(180,0)', 180.0, 0.0, 'water'),
    ('死海(35.5,31.5)', 35.5, 31.5, 'lake'),   # 内陆湖（湖面高程≠0）
    ('地中海(15,35)', 15.0, 35.0, 'water'),
    ('东太平洋(-150,10)', -150.0, 10.0, 'water'),
    ('格陵兰(-40,72)', -40.0, 72.0, 'land'),
    ('南极半岛(-60,-75)', -60.0, -75.0, 'water'),  # 威德尔海/接地线外
    ('南极内陆(0,-85)', 0.0, -85.0, 'land'),  # 南极补全（-85.15°S 以南）
    ('南极极点(0,-89)', 0.0, -89.0, 'land'),  # 补全区
    ('大西洋中部(-30,10)', -30.0, 10.0, 'water'),
    ('印度洋中部(80,-10)', 80.0, -10.0, 'water'),
    ('撒哈拉(15,25)', 15.0, 25.0, 'land'),
    ('亚马逊(-60,-5)', -60.0, -5.0, 'land'),
    ('青藏高原(90,33)', 90.0, 33.0, 'land'),
    ('里海(50,42)', 50.0, 42.0, 'lake'),    # 内陆湖
    ('贝加尔湖(108,53.5)', 108.0, 53.5, 'lake'),  # 内陆湖
    ('日本东京(139.7,35.7)', 139.7, 35.7, 'land'),
    ('澳大利亚中部(134,-25)', 134.0, -25.0, 'land'),
    ('新西兰(172,-42)', 172.0, -42.0, 'land'),
    ('吐鲁番盆地(89.2,42.9)', 89.2, 42.9, 'land'),  # 内陆负高程但陆地（非湖泊）
    ('台湾(121,23.5)', 121.0, 23.5, 'land'),
    ('海南(109.5,19)', 109.5, 19.0, 'land'),
    ('杭州湾(121.5,30.5)', 121.5, 30.5, 'water'),  # 杭州湾是海
    ('波斯湾(51,26)', 51.0, 26.0, 'water'),
    ('红海(38,22)', 38.0, 22.0, 'water'),
    ('英吉利海峡(-1,50)', -1.0, 50.0, 'water'),
    ('伦敦(0,51.5)', 0.0, 51.5, 'land'),
    ('直布罗陀海峡(-5.4,35.9)', -5.4, 35.9, 'land'),  # 摩洛哥丹吉尔附近陆地
    ('巴拿马运河(-80,9)', -80.0, 9.0, 'land'),
    ('美国中部(-100,40)', -100.0, 40.0, 'land'),
    ('北极点(0,88)', 0.0, 88.0, 'water'),  # 北冰洋（GSHHG 无北极陆地）
]
bad = 0
for name, lon, lat, exp in tests:
    got = query(m, lon, lat)
    ok = got == exp
    if not ok:
        bad += 1
        print(f'  MISMATCH {name}: {got} (期望 {exp})')
print(f'测试 {len(tests)} 项, {len(tests)-bad} OK, {bad} MISMATCH')
lr, lk = land_ratio(m)
print(f'陆地占比: {lr*100:.2f}%  (理论 29.2%)')
print(f'内陆湖占比: {lk*100:.4f}%  (理论 ~1%)')
