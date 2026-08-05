import json, subprocess, math

def run(radars, name):
    d = {
        "schema_version": "0.20",
        "mission": {
            "start": {"lon": 115.0, "lat": 39.0, "alt_m": 3000},
            "target": {"lon": 116.5, "lat": 39.9, "alt_m": 3000},
            "terrain": {"source": "none"},
            "red_forces": {"radars": radars},
        },
    }
    out = subprocess.run(["target/release/aircraft-router-planner-cli"],
                         input=json.dumps(d).encode(), capture_output=True)
    r = json.loads(out.stdout)
    v = r["vehicles"][0]
    print(f"== {name} ==")
    print("  status:", v["status"], "| dist_m:", round(v["distance_m"]), "| pts:", len(v["path"]))
    print("  deps:", r["stats"]["degradations"])
    print("  warns:", v["warnings"])
    # 离雷达中心最近距离
    rl, rr = radars[0]["lon"], radars[0]["lat"]
    eff = radars[0]["radius_km"] * 1.2
    mind = 1e9
    for p in v["path"]:
        dd = math.hypot((p["x"] - rl) * 90.0, (p["y"] - rr) * 111.0)
        mind = min(mind, dd)
    print("  min dist to radar center (km):", round(mind), "| eff radius:", round(eff))

run([{"id": "r1", "lon": 115.75, "lat": 39.45, "radar_type": "tracking", "radius_km": 40}], "A: big 40km radar")
run([{"id": "r1", "lon": 115.75, "lat": 39.45, "radar_type": "tracking", "radius_km": 5}], "B: small 5km radar")
