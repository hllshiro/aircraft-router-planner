import json, subprocess

def run(m, name):
    d = {"schema_version": "0.20", "mission": m}
    out = subprocess.run(["target/release/aircraft-router-planner-cli"],
                         input=json.dumps(d).encode(), capture_output=True)
    r = json.loads(out.stdout)
    if not r.get("vehicles"):
        print(f"== {name} == status: {r['status']} err: {r.get('error')}")
        return
    v = r["vehicles"][0]
    print(f"== {name} ==")
    print("  status:", v["status"], "| dist_m:", round(v["distance_m"]), "| pts:", len(v["path"]))
    print("  deps:", r["stats"]["degradations"])

base = {"start": {"lon": 115.0, "lat": 39.0, "alt_m": 3000},
        "target": {"lon": 116.5, "lat": 39.9, "alt_m": 3000},
        "terrain": {"source": "none"},
        "red_forces": {"radars": []}}

run({**base, "no_fly_zones": [{"id": "mid", "zone_type": "no_fly", "shape": "circle",
     "geometry": {"center": [115.75, 39.45], "radius_km": 15}, "alt_min_m": 0, "alt_max_m": 10000}]},
    "No-fly circle")
run({**base, "no_fly_zones": [{"id": "mid", "zone_type": "no_fly", "shape": "polygon",
     "geometry": {"vertices": [[115.7, 39.2], [115.9, 39.2], [115.9, 39.7], [115.7, 39.7]]},
     "alt_min_m": 0, "alt_max_m": 10000}]},
    "No-fly polygon")
