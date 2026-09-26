"""Build the static Hugging Face Space in site/ from the URDFs and results/*.json.

site/fk.js holds the kinematics and the limit check; scripts/check_space_fk.mjs tests it against the C++ library.
"""
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SITE = ROOT / "site"
MODELS = ["yaro_0808", "yaro_1105", "yaro_1115", "yaro_1310", "yaro_1608"]


def chain(model):
    root = ET.parse(ROOT / "models" / model / "robot.urdf").getroot()
    by_parent = {j.find("parent").get("link"): j for j in root.findall("joint")}
    link, joints = "link_0", []
    while link in by_parent:
        j = by_parent[link]
        o = j.find("origin")
        entry = {
            "name": j.get("name"), "type": j.get("type"),
            "xyz": [float(v) for v in o.get("xyz", "0 0 0").split()],
            "rpy": [float(v) for v in o.get("rpy", "0 0 0").split()],
        }
        if j.get("type") == "revolute":
            a = [float(v) for v in j.find("axis").get("xyz").split()]
            n = sum(v * v for v in a) ** 0.5
            lim = j.find("limit")
            entry.update(axis=[v / n for v in a], lower=float(lim.get("lower")), upper=float(lim.get("upper")),
                         velocity=float(lim.get("velocity")))
        joints.append(entry)
        link = j.find("child").get("link")
    assert link == "ee_frame", model
    return {"joints": joints}


audit = json.loads((ROOT / "results" / "audit.json").read_text())
tests = json.loads((ROOT / "results" / "tests.json").read_text())
bench = json.loads((ROOT / "results" / "bench_yaro_1105.json").read_text())
data = {"models": {m: chain(m) for m in MODELS}, "audit": audit, "tests": tests, "bench": bench,
        "cpu": (ROOT / "results" / "bench_cpu.txt").read_text().strip()}
for m in audit["models"]:
    data["models"][m["model"]]["datasheet"] = m["datasheet"]
    data["models"][m["model"]]["comparisons"] = m["comparisons"]
(SITE / "data.json").write_text(json.dumps(data))
print("wrote site/data.json")
