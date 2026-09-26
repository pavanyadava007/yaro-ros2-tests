"""Run inside the image after `colcon test`: summarise the JUnit XML files into results/tests.json."""
import glob
import json
import sys
import xml.etree.ElementTree as ET

out = {"suites": [], "total": 0, "failures": 0}
for f in sorted(glob.glob("/ws/ros2_ws/build/yaro_check/test_results/yaro_check/*.xml")):
    root = ET.parse(f).getroot()
    suites = [root] if root.tag == "testsuite" else root.findall("testsuite")
    n = sum(int(s.get("tests", 0)) for s in suites)
    bad = sum(int(s.get("failures", 0)) + int(s.get("errors", 0)) for s in suites)
    props = {}
    for case in root.iter("testcase"):
        for p in case.iter("property"):
            props.setdefault(case.get("name"), {})[p.get("name")] = float(p.get("value"))
    out["suites"].append({"file": f.rsplit("/", 1)[1], "tests": n, "failures": bad, "properties": props})
    out["total"] += n
    out["failures"] += bad
json.dump(out, open(sys.argv[1], "w"), indent=2)
print(f"{out['total']} test cases, {out['failures']} failures")
sys.exit(1 if out["failures"] else 0)
