// node scripts/check_space_fk.mjs : the Space's JavaScript FK must match the C++ library (results/fk_reference.json).
import { readFileSync } from "node:fs";
import { forward } from "../site/fk.js";

const data = JSON.parse(readFileSync(new URL("../site/data.json", import.meta.url)));
const ref = JSON.parse(readFileSync(new URL("../results/fk_reference.json", import.meta.url)));
let worst = 0;
for (const r of ref) {
  const p = forward(data.models[r.model], r.q);
  for (let i = 0; i < 3; i++) worst = Math.max(worst, Math.abs(p[i] - r.p[i]));
}
console.log(`${ref.length} configurations, worst position difference JS vs C++: ${worst.toExponential(2)} m`);
if (worst > 1e-9) { console.error("FAIL"); process.exit(1); }
