// Forward kinematics and limit check for the YARO chains, the same rules as yaro_check (C++).
// Tested against the C++ library by scripts/check_space_fk.mjs.

function matMul(a, b) {
  const r = new Array(16).fill(0);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) for (let k = 0; k < 4; k++) r[i * 4 + j] += a[i * 4 + k] * b[k * 4 + j];
  return r;
}

// URDF rpy: fixed-axis roll, pitch, yaw -> R = Rz(yaw) Ry(pitch) Rx(roll)
function originMatrix(xyz, rpy) {
  const [r, p, y] = rpy;
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0],
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1],
    -sp, cp * sr, cp * cr, xyz[2],
    0, 0, 0, 1,
  ];
}

// Rotation by angle q about a unit axis (Rodrigues).
function axisRotation(axis, q) {
  const [x, y, z] = axis;
  const c = Math.cos(q), s = Math.sin(q), t = 1 - c;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0,
    0, 0, 0, 1,
  ];
}

export function forward(model, q) {
  let T = originMatrix([0, 0, 0], [0, 0, 0]);
  let k = 0;
  for (const j of model.joints) {
    T = matMul(T, originMatrix(j.xyz, j.rpy));
    if (j.type === "revolute") T = matMul(T, axisRotation(j.axis, q[k++]));
  }
  return [T[3], T[7], T[11]];
}

// level: 0 OK, 1 WARN, 2 ERROR (same order of checks as LimitChecker::check)
export function checkJoint(j, position, velocity, warnMargin, velocityScale) {
  if (!Number.isFinite(position) || !Number.isFinite(velocity)) return { level: 2, message: "non-finite value" };
  if (position < j.lower || position > j.upper) return { level: 2, message: "position outside limits" };
  if (j.velocity > 0 && Math.abs(velocity) > velocityScale * j.velocity) return { level: 2, message: "velocity above limit" };
  if (position - j.lower < warnMargin || j.upper - position < warnMargin) return { level: 1, message: "close to a limit" };
  return { level: 0, message: "ok" };
}
