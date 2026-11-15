const EPS = 1e-9;

export const v3 = (x = 0, y = 0, z = 0) => ({ x, y, z });
export const add = (a, b) => v3(a.x + b.x, a.y + b.y, a.z + b.z);
export const sub = (a, b) => v3(a.x - b.x, a.y - b.y, a.z - b.z);
export const scale = (a, s) => v3(a.x * s, a.y * s, a.z * s);
export const dot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
export const cross = (a, b) => v3(
  a.y * b.z - a.z * b.y,
  a.z * b.x - a.x * b.z,
  a.x * b.y - a.y * b.x,
);
export const len = (a) => Math.hypot(a.x, a.y, a.z);
export const normalize = (a) => {
  const l = len(a);
  return l < EPS ? v3(0, 0, 0) : scale(a, 1 / l);
};
export const lerp = (a, b, t) => v3(
  a.x + (b.x - a.x) * t,
  a.y + (b.y - a.y) * t,
  a.z + (b.z - a.z) * t,
);
export const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
export const angleBetween = (a, b) => {
  const d = clamp(dot(normalize(a), normalize(b)), -1, 1);
  return Math.acos(d);
};

export function orthonormalize(up, forward) {
  const u = normalize(up);
  let f = sub(forward, scale(u, dot(forward, u)));
  if (len(f) < EPS) return { up: u, forward: null };
  f = normalize(f);
  return { up: u, forward: f, right: cross(f, u) };
}

export function rotateAroundAxis(v, axis, angle) {
  const a = normalize(axis);
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return add(
    add(scale(v, c), scale(cross(a, v), s)),
    scale(a, dot(a, v) * (1 - c)),
  );
}
