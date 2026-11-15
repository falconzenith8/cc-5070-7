import { dot, normalize, v3 } from './vec3.js';

export function quatFromBasis(right, up, back) {
  const m00 = right.x, m01 = up.x, m02 = back.x;
  const m10 = right.y, m11 = up.y, m12 = back.y;
  const m20 = right.z, m21 = up.z, m22 = back.z;
  const trace = m00 + m11 + m22;
  let x, y, z, w;
  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    w = 0.25 * s;
    x = (m21 - m12) / s;
    y = (m02 - m20) / s;
    z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    w = (m21 - m12) / s;
    x = 0.25 * s;
    y = (m01 + m10) / s;
    z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    w = (m02 - m20) / s;
    x = (m01 + m10) / s;
    y = 0.25 * s;
    z = (m12 + m21) / s;
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    w = (m10 - m01) / s;
    x = (m02 + m20) / s;
    y = (m12 + m21) / s;
    z = 0.25 * s;
  }
  const q = { x, y, z, w };
  const l = Math.hypot(q.x, q.y, q.z, q.w);
  if (!Number.isFinite(l) || l < 1e-12) return { x: 0, y: 0, z: 0, w: 1 };
  return { x: q.x / l, y: q.y / l, z: q.z / l, w: q.w / l };
}

export function quatDot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

export function quatAngle(a, b) {
  const d = Math.min(1, Math.abs(quatDot(a, b)));
  return 2 * Math.acos(d);
}

export function quatToMatrix(q) {
  const { x, y, z, w } = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  return {
    right: v3(1 - (yy + zz), xy + wz, xz - wy),
    up: v3(xy - wz, 1 - (xx + zz), yz + wx),
    back: v3(xz + wy, yz - wx, 1 - (xx + yy)),
  };
}

export const quatIdentity = () => ({ x: 0, y: 0, z: 0, w: 1 });

export function quatNormalize(q) {
  const l = Math.hypot(q.x, q.y, q.z, q.w);
  return l < 1e-12 ? quatIdentity() : { x: q.x / l, y: q.y / l, z: q.z / l, w: q.w / l };
}

export function quatMultiply(a, b) {
  return quatNormalize({
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  });
}

export function quatFromAxisAngle(axis, angle) {
  const a = normalize(axis);
  const h = angle * 0.5;
  const s = Math.sin(h);
  return { x: a.x * s, y: a.y * s, z: a.z * s, w: Math.cos(h) };
}

export function quatRotate(q, v) {
  const { x, y, z, w } = q;
  const ix = w * v.x + y * v.z - z * v.y;
  const iy = w * v.y + z * v.x - x * v.z;
  const iz = w * v.z + x * v.y - y * v.x;
  const iw = -x * v.x - y * v.y - z * v.z;
  return v3(
    ix * w + iw * -x + iy * -z - iz * -y,
    iy * w + iw * -y + iz * -x - ix * -z,
    iz * w + iw * -z + ix * -y - iy * -x,
  );
}

export function quatForCamera(right, up, forward) {
  return quatFromBasis(normalize(right), normalize(up), normalize({ x: -forward.x, y: -forward.y, z: -forward.z }));
}

export function basisDot(a, b) {
  return dot(a, b);
}
