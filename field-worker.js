// Slipstream: potential-flow velocity field around the aircraft from the vortex lattice.
// Sums every horseshoe vortex (Biot–Savart with a smoothed core) on a 3D grid; the half-span grid is
// mirrored by the caller. Body axes: x aft, y starboard, z up; freestream (cos α, 0, sin α).
onmessage = e => {
  const { id, A, B, G, alpha, box, dims, rc, far } = e.data, n = G.length;
  const [NX, NY, NZ] = dims, [x0, x1, y0, y1, z0, z1] = box;
  const out = new Float32Array(NX * NY * NZ * 3), ca = Math.cos(alpha), sa = Math.sin(alpha), rc2 = rc * rc;
  const seg = (px, py, pz, ax, ay, az, bx, by, bz, g, o) => {
    const r1x = px - ax, r1y = py - ay, r1z = pz - az, r2x = px - bx, r2y = py - by, r2z = pz - bz;
    const cx = r1y * r2z - r1z * r2y, cy = r1z * r2x - r1x * r2z, cz = r1x * r2y - r1y * r2x;
    const r0x = bx - ax, r0y = by - ay, r0z = bz - az, l2 = r0x * r0x + r0y * r0y + r0z * r0z;
    const c2 = cx * cx + cy * cy + cz * cz + rc2 * l2;          // smoothed (Rankine-like) core
    const r1 = Math.sqrt(r1x * r1x + r1y * r1y + r1z * r1z) + 1e-9, r2 = Math.sqrt(r2x * r2x + r2y * r2y + r2z * r2z) + 1e-9;
    const k = g / (4 * Math.PI * c2) * (r0x * (r1x / r1 - r2x / r2) + r0y * (r1y / r1 - r2y / r2) + r0z * (r1z / r1 - r2z / r2));
    o[0] += k * cx; o[1] += k * cy; o[2] += k * cz;
  };
  const v = [0, 0, 0];
  for (let k = 0; k < NZ; k++) for (let j = 0; j < NY; j++) for (let i = 0; i < NX; i++) {
    const px = x0 + (x1 - x0) * i / (NX - 1), py = y0 + (y1 - y0) * j / (NY - 1), pz = z0 + (z1 - z0) * k / (NZ - 1);
    v[0] = ca; v[1] = 0; v[2] = sa;
    for (let m = 0; m < n; m++) {
      const g = G[m]; if (!g) continue;
      const a = m * 3;
      seg(px, py, pz, A[a] + far, A[a + 1], A[a + 2], A[a], A[a + 1], A[a + 2], g, v);
      seg(px, py, pz, A[a], A[a + 1], A[a + 2], B[a], B[a + 1], B[a + 2], g, v);
      seg(px, py, pz, B[a], B[a + 1], B[a + 2], B[a] + far, B[a + 1], B[a + 2], g, v);
    }
    const o = ((k * NY + j) * NX + i) * 3; out[o] = v[0]; out[o + 1] = v[1]; out[o + 2] = v[2];
  }
  postMessage({ id, field: out, dims, box }, [out.buffer]);
};
