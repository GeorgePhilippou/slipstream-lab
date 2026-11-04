// Slipstream Lab: vortex-lattice method (VLM) for wing + tail configurations.
// Horseshoe vortices on the cambered mean surface (bound leg at panel 1/4 chord, control point at 3/4 chord,
// trailing legs to +x infinity). Lift and moments from Kutta–Joukowski on the bound legs; induced drag in the
// Trefftz plane. Axes: x downstream, y to the right (starboard), z up. Freestream V = (cos α, 0, sin α).
(() => {
const VLM = (typeof window !== 'undefined' ? window : globalThis).VLM = {};
const FAR = 1e4;

// ---------- Biot–Savart for a straight segment A→B, unit circulation; adds to out ----------
function segVel(px, py, pz, ax, ay, az, bx, by, bz, out, g) {
  const r1x = px - ax, r1y = py - ay, r1z = pz - az, r2x = px - bx, r2y = py - by, r2z = pz - bz;
  const cx = r1y * r2z - r1z * r2y, cy = r1z * r2x - r1x * r2z, cz = r1x * r2y - r1y * r2x;
  const c2 = cx * cx + cy * cy + cz * cz;
  const r0x = bx - ax, r0y = by - ay, r0z = bz - az, l2 = r0x * r0x + r0y * r0y + r0z * r0z;
  if (c2 < 1e-10 * l2 + 1e-14) return;   // point (nearly) on the vortex line
  const r1 = Math.sqrt(r1x * r1x + r1y * r1y + r1z * r1z), r2 = Math.sqrt(r2x * r2x + r2y * r2y + r2z * r2z);
  if (r1 < 1e-9 || r2 < 1e-9) return;
  const k = g / (4 * Math.PI * c2) * (r0x * (r1x / r1 - r2x / r2) + r0y * (r1y / r1 - r2y / r2) + r0z * (r1z / r1 - r2z / r2));
  out[0] += k * cx; out[1] += k * cy; out[2] += k * cz;
}
function horseshoe(P, pan, out, g) {
  const A = pan.A, B = pan.B;
  segVel(P[0], P[1], P[2], A[0] + FAR, A[1], A[2], A[0], A[1], A[2], out, g);
  segVel(P[0], P[1], P[2], A[0], A[1], A[2], B[0], B[1], B[2], out, g);
  segVel(P[0], P[1], P[2], B[0], B[1], B[2], B[0] + FAR, B[1], B[2], out, g);
}

// ---------- NACA 4-digit camber ----------
const camber = (m, p, x) => (m === 0 || p === 0) ? 0 : x < p ? m / (p * p) * (2 * p * x - x * x) : m / ((1 - p) * (1 - p)) * (1 - 2 * p + 2 * p * x - x * x);

// ---------- geometry ----------
// surface: { id, name, sections: [{le:[x,y,z], c, twist(deg)}], segs: [{n, flap(deg)}], m, p, t, nc, lifting:true }
// Builds right half then mirrors. Strips are ordered left tip → right tip for the Trefftz analysis.
function buildSurface(sf) {
  const right = [];
  const nc = sf.nc;
  // section 'up' vectors: bisector of the neighbouring segments, so adjacent strips share edges exactly
  const segUp = [];
  for (let g = 0; g < sf.sections.length - 1; g++) {
    const s0 = sf.sections[g], s1 = sf.sections[g + 1], ph = Math.atan2(s1.le[2] - s0.le[2], s1.le[1] - s0.le[1]);
    segUp.push([0, -Math.sin(ph), Math.cos(ph)]);
  }
  const secUp = sf.sections.map((_, k) => {
    const a = segUp[Math.max(0, k - 1)], b = segUp[Math.min(segUp.length - 1, k)];
    const v = k === 0 ? [0, 0, 1] : [0, a[1] + b[1], a[2] + b[2]], l = Math.hypot(v[1], v[2]) || 1;
    return [0, v[1] / l, v[2] / l];
  });
  for (let g = 0; g < sf.sections.length - 1; g++) {
    const s0 = sf.sections[g], s1 = sf.sections[g + 1], seg = sf.segs[g];
    const dy = s1.le[1] - s0.le[1], dz = s1.le[2] - s0.le[2], phi = Math.atan2(dz, dy);
    const u0 = segUp[g], ua = secUp[g], ub = secUp[g + 1];
    const P = (s, xc) => {
      const le = [0, 1, 2].map(a => s0.le[a] + (s1.le[a] - s0.le[a]) * s), c = s0.c + (s1.c - s0.c) * s;
      const uy = ua[1] + (ub[1] - ua[1]) * s, uz = ua[2] + (ub[2] - ua[2]) * s, ul = Math.hypot(uy, uz) || 1, u0 = [0, uy / ul, uz / ul];
      const t = (s0.twist + (s1.twist - s0.twist) * s) * Math.PI / 180;
      const d = [Math.cos(t), -Math.sin(t) * u0[1], -Math.sin(t) * u0[2]], nu = [Math.sin(t), Math.cos(t) * u0[1], Math.cos(t) * u0[2]];
      let zc = camber(sf.m, sf.p, xc);
      if (seg.flap && xc > 0.75) zc -= (xc - 0.75) * Math.tan(seg.flap * Math.PI / 180);
      return [le[0] + c * (xc * d[0] + zc * nu[0]), le[1] + c * (xc * d[1] + zc * nu[1]), le[2] + c * (xc * d[2] + zc * nu[2])];
    };
    for (let i = 0; i < seg.n; i++) {
      const sa = 0.5 * (1 - Math.cos(Math.PI * i / seg.n)), sb = 0.5 * (1 - Math.cos(Math.PI * (i + 1) / seg.n));
      const sm = 0.5 * (sa + sb);
      const strip = { surf: sf.id, panels: [], c: (s0.c + (s1.c - s0.c) * sm), flap: !!seg.flap, phi };
      for (let j = 0; j < nc; j++) {
        const xa = j / nc, xb = (j + 1) / nc, xq = xa + 0.25 / nc, x3 = xa + 0.75 / nc;
        const A = P(sa, xq), B = P(sb, xq), C = P(sm, x3);
        const c00 = P(sa, xa), c10 = P(sb, xa), c11 = P(sb, xb), c01 = P(sa, xb);
        const d1 = [c11[0] - c00[0], c11[1] - c00[1], c11[2] - c00[2]], d2 = [c01[0] - c10[0], c01[1] - c10[1], c01[2] - c10[2]];
        let n = [d1[1] * d2[2] - d1[2] * d2[1], d1[2] * d2[0] - d1[0] * d2[2], d1[0] * d2[1] - d1[1] * d2[0]];
        const nl = Math.hypot(...n); n = n.map(v => v / nl);
        if (n[2] * u0[2] + n[1] * u0[1] < 0) n = n.map(v => -v);
        strip.panels.push({ A, B, C, n, corners: [c00, c10, c11, c01], dx: (xb - xa) * strip.c });
      }
      const ta = P(sa, 1), tb = P(sb, 1);
      strip.te = [ta, tb]; strip.ds = Math.hypot(tb[1] - ta[1], tb[2] - ta[2]); strip.dyProj = Math.abs(tb[1] - ta[1]);
      strip.mid = P(sm, 0.25);
      right.push(strip);
    }
  }
  const mir = v => [v[0], -v[1], v[2]];
  const left = right.slice().reverse().map(st => ({
    ...st, panels: st.panels.map(p => ({ A: mir(p.B), B: mir(p.A), C: mir(p.C), n: mir(p.n), corners: p.corners.map(mir), dx: p.dx })),
    te: [mir(st.te[1]), mir(st.te[0])], mid: mir(st.mid), side: -1,
  }));
  right.forEach(st => st.side = 1);
  return left.concat(right);
}

// ---------- dense solver (Gaussian elimination, partial pivoting) ----------
function lu(Ain, n) {
  const A = Float64Array.from(Ain), piv = new Int32Array(n);
  for (let k = 0; k < n; k++) {
    let p = k, mx = Math.abs(A[k * n + k]);
    for (let i = k + 1; i < n; i++) { const v = Math.abs(A[i * n + k]); if (v > mx) { mx = v; p = i; } }
    piv[k] = p;
    if (p !== k) for (let j = 0; j < n; j++) { const t = A[k * n + j]; A[k * n + j] = A[p * n + j]; A[p * n + j] = t; }
    const d = A[k * n + k];
    for (let i = k + 1; i < n; i++) {
      const f = (A[i * n + k] /= d);
      if (f) for (let j = k + 1; j < n; j++) A[i * n + j] -= f * A[k * n + j];
    }
  }
  return { A, piv, n };
}
function luSolve(F, b) {
  const { A, piv, n } = F, x = Float64Array.from(b);
  for (let k = 0; k < n; k++) { const p = piv[k]; if (p !== k) { const t = x[k]; x[k] = x[p]; x[p] = t; } }
  for (let i = 0; i < n; i++) { let s = x[i]; for (let j = 0; j < i; j++) s -= A[i * n + j] * x[j]; x[i] = s; }
  for (let i = n - 1; i >= 0; i--) { let s = x[i]; for (let j = i + 1; j < n; j++) s -= A[i * n + j] * x[j]; x[i] = s / A[i * n + i]; }
  return x;
}

// ---------- analysis of one geometry ----------
// Returns linearised aerodynamics: everything is evaluated at α = 0 and α = 1 rad (unit RHS) and combined.
function analyse(surfaces, ref) {
  const strips = [], panels = [];
  for (const sf of surfaces) { const st = buildSurface(sf); st.forEach(s => { s.surfObj = sf; strips.push(s); s.panels.forEach(p => { p.strip = s; panels.push(p); }); }); }
  const n = panels.length, AIC = new Float64Array(n * n), v = [0, 0, 0];
  for (let i = 0; i < n; i++) {
    const pi = panels[i];
    for (let j = 0; j < n; j++) { v[0] = v[1] = v[2] = 0; horseshoe(pi.C, panels[j], v, 1); AIC[i * n + j] = v[0] * pi.n[0] + v[1] * pi.n[1] + v[2] * pi.n[2]; }
  }
  const F = lu(AIC, n);
  // RHS for V = (1,0,0) and V = (0,0,1); solution for any small α: Γ = Γx cosα + Γz sinα
  const bx = new Float64Array(n), bz = new Float64Array(n);
  for (let i = 0; i < n; i++) { bx[i] = -panels[i].n[0]; bz[i] = -panels[i].n[2]; }
  const Gx = luSolve(F, bx), Gz = luSolve(F, bz);
  // induced velocity at each bound midpoint per unit Γ_j is needed for forces: build matrix columns lazily (n² again)
  const mids = panels.map(p => [(p.A[0] + p.B[0]) / 2, (p.A[1] + p.B[1]) / 2, (p.A[2] + p.B[2]) / 2]);
  const Wx = new Float64Array(n * 3), Wz = new Float64Array(n * 3);   // induced velocity at mids for the two base solutions
  for (let i = 0; i < n; i++) {
    let ax = 0, ay = 0, az = 0, cx = 0, cy = 0, cz = 0;
    for (let j = 0; j < n; j++) {
      v[0] = v[1] = v[2] = 0; horseshoe(mids[i], panels[j], v, 1);
      ax += v[0] * Gx[j]; ay += v[1] * Gx[j]; az += v[2] * Gx[j];
      cx += v[0] * Gz[j]; cy += v[1] * Gz[j]; cz += v[2] * Gz[j];
    }
    Wx[i * 3] = ax; Wx[i * 3 + 1] = ay; Wx[i * 3 + 2] = az; Wz[i * 3] = cx; Wz[i * 3 + 1] = cy; Wz[i * 3 + 2] = cz;
  }
  // Trefftz plane: each surface's strip circulation is interpolated (linearly in θ, where loading is smooth)
  // onto fine cosine grids of M and 2M stations; the point-vortex midpoint rule is applied on both and
  // Richardson-extrapolated, which removes the O(1/M) bias of the midpoint rule.
  const tre = [];
  for (const sf of surfaces) {
    const ss = strips.filter(s => s.surfObj === sf);
    const pts = [ss[0].te[0]].concat(ss.map(s => s.te[1])).map(p => [p[1], p[2]]);
    const arc = [0]; for (let k = 1; k < pts.length; k++) arc.push(arc[k - 1] + Math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]));
    const Lt = arc[arc.length - 1];
    const thMid = ss.map((_, i) => Math.acos(1 - 2 * ((arc[i] + arc[i + 1]) / 2) / Lt));
    const at = a => { let k = 1; while (k < arc.length - 1 && arc[k] < a) k++; const f = (a - arc[k - 1]) / ((arc[k] - arc[k - 1]) || 1); return [pts[k - 1][0] + (pts[k][0] - pts[k - 1][0]) * f, pts[k - 1][1] + (pts[k][1] - pts[k - 1][1]) * f]; };
    tre.push({ ss, Lt, thMid, at });
  }
  const grids = [120, 240].map(M => {
    const edges = [], fs = [];
    tre.forEach((t, ti) => {
      const e = []; for (let k = 0; k <= M; k++) e.push(t.at(t.Lt * (1 - Math.cos(Math.PI * k / M)) / 2));
      for (let k = 0; k < M; k++) {
        const a = e[k], b = e[k + 1], ty = b[0] - a[0], tz = b[1] - a[1], ds = Math.hypot(ty, tz) || 1e-12;
        fs.push({ ti, th: Math.PI * (k + 0.5) / M, m: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2], n: [-tz / ds, ty / ds], ds, k, M });
      }
      e.forEach((p, k) => edges.push({ ti, k, p, M }));
    });
    const D = new Float64Array(fs.length * edges.length);
    fs.forEach((f, i) => edges.forEach((e, k) => {
      const ry = f.m[0] - e.p[0], rz = f.m[1] - e.p[1], r2 = ry * ry + rz * rz + 1e-14;
      D[i * edges.length + k] = (-rz * f.n[0] + ry * f.n[1]) / (2 * Math.PI * r2);
    }));
    return { M, edges, fs, D };
  });
  return { surfaces, strips, panels, n, Gx, Gz, Wx, Wz, mids, tre, grids, ref };
}

// evaluate at angle α (rad) with unit speed, ρ = 1
function evaluate(G, alpha) {
  const ca = Math.cos(alpha), sa = Math.sin(alpha), { panels, n, ref } = G;
  const Vx = ca, Vz = sa;
  const gam = new Float64Array(n);
  let Fx = 0, Fy = 0, Fz = 0, My = 0;
  for (let i = 0; i < n; i++) {
    const g = G.Gx[i] * ca + G.Gz[i] * sa; gam[i] = g;
    const p = panels[i], m = G.mids[i];
    const vx = Vx + G.Wx[i * 3] * ca + G.Wz[i * 3] * sa, vy = G.Wx[i * 3 + 1] * ca + G.Wz[i * 3 + 1] * sa, vz = Vz + G.Wx[i * 3 + 2] * ca + G.Wz[i * 3 + 2] * sa;
    const lx = p.B[0] - p.A[0], ly = p.B[1] - p.A[1], lz = p.B[2] - p.A[2];
    const fx = g * (vy * lz - vz * ly), fy = g * (vz * lx - vx * lz), fz = g * (vx * ly - vy * lx);
    p.F = [fx, fy, fz]; p.gam = g;
    Fx += fx; Fy += fy; Fz += fz;
    const rx = m[0] - ref.xcg, rz = m[2] - ref.zcg;
    My += rz * fx - rx * fz;
  }
  const q = 0.5, S = ref.S, c = ref.cbar;
  const L = -Fx * sa + Fz * ca;
  // strips
  for (const s of G.strips) {
    let g = 0, fn = 0;
    for (const p of s.panels) { g += p.gam; }
    s.G = g; s.cl = 2 * g / s.c;
    for (const p of s.panels) p.dcp = p.gam / p.dx;   // pressure-difference coefficient ΔCp = ρVΓ/(q dx) = 2Γ/dx
  }
  // Trefftz drag (Richardson-extrapolated from two fine grids)
  const interp = (t, th) => {
    const T = t.thMid, ss = t.ss;
    if (th <= T[0]) return ss[0].G * th / T[0];
    if (th >= T[T.length - 1]) return ss[ss.length - 1].G * (Math.PI - th) / (Math.PI - T[T.length - 1]);
    let i = 1; while (T[i] < th) i++;
    const f = (th - T[i - 1]) / (T[i] - T[i - 1]); return ss[i - 1].G + (ss[i].G - ss[i - 1].G) * f;
  };
  const dis = G.grids.map(gr => {
    const g = gr.fs.map(f => interp(G.tre[f.ti], f.th));
    const str = new Float64Array(gr.edges.length);
    let base = 0;
    G.tre.forEach((t, ti) => { const M = gr.M; for (let k = 0; k <= M; k++) str[base + k] = (k > 0 ? g[ti * M + k - 1] : 0) - (k < M ? g[ti * M + k] : 0); base += M + 1; });
    let d = 0;
    for (let i = 0; i < gr.fs.length; i++) { let w = 0; const row = i * gr.edges.length; for (let k = 0; k < gr.edges.length; k++) w += gr.D[row + k] * str[k]; d -= 0.5 * g[i] * w * gr.fs[i].ds; }
    return d;
  });
  const Di = 2 * dis[1] - dis[0];
  return { CL: L / (q * S), CDi: Di / (q * S), Cm: My / (q * S * c), CY: Fy / (q * S) };
}

VLM.camber = camber;
VLM.analyse = analyse;
VLM.evaluate = evaluate;
})();
