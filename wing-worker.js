// Slipstream Lab: 2D wing-section solver.
// D2Q9 lattice Boltzmann (BGK + Smagorinsky LES), bounce-back walls, momentum-exchange forces,
// Lagrangian supercooled droplets with Schiller–Naumann drag, and a cell-based ice accretion
// model with a freezing fraction and surface runback (rime vs glaze).
const EX = [0, 1, 0, -1, 0, 1, -1, -1, 1], EY = [0, 0, 1, 0, -1, 1, 1, -1, -1], OPP = [0, 3, 4, 1, 2, 7, 8, 5, 6];
const W = [4 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 9, 1 / 36, 1 / 36, 1 / 36, 1 / 36];
const NB8 = [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [-1, 1], [1, -1], [-1, -1]];
let NX = 0, NY = 0, N = 0, f, g, solid, clean, ice, ux, uy, rho, dye, dye2, iceMass, water, wetFlag, hotFlag;
const OFF = new Int32Array(9);
let U0 = 0.1, tau0 = 0.53, CS = 0.15, running = true, want = false, steps = 0, fx = 0, fy = 0, fn = 0, batch = 0, spacing = 8, timer = null;
let geom = { px: 0, py: 0, L: 1, aoa: 0, ox: 0.25 };
let icing = { on: false, tp: 10, rpk: 0, n0: 1, mFrac: 0.05, spawn: 2 };
let expSteps = 0, iceBody = [], iceVersion = 0, iceSent = -1, iceK = 0, iceCount = 0, hits = 0;
const MAXD = 7000, drops = new Float32Array(MAXD * 4); let nd = 0;
let wet = [], hot = [];

function eqInto(arr, k, r, vx, vy) { const usq = 1.5 * (vx * vx + vy * vy); for (let i = 0; i < 9; i++) { const cu = 3 * (EX[i] * vx + EY[i] * vy); arr[i * N + k] = W[i] * r * (1 + cu + 0.5 * cu * cu - usq); } }
function init(nx, ny) {
  NX = nx; NY = ny; N = nx * ny;
  f = new Float32Array(9 * N); g = new Float32Array(9 * N);
  solid = new Uint8Array(N); clean = new Uint8Array(N); ice = new Uint8Array(N);
  ux = new Float32Array(N); uy = new Float32Array(N); rho = new Float32Array(N);
  dye = new Float32Array(N); dye2 = new Float32Array(N);
  iceMass = new Float32Array(N); water = new Float32Array(N); wetFlag = new Uint8Array(N); hotFlag = new Uint8Array(N);
  for (let i = 0; i < 9; i++) OFF[i] = EX[i] + EY[i] * NX;
  spacing = Math.max(6, Math.round(NY / 24));
  iceBody = []; wet = []; hot = []; nd = 0; expSteps = 0;
}
function resetFlow() {
  for (let k = 0; k < N; k++) {
    const s = solid[k], vx = s ? 0 : U0 + (Math.random() - 0.5) * 0.004, vy = s ? 0 : (Math.random() - 0.5) * 0.004;
    eqInto(f, k, 1, vx, vy); ux[k] = vx; uy[k] = vy; rho[k] = 1;
  }
  dye.fill(0); steps = 0; fx = fy = fn = 0;
}
function setSolid(m) {
  for (let k = 0; k < N; k++) {
    const s = m[k];
    if (s && !solid[k]) { ux[k] = 0; uy[k] = 0; rho[k] = 1; dye[k] = 0; }
    else if (!s && solid[k]) eqInto(f, k, 1, 0, 0);
    solid[k] = s;
  }
}
function rebuildSolid() { const m = new Uint8Array(N); for (let k = 0; k < N; k++) m[k] = clean[k] | ice[k]; setSolid(m); }

// ---- ice bookkeeping in the body frame, so ice stays attached when the wing is re-tilted ----
function toBody(X, Y) {
  const a = -geom.aoa * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), rx = X - geom.px, ry = Y - geom.py;
  return [(rx * c + ry * s) / geom.L + geom.ox, (-rx * s + ry * c) / geom.L];
}
function toGrid(bx, by) {
  const a = -geom.aoa * Math.PI / 180, c = Math.cos(a), s = Math.sin(a), dx = bx - geom.ox, dy = by;
  return [geom.px + geom.L * (dx * c - dy * s), geom.py + geom.L * (dx * s + dy * c)];
}
function reRasterIce() {
  ice.fill(0);
  for (let i = 0; i < iceBody.length; i += 2) {
    const [X, Y] = toGrid(iceBody[i], iceBody[i + 1]);
    const x = Math.floor(X), y = Math.floor(Y);
    if (x < 2 || x > NX - 3 || y < 2 || y > NY - 3) continue;
    const k = y * NX + x; if (!clean[k]) ice[k] = 1;
  }
  // close single-cell pinholes left by rotation
  const add = [];
  for (let y = 2; y < NY - 2; y++) for (let x = 2; x < NX - 2; x++) {
    const k = y * NX + x; if (clean[k] || ice[k]) continue;
    let n = 0; for (const [dx, dy] of NB8) { const j = k + dx + dy * NX; if (clean[j] || ice[j]) n++; }
    if (n >= 5) add.push(k);
  }
  for (const k of add) ice[k] = 1;
}
function computeThickness() {
  iceCount = 0; iceK = 0;
  const dist = new Int16Array(N).fill(-1), q = [];
  for (let k = 0; k < N; k++) {
    if (!ice[k]) continue; iceCount++;
    if (clean[k - 1] || clean[k + 1] || clean[k - NX] || clean[k + NX]) { dist[k] = 1; q.push(k); }
  }
  for (let h = 0; h < q.length; h++) {
    const k = q[h], d = dist[k]; if (d > iceK) iceK = d;
    for (const j of [k - 1, k + 1, k - NX, k + NX]) if (ice[j] && dist[j] < 0) { dist[j] = d + 1; q.push(j); }
  }
  if (iceCount && !q.length) iceK = 1;
}
function clearIce() {
  ice.fill(0); iceBody = []; iceMass.fill(0); water.fill(0); wetFlag.fill(0); hotFlag.fill(0);
  wet = []; hot = []; nd = 0; expSteps = 0; hits = 0;
  rebuildSolid(); computeThickness(); iceVersion++;
}
const isSurface = n => !solid[n] && (solid[n - 1] || solid[n + 1] || solid[n - NX] || solid[n + NX]);
// spread each impact over nearby surface cells (sub-cell impact spread; mass-conserving)
const KER = []; for (let dy = -3; dy <= 3; dy++) for (let dx = -3; dx <= 3; dx++) { const d2 = dx * dx + dy * dy; if (d2 <= 10) KER.push([dx, dy, 1 / (1 + 0.5 * d2)]); }
function deposit(k0, m) {
  const tgt = [], wts = []; let ws = 0;
  for (const [dx, dy, w] of KER) { const n = k0 + dx + dy * NX; if (n === k0 || isSurface(n)) { tgt.push(n); wts.push(w); ws += w; } }
  for (let i = 0; i < tgt.length; i++) depositOne(tgt[i], m * wts[i] / ws);
}
function depositOne(k, m) {
  const n0 = icing.n0;
  iceMass[k] += m * n0;
  if (!hotFlag[k]) { hotFlag[k] = 1; hot.push(k); }
  const w = m * (1 - n0);
  if (w > 0) { water[k] += w; if (!wetFlag[k]) { wetFlag[k] = 1; wet.push(k); } }
}
function runback() {
  const list = wet; wet = [];
  const fr = Math.max(icing.n0, 0.3);
  for (const k of list) {
    wetFlag[k] = 0; const w = water[k]; water[k] = 0;
    if (solid[k] || w <= 0) continue;
    iceMass[k] += w * fr; if (!hotFlag[k]) { hotFlag[k] = 1; hot.push(k); }
    const rem = w * (1 - fr); if (rem < 1e-7) continue;
    // water is pushed along the surface by the airflow
    const u = ux[k], v = uy[k], sp = Math.hypot(u, v) + 1e-9;
    let best = -1, bs = 0.2;
    for (const [dx, dy] of NB8) {
      const n = k + dx + dy * NX;
      if (!isSurface(n)) continue;
      const s = (dx * u + dy * v) / (sp * Math.hypot(dx, dy));
      if (s > bs) { bs = s; best = n; }
    }
    if (best >= 0) { water[best] += rem; if (!wetFlag[best]) { wetFlag[best] = 1; wet.push(best); } }
    else iceMass[k] += rem;
  }
}
function freezeCheck() {
  let changed = false; const keep = [];
  for (const k of hot) {
    if (solid[k]) { hotFlag[k] = 0; iceMass[k] = 0; continue; }
    if (iceMass[k] >= 1) {
      ice[k] = 1; iceMass[k] = 0; hotFlag[k] = 0; changed = true;
      const x = k % NX, y = (k / NX) | 0, [bx, by] = toBody(x + 0.5, y + 0.5);
      iceBody.push(bx, by);
    } else keep.push(k);
  }
  hot = keep;
  if (changed) { rebuildSolid(); computeThickness(); iceVersion++; }
}
function dropsStep() {
  const { px, py, L, ox } = geom;
  const x0 = Math.max(2, px - (ox + 0.55) * L), xEnd = px + (1.1 - ox) * L;
  for (let s = 0; s < icing.spawn && nd < MAXD; s++) {
    const x = x0 + Math.random(), y = py + (Math.random() - 0.5) * 0.9 * L;
    const k = (y | 0) * NX + (x | 0); if (solid[k]) continue;
    const o = nd * 4; drops[o] = x; drops[o + 1] = y; drops[o + 2] = ux[k]; drops[o + 3] = uy[k]; nd++;
  }
  const tp = icing.tp, rpk = icing.rpk;
  for (let i = 0; i < nd; i++) {
    const o = i * 4; let x = drops[o], y = drops[o + 1], vx = drops[o + 2], vy = drops[o + 3];
    const k = (y | 0) * NX + (x | 0), u = ux[k], v = uy[k];
    const du = u - vx, dv = v - vy, Rep = rpk * Math.sqrt(du * du + dv * dv);
    const kk = (1 + 0.15 * Math.pow(Rep, 0.687)) / tp;     // Schiller–Naumann drag, implicit step
    vx = (vx + u * kk) / (1 + kk); vy = (vy + v * kk) / (1 + kk);
    const nx = x + vx, ny = y + vy;
    let dead = nx < 1 || nx > NX - 2 || ny < 1 || ny > NY - 2 || nx > xEnd;
    if (!dead) {
      const k2 = (ny | 0) * NX + (nx | 0);
      if (solid[k2]) { dead = true; if (!solid[k]) { deposit(k, icing.mFrac); hits++; } }
    }
    if (dead) { nd--; if (i < nd) { const e = nd * 4; drops[o] = drops[e]; drops[o + 1] = drops[e + 1]; drops[o + 2] = drops[e + 2]; drops[o + 3] = drops[e + 3]; } i--; continue; }
    drops[o] = nx; drops[o + 1] = ny; drops[o + 2] = vx; drops[o + 3] = vy;
  }
}

function collide() {
  const n = N, c = 18 * Math.SQRT2 * CS * CS, t0 = tau0, t02 = t0 * t0;
  const n2 = 2 * n, n3 = 3 * n, n4 = 4 * n, n5 = 5 * n, n6 = 6 * n, n7 = 7 * n, n8 = 8 * n;
  for (let k = 0; k < n; k++) {
    if (solid[k]) continue;
    const f0 = f[k], f1 = f[k + n], f2 = f[k + n2], f3 = f[k + n3], f4 = f[k + n4], f5 = f[k + n5], f6 = f[k + n6], f7 = f[k + n7], f8 = f[k + n8];
    const r = f0 + f1 + f2 + f3 + f4 + f5 + f6 + f7 + f8;
    let vx = (f1 - f3 + f5 - f6 - f7 + f8) / r, vy = (f2 - f4 + f5 + f6 - f7 - f8) / r;
    const sp = vx * vx + vy * vy; if (sp > 0.1225) { const s = 0.35 / Math.sqrt(sp); vx *= s; vy *= s; }
    rho[k] = r; ux[k] = vx; uy[k] = vy;
    const usq = 1.5 * (vx * vx + vy * vy), a = r * 4 / 9, b = r / 9, d = r / 36;
    let cu;
    const e0 = a * (1 - usq);
    cu = 3 * vx; const e1 = b * (1 + cu + 0.5 * cu * cu - usq);
    cu = 3 * vy; const e2 = b * (1 + cu + 0.5 * cu * cu - usq);
    cu = -3 * vx; const e3 = b * (1 + cu + 0.5 * cu * cu - usq);
    cu = -3 * vy; const e4 = b * (1 + cu + 0.5 * cu * cu - usq);
    cu = 3 * (vx + vy); const e5 = d * (1 + cu + 0.5 * cu * cu - usq);
    cu = 3 * (-vx + vy); const e6 = d * (1 + cu + 0.5 * cu * cu - usq);
    cu = 3 * (-vx - vy); const e7 = d * (1 + cu + 0.5 * cu * cu - usq);
    cu = 3 * (vx - vy); const e8 = d * (1 + cu + 0.5 * cu * cu - usq);
    const q0 = f0 - e0, q1 = f1 - e1, q2 = f2 - e2, q3 = f3 - e3, q4 = f4 - e4, q5 = f5 - e5, q6 = f6 - e6, q7 = f7 - e7, q8 = f8 - e8;
    const pxx = q1 + q3 + q5 + q6 + q7 + q8, pyy = q2 + q4 + q5 + q6 + q7 + q8, pxy = q5 - q6 + q7 - q8;
    const Q = Math.sqrt(pxx * pxx + pyy * pyy + 2 * pxy * pxy);
    const om = 2 / (t0 + Math.sqrt(t02 + c * Q / r));
    f[k] = f0 - om * q0; f[k + n] = f1 - om * q1; f[k + n2] = f2 - om * q2; f[k + n3] = f3 - om * q3; f[k + n4] = f4 - om * q4;
    f[k + n5] = f5 - om * q5; f[k + n6] = f6 - om * q6; f[k + n7] = f7 - om * q7; f[k + n8] = f8 - om * q8;
  }
}
const EQ = new Float64Array(9);
function stream() {
  const n = N; let Fx = 0, Fy = 0;
  for (let y = 1; y < NY - 1; y++) {
    const row = y * NX;
    for (let x = 1; x < NX - 1; x++) {
      const k = row + x; if (solid[k]) continue;
      g[k] = f[k];
      for (let i = 1; i < 9; i++) {
        const s = k - OFF[i];
        if (solid[s]) { const v = f[OPP[i] * n + k]; g[i * n + k] = v; Fx -= 2 * v * EX[i]; Fy -= 2 * v * EY[i]; }
        else g[i * n + k] = f[i * n + s];
      }
    }
  }
  const usq = 1.5 * U0 * U0;
  for (let i = 0; i < 9; i++) { const cu = 3 * EX[i] * U0; EQ[i] = W[i] * (1 + cu + 0.5 * cu * cu - usq); }
  for (let y = 0; y < NY; y++) { const k = y * NX; for (let i = 0; i < 9; i++) g[i * n + k] = EQ[i]; }
  for (let x = 0; x < NX; x++) { const k1 = x, k2 = (NY - 1) * NX + x; for (let i = 0; i < 9; i++) { g[i * n + k1] = EQ[i]; g[i * n + k2] = EQ[i]; } }
  for (let y = 1; y < NY - 1; y++) { const k = y * NX + NX - 1; for (let i = 0; i < 9; i++) g[i * n + k] = g[i * n + k - 1]; }
  const t = f; f = g; g = t;
  fx += Fx; fy += Fy; fn++;
}
function advectDye(dt) {
  for (let y = 1; y < NY - 1; y++) for (let x = 1; x < NX - 1; x++) {
    const k = y * NX + x;
    if (solid[k]) { dye2[k] = 0; continue; }
    let px = x - ux[k] * dt, py = y - uy[k] * dt;
    if (px < 0) px = 0; else if (px > NX - 1.001) px = NX - 1.001;
    if (py < 0) py = 0; else if (py > NY - 1.001) py = NY - 1.001;
    const ix = px | 0, iy = py | 0, a = px - ix, b = py - iy, j = iy * NX + ix;
    dye2[k] = ((dye[j] * (1 - a) + dye[j + 1] * a) * (1 - b) + (dye[j + NX] * (1 - a) + dye[j + NX + 1] * a) * b) * 0.9993;
  }
  for (let y = 0; y < NY; y++) { dye2[y * NX] = (y % spacing) < 2 ? 1 : 0; dye2[y * NX + NX - 1] = dye2[y * NX + NX - 2]; }
  for (let x = 0; x < NX; x++) { dye2[x] = 0; dye2[(NY - 1) * NX + x] = 0; }
  const t = dye; dye = dye2; dye2 = t;
}
function stable() { const st = Math.max(1, (N / 400) | 0); for (let k = 0; k < N; k += st) { const r = rho[k]; if (!(r > 0.4 && r < 1.6)) return false; } return true; }
function post() {
  const dp = new Float32Array(nd * 2);
  for (let i = 0; i < nd; i++) { dp[i * 2] = drops[i * 4]; dp[i * 2 + 1] = drops[i * 4 + 1]; }
  const m = { type: 'field', nx: NX, ny: NY, ux: ux.slice(), uy: uy.slice(), rho: rho.slice(), dye: dye.slice(), drops: dp,
    steps, batch, Fx: fn ? fx / fn : 0, Fy: fn ? fy / fn : 0, fn, expSteps, iceK, iceCount, iceVersion, hits };
  const tr = [m.ux.buffer, m.uy.buffer, m.rho.buffer, m.dye.buffer, dp.buffer];
  if (iceVersion !== iceSent) { m.ice = ice.slice(); m.iceBody = new Float32Array(iceBody); tr.push(m.ice.buffer, m.iceBody.buffer); iceSent = iceVersion; }
  fx = fy = fn = 0; batch = 0; want = false;
  postMessage(m, tr);
}
function loop() {
  timer = null;
  if (running && N) {
    const t = performance.now(); let n = 0;
    do {
      collide(); stream(); n++; steps++;
      if (icing.on && steps > icing.settle) { dropsStep(); expSteps++; if ((steps & 3) === 0) runback(); if ((steps & 7) === 0) freezeCheck(); }
    } while (performance.now() - t < 10 && n < 200);   // ~10 ms of work, then rest: keeps one core near half load
    batch += n; advectDye(n);
    if (!stable()) { resetFlow(); postMessage({ type: 'blowup' }); }
  }
  if (want && N) post();
  timer = setTimeout(loop, running ? 8 : 60);
}
onmessage = e => {
  const m = e.data;
  switch (m.type) {
    case 'init': U0 = m.u0; tau0 = m.tau; init(m.nx, m.ny); clean.set(m.clean); geom = m.geom; rebuildSolid(); resetFlow(); computeThickness(); iceVersion++; if (!timer) loop(); break;
    case 'geom': clean.set(m.clean); geom = m.geom; iceMass.fill(0); water.fill(0); wetFlag.fill(0); hotFlag.fill(0); wet = []; hot = []; nd = 0;
      reRasterIce(); rebuildSolid(); computeThickness(); iceVersion++; break;
    case 'tau': tau0 = m.tau; break;
    case 'icing': icing = m.icing; if (!icing.on) nd = 0; break;
    case 'deice': clearIce(); break;
    case 'run': running = m.running; break;
    case 'reset': resetFlow(); break;
    case 'frame': want = true; break;
  }
};
