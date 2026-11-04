// Designer view: build a wing + tail, analyse it with the vortex-lattice method, check stall, stability and trim.
(() => {
const { $, fmt, clamp, D2R } = SL;
const G0 = 9.80665;
const AIRFOILS = { '0009': [0, 0, 9], '0012': [0, 0, 12], '2412': [2, 4, 12], '2415': [2, 4, 15], '4412': [4, 4, 12], '4415': [4, 4, 15], '6409': [6, 4, 9] };
const BASE = {
  b: 11, cr: 1.65, taper: 0.65, sweep: 0, dihedral: 5, twist: -2, inc: 2, sec: '2412', flapSpan: 0.6, flap: 0,
  wlH: 0, wlCant: 75,
  tail: true, bt: 3.6, ct: 0.85, lt: 4.6, zt: 0.3, it: -2, tsweep: 5,
  cg: 25, lf: 7.8, df: 1.2, xnose: -1.9,
  mass: 1100, h: 2500, V: 60, engine: 'prop', power: 134, thrust: 20,
};
const PRESETS = {
  trainer: { name: 'Light trainer' },
  glider: { name: 'Sailplane', b: 18, cr: 0.95, taper: 0.45, sweep: 0, dihedral: 3, twist: -2.5, inc: 2, sec: '4415', bt: 2.6, ct: 0.6, lt: 4.6, zt: 1.0, it: -1.5, tsweep: 5, lf: 7, df: 0.7, xnose: -1.6, mass: 420, h: 1500, V: 28, engine: 'none' },
  airliner: { name: 'Airliner on approach', b: 34, cr: 7.0, taper: 0.24, sweep: 25, dihedral: 6, twist: -3, inc: 2, sec: '2412', flapSpan: 0.6, flap: 15, bt: 12.5, ct: 3.6, lt: 17.5, zt: 0.8, it: -2, tsweep: 30, lf: 38, df: 4, xnose: -13, mass: 64000, h: 300, V: 75, engine: 'jet', thrust: 240 },
  swept: { name: 'Swept, no twist', sweep: 35, taper: 0.3, twist: 0, dihedral: 0, cr: 2.3 },
  forward: { name: 'Forward-swept', sweep: -25, taper: 0.5, twist: 0, dihedral: 0, cr: 2.0 },
  winglets: { name: 'Trainer with winglets', wlH: 0.14, wlCant: 80 },
  unstable: { name: 'CG too far aft', cg: 62 },
};
let D = { ...BASE }, A = null, active = false, dirty = true, last = 0, cam = { yaw: 3.75, pitch: 0.55, zoom: 1 }, colourMode = 'load';

// ---------- geometry ----------
function wingGeom(d) {
  const s = d.b / 2, ct = d.cr * d.taper, tS = Math.tan(d.sweep * D2R), tG = Math.tan(d.dihedral * D2R);
  const c = y => d.cr + (ct - d.cr) * y / s, xle = y => y * tS + 0.25 * (d.cr - c(y)), tw = y => d.inc + d.twist * y / s;
  return { s, ct, c, xle, tw, tG };
}
function buildSurfaces(d, itExtra = 0) {
  const [mPct, pT, tPct] = AIRFOILS[d.sec], m = mPct / 100, p = pT / 10, t = tPct / 100;
  const w = wingGeom(d), s = w.s;
  const secs = [{ le: [0, 0, 0], c: d.cr, twist: d.inc }], segs = [];
  const NS = 16;
  if (d.flap && d.flapSpan > 0.05 && d.flapSpan < 0.98) {
    const yf = d.flapSpan * s;
    secs.push({ le: [w.xle(yf), yf, yf * w.tG], c: w.c(yf), twist: w.tw(yf) });
    segs.push({ n: Math.max(4, Math.round(NS * d.flapSpan)), flap: d.flap });
    segs.push({ n: Math.max(4, Math.round(NS * (1 - d.flapSpan))) });
  } else segs.push({ n: NS, flap: d.flapSpan >= 0.98 ? d.flap : 0 });
  const tip = { le: [w.xle(s), s, s * w.tG], c: w.ct, twist: w.tw(s) };
  secs.push(tip);
  if (d.wlH > 0.01) {
    const hw = d.wlH * s, ph = d.wlCant * D2R;
    secs.push({ le: [tip.le[0] + hw * Math.tan(35 * D2R) + 0.45 * w.ct * 0.5, s + hw * Math.cos(ph), tip.le[2] + hw * Math.sin(ph)], c: 0.55 * w.ct, twist: tip.twist });
    segs.push({ n: 6 });
  }
  const surfs = [{ id: 'wing', sections: secs, segs, m, p, t, nc: 5 }];
  if (d.tail) {
    const sh = d.bt / 2, ctt = d.ct * 0.6, it = d.it + itExtra;
    surfs.push({ id: 'tail', sections: [{ le: [d.lt, 0, d.zt], c: d.ct, twist: it }, { le: [d.lt + sh * Math.tan(d.tsweep * D2R) + 0.25 * (d.ct - ctt), sh, d.zt], c: ctt, twist: it }], segs: [{ n: 8 }], m: 0, p: 0, t: 0.1, nc: 4 });
  }
  return { surfs, w, m, p, t };
}
function reference(d) {
  const w = wingGeom(d), l = d.taper, S = d.b * d.cr * (1 + l) / 2;
  const mac = 2 / 3 * d.cr * (1 + l + l * l) / (1 + l), ymac = d.b / 6 * (1 + 2 * l) / (1 + l), xmac = w.xle(ymac);
  return { S, cbar: mac, ymac, xmac, xcg: xmac + d.cg / 100 * mac, zcg: 0, AR: d.b * d.b / S };
}

// ---------- drag / stall models ----------
const cfTurb = Re => 0.455 / Math.pow(Math.log10(Math.max(Re, 1e4)), 2.58);
function clmaxSection(tPct, mPct, Re, flap) {
  let c = 1.6 - 0.0194 * Math.max(0, 12 - tPct) ** 2 - 0.006 * Math.max(0, tPct - 15) ** 2 + 0.05 * Math.min(mPct, 4);
  c *= clamp(Math.pow(Re / 6e6, 0.08), 0.75, 1.05);
  if (flap) c += 0.9 * clamp(flap / 40, 0, 1);
  return c;
}

// ---------- analysis ----------
function analyse() {
  const ref = reference(D), geo = buildSurfaces(D), atm = SL.isa(D.h);
  const g = VLM.analyse(geo.surfs, ref);
  let gT = null; if (D.tail) gT = VLM.analyse(buildSurfaces(D, 1).surfs, ref);
  const a1 = D2R, r0 = VLM.evaluate(g, 0);
  const cl0 = g.strips.map(s => s.cl);
  const r1 = VLM.evaluate(g, a1);
  const cla = g.strips.map((s, i) => (s.cl - cl0[i]) / a1);
  const CL0 = r0.CL, CLa = (r1.CL - r0.CL) / a1, Cm0 = r0.Cm, Cma = (r1.Cm - r0.Cm) / a1;
  let CLit = 0, Cmit = 0; if (gT) { const t0 = VLM.evaluate(gT, 0); CLit = (t0.CL - CL0) / D2R; Cmit = (t0.Cm - Cm0) / D2R; }
  const [mPct, , tPct] = AIRFOILS[D.sec];
  const V = D.V, rho = atm.rho, mu = atm.mu;
  g.strips.forEach((s, i) => {
    const Re = rho * V * s.c / mu;
    s.Re = Re; s.cl0 = cl0[i]; s.cla = cla[i];
    const isWing = s.surf === 'wing';
    s.clmax = isWing ? clmaxSection(tPct, mPct, Re, s.flap ? D.flap : 0) : 1.2;
    const tt = isWing ? tPct / 100 : 0.1;
    s.cd0 = 2 * cfTurb(Re) * (1 + 2 * tt + 100 * tt ** 4) * 1.02;
    s.clmd = isWing ? 12 * mPct / 100 : 0;
  });
  // critical-section stall on the main wing
  let aStall = Infinity, stallStrip = null;
  g.strips.forEach(s => { if (s.surf !== 'wing' || s.side !== 1 || s.cla <= 0) return; const a = (s.clmax - s.cl0) / s.cla; if (a < aStall) { aStall = a; stallStrip = s; } });
  const CLmax = CL0 + CLa * aStall;
  // fuselage + fin parasite drag
  const Ref = rho * V * D.lf / mu, f = D.lf / D.df, Swf = 0.75 * Math.PI * D.df * D.lf;
  const CDf = cfTurb(Ref) * (1 + 60 / f ** 3 + f / 400) * Swf / ref.S;
  const Svt = D.tail ? 0.35 * D.bt * D.ct : 0, CDvt = D.tail ? 2 * cfTurb(rho * V * D.ct / mu) * 1.25 * Svt / ref.S : 0;
  const strips = g.strips;
  const profile = al => { let s = 0; for (const st of strips) { const cl = st.cl0 + st.cla * al; s += (st.cd0 + 0.006 * (cl - st.clmd) ** 2) * st.c * st.dyProj; } return s / ref.S; };
  const polarAt = al => { const r = VLM.evaluate(g, al); const CDp = (profile(al) + CDf + CDvt) * 1.05; return { al, CL: r.CL, CDi: r.CDi, CDp, CD: r.CDi + CDp, Cm: r.Cm }; };
  // flight condition
  const W = D.mass * G0, q = 0.5 * rho * V * V, CLreq = W / (q * ref.S), alReq = (CLreq - CL0) / CLa;
  const op = polarAt(alReq);
  const polar = []; for (let a = -4; a <= aStall / D2R + 0.01; a += Math.max(0.5, (aStall / D2R + 4) / 30)) polar.push(polarAt(a * D2R));
  let best = polar[0]; polar.forEach(pt => { if (pt.CL > 0 && pt.CL / pt.CD > best.CL / best.CD) best = pt; });
  // trim (tail incidence) for the required lift
  let trim = null;
  if (D.tail && Math.abs(CLit) + Math.abs(Cmit) > 0) {
    const det = CLa * Cmit - CLit * Cma;
    if (Math.abs(det) > 1e-9) { const da = ((CLreq - CL0) * Cmit - CLit * (-Cm0)) / det, di = (CLa * (-Cm0) - Cma * (CLreq - CL0)) / det; trim = { alpha: da, dit: di }; }
  }
  // evaluate at operating α so the drawing shows the cruise loading
  const rOp = VLM.evaluate(g, alReq);
  const SM = -Cma / CLa, xnp = ref.xcg + SM * ref.cbar;
  const Vs = Math.sqrt(2 * W / (rho * ref.S * Math.max(0.1, CLmax)));
  const e = op.CL ** 2 / (Math.PI * ref.AR * Math.max(1e-6, op.CDi));
  const Dn = q * ref.S * op.CD, Preq = Dn * V;
  let avail = null;
  if (D.engine === 'prop') avail = { T: Math.min(D.power * 1000 * 0.8 * atm.sigma / V, 30 * D.power * atm.sigma) };
  else if (D.engine === 'jet') avail = { T: D.thrust * 1000 * atm.sigma ** 0.85 * (1 - 0.25 * V / atm.a) };
  A = { ref, geo, g, atm, CL0, CLa, Cm0, Cma, CLit, Cmit, aStall, stallStrip, CLmax, CDf, CDvt, W, q, CLreq, alReq, op, polar, best, trim, SM, xnp, Vs, e, Dn, Preq, avail, rOp, profile, polarAt, M: V / atm.a };
}

// ---------- 3D drawing ----------
const view = $('d-view'), ctx = view.getContext('2d');
let W = 0, H = 0, dpr = 1;
function size() {
  dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = view.parentElement.clientWidth; if (!w) return;
  const h = Math.round(Math.min(560, Math.max(320, w * 0.52)));
  W = Math.round(w * dpr); H = Math.round(h * dpr); view.width = W; view.height = H; view.style.height = h + 'px';
}
new ResizeObserver(() => { if (active) { size(); draw(); } }).observe($('d-stage'));
function camera() {
  const span = Math.max(D.b, D.lf), tgt = [D.lt * 0.45, 0, 0], dist = span * 1.55 / cam.zoom;
  const eye = [tgt[0] + dist * Math.cos(cam.pitch) * Math.cos(cam.yaw), tgt[1] + dist * Math.cos(cam.pitch) * Math.sin(cam.yaw), tgt[2] + dist * Math.sin(cam.pitch)];
  const f = SL3.norm(SL3.sub(tgt, eye)), r = SL3.norm(SL3.cross(f, [0, 0, 1])), u = SL3.cross(r, f);
  const foc = Math.min(W, H) * 1.25;
  return { eye, f, r, u, project(p) { const q = SL3.sub(p, eye), z = SL3.dot(q, f); return [W / 2 + SL3.dot(q, r) / z * foc, H * 0.52 - SL3.dot(q, u) / z * foc, z]; } };
}
const SL3 = { sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]], dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2], cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]], norm: a => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; } };
const lerpC = (a, b, t) => a.map((v, i) => Math.round(v + (b[i] - v) * t));
function stallColour(r) {
  if (r < 0.7) return lerpC([46, 160, 110], [214, 200, 70], clamp(r / 0.7, 0, 1));
  return lerpC([214, 200, 70], [215, 55, 50], clamp((r - 0.7) / 0.3, 0, 1));
}
function fuselageQuads() {
  const qs = [], NA = 22, NR = 14, R = D.df / 2;
  const rad = u => u < 0.2 ? R * Math.sqrt(1 - ((0.2 - u) / 0.2) ** 2) : u < 0.62 ? R : R * (1 - 0.75 * ((u - 0.62) / 0.38) ** 1.3);
  const zc = u => u > 0.62 ? 0.25 * R * ((u - 0.62) / 0.38) ** 2 : 0;
  const P = (i, j) => { const u = i / NA, th = 2 * Math.PI * j / NR, r = rad(u); return [D.xnose + u * D.lf, r * Math.cos(th), zc(u) - 0.25 * R + r * Math.sin(th)]; };
  for (let i = 0; i < NA; i++) for (let j = 0; j < NR; j++) qs.push({ pts: [P(i, j), P(i + 1, j), P(i + 1, j + 1), P(i, j + 1)], kind: 'body' });
  if (D.tail) {
    const h = 0.38 * D.bt, x0 = D.lt - 0.15 * D.ct, c0 = D.ct * 1.2;
    qs.push({ pts: [[x0, 0, 0.2 * R], [x0 + c0, 0, 0.2 * R], [x0 + c0 + 0.1 * c0, 0, h], [x0 + 0.55 * c0, 0, h]], kind: 'fin' });
  }
  return qs;
}
function draw() {
  if (!W || !A) return;
  const g = A.g, C = camera();
  const bg = ctx.createLinearGradient(0, 0, 0, H); bg.addColorStop(0, '#0b1724'); bg.addColorStop(1, '#16283a');
  ctx.fillStyle = bg; ctx.fillRect(0, 0, W, H);
  // ground grid for orientation
  ctx.strokeStyle = 'rgba(140,180,210,0.08)'; ctx.lineWidth = dpr;
  const gz = -D.df * 1.2, ext = Math.max(D.b, D.lf) * 0.9;
  for (let k = -6; k <= 6; k++) {
    const a = C.project([D.lt * 0.45 + k * ext / 6, -ext, gz]), b = C.project([D.lt * 0.45 + k * ext / 6, ext, gz]);
    const c = C.project([D.lt * 0.45 - ext, k * ext / 6, gz]), d = C.project([D.lt * 0.45 + ext, k * ext / 6, gz]);
    if (a[2] > 0 && b[2] > 0) { ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); }
    if (c[2] > 0 && d[2] > 0) { ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(d[0], d[1]); ctx.stroke(); }
  }
  // trailing vortices from strip edges (strength ∝ circulation jump)
  let gmax = 1e-9;
  for (const t of g.tre) for (let k = 0; k <= t.ss.length; k++) gmax = Math.max(gmax, Math.abs((k > 0 ? t.ss[k - 1].G : 0) - (k < t.ss.length ? t.ss[k].G : 0)));
  for (const t of g.tre) {
    const ss = t.ss;
    for (let k = 0; k <= ss.length; k++) {
      const gL = k > 0 ? ss[k - 1].G : 0, gR = k < ss.length ? ss[k].G : 0, str = Math.abs(gL - gR) / gmax;
      if (str < 0.04) continue;
      const e = k < ss.length ? ss[k].te[0] : ss[k - 1].te[1];
      const a = C.project(e), b = C.project([e[0] + D.b * 0.9, e[1], e[2] - D.b * 0.06 * Math.sin(A.alReq)]);
      if (a[2] <= 0 || b[2] <= 0) continue;
      ctx.strokeStyle = `rgba(150,215,255,${clamp(0.15 + 0.75 * str, 0.15, 0.9)})`; ctx.lineWidth = dpr * (0.8 + 2.2 * str);
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    }
  }
  // collect quads
  const quads = fuselageQuads();
  let dmax = 1e-9; g.panels.forEach(p => dmax = Math.max(dmax, p.dcp));
  g.panels.forEach(p => quads.push({ pts: p.corners, kind: 'panel', p }));
  const L1 = SL3.norm([-0.3, 0.4, 0.85]);
  quads.forEach(q => { q.pr = q.pts.map(C.project); q.z = q.pr.reduce((s, v) => s + v[2], 0) / 4; });
  quads.sort((a, b) => b.z - a.z);
  for (const q of quads) {
    if (q.pr.some(v => v[2] <= 0)) continue;
    const n = SL3.norm(SL3.cross(SL3.sub(q.pts[2], q.pts[0]), SL3.sub(q.pts[3], q.pts[1])));
    const lam = 0.55 + 0.45 * Math.abs(SL3.dot(n, L1));
    let col;
    if (q.kind === 'panel') {
      if (colourMode === 'stall') col = stallColour(q.p.strip.surf === 'wing' ? (q.p.strip.cl0 + q.p.strip.cla * A.alReq) / q.p.strip.clmax : 0.2);
      else { const t = clamp(q.p.dcp / dmax, 0, 1), i = Math.round(t * 255) * 3, M = SL.MAPS.speed; col = [M[i], M[i + 1], M[i + 2]]; }
    } else col = q.kind === 'fin' ? [150, 160, 172] : [196, 204, 214];
    ctx.fillStyle = `rgb(${Math.round(col[0] * lam)},${Math.round(col[1] * lam)},${Math.round(col[2] * lam)})`;
    ctx.beginPath(); q.pr.forEach((v, i) => i ? ctx.lineTo(v[0], v[1]) : ctx.moveTo(v[0], v[1])); ctx.closePath(); ctx.fill();
    if (q.kind === 'panel') { ctx.strokeStyle = 'rgba(10,20,30,0.25)'; ctx.lineWidth = 0.6 * dpr; ctx.stroke(); }
  }
  // CG and neutral point markers above the fuselage
  const zm = D.df * 0.55;
  const mark = (x, col, label, dy) => {
    const p = C.project([x, 0, zm]); if (p[2] <= 0) return;
    ctx.fillStyle = col; ctx.beginPath(); ctx.arc(p[0], p[1], 6 * dpr, 0, 7); ctx.fill();
    ctx.strokeStyle = '#08131d'; ctx.lineWidth = 1.5 * dpr; ctx.stroke();
    ctx.font = `700 ${12 * dpr}px ${SL.css('--f-body')}`; ctx.textAlign = 'center';
    ctx.lineWidth = 3 * dpr; ctx.strokeStyle = 'rgba(5,15,25,0.8)'; ctx.strokeText(label, p[0], p[1] + dy * dpr); ctx.fillStyle = '#fff'; ctx.fillText(label, p[0], p[1] + dy * dpr);
  };
  mark(A.ref.xcg, '#ffd27a', 'CG', -12);
  if (isFinite(A.xnp)) mark(A.xnp, '#7fe0ff', 'Neutral point', 22);
}
let drag = null;
view.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, yaw: cam.yaw, pitch: cam.pitch }; view.setPointerCapture(e.pointerId); });
view.addEventListener('pointermove', e => { if (!drag) return; cam.yaw = drag.yaw - (e.clientX - drag.x) * 0.008; cam.pitch = clamp(drag.pitch + (e.clientY - drag.y) * 0.006, -0.2, 1.5); draw(); });
view.addEventListener('pointerup', () => drag = null);
view.addEventListener('pointercancel', () => drag = null);
view.addEventListener('wheel', e => { e.preventDefault(); cam.zoom = clamp(cam.zoom * Math.exp(-e.deltaY * 0.001), 0.4, 4); draw(); }, { passive: false });
view.style.touchAction = 'none';
document.querySelector('#d-stage .camctl').addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b) return;
  ({ persp: () => { cam.yaw = 3.75; cam.pitch = 0.55; }, top: () => { cam.yaw = Math.PI; cam.pitch = 1.5; }, front: () => { cam.yaw = Math.PI; cam.pitch = 0.08; }, side: () => { cam.yaw = -Math.PI / 2; cam.pitch = 0.05; } })[b.dataset.cam]();
  draw();
});

// ---------- charts ----------
function drawCharts() {
  const g = A.g, wing = g.strips.filter(s => s.surf === 'wing' && s.side === 1), s = D.b / 2, al = A.alReq;
  // 1. spanwise loading vs elliptic
  {
    const cbar = A.ref.cbar, CL = Math.max(1e-3, A.op.CL);
    const pts = wing.map(st => [Math.hypot(st.mid[1], st.mid[2] * 0) / s, st.c * (st.cl0 + st.cla * al) / (cbar * CL)]);
    const ymax = Math.max(1.6, ...pts.map(p => p[1])) * 1.1;
    const ch = SL.chart($('d-c1'), { x0: 0, x1: Math.max(1, ...pts.map(p => p[0])), y0: 0, y1: ymax, dy: 0.4, dx: 0.2, xLabel: 'position along span (root → tip)', xFmt: v => Math.round(v * 100) + '%', padL: 34 });
    if (ch) {
      const { c, X, Y } = ch;
      c.setLineDash([5, 4]); c.strokeStyle = SL.css('--muted'); c.lineWidth = 1.6; c.beginPath();
      for (let e = 0; e <= 1.0001; e += 0.01) { const v = 4 / Math.PI * Math.sqrt(Math.max(0, 1 - e * e)) * (A.ref.S / (D.b * cbar)); e === 0 ? c.moveTo(X(e), Y(v)) : c.lineTo(X(e), Y(v)); }
      c.stroke(); c.setLineDash([]);
      c.strokeStyle = SL.css('--accent'); c.lineWidth = 2.2; c.beginPath(); pts.forEach((p, i) => i ? c.lineTo(X(p[0]), Y(p[1])) : c.moveTo(X(p[0]), Y(p[1]))); c.stroke();
    }
  }
  // 2. local lift coefficient vs section limit, now and at stall
  {
    const now = wing.map(st => [st.mid[1] / s, st.cl0 + st.cla * al]), atS = wing.map(st => [st.mid[1] / s, st.cl0 + st.cla * A.aStall]), lim = wing.map(st => [st.mid[1] / s, st.clmax]);
    const ymax = Math.max(...lim.map(p => p[1]), ...atS.map(p => p[1])) * 1.15;
    const ch = SL.chart($('d-c2'), { x0: 0, x1: Math.max(1, ...now.map(p => p[0])), y0: 0, y1: ymax, dy: 0.4, dx: 0.2, xLabel: 'position along span (root → tip)', xFmt: v => Math.round(v * 100) + '%', padL: 34 });
    if (ch) {
      const { c, X, Y } = ch;
      const line = (pts, col, w, dash) => { c.strokeStyle = col; c.lineWidth = w; c.setLineDash(dash ? [5, 4] : []); c.beginPath(); pts.forEach((p, i) => i ? c.lineTo(X(p[0]), Y(p[1])) : c.moveTo(X(p[0]), Y(p[1]))); c.stroke(); c.setLineDash([]); };
      line(lim, SL.css('--crit'), 1.6, true); line(atS, SL.css('--warm'), 1.6, false); line(now, SL.css('--accent'), 2.2, false);
      if (A.stallStrip) { const x = X(A.stallStrip.mid[1] / s), y = Y(A.stallStrip.clmax); c.fillStyle = SL.css('--crit'); c.beginPath(); c.arc(x, y, 5, 0, 7); c.fill(); }
    }
  }
  // 3. drag polar
  {
    const P = A.polar, xmax = Math.max(...P.map(p => p.CD)) * 1.08, ymin = Math.min(0, ...P.map(p => p.CL)), ymax = Math.max(...P.map(p => p.CL)) * 1.1;
    const dx = [0.005, 0.01, 0.02, 0.05, 0.1].find(v => xmax / v <= 6) || 0.1;
    const ch = SL.chart($('d-c3'), { x0: 0, x1: xmax, y0: Math.floor(ymin * 5) / 5, y1: ymax, dy: ymax > 2 ? 0.5 : 0.25, dx, xLabel: 'drag coefficient C_D', xFmt: v => v.toFixed(dx < 0.01 ? 3 : 2), padL: 38 });
    if (ch) {
      const { c, X, Y } = ch;
      c.strokeStyle = SL.css('--muted'); c.setLineDash([5, 4]); c.lineWidth = 1.6; c.beginPath(); P.forEach((p, i) => i ? c.lineTo(X(p.CDi), Y(p.CL)) : c.moveTo(X(p.CDi), Y(p.CL))); c.stroke(); c.setLineDash([]);
      c.strokeStyle = SL.css('--accent'); c.lineWidth = 2.2; c.beginPath(); P.forEach((p, i) => i ? c.lineTo(X(p.CD), Y(p.CL)) : c.moveTo(X(p.CD), Y(p.CL))); c.stroke();
      const b = A.best; c.strokeStyle = SL.css('--good'); c.lineWidth = 1.2; c.beginPath(); c.moveTo(X(0), Y(0)); c.lineTo(X(b.CD * 1.4), Y(b.CL * 1.4)); c.stroke();
      c.fillStyle = SL.css('--good'); c.beginPath(); c.arc(X(b.CD), Y(b.CL), 4, 0, 7); c.fill();
      c.fillStyle = SL.css('--ink'); c.beginPath(); c.arc(X(A.op.CD), Y(A.op.CL), 5, 0, 7); c.fill();
    }
  }
  // 4. pitching moment vs angle of attack
  {
    const a0 = -4, a1 = Math.max(4, A.aStall / D2R);
    const cm = a => A.Cm0 + A.Cma * a * D2R;
    const vals = [cm(a0), cm(a1)], lo = Math.min(-0.2, ...vals), hi = Math.max(0.2, ...vals);
    const dy = [0.05, 0.1, 0.2, 0.25, 0.5, 1].find(v => (hi - lo) / v <= 6) || 1;
    const ch = SL.chart($('d-c4'), { x0: a0, x1: a1, y0: Math.floor(lo / dy) * dy, y1: Math.ceil(hi / dy) * dy, dy, dx: a1 - a0 > 16 ? 4 : 2, xLabel: 'angle of attack (°)', xFmt: v => v + '°', padL: 40 });
    if (ch) {
      const { c, X, Y } = ch;
      c.strokeStyle = A.Cma < 0 ? SL.css('--accent') : SL.css('--crit'); c.lineWidth = 2.2; c.beginPath(); c.moveTo(X(a0), Y(cm(a0))); c.lineTo(X(a1), Y(cm(a1))); c.stroke();
      const ar = A.alReq / D2R; if (ar >= a0 && ar <= a1) { c.fillStyle = SL.css('--ink'); c.beginPath(); c.arc(X(ar), Y(cm(ar)), 5, 0, 7); c.fill(); }
    }
  }
}

// ---------- read-outs ----------
const kt = v => Math.round(v * 1.944);
function summary() {
  const a = A, r = a.ref;
  $('d-cl').textContent = fmt(a.op.CL); $('d-cd').textContent = fmt(a.op.CD, 4); $('d-ld').textContent = fmt(a.op.CL / a.op.CD, 1); $('d-e').textContent = fmt(a.e, 3);
  const sm = $('d-chip-sm'), smv = a.SM * 100;
  if (!D.tail) { sm.textContent = 'No tail: pitch stability not assessed'; sm.className = 'chip neutral'; }
  else if (smv < 0) { sm.textContent = `Unstable: static margin ${fmt(smv, 0)}%`; sm.className = 'chip crit'; }
  else if (smv < 5) { sm.textContent = `Marginally stable: ${fmt(smv, 0)}%`; sm.className = 'chip warn'; }
  else if (smv > 35) { sm.textContent = `Very stable, heavy in pitch: ${fmt(smv, 0)}%`; sm.className = 'chip warn'; }
  else { sm.textContent = `Stable: static margin ${fmt(smv, 0)}%`; sm.className = 'chip good'; }
  const st = $('d-chip-stall'), frac = a.stallStrip ? a.stallStrip.mid[1] / (D.b / 2) : 0;
  st.textContent = !a.stallStrip ? 'Stall not found' : frac > 0.6 ? `Tip stalls first (${Math.round(frac * 100)}% span)` : frac > 0.35 ? `Stall starts mid-span (${Math.round(frac * 100)}%)` : `Root stalls first (${Math.round(frac * 100)}% span)`;
  st.className = 'chip ' + (frac > 0.6 ? 'crit' : frac > 0.35 ? 'warn' : 'good');
  const fl = $('d-chip-fly');
  const margin = D.V / a.Vs;
  if (a.CLreq > a.CLmax) { fl.textContent = 'Too slow: below stall speed'; fl.className = 'chip crit'; }
  else if (a.avail && a.Dn > a.avail.T) { fl.textContent = `Not enough thrust (${Math.round(a.Dn / a.avail.T * 100)}% needed)`; fl.className = 'chip crit'; }
  else if (margin < 1.3) { fl.textContent = `Close to stall (${fmt(margin, 2)}× stall speed)`; fl.className = 'chip warn'; }
  else { fl.textContent = 'Can hold level flight'; fl.className = 'chip good'; }
  if (a.M > 0.5) $('d-warn').textContent = `Mach ${a.M.toFixed(2)}: this method ignores compressibility, so results above about Mach 0.5 are optimistic.`;
  else $('d-warn').textContent = '';
  const rows = [
    ['Wing area', `${fmt(r.S, r.S < 50 ? 1 : 0)} m²`], ['Span', `${fmt(D.b, 1)} m`], ['Aspect ratio', fmt(r.AR, 1)], ['Mean aerodynamic chord', `${fmt(r.cbar, 2)} m`],
    ['Lift slope C<sub>Lα</sub>', `${fmt(a.CLa * D2R, 3)} per °`], ['Zero-lift angle', `${fmt(-a.CL0 / a.CLa / D2R, 1)}°`],
    ['Maximum lift C<sub>L max</sub>', fmt(a.CLmax, 2)], ['Stall speed', `${kt(a.Vs)} kt · ${fmt(a.Vs, 1)} m/s`],
    ['Angle for level flight', `${fmt(a.alReq / D2R, 1)}°`], ['Lift coefficient needed', fmt(a.CLreq, 3)],
    ['Induced drag', `${fmt(a.op.CDi, 4)} (${Math.round(a.op.CDi / a.op.CD * 100)}%)`], ['Profile and body drag', `${fmt(a.op.CDp, 4)} (${Math.round(a.op.CDp / a.op.CD * 100)}%)`],
    ['Span efficiency e (projected span)', fmt(a.e, 3)], ['Best L/D', `${fmt(a.best.CL / a.best.CD, 1)} at ${kt(Math.sqrt(2 * a.W / (a.atm.rho * r.S * Math.max(0.05, a.best.CL))))} kt`],
    ['Drag at this speed', `${fmt(a.Dn / 1000, 2)} kN · ${fmt(a.Preq / 1000, 0)} kW`], ['Thrust available', a.avail ? `${fmt(a.avail.T / 1000, 2)} kN` : 'none (glider)'],
    ['Neutral point', D.tail ? `${fmt((a.xnp - r.xmac) / r.cbar * 100, 0)}% MAC` : '—'], ['Centre of gravity', `${D.cg}% MAC`],
    ['Tail change to trim', a.trim ? `${a.trim.dit >= 0 ? '+' : '−'}${fmt(Math.abs(a.trim.dit / D2R), 1)}° → ${fmt(D.it + a.trim.dit / D2R, 1)}°` : '—'],
    ['Reynolds number (MAC)', `${fmt(a.atm.rho * D.V * r.cbar / a.atm.mu / 1e6, 1)} million`],
  ];
  $('d-table').innerHTML = rows.map(([k, v]) => `<div><dt>${k}</dt><dd>${v}</dd></div>`).join('');
}

// ---------- controls ----------
const CTL = [
  ['b', 'Span', v => `${fmt(v, 1)} m`], ['cr', 'Root chord', v => `${fmt(v, 2)} m`], ['taper', 'Taper ratio', v => fmt(v, 2)], ['sweep', 'Sweep', v => `${fmt(v, 0)}°`],
  ['dihedral', 'Dihedral', v => `${fmt(v, 1)}°`], ['twist', 'Twist at tip', v => `${fmt(v, 1)}°`], ['inc', 'Wing incidence', v => `${fmt(v, 1)}°`],
  ['flapSpan', 'Flap span', v => `${Math.round(v * 100)}%`], ['flap', 'Flap angle', v => `${fmt(v, 0)}°`],
  ['wlH', 'Winglet height', v => v < 0.01 ? 'none' : `${Math.round(v * 100)}% semispan`], ['wlCant', 'Winglet cant', v => `${fmt(v, 0)}°`],
  ['bt', 'Tail span', v => `${fmt(v, 1)} m`], ['ct', 'Tail root chord', v => `${fmt(v, 2)} m`], ['lt', 'Tail arm', v => `${fmt(v, 1)} m`], ['zt', 'Tail height', v => `${fmt(v, 2)} m`], ['it', 'Tail incidence', v => `${fmt(v, 1)}°`],
  ['cg', 'Centre of gravity', v => `${fmt(v, 0)}% MAC`], ['lf', 'Fuselage length', v => `${fmt(v, 1)} m`], ['df', 'Fuselage diameter', v => `${fmt(v, 2)} m`],
  ['mass', 'Mass', v => `${SL.int(v)} kg`], ['h', 'Altitude', v => `${SL.int(v)} m`], ['V', 'Airspeed', v => `${fmt(v, 0)} m/s · ${kt(v)} kt`],
  ['power', 'Engine power', v => `${SL.int(v)} kW`], ['thrust', 'Engine thrust', v => `${SL.int(v)} kN`],
];
function rangeFor(k) {   // slider ranges scale with the design's size
  const big = D.b > 20;
  return { b: [4, 70, 0.1], cr: [0.3, 12, 0.05], taper: [0.1, 1, 0.01], sweep: [-35, 50, 1], dihedral: [-8, 12, 0.5], twist: [-8, 4, 0.5], inc: [-4, 8, 0.5],
    flapSpan: [0, 1, 0.05], flap: [0, 40, 1], wlH: [0, 0.3, 0.01], wlCant: [30, 90, 1],
    bt: [0.5, big ? 25 : 8, 0.1], ct: [0.2, big ? 8 : 2.5, 0.05], lt: [1, big ? 40 : 12, 0.1], zt: [-1, big ? 8 : 2.5, 0.05], it: [-8, 4, 0.25],
    cg: [-10, 80, 1], lf: [2, 75, 0.5], df: [0.3, 7, 0.05], mass: big ? [5000, 400000, 500] : [100, 6000, 10], h: [0, 13000, 100], V: [15, 260, 1],
    power: [0, 3000, 5], thrust: [0, 600, 1] }[k];
}
function buildControls() {
  for (const [k, label, f] of CTL) {
    const el = $('d-' + k); if (!el) continue;
    const [lo, hi, st] = rangeFor(k); el.min = lo; el.max = hi; el.step = st; el.value = D[k];
    $('d-' + k + '-o').textContent = f(D[k]);
  }
  $('d-sec').value = D.sec; $('d-tail').checked = D.tail; SL.syncSeg('d-engine', D.engine);
  $('d-power-ctl').hidden = D.engine !== 'prop'; $('d-thrust-ctl').hidden = D.engine !== 'jet';
  document.querySelectorAll('.d-tailonly').forEach(e => e.style.opacity = D.tail ? 1 : 0.45);
}
for (const [k, , f] of CTL) {
  const el = $('d-' + k); if (!el) continue;
  el.addEventListener('input', () => { D[k] = +el.value; $('d-' + k + '-o').textContent = f(D[k]); dirty = true; });
}
$('d-sec').onchange = e => { D.sec = e.target.value; dirty = true; };
$('d-tail').onchange = e => { D.tail = e.target.checked; buildControls(); dirty = true; };
SL.seg('d-engine', v => { D.engine = v; buildControls(); dirty = true; });
SL.seg('d-colour', v => { colourMode = v; draw(); $('d-lname').textContent = v === 'stall' ? 'Stall margin' : 'Lift loading'; $('d-lmin').textContent = v === 'stall' ? 'safe' : 'low'; $('d-lmax').textContent = v === 'stall' ? 'stalling' : 'high'; legendBar(); });
$('d-trim').onclick = () => { if (A && A.trim) { D.it = +(D.it + A.trim.dit / D2R).toFixed(2); buildControls(); dirty = true; } };
$('d-try').addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b) return;
  const p = PRESETS[b.dataset.p]; D = { ...BASE, ...p }; delete D.name; buildControls(); dirty = true;
});
$('d-fly').onclick = () => {
  if (!A) return;
  const r = A.ref, aw = A.CLa, AR = r.AR, e = clamp(A.e, 0.5, 1.05);
  const a2d = aw / Math.max(0.15, 1 - aw / (Math.PI * e * AR));
  const CD0 = A.polarAt((0 - A.CL0) / A.CLa).CDp;
  const prop = D.engine !== 'jet';
  const Vs = A.Vs, vt = D.V;
  SL.emit('design', {
    name: 'Your design', desc: `Your Designer aircraft: ${fmt(D.b, 1)} m span, aspect ratio ${fmt(AR, 1)}, ${SL.int(D.mass)} kg. Lift slope, span efficiency, maximum lift and parasite drag come from the vortex-lattice analysis.`,
    m: D.mass, S: r.S, b: D.b, c: r.cbar, CD0, e, CLmax: A.CLmax, a0: a2d, alpha0: -A.CL0 / A.CLa,
    prop, P: D.engine === 'prop' ? D.power * 1000 : 0, eta: 0.8, Tstatic: D.engine === 'prop' ? 30 * D.power : 0, lapse: 1.0, T0: D.thrust * 1000,
    Mcrit: prop ? 0.62 : 0.72, v: [Math.max(10, Math.round(Vs * 1.05)), Math.max(Math.round(Vs * 1.2) + 5, Math.round(vt * 1.7))], vt, ht: D.h, deice: false, high: false,
  });
  location.hash = 'aircraft';
};
function legendBar() {
  const c = $('d-bar'), x = c.getContext('2d');
  for (let i = 0; i < c.width; i++) {
    const t = i / (c.width - 1);
    const col = colourMode === 'stall' ? stallColour(t) : (() => { const j = Math.round(t * 255) * 3, M = SL.MAPS.speed; return [M[j], M[j + 1], M[j + 2]]; })();
    x.fillStyle = `rgb(${col[0]},${col[1]},${col[2]})`; x.fillRect(i, 0, 1, c.height);
  }
}

function loop(now) {
  if (!active) return;
  requestAnimationFrame(loop);
  if (!dirty || now - last < 60) return;
  dirty = false; last = now;
  const t0 = performance.now();
  try { analyse(); } catch (e) { console.error(e); $('d-warn').textContent = 'This geometry could not be analysed: ' + e.message; return; }
  draw(); drawCharts(); summary();
  $('d-perf').textContent = `${A.g.n} panels · ${Math.round(performance.now() - t0)} ms`;
}
buildControls(); legendBar();
SL.views.designer = {
  activate() { active = true; size(); dirty = true; requestAnimationFrame(loop); },
  deactivate() { active = false; },
};
})();
