// Slipstream: aircraft geometry (VLM surfaces, reference values, closed triangle mesh) and a canvas 3D renderer.
(() => {
const { D2R, clamp } = SL;
const G = SL.geom = {};

G.wing = P => {
  const w = P.wing, s = w.b / 2, ct = w.cr * w.taper, tS = Math.tan(w.sweep * D2R), tG = Math.tan(w.dihedral * D2R);
  const c = y => w.cr + (ct - w.cr) * y / s, xle = y => y * tS + 0.25 * (w.cr - c(y)), tw = y => w.inc + w.twist * y / s;
  return { s, ct, c, xle, tw, tG };
};
G.reference = P => {
  const w = P.wing, l = w.taper, S = w.b * w.cr * (1 + l) / 2, W = G.wing(P);
  const mac = 2 / 3 * w.cr * (1 + l + l * l) / (1 + l), ymac = w.b / 6 * (1 + 2 * l) / (1 + l), xmac = W.xle(ymac);
  return { S, cbar: mac, ymac, xmac, xcg: xmac + P.cg / 100 * mac, zcg: 0, AR: w.b * w.b / S };
};
// sections for the lifting surfaces; used by the VLM and by the solid mesh
G.surfaces = (P, itExtra = 0) => {
  const w = P.wing, [mP, pT, tP] = SL.AIRFOILS[w.sec], W = G.wing(P), s = W.s, NS = 16;
  const secs = [{ le: [0, 0, 0], c: w.cr, twist: w.inc }], segs = [];
  if (w.flap && w.flapSpan > 0.05 && w.flapSpan < 0.98) {
    const yf = w.flapSpan * s;
    secs.push({ le: [W.xle(yf), yf, yf * W.tG], c: W.c(yf), twist: W.tw(yf) });
    segs.push({ n: Math.max(4, Math.round(NS * w.flapSpan)), flap: w.flap }, { n: Math.max(4, Math.round(NS * (1 - w.flapSpan))) });
  } else segs.push({ n: NS, flap: w.flapSpan >= 0.98 ? w.flap : 0 });
  const tip = { le: [W.xle(s), s, s * W.tG], c: W.ct, twist: W.tw(s) };
  secs.push(tip);
  if (w.wlH > 0.01) {
    const hw = w.wlH * s, ph = w.wlCant * D2R;
    secs.push({ le: [tip.le[0] + hw * Math.tan(35 * D2R) + 0.225 * W.ct, s + hw * Math.cos(ph), tip.le[2] + hw * Math.sin(ph)], c: 0.55 * W.ct, twist: tip.twist });
    segs.push({ n: 6 });
  }
  const out = [{ id: 'wing', sections: secs, segs, m: mP / 100, p: pT / 10, t: tP / 100, nc: 5 }];
  const T = P.tail;
  if (T.on) {
    const sh = T.bt / 2, ctt = T.ct * 0.6, it = T.it + itExtra;
    out.push({ id: 'tail', sections: [{ le: [T.lt, 0, T.zt], c: T.ct, twist: it }, { le: [T.lt + sh * Math.tan(T.tsweep * D2R) + 0.25 * (T.ct - ctt), sh, T.zt], c: ctt, twist: it }], segs: [{ n: 8 }], m: 0, p: 0, t: 0.1, nc: 4 });
  }
  return out;
};

// ---------- closed triangle mesh (both halves), in metres; x aft, y starboard, z up ----------
G.mesh = (P, opts = {}) => {
  const pos = [], idx = [], part = [];
  const addV = (v) => { pos.push(v[0], v[1], v[2]); return pos.length / 3 - 1; };
  const quad = (a, b, c, d, pt) => { idx.push(a, b, c, a, c, d); part.push(pt, pt); };
  const surf = (sf, pt, ring = 22) => {
    const prof = SL.naca.points(sf.m, sf.p, Math.max(0.06, sf.t), 0, ring);   // closed loop, upper TE → LE → lower
    const secs = sf.sections, segUp = [];
    for (let g = 0; g < secs.length - 1; g++) { const a = secs[g], b = secs[g + 1], ph = Math.atan2(b.le[2] - a.le[2], b.le[1] - a.le[1]); segUp.push([0, -Math.sin(ph), Math.cos(ph)]); }
    const secUp = secs.map((_, k) => { const a = segUp[Math.max(0, k - 1)], b = segUp[Math.min(segUp.length - 1, k)]; const v = k === 0 ? [0, 0, 1] : [0, a[1] + b[1], a[2] + b[2]], l = Math.hypot(v[1], v[2]) || 1; return [0, v[1] / l, v[2] / l]; });
    const stations = [];
    for (let g = 0; g < secs.length - 1; g++) {
      const n = Math.max(3, Math.round((sf.segs[g].n || 8) * 0.75));
      for (let i = (g === 0 ? 0 : 1); i <= n; i++) stations.push({ g, s: i / n });
    }
    const ringAt = (st, side) => {
      const a = secs[st.g], b = secs[st.g + 1], s = st.s;
      const le = [0, 1, 2].map(k => a.le[k] + (b.le[k] - a.le[k]) * s), c = a.c + (b.c - a.c) * s, t = (a.twist + (b.twist - a.twist) * s) * D2R;
      const ua = secUp[st.g], ub = secUp[st.g + 1], uy = ua[1] + (ub[1] - ua[1]) * s, uz = ua[2] + (ub[2] - ua[2]) * s, ul = Math.hypot(uy, uz) || 1, u = [0, uy / ul, uz / ul];
      const d = [Math.cos(t), -Math.sin(t) * u[1], -Math.sin(t) * u[2]], nu = [Math.sin(t), Math.cos(t) * u[1], Math.cos(t) * u[2]];
      return prof.map(([x, y]) => addV([le[0] + c * (x * d[0] + y * nu[0]), side * (le[1] + c * (x * d[1] + y * nu[1])), le[2] + c * (x * d[2] + y * nu[2])]));
    };
    for (const side of [1, -1]) {
      const rings = stations.map(st => ringAt(st, side)), n = prof.length;
      for (let r = 0; r + 1 < rings.length; r++) for (let k = 0; k < n; k++) {
        const a = rings[r][k], b = rings[r][(k + 1) % n], c = rings[r + 1][(k + 1) % n], d = rings[r + 1][k];
        side > 0 ? quad(a, b, c, d, pt) : quad(a, d, c, b, pt);
      }
      // tip cap
      const last = rings[rings.length - 1]; let cx = 0, cy = 0, cz = 0;
      for (const i of last) { cx += pos[i * 3]; cy += pos[i * 3 + 1]; cz += pos[i * 3 + 2]; }
      const ci = addV([cx / n, cy / n, cz / n]);
      for (let k = 0; k < n; k++) { idx.push(ci, last[k], last[(k + 1) % n]); part.push(pt); }
    }
  };
  const S = G.surfaces(P);
  surf(S[0], 'wing'); if (S[1]) surf(S[1], 'tail', 16);
  // fuselage: body of revolution with a raised tail cone
  const B = P.body, R = B.df / 2, NA = 26, NR = 18;
  const rad = u => u < 0.18 ? R * Math.sqrt(Math.max(0, 1 - ((0.18 - u) / 0.18) ** 2)) : u < 0.6 ? R : R * (1 - 0.82 * ((u - 0.6) / 0.4) ** 1.3);
  const zc = u => (u > 0.6 ? 0.3 * R * ((u - 0.6) / 0.4) ** 2 : 0) - 0.2 * R;
  const rings = [];
  for (let i = 0; i <= NA; i++) {
    const u = i / NA, r = Math.max(rad(u), i === 0 ? 0 : 0.02 * R), x = B.xnose + u * B.lf, ring = [];
    for (let j = 0; j < NR; j++) { const th = 2 * Math.PI * j / NR; ring.push(addV([x, r * Math.cos(th), zc(u) + r * Math.sin(th)])); }
    rings.push(ring);
  }
  for (let i = 0; i < NA; i++) for (let j = 0; j < NR; j++) quad(rings[i][j], rings[i + 1][j], rings[i + 1][(j + 1) % NR], rings[i][(j + 1) % NR], 'body');
  { const last = rings[NA], ci = addV([B.xnose + B.lf, 0, zc(1)]); for (let j = 0; j < NR; j++) { idx.push(ci, last[(j + 1) % NR], last[j]); part.push('body'); } }
  // fin (thin symmetric slab)
  if (P.tail.on) {
    const T = P.tail, h = Math.max(0.38 * T.bt, T.zt + 0.3 * T.bt), x0 = T.lt - 0.15 * T.ct, c0 = T.ct * 1.25, c1 = c0 * 0.6, sw = 0.6 * c0, th = 0.05;
    const prof = SL.naca.points(0, 0, 0.1, 0, 10);
    const ringF = (z, xl, c) => prof.map(([x, y]) => addV([xl + x * c, y * c, z]));
    const r0 = ringF(0.1 * R, x0, c0), r1 = ringF(h, x0 + sw, c1), n = prof.length;
    for (let k = 0; k < n; k++) quad(r0[k], r1[k], r1[(k + 1) % n], r0[(k + 1) % n], 'fin');
    let cx = 0, cy = 0, cz = 0; for (const i of r1) { cx += pos[i * 3]; cy += pos[i * 3 + 1]; cz += pos[i * 3 + 2]; }
    const ci = addV([cx / n, cy / n, cz / n]); for (let k = 0; k < n; k++) { idx.push(ci, r1[(k + 1) % n], r1[k]); part.push('fin'); }
    void th;
  }
  return { pos: new Float32Array(pos), idx: new Uint32Array(idx), part };
};

// ---------- canvas 3D renderer (flat-shaded, depth-sorted) ----------
G.View3D = class {
  constructor(canvas) {
    this.cv = canvas; this.ctx = canvas.getContext('2d');
    this.cam = { yaw: 3.75, pitch: 0.5, zoom: 1, ortho: false, target: [0, 0, 0], size: 10 };
    this.items = []; this.lines = []; this.markers = [];
    let drag = null;
    canvas.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, yaw: this.cam.yaw, pitch: this.cam.pitch }; canvas.setPointerCapture(e.pointerId); });
    canvas.addEventListener('pointermove', e => { if (!drag) return; this.cam.yaw = drag.yaw - (e.clientX - drag.x) * 0.008; this.cam.pitch = clamp(drag.pitch + (e.clientY - drag.y) * 0.006, -1.5, 1.5); this.draw(); });
    canvas.addEventListener('pointerup', () => drag = null); canvas.addEventListener('pointercancel', () => drag = null);
    canvas.addEventListener('wheel', e => { e.preventDefault(); this.cam.zoom = clamp(this.cam.zoom * Math.exp(-e.deltaY * 0.001), 0.3, 6); this.draw(); }, { passive: false });
    canvas.style.touchAction = 'none';
  }
  preset(k) {
    const P = { iso: [3.75, 0.5, false], top: [Math.PI, Math.PI / 2 - 1e-4, true], front: [Math.PI, 0, true], side: [-Math.PI / 2, 0, true], rear: [0, 0.12, false] }[k];
    [this.cam.yaw, this.cam.pitch, this.cam.ortho] = P; this.cam.zoom = 1; this.draw();
  }
  size() {
    const dpr = Math.min(2, window.devicePixelRatio || 1), w = this.cv.clientWidth, h = this.cv.clientHeight;
    if (!w || !h) return false;
    if (this.cv.width !== Math.round(w * dpr)) { this.cv.width = Math.round(w * dpr); this.cv.height = Math.round(h * dpr); }
    this.dpr = dpr; this.W = this.cv.width; this.H = this.cv.height; return true;
  }
  basis() {
    const c = this.cam, cp = Math.cos(c.pitch), dir = [cp * Math.cos(c.yaw), cp * Math.sin(c.yaw), Math.sin(c.pitch)];
    const dist = c.size * 2.5 / c.zoom, eye = [c.target[0] + dir[0] * dist, c.target[1] + dir[1] * dist, c.target[2] + dir[2] * dist];
    const f = norm([-dir[0], -dir[1], -dir[2]]);
    const upW = Math.abs(f[2]) > 0.999 ? [-1, 0, 0] : [0, 0, 1];
    const r = norm(cross(f, upW)), u = cross(r, f);
    const foc = Math.min(this.W, this.H) * 1.15, ortho = c.ortho, scale = Math.min(this.W, this.H) * 0.45 / (c.size / c.zoom);
    return { eye, f, r, u, project: p => {
      const q = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]], z = dot(q, f);
      if (ortho) return [this.W / 2 + dot(q, r) * scale, this.H / 2 - dot(q, u) * scale, z];
      return [this.W / 2 + dot(q, r) / z * foc, this.H / 2 - dot(q, u) / z * foc, z];
    } };
  }
  draw() {
    if (!this.size()) return;
    const x = this.ctx, B = this.basis(), dpr = this.dpr;
    x.fillStyle = SL.css('--vp'); x.fillRect(0, 0, this.W, this.H);
    // reference grid on the ground plane
    const gz = this.groundZ ?? -1, ext = this.cam.size * 1.2, tg = this.cam.target, step = SL.niceStep(ext * 2, 10);
    x.strokeStyle = SL.css('--vp-grid'); x.lineWidth = dpr;
    for (let k = -Math.ceil(ext / step); k <= Math.ceil(ext / step); k++) {
      const segs = [[[tg[0] + k * step, tg[1] - ext, gz], [tg[0] + k * step, tg[1] + ext, gz]], [[tg[0] - ext, tg[1] + k * step, gz], [tg[0] + ext, tg[1] + k * step, gz]]];
      for (const [a, b] of segs) { const A = B.project(a), C = B.project(b); if (A[2] > 0 && C[2] > 0) { x.beginPath(); x.moveTo(A[0], A[1]); x.lineTo(C[0], C[1]); x.stroke(); } }
    }
    for (const l of this.lines) {
      const A = B.project(l.a), C = B.project(l.b); if (A[2] <= 0 || C[2] <= 0) continue;
      x.strokeStyle = l.col; x.lineWidth = (l.w || 1) * dpr; x.beginPath(); x.moveTo(A[0], A[1]); x.lineTo(C[0], C[1]); x.stroke();
    }
    const L = norm([-0.35, 0.3, 0.88]);
    const list = this.items.map(it => { const pr = it.pts.map(B.project); return { it, pr, z: pr.reduce((s, v) => s + v[2], 0) / pr.length }; });
    list.sort((a, b) => b.z - a.z);
    for (const { it, pr } of list) {
      if (pr.some(v => v[2] <= 0)) continue;
      const p = it.pts, n = norm(cross(sub(p[1], p[0]), sub(p[2], p[0])));
      const lam = 0.42 + 0.58 * Math.abs(dot(n, L));
      const c = it.col;
      x.fillStyle = `rgb(${c[0] * lam | 0},${c[1] * lam | 0},${c[2] * lam | 0})`;
      x.beginPath(); pr.forEach((v, i) => i ? x.lineTo(v[0], v[1]) : x.moveTo(v[0], v[1])); x.closePath(); x.fill();
      if (it.edge) { x.strokeStyle = it.edge; x.lineWidth = 0.6 * dpr; x.stroke(); }
      else { x.strokeStyle = x.fillStyle; x.lineWidth = 0.6 * dpr; x.stroke(); }
    }
    for (const m of this.markers) {
      const v = B.project(m.p); if (v[2] <= 0) continue;
      x.fillStyle = m.col; x.strokeStyle = SL.css('--vp'); x.lineWidth = 2 * dpr;
      x.beginPath(); x.arc(v[0], v[1], 4.5 * dpr, 0, 7); x.fill(); x.stroke();
      x.font = `${11 * dpr}px ${SL.css('--mono')}`; x.textAlign = 'left'; x.textBaseline = 'middle';
      x.fillStyle = SL.css('--text'); x.fillText(m.label, v[0] + 9 * dpr, v[1] + (m.dy || 0) * dpr);
    }
    this.drawAxes(B);
  }
  drawAxes(B) {
    const x = this.ctx, dpr = this.dpr, o = [44 * dpr, this.H - 40 * dpr], L = 22 * dpr;
    for (const [v, lab, col] of [[[1, 0, 0], 'x', '#e0605a'], [[0, 1, 0], 'y', '#57b37a'], [[0, 0, 1], 'z', '#5b9cf2']]) {
      const sx = dot(v, B.r), sy = dot(v, B.u);
      x.strokeStyle = col; x.lineWidth = 1.5 * dpr; x.beginPath(); x.moveTo(o[0], o[1]); x.lineTo(o[0] + sx * L, o[1] - sy * L); x.stroke();
      x.fillStyle = col; x.font = `${10 * dpr}px ${SL.css('--mono')}`; x.textAlign = 'center'; x.textBaseline = 'middle'; x.fillText(lab, o[0] + sx * (L + 8 * dpr), o[1] - sy * (L + 8 * dpr));
    }
  }
};
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]], dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]], norm = a => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
G.vec = { sub, dot, cross, norm };

// mesh → renderer items (triangles), coloured by part
G.meshItems = (M, colours, hideR = 0) => {
  const items = [], p = M.pos;
  for (let t = 0; t < M.idx.length / 3; t++) {
    const a = M.idx[t * 3] * 3, b = M.idx[t * 3 + 1] * 3, c = M.idx[t * 3 + 2] * 3;
    if (hideR && M.part[t] !== 'body' && M.part[t] !== 'fin' && Math.max(Math.abs(p[a + 1]), Math.abs(p[b + 1]), Math.abs(p[c + 1])) < hideR) continue;
    items.push({ pts: [[p[a], p[a + 1], p[a + 2]], [p[b], p[b + 1], p[b + 2]], [p[c], p[c + 1], p[c + 2]]], col: colours[M.part[t]] || colours.body });
  }
  return items;
};
})();
