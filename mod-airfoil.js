// Airfoil module: 2D lattice Boltzmann flow around the project's wing section, with ice accretion.
(() => {
const { $, fmt, int, D2R, clamp } = SL;
const U0 = 0.1, CHORD = 2.0, TIMELAPSE = 100000;
const RES = { coarse: [300, 140], std: [420, 196], fine: [560, 260] };
const S = { shape: 'airfoil', useFlap: false, aoa: 4, re: 3000, field: 'speed', stream: true, parts: false, res: 'std', running: true,
  icing: false, temp: -12, lwc: 0.5, mvd: 20, vel: 120 };
let NX, NY, N, clean, iceM, polys = [], field = null, fresh = false, active = false, started = false;
const hist = { cl: [], cd: [] }, HMAX = 500;
let baseline = null, iceStats = { k: 0, count: 0, exp: 0 }, lastSteps = 0, rate = 0, rateT = 0, rateS = 0, frames = 0;
let worker = null, secKey = '';
const cv = () => $('cv-airfoil');

// ---------- geometry ----------
const sec = () => { const s = SL.section(); return { m: s.m / 100, p: s.p / 10, t: s.t / 100, flap: S.useFlap ? SL.P.wing.flap : 0 }; };
const refLen = () => S.shape === 'cylinder' ? Math.round(0.13 * NY) : Math.round(0.2 * NX);
const geomInfo = () => { const L = refLen(); return S.shape === 'cylinder' ? { px: 0.25 * NX, py: NY / 2 + 0.37, L, aoa: S.aoa, ox: 0 } : { px: 0.22 * NX + 0.25 * L, py: NY / 2, L, aoa: S.aoa, ox: 0.25 }; };
function buildPolys() {
  const gi = geomInfo(), L = gi.L, a = -S.aoa * D2R, ca = Math.cos(a), sa = Math.sin(a), s = sec();
  let pts;
  if (S.shape === 'airfoil') pts = SL.naca.points(s.m, s.p, s.t, s.flap);
  else { pts = []; for (let i = 0; i < 72; i++) { const t = i / 72 * 2 * Math.PI; pts.push([0.5 * Math.cos(t), 0.5 * Math.sin(t)]); } }
  return [pts.map(([x, y]) => { const dx = x - gi.ox; return [gi.px + L * (dx * ca - y * sa), gi.py + L * (dx * sa + y * ca)]; })];
}
function rasterize(ps) {
  const m = new Uint8Array(N), xs = [];
  for (const poly of ps) for (let y = 0; y < NY; y++) {
    const yc = y + 0.5; xs.length = 0;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) { const [x1, y1] = poly[i], [x2, y2] = poly[j]; if ((y1 <= yc) !== (y2 <= yc)) xs.push(x1 + (yc - y1) * (x2 - x1) / (y2 - y1)); }
    xs.sort((a, b) => a - b);
    for (let i = 0; i + 1 < xs.length; i += 2) { const a = Math.max(2, Math.ceil(xs[i] - 0.5)), b = Math.min(NX - 3, Math.floor(xs[i + 1] - 0.5)); for (let x = a; x <= b; x++) m[y * NX + x] = 1; }
  }
  return m;
}
const reFromSlider = v => 20 * Math.pow(10, v / 1000 * 3);
const tau = () => Math.max(0.5005, 3 * U0 * refLen() / S.re + 0.5);
function sendGeom() { if (!worker) return; polys = buildPolys(); clean = rasterize(polys); worker.postMessage({ type: 'geom', clean, geom: geomInfo() }); worker.postMessage({ type: 'tau', tau: tau() }); sendIcing(); }
function icingParams() {
  const kind = SL.iceKind(S.temp, S.lwc), on = S.icing && kind.key !== 'none';
  const L = refLen(), V = S.vel, d = S.mvd * 1e-6, mu = 1.72e-5, rhoA = 0.9, rhoIce = kind.key === 'rime' ? 880 : 917;
  const St = 1000 * d * d * V / (18 * mu * CHORD), dtReal = CHORD * U0 / (L * V), cell = CHORD / L, spawn = 3;
  const mFrac = (S.lwc * 1e-3) * V * 0.9 * CHORD * dtReal * TIMELAPSE / (spawn * rhoIce * cell * cell);
  return { kind, St, dtReal, cell, icing: { on, tp: Math.max(0.5, St * L / U0), rpk: rhoA * d * (V / U0) / mu, n0: Math.max(0.05, kind.n0), mFrac, spawn, settle: Math.round(2.5 * L / U0) } };
}
function sendIcing() { if (worker && NX) worker.postMessage({ type: 'icing', icing: icingParams().icing }); }
function thin() { if (S.shape === 'cylinder') return null; const s = sec(), a0 = SL.alpha0(s.m, s.p, s.flap * D2R); return { cl: 2 * Math.PI * (S.aoa * D2R - a0), a0: a0 / D2R }; }

// ---------- rendering ----------
const fieldC = document.createElement('canvas'), fctx = fieldC.getContext('2d');
let img, rect = { x: 0, y: 0, w: 1, h: 1 }, dpr = 1, parts = null;
function layout() {
  const c = cv(); dpr = Math.min(2, window.devicePixelRatio || 1);
  const W = c.clientWidth, H = c.clientHeight; if (!W || !H) return false;
  if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
  const ar = NX / NY, cw = c.width, ch = c.height;
  let w = cw - 16 * dpr, h = w / ar; if (h > ch - 16 * dpr) { h = ch - 16 * dpr; w = h * ar; }
  rect = { x: (cw - w) / 2, y: (ch - h) / 2, w, h }; return true;
}
const toScr = (x, y) => [rect.x + x / NX * rect.w, rect.y + (1 - y / NY) * rect.h];
const solidAt = k => clean[k] || iceM[k];
function renderField() {
  if (!field) return;
  const { ux, uy, rho, dye } = field, d = img.data, v = S.field, L = refLen();
  const map = SL.MAPS[v === 'speed' ? 'viridis' : v === 'dye' ? 'grey' : 'coolwarm'];
  for (let y = 0; y < NY; y++) {
    const r = (NY - 1 - y) * NX;
    for (let x = 0; x < NX; x++) {
      const k = y * NX + x, o = (r + x) * 4;
      if (clean[k]) { d[o] = 84; d[o + 1] = 91; d[o + 2] = 100; d[o + 3] = 255; continue; }
      if (iceM[k]) { d[o] = 226; d[o + 1] = 238; d[o + 2] = 246; d[o + 3] = 255; continue; }
      let t;
      if (v === 'speed') t = Math.sqrt(ux[k] * ux[k] + uy[k] * uy[k]) / U0 / 1.8;
      else if (v === 'pressure') { const cp = (rho[k] - 1) / (1.5 * U0 * U0); t = cp < 0 ? 0.5 + 0.5 * cp / 2.5 : 0.5 + 0.5 * cp; }
      else if (v === 'vort') { let w = 0; if (x > 0 && x < NX - 1 && y > 0 && y < NY - 1) w = (uy[k + 1] - uy[k - 1] - ux[k + NX] + ux[k - NX]) * 0.5 * L / U0; t = 0.5 + 0.5 * Math.tanh(w / 8); }
      else t = Math.min(1, dye[k] * 1.3) * 0.92 + 0.04;
      const i = Math.max(0, Math.min(255, (t * 255) | 0)) * 3;
      d[o] = map[i]; d[o + 1] = map[i + 1]; d[o + 2] = map[i + 2]; d[o + 3] = 255;
    }
  }
  fctx.putImageData(img, 0, 0);
}
function sampleU(x, y) {
  const { ux, uy } = field; if (x < 0.5 || y < 0.5 || x > NX - 1.5 || y > NY - 1.5) return null;
  const px = x - 0.5, py = y - 0.5, ix = px | 0, iy = py | 0, a = px - ix, b = py - iy, k = iy * NX + ix;
  if (solidAt(k) || solidAt(k + 1) || solidAt(k + NX) || solidAt(k + NX + 1)) return null;
  return [(ux[k] * (1 - a) + ux[k + 1] * a) * (1 - b) + (ux[k + NX] * (1 - a) + ux[k + NX + 1] * a) * b, (uy[k] * (1 - a) + uy[k + 1] * a) * (1 - b) + (uy[k + NX] * (1 - a) + uy[k + NX + 1] * a) * b];
}
function drawStreamlines(x) {
  const nS = Math.round(NY / 7), maxSteps = NX * 3;
  x.strokeStyle = 'rgba(255,255,255,0.42)'; x.lineWidth = Math.max(1, 0.8 * dpr);
  for (let i = 0; i < nS; i++) {
    let px = 1.5, py = (i + 0.5) / nS * NY; x.beginPath(); let [sx, sy] = toScr(px, py); x.moveTo(sx, sy);
    for (let s = 0; s < maxSteps; s++) {
      const u1 = sampleU(px, py); if (!u1) break; const sp = Math.hypot(u1[0], u1[1]); if (sp < 1e-4) break;
      const h = 0.9 / sp, mx = px + u1[0] * h * 0.5, my = py + u1[1] * h * 0.5, u2 = sampleU(mx, my); if (!u2) break;
      const sp2 = Math.hypot(u2[0], u2[1]) || 1; px += u2[0] / sp2 * 0.9; py += u2[1] / sp2 * 0.9;
      if (px > NX - 2) break;
      if (s % 3 === 0) { [sx, sy] = toScr(px, py); x.lineTo(sx, sy); }
    }
    x.stroke();
  }
}
function initParts() { const n = Math.round(N / 50); parts = new Float32Array(n * 2); for (let i = 0; i < n; i++) respawn(i, true); }
function respawn(i, any) { for (let t = 0; t < 20; t++) { const x = any ? 2 + Math.random() * (NX - 4) : 1 + Math.random() * 3, y = 1 + Math.random() * (NY - 2); if (!solidAt((y | 0) * NX + (x | 0))) { parts[i * 2] = x; parts[i * 2 + 1] = y; return; } } }
function stepParts(dt) {
  if (!field || !parts) return; const { ux, uy } = field; dt = Math.min(dt, 80);
  for (let i = 0; i < parts.length / 2; i++) {
    let x = parts[i * 2], y = parts[i * 2 + 1]; const k = (y | 0) * NX + (x | 0); x += ux[k] * dt; y += uy[k] * dt;
    if (x < 1 || x >= NX - 1 || y < 1 || y >= NY - 1 || solidAt((y | 0) * NX + (x | 0))) respawn(i, false); else { parts[i * 2] = x; parts[i * 2 + 1] = y; }
  }
}
function draw() {
  if (!field || !layout()) return;
  const c = cv(), x = c.getContext('2d');
  x.fillStyle = SL.css('--vp'); x.fillRect(0, 0, c.width, c.height);
  x.imageSmoothingEnabled = true; x.drawImage(fieldC, rect.x, rect.y, rect.w, rect.h);
  if (S.stream && S.field !== 'dye') drawStreamlines(x);
  if (S.parts) { x.fillStyle = 'rgba(255,255,255,0.7)'; const s = Math.max(1, dpr); for (let i = 0; i < parts.length; i += 2) { const [sx, sy] = toScr(parts[i], parts[i + 1]); x.fillRect(sx - s / 2, sy - s / 2, s, s); } }
  for (const poly of polys) {
    x.beginPath(); poly.forEach(([px, py], i) => { const [sx, sy] = toScr(px, py); i ? x.lineTo(sx, sy) : x.moveTo(sx, sy); }); x.closePath();
    x.fillStyle = '#545b64'; x.fill(); x.strokeStyle = '#c9cfd6'; x.lineWidth = dpr; x.stroke();
  }
  if (field.drops && field.drops.length) { x.fillStyle = 'rgba(160,220,255,0.6)'; const dp = field.drops, s = Math.max(1, dpr); for (let i = 0; i < dp.length; i += 2) { const [sx, sy] = toScr(dp[i], dp[i + 1]); x.fillRect(sx - s / 2, sy - s / 2, s, s); } }
  x.strokeStyle = SL.css('--line'); x.lineWidth = dpr; x.strokeRect(rect.x + 0.5, rect.y + 0.5, rect.w - 1, rect.h - 1);
}
function legend() {
  const L = { speed: ['viridis', '0', '1.8', '|u| / U∞'], pressure: ['coolwarm', '−2.5', '+1', 'Pressure coefficient C_p'], vort: ['coolwarm', 'cw', 'ccw', 'Vorticity ω·c/U∞'], dye: ['grey', '0', '1', 'Dye concentration'] }[S.field];
  SL.colorbar(...L);
}

// ---------- results ----------
function results() {
  const L = refLen(), tstar = field ? field.steps * U0 / L : 0, win = 150;
  const cl = SL.mean(hist.cl, win), cd = SL.mean(hist.cd, win), sd = SL.std(hist.cl, win), settled = tstar > 1.5 && hist.cl.length > 20;
  const th = thin(), ip = icingParams(), kc = iceStats.k / L, kmm = iceStats.k * ip.cell * 1000, hasIce = iceStats.count > 0;
  if (iceStats.count === 0 && settled) baseline = { cl, cd };
  const state = !settled ? ['Settling', 'neutral'] : sd < 0.01 ? ['Steady', 'ok'] : S.re < 1500 ? ['Periodic shedding', 'warn'] : ['Unsteady wake', 'warn'];
  const stalled = th && settled && S.aoa > 9 && th.cl > 0.6 && cl < 0.7 * th.cl;
  const mins = iceStats.exp * ip.dtReal * TIMELAPSE / 60;
  let dl = '—'; if (hasIce && baseline && settled) { const a = (cl - baseline.cl) / Math.max(0.05, Math.abs(baseline.cl)) * 100, b = (cd - baseline.cd) / Math.max(1e-3, baseline.cd) * 100; dl = `C<sub>l</sub> ${a >= 0 ? '+' : '−'}${Math.abs(a).toFixed(0)}% · C<sub>d</sub> ${b >= 0 ? '+' : '−'}${Math.abs(b).toFixed(0)}%`; }
  const s = SL.section();
  $('results').innerHTML = `
    <div class="badges">${SL.badge(state[0], state[1])}${stalled ? SL.badge('Stalled', 'crit') : ''}${S.icing && ip.kind.key !== 'none' ? SL.badge(ip.kind.name + ' accreting', 'warn') : ''}</div>
    <h4>Forces <span>mean of last ${win} samples</span></h4>
    ${SL.kv([['C<sub>l</sub>', settled ? fmt(cl, 3) : '…', 'big'], ['C<sub>d</sub>', settled ? fmt(cd, 4) : '…', 'big'], ['L/D', settled && Math.abs(cd) > 1e-3 ? fmt(cl / cd, 1) : '…', 'big'], null,
      ['C<sub>l</sub> fluctuation (σ)', settled ? fmt(sd, 3) : '…'], ['Thin-airfoil C<sub>l</sub>', th ? fmt(th.cl, 3) : '—'], ['α zero lift (theory)', th ? `${fmt(th.a0, 2)}°` : '—'], ['Flow time', `${fmt(tstar, 1)} c/U∞`]])}
    <h4>Force history</h4><canvas class="plot" id="a-p1"></canvas><div class="leg"><span><i style="background:var(--accent)"></i>C<sub>l</sub></span><span><i style="background:var(--warn)"></i>C<sub>d</sub></span>${th ? '<span style="color:var(--muted)"><i class="dash"></i>theory C<sub>l</sub></span>' : ''}</div>
    <h4>Ice</h4>${SL.kv([['Type', S.icing ? ip.kind.name : 'icing off'], ['Thickness', hasIce ? `${Math.round(kmm)} mm · ${(kc * 100).toFixed(1)}% c` : '0'], ['Exposure (real time)', mins < 1 ? `${Math.round(mins * 60)} s` : `${fmt(mins, 1)} min`], ['Change vs clean', dl], ['Droplet Stokes number', fmt(ip.St, ip.St < 1 ? 3 : 1)]])}
    <p class="ref">${S.icing ? ip.kind.desc : 'Switch on icing in Properties to grow ice on the leading edge.'} ${S.mvd > 50 && S.icing ? 'Large droplets strike far back, beyond the protected leading edge.' : ''}</p>
    <h4>Method</h4><p class="ref">D2Q9 lattice Boltzmann with a Smagorinsky subgrid model; bounce-back walls; forces by momentum exchange. Droplets follow Schiller–Naumann drag; ice forms cell by cell with a temperature-based freezing fraction and surface runback. 2D, at simulation Reynolds numbers far below flight (10⁶–10⁷). Section: NACA ${SL.P.wing.sec}${S.useFlap && SL.P.wing.flap ? ` with ${SL.P.wing.flap}° flap` : ''}; ice clock assumes a ${CHORD} m chord.</p>`;
  const P1 = $('a-p1'); if (!P1) return;
  const vals = hist.cl.concat(hist.cd); if (th) vals.push(th.cl);
  let lo = Math.max(-3, Math.min(0, ...vals)), hi = Math.min(4, Math.max(0.5, ...vals)); const sp = hi - lo || 1; lo -= sp * 0.05; hi += sp * 0.05;
  const pl = SL.plot(P1, { x0: 0, x1: HMAX - 1, y0: lo, y1: hi, dy: SL.niceStep(hi - lo, 4) });
  if (pl) { const off = HMAX - hist.cl.length; if (th) pl.line([[0, th.cl], [HMAX - 1, th.cl]], SL.css('--muted'), 1, [4, 3]); pl.line(hist.cd.map((v, i) => [i + off, v]), SL.css('--warn'), 1.4); pl.line(hist.cl.map((v, i) => [i + off, v]), SL.css('--accent'), 1.4); }
  SL.ice.kc = hasIce ? kc : 0; SL.ice.kmm = hasIce ? kmm : 0;
}

// ---------- worker ----------
const clearHist = () => { hist.cl.length = 0; hist.cd.length = 0; };
function onMsg(e) {
  const m = e.data;
  if (m.type === 'blowup') { clearHist(); SL.toast('The flow became numerically unstable and was restarted. Lower the Reynolds number or use a finer grid.'); return; }
  if (m.type !== 'field' || m.nx !== NX || m.ny !== NY) { if (active) worker.postMessage({ type: 'frame' }); return; }
  if (m.ice) { iceM = m.ice; Object.assign(SL.ice, { points: m.iceBody, cell: 1 / refLen(), shape: S.shape, kind: SL.iceKind(S.temp, S.lwc).key }); SL.ice.version++; }
  iceStats = { k: m.iceK, count: m.iceCount, exp: m.expSteps };
  field = m; fresh = true;
  if (m.fn > 0 && m.steps * U0 / refLen() > 0.6) { const q = 0.5 * U0 * U0 * refLen(); hist.cl.push(m.Fy / q); hist.cd.push(m.Fx / q); if (hist.cl.length > HMAX) { hist.cl.shift(); hist.cd.shift(); } }
  if (m.steps < lastSteps) clearHist();
  lastSteps = m.steps;
}
function start() {
  [NX, NY] = RES[S.res]; N = NX * NY;
  fieldC.width = NX; fieldC.height = NY; img = fctx.createImageData(NX, NY);
  field = null; clearHist(); lastSteps = 0; iceM = new Uint8Array(N); baseline = null;
  polys = buildPolys(); clean = rasterize(polys); initParts();
  worker.postMessage({ type: 'init', nx: NX, ny: NY, u0: U0, tau: tau(), clean, geom: geomInfo() });
  worker.postMessage({ type: 'run', running: S.running && active });
  sendIcing(); worker.postMessage({ type: 'frame' });
}
function tick() {
  if (!active) return; requestAnimationFrame(tick);
  if (!fresh) return; fresh = false;
  if (S.parts) stepParts(field.batch);
  renderField(); draw();
  if (++frames % 4 === 0) results();
  const now = performance.now();
  if (now - rateT > 1000) { rate = (field.steps - rateS) / ((now - rateT) / 1000); rateS = field.steps; rateT = now; SL.status(`2D LBM · NACA ${SL.P.wing.sec} · α ${fmt(S.aoa, 1)}° · Re ${int(S.re)}${S.running ? '' : ' · paused'}`, `${int(Math.max(0, rate))} steps/s · ${NX}×${NY}`); }
  worker.postMessage({ type: 'frame' });
}
const restartFlow = () => { clearHist(); worker && worker.postMessage({ type: 'reset' }); };
function setRunning(r) { S.running = r; worker && worker.postMessage({ type: 'run', running: r }); const b = $('a-run'); if (b) b.textContent = r ? 'Pause' : 'Run'; }

SL.register('airfoil', {
  title: 'Airfoil',
  inspector: () => [
    { section: 'Section', fields: [
      { type: 'seg', label: 'Shape', get: () => S.shape, set: v => { S.shape = v; worker && worker.postMessage({ type: 'deice' }); baseline = null; sendGeom(); restartFlow(); }, options: [['airfoil', 'Project airfoil'], ['cylinder', 'Cylinder']] },
      { type: 'readout', label: 'Airfoil', get: () => `NACA ${SL.P.wing.sec}` },
      { type: 'toggle', label: 'Include flap', get: () => S.useFlap, set: v => { S.useFlap = v; sendGeom(); } },
    ] },
    { section: 'Flow', fields: [
      { type: 'num', label: 'Angle of attack', get: () => S.aoa, set: v => { S.aoa = v; if (iceStats.count) baseline = null; sendGeom(); }, unit: '°', min: -10, max: 25, step: 0.5, dp: 1 },
      { type: 'num', label: 'Reynolds number', get: () => Math.round(Math.log10(S.re / 20) / 3 * 1000), set: v => { S.re = reFromSlider(v); worker && worker.postMessage({ type: 'tau', tau: tau() }); }, min: 0, max: 1000, step: 1, fmt: () => int(S.re), parse: t => Math.round(Math.log10(clamp(t, 20, 20000) / 20) / 3 * 1000), unit: '', hardMin: 0, hardMax: 1000 },
      { type: 'select', label: 'Grid', get: () => S.res, set: v => { S.res = v; start(); }, options: [['coarse', '300 × 140 (fast)'], ['std', '420 × 196'], ['fine', '560 × 260 (slow)']] },
      { type: 'button', buttons: [{ label: 'Use level-flight α', onclick: () => { const a = SL.aero(); S.aoa = Math.round(a.alReq / D2R * 2) / 2; sendGeom(); SL.refreshInspector(); } }, { label: 'Restart flow', onclick: restartFlow }] },
    ] },
    { section: 'Icing', fields: [
      { type: 'toggle', label: 'Supercooled cloud', get: () => S.icing, set: v => { S.icing = v; sendIcing(); } },
      { type: 'num', label: 'Temperature', get: () => S.temp, set: v => { S.temp = v; sendIcing(); }, unit: '°C', min: -30, max: 2, step: 0.5, dp: 1 },
      { type: 'num', label: 'Liquid water', get: () => S.lwc, set: v => { S.lwc = v; sendIcing(); }, unit: 'g/m³', min: 0.1, max: 2, step: 0.05, dp: 2 },
      { type: 'num', label: 'Droplet MVD', get: () => S.mvd, set: v => { S.mvd = v; sendIcing(); }, unit: 'µm', min: 10, max: 200, step: 1, dp: 0 },
      { type: 'num', label: 'Airspeed', get: () => S.vel, set: v => { S.vel = v; sendIcing(); }, unit: 'm/s', min: 40, max: 220, step: 5, dp: 0 },
      { type: 'readout', label: 'Ice type', get: () => SL.iceKind(S.temp, S.lwc).name },
      { type: 'button', buttons: [{ label: 'Use flight condition', onclick: () => { S.temp = Math.round(SL.isa(SL.P.flight.h).Tc * 2) / 2; S.vel = Math.round(SL.P.flight.V / 5) * 5; sendIcing(); SL.refreshInspector(); } }, { label: 'Shed ice', onclick: () => { worker && worker.postMessage({ type: 'deice' }); baseline = null; } }] },
    ] },
  ],
  toolbar(bar) {
    bar.innerHTML = '<span class="lab">Field</span>';
    const g = SL.el('div', { class: 'segm' });
    for (const [k, t] of [['speed', 'Velocity'], ['pressure', 'Pressure'], ['vort', 'Vorticity'], ['dye', 'Dye']]) g.appendChild(SL.el('button', { type: 'button', 'aria-pressed': String(k === S.field), onclick: e => { S.field = k; g.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', String(b === e.target))); legend(); renderField(); draw(); } }, t));
    bar.appendChild(g);
    for (const [k, t] of [['stream', 'Streamlines'], ['parts', 'Particles']]) { const l = SL.el('label', { class: 'chk' }), c = SL.el('input', { type: 'checkbox', class: 'tog' }); c.checked = S[k]; c.onchange = () => { S[k] = c.checked; draw(); }; l.append(c, document.createTextNode(t)); bar.appendChild(l); }
    bar.appendChild(SL.el('span', { class: 'grow' }));
    bar.appendChild(SL.el('button', { class: 'btn', id: 'a-run', type: 'button', onclick: () => setRunning(!S.running) }, S.running ? 'Pause' : 'Run'));
  },
  activate() {
    active = true; legend();
    if (!worker) { worker = new Worker('wing-worker.js?v=5'); worker.onmessage = onMsg; }
    if (!started) { started = true; start(); } else { sendGeom(); worker.postMessage({ type: 'run', running: S.running }); worker.postMessage({ type: 'frame' }); }
    requestAnimationFrame(tick);
  },
  deactivate() { active = false; worker && worker.postMessage({ type: 'run', running: false }); },
  onProject(d) { const k = SL.P.wing.sec + '/' + (S.useFlap ? SL.P.wing.flap : 0); if (started && k !== secKey) { secKey = k; sendGeom(); } },
});
document.addEventListener('visibilitychange', () => { if (active && worker) worker.postMessage({ type: 'run', running: S.running && !document.hidden }); });
})();
