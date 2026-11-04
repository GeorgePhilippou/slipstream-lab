// Wing section view: main-thread rendering, controls and icing set-up
(() => {
const { $, fmt } = SL;
const U0 = 0.1, CHORD = 2.0, TIMELAPSE = 100000;
const RES = { coarse: [300, 140], std: [420, 196], fine: [560, 260] };
const S = { shape: 'airfoil', m: 2, p: 4, t: 12, flap: 0, aoa: 4, re: 3000, reSlider: 0, view: 'speed', particles: true, res: 'std', running: true,
  icing: false, temp: -12, lwc: 0.5, mvd: 20, vel: 120 };
let NX, NY, N, clean, iceM, polys = [], field = null, fresh = false, active = false, started = false;
const hist = { cl: [], cd: [] }, HMAX = 600;
let baseline = null, iceStats = { k: 0, count: 0, exp: 0 }, lastSteps = 0, stepRate = 0, lastRateT = 0, lastRateSteps = 0, frames = 0, probe = null;
const worker = new Worker('wing-worker.js');

const VIEWS = {
  speed: { name: 'Speed', min: '0', max: '1.8× free stream', note: 'how fast the air moves' },
  pressure: { name: 'Pressure Cp', min: '−2.5 suction', max: '+1 stagnation', note: '' },
  vorticity: { name: 'Spin', min: 'clockwise', max: 'anticlockwise', note: 'vorticity: boundary layers and eddies' },
  smoke: { name: 'Smoke', min: '', max: '', note: 'dye released upstream; blue dots are droplets' },
};

// ---------- geometry ----------
function refLen() { return S.shape === 'cylinder' ? Math.round(0.13 * NY) : Math.round(0.2 * NX); }
function geomInfo() {
  const L = refLen();
  if (S.shape === 'cylinder') return { px: 0.25 * NX, py: NY / 2 + 0.37, L, aoa: S.aoa, ox: 0 };
  return { px: 0.22 * NX + 0.25 * L, py: NY / 2, L, aoa: S.aoa, ox: 0.25 };
}
function buildPolys() {
  const gi = geomInfo(), L = gi.L, a = -S.aoa * SL.D2R, ca = Math.cos(a), sa = Math.sin(a);
  let pts;
  if (S.shape === 'airfoil') pts = SL.naca.points(S.m / 100, S.p / 10, S.t / 100, S.flap);
  else if (S.shape === 'plate') { const h = Math.max(0.012, 1.1 / L); pts = [[0, -h], [1, -h], [1, h], [0, h]]; }
  else { pts = []; for (let i = 0; i < 72; i++) { const t = i / 72 * 2 * Math.PI; pts.push([0.5 * Math.cos(t), 0.5 * Math.sin(t)]); } }
  return [pts.map(([x, y]) => { const dx = x - gi.ox, dy = y; return [gi.px + L * (dx * ca - dy * sa), gi.py + L * (dx * sa + dy * ca)]; })];
}
function rasterize(ps) {
  const m = new Uint8Array(N), xs = [];
  for (const poly of ps) for (let y = 0; y < NY; y++) {
    const yc = y + 0.5; xs.length = 0;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const [x1, y1] = poly[i], [x2, y2] = poly[j];
      if ((y1 <= yc) !== (y2 <= yc)) xs.push(x1 + (yc - y1) * (x2 - x1) / (y2 - y1));
    }
    xs.sort((a, b) => a - b);
    for (let i = 0; i + 1 < xs.length; i += 2) {
      const a = Math.max(2, Math.ceil(xs[i] - 0.5)), b = Math.min(NX - 3, Math.floor(xs[i + 1] - 0.5));
      for (let x = a; x <= b; x++) m[y * NX + x] = 1;
    }
  }
  return m;
}
function sendGeom() {
  polys = buildPolys(); clean = rasterize(polys);
  worker.postMessage({ type: 'geom', clean, geom: geomInfo() });
  worker.postMessage({ type: 'tau', tau: tau() });
  sendIcing();
}
const solidAt = k => clean[k] || iceM[k];

// ---------- physics parameters ----------
const reFromSlider = v => 20 * Math.pow(10, v / 1000 * 3);
const sliderFromRe = re => Math.round(Math.log10(re / 20) / 3 * 1000);
const tau = () => Math.max(0.5005, 3 * U0 * refLen() / S.re + 0.5);
function thinAirfoil() {
  if (S.shape === 'cylinder') return null;
  const m = S.shape === 'airfoil' ? S.m / 100 : 0, fl = S.shape === 'airfoil' ? S.flap * SL.D2R : 0;
  const a0 = SL.alpha0(m, S.p / 10, fl);
  return { cl: 2 * Math.PI * (S.aoa * SL.D2R - a0), a0: a0 / SL.D2R };
}
function icingParams() {
  const kind = SL.iceKind(S.temp, S.lwc);
  const on = S.icing && kind.key !== 'none';
  const L = refLen(), V = S.vel, d = S.mvd * 1e-6, mu = 1.72e-5, rhoA = 0.9, rhoIce = kind.key === 'rime' ? 880 : 917;
  const St = 1000 * d * d * V / (18 * mu * CHORD);           // droplet Stokes number based on chord
  const dtReal = CHORD * U0 / (L * V), cell = CHORD / L, spawn = 3, band = 0.9 * CHORD;
  const mFrac = (S.lwc * 1e-3) * V * band * dtReal * TIMELAPSE / (spawn * rhoIce * cell * cell);
  return { kind, St, dtReal, cell, icing: { on, tp: Math.max(0.5, St * L / U0), rpk: rhoA * d * (V / U0) / mu, n0: Math.max(0.05, kind.n0), mFrac, spawn, settle: Math.round(2.5 * L / U0) } };
}
function sendIcing() { if (!NX) return; const ip = icingParams(); worker.postMessage({ type: 'icing', icing: ip.icing }); syncIcingUI(ip); }

// ---------- canvases ----------
const view = $('w-view'), vctx = view.getContext('2d');
const fieldC = document.createElement('canvas'), fctx = fieldC.getContext('2d');
const trailC = document.createElement('canvas'), tctx = trailC.getContext('2d');
let img, W = 0, H = 0, dpr = 1, parts;
function sizeCanvas() {
  dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = view.parentElement.clientWidth; if (!w) return;
  W = Math.round(w * dpr); H = Math.round(w * NY / NX * dpr);
  view.width = W; view.height = H; trailC.width = W; trailC.height = H;
  view.style.height = (w * NY / NX) + 'px';
}
new ResizeObserver(() => { if (NX && active) { sizeCanvas(); draw(); } }).observe($('w-stage'));
function initParticles() { const n = Math.round(N / 40); parts = new Float32Array(n * 2); for (let i = 0; i < n; i++) respawn(i, true); }
function respawn(i, anywhere) {
  for (let t = 0; t < 20; t++) {
    const x = anywhere ? 2 + Math.random() * (NX - 4) : 1 + Math.random() * 3, y = 1 + Math.random() * (NY - 2);
    if (!solidAt((y | 0) * NX + (x | 0))) { parts[i * 2] = x; parts[i * 2 + 1] = y; return; }
  }
}
function advectParticles(dt) {
  if (!field || dt <= 0) return;
  dt = Math.min(dt, 120);
  const { ux, uy } = field, sx = W / NX, sy = H / NY;
  tctx.globalCompositeOperation = 'destination-out'; tctx.fillStyle = 'rgba(0,0,0,0.2)'; tctx.fillRect(0, 0, W, H);
  tctx.globalCompositeOperation = 'source-over'; tctx.strokeStyle = 'rgba(255,255,255,0.75)'; tctx.lineWidth = Math.max(1, dpr * 0.9);
  tctx.beginPath();
  const sub = Math.max(1, Math.ceil(dt * 0.25)), h = dt / sub;
  for (let i = 0; i < parts.length / 2; i++) {
    let x = parts[i * 2], y = parts[i * 2 + 1]; const x0 = x, y0 = y; let ok = true;
    for (let s = 0; s < sub; s++) {
      const k = (y | 0) * NX + (x | 0); x += ux[k] * h; y += uy[k] * h;
      if (x < 1 || x >= NX - 1 || y < 1 || y >= NY - 1 || solidAt((y | 0) * NX + (x | 0))) { ok = false; break; }
    }
    if (!ok) { respawn(i, false); continue; }
    parts[i * 2] = x; parts[i * 2 + 1] = y;
    tctx.moveTo(x0 * sx, H - y0 * sy); tctx.lineTo(x * sx, H - y * sy);
  }
  tctx.stroke();
}
function renderField() {
  if (!field) return;
  const { ux, uy, rho, dye } = field, d = img.data, v = S.view, L = refLen(), map = SL.MAPS[v === 'smoke' ? 'speed' : v];
  for (let y = 0; y < NY; y++) {
    const r = (NY - 1 - y) * NX;
    for (let x = 0; x < NX; x++) {
      const k = y * NX + x, o = (r + x) * 4;
      if (clean[k]) { d[o] = 52; d[o + 1] = 60; d[o + 2] = 70; d[o + 3] = 255; continue; }
      if (iceM[k]) { d[o] = 214; d[o + 1] = 238; d[o + 2] = 250; d[o + 3] = 255; continue; }
      let t;
      if (v === 'speed' || v === 'smoke') t = Math.sqrt(ux[k] * ux[k] + uy[k] * uy[k]) / U0 / 1.8;
      else if (v === 'pressure') { const cp = (rho[k] - 1) / (1.5 * U0 * U0); t = cp < 0 ? 0.5 + 0.5 * cp / 2.5 : 0.5 + 0.5 * cp; }
      else {
        let w = 0;
        if (x > 0 && x < NX - 1 && y > 0 && y < NY - 1) w = (uy[k + 1] - uy[k - 1] - ux[k + NX] + ux[k - NX]) * 0.5 * L / U0;
        t = 0.5 + 0.5 * Math.tanh(w / 8);
      }
      const i = Math.max(0, Math.min(255, (t * 255) | 0)) * 3;
      if (v === 'smoke') { const s = Math.min(1, dye[k] * 1.4); d[o] = map[i] * 0.28 * (1 - s) + 246 * s; d[o + 1] = map[i + 1] * 0.28 * (1 - s) + 244 * s; d[o + 2] = map[i + 2] * 0.28 * (1 - s) + 238 * s; }
      else { d[o] = map[i]; d[o + 1] = map[i + 1]; d[o + 2] = map[i + 2]; }
      d[o + 3] = 255;
    }
  }
  fctx.putImageData(img, 0, 0);
}
function draw() {
  if (!W || !img) return;
  vctx.imageSmoothingEnabled = true;
  vctx.drawImage(fieldC, 0, 0, W, H);
  if (S.particles && S.view !== 'smoke') vctx.drawImage(trailC, 0, 0);
  const sx = W / NX, sy = H / NY;
  for (const poly of polys) {
    vctx.beginPath();
    poly.forEach(([x, y], i) => i ? vctx.lineTo(x * sx, H - y * sy) : vctx.moveTo(x * sx, H - y * sy));
    vctx.closePath(); vctx.fillStyle = '#343c46'; vctx.fill();
    vctx.strokeStyle = 'rgba(235,240,245,0.9)'; vctx.lineWidth = 1.2 * dpr; vctx.stroke();
  }
  if (field && field.drops && field.drops.length) {
    vctx.fillStyle = 'rgba(140,220,255,0.55)';
    const dp = field.drops, s = Math.max(1, 1.1 * dpr);
    for (let i = 0; i < dp.length; i += 2) vctx.fillRect(dp[i] * sx - s / 2, H - dp[i + 1] * sy - s / 2, s, s);
  }
  if (probe) {
    vctx.strokeStyle = 'rgba(255,255,255,0.9)'; vctx.lineWidth = dpr;
    const X = (probe.x + 0.5) * sx, Y = H - (probe.y + 0.5) * sy, r = 6 * dpr;
    vctx.beginPath(); vctx.arc(X, Y, r, 0, 7); vctx.moveTo(X - 2 * r, Y); vctx.lineTo(X - r, Y); vctx.moveTo(X + r, Y); vctx.lineTo(X + 2 * r, Y); vctx.stroke();
  }
}
function drawLegend() {
  const info = VIEWS[S.view];
  $('w-lname').textContent = info.name; $('w-lmin').textContent = info.min; $('w-lmax').textContent = info.max; $('w-lnote').textContent = info.note;
  $('w-bar').hidden = S.view === 'smoke';
  if (S.view !== 'smoke') SL.drawBar($('w-bar'), SL.MAPS[S.view]);
}

// ---------- read-outs ----------
function updateGauges() {
  const L = refLen(), tstar = field ? field.steps * U0 / L : 0, win = 150;
  const cl = SL.mean(hist.cl, win), cd = SL.mean(hist.cd, win), sd = SL.std(hist.cl, win), settled = tstar > 1.5 && hist.cl.length > 20;
  $('w-cl').textContent = settled ? fmt(cl) : '…';
  $('w-cd').textContent = settled ? fmt(cd, 3) : '…';
  $('w-ld').textContent = settled && Math.abs(cd) > 1e-3 ? fmt(cl / cd, 1) : '…';
  const chip = $('w-chip');
  if (!settled) { chip.textContent = 'Flow settling'; chip.className = 'chip neutral'; }
  else if (sd < 0.01) { chip.textContent = 'Steady flow'; chip.className = 'chip good'; }
  else if (S.re < 1500) { chip.textContent = 'Periodic vortex shedding'; chip.className = 'chip warn'; }
  else { chip.textContent = 'Unsteady, turbulent-like wake'; chip.className = 'chip crit'; }

  const th = thinAirfoil();
  $('w-thkey').hidden = !th;
  if (th) {
    $('w-theory').innerHTML = `Thin-airfoil theory, ideal inviscid air: C<sub>l</sub> = 2π(α − α<sub>0</sub>) = <b>${fmt(th.cl)}</b> · Reynolds ${SL.int(S.re)}`;
    $('w-stall').hidden = !(settled && S.aoa > 9 && th.cl > 0.6 && cl < 0.7 * th.cl);
  } else { $('w-theory').textContent = 'Lab reference: a cylinder at Re 100 has Cd ≈ 1.35 and sheds vortices at a Strouhal number near 0.16.'; $('w-stall').hidden = true; }

  // ice
  const ip = icingParams();
  const kmm = iceStats.k * ip.cell * 1000, kc = iceStats.k / L;
  if (iceStats.count === 0 && settled) baseline = { cl, cd };
  const hasIce = iceStats.count > 0;
  $('w-icehud').hidden = !hasIce && !S.icing;
  $('w-icek').innerHTML = hasIce ? `${Math.round(kmm)} mm <small>${(kc * 100).toFixed(1)}% chord</small>` : '0 mm';
  const mins = iceStats.exp * ip.dtReal * TIMELAPSE / 60;
  $('w-iceexp').textContent = mins < 1 ? `${Math.round(mins * 60)} s` : `${mins.toFixed(1)} min`;
  if (hasIce && baseline && settled) {
    const dcl = (cl - baseline.cl) / Math.max(0.05, Math.abs(baseline.cl)) * 100, dcd = (cd - baseline.cd) / Math.max(1e-3, baseline.cd) * 100;
    $('w-icedelta').innerHTML = `lift ${dcl >= 0 ? '+' : '−'}${Math.abs(dcl).toFixed(0)}% <small>·</small> drag ${dcd >= 0 ? '+' : '−'}${Math.abs(dcd).toFixed(0)}%`;
  } else $('w-icedelta').textContent = hasIce ? 'measuring…' : '—';
  SL.ice.kc = hasIce ? kc : 0; SL.ice.kmm = hasIce ? kmm : 0;
  if (S.icing) syncIcingUI(ip);
}
function syncIcingUI(ip) {
  const k = ip.kind;
  $('w-kind').innerHTML = `<b>${k.name}</b>${S.mvd > 50 && k.key !== 'none' ? ' · <b>large droplets</b>' : ''}<p>${k.desc}${S.mvd > 50 && k.key !== 'none' ? ' Large droplets hit far back on the wing, beyond where de-icing boots reach.' : ''} Droplet inertia (Stokes number): ${ip.St.toFixed(ip.St < 1 ? 3 : 1)}.</p>`;
  $('w-icestate').textContent = !S.icing ? 'off' : k.key === 'none' ? 'no freezing' : (field && field.steps < 2.5 * refLen() / U0 ? 'waiting for flow to settle' : 'icing');
  const chip = $('w-icechip');
  chip.hidden = !(S.icing && k.key !== 'none'); chip.textContent = k.name + ' accreting';
}

const histC = $('w-hist');
function drawHist() {
  const th = thinAirfoil();
  const vals = hist.cl.concat(hist.cd); if (th) vals.push(th.cl);
  let lo = Math.max(-3, Math.min(0, ...vals)), hi = Math.min(4, Math.max(0.5, ...vals));
  const sp = hi - lo || 1; lo -= sp * 0.05; hi += sp * 0.05;
  const dy = [0.1, 0.2, 0.25, 0.5, 1, 2].find(s => (hi - lo) / s <= 5) || 2;
  const ch = SL.chart(histC, { x0: 0, x1: HMAX - 1, y0: lo, y1: hi, dy, padL: 40 }); if (!ch) return;
  const { c, X, Y } = ch;
  if (th) { c.setLineDash([4, 4]); c.strokeStyle = SL.css('--muted'); c.beginPath(); c.moveTo(X(0), Y(th.cl)); c.lineTo(X(HMAX - 1), Y(th.cl)); c.stroke(); c.setLineDash([]); }
  const plot = (a, col) => {
    if (a.length < 2) return; c.strokeStyle = col; c.lineWidth = 1.75; c.beginPath();
    const off = HMAX - a.length;
    a.forEach((v, i) => { const y = Y(Math.max(lo, Math.min(hi, v))); i ? c.lineTo(X(i + off), y) : c.moveTo(X(i + off), y); });
    c.stroke(); c.fillStyle = col; c.beginPath(); c.arc(X(HMAX - 1), Y(Math.max(lo, Math.min(hi, a[a.length - 1]))), 3, 0, 7); c.fill();
  };
  plot(hist.cd, SL.css('--warm')); plot(hist.cl, SL.css('--accent'));
}

// ---------- worker plumbing ----------
const clearHist = () => { hist.cl.length = 0; hist.cd.length = 0; };
worker.onmessage = e => {
  const m = e.data;
  if (m.type === 'blowup') { clearHist(); SL.toast($('w-toast'), 'The flow went numerically unstable and was restarted. Try a lower Reynolds number or a finer grid.'); return; }
  if (m.type !== 'field' || m.nx !== NX || m.ny !== NY) { if (active) worker.postMessage({ type: 'frame' }); return; }
  if (m.ice) {
    iceM = m.ice;
    SL.ice.points = m.iceBody; SL.ice.cell = 1 / refLen(); SL.ice.version++; SL.ice.shape = S.shape;
    SL.ice.kind = SL.iceKind(S.temp, S.lwc).key;
    SL.emit('ice', SL.ice);
  }
  iceStats = { k: m.iceK, count: m.iceCount, exp: m.expSteps };
  field = m; fresh = true;
  if (m.fn > 0 && m.steps * U0 / refLen() > 0.6) {
    const q = 0.5 * U0 * U0 * refLen();
    hist.cl.push(m.Fy / q); hist.cd.push(m.Fx / q);
    if (hist.cl.length > HMAX) { hist.cl.shift(); hist.cd.shift(); }
  }
  if (m.steps < lastSteps) clearHist();
  lastSteps = m.steps;
};
function start(resKey) {
  [NX, NY] = RES[resKey]; N = NX * NY;
  fieldC.width = NX; fieldC.height = NY; img = fctx.createImageData(NX, NY);
  field = null; clearHist(); lastSteps = 0; iceM = new Uint8Array(N); baseline = null;
  polys = buildPolys(); clean = rasterize(polys);
  sizeCanvas(); initParticles();
  worker.postMessage({ type: 'init', nx: NX, ny: NY, u0: U0, tau: tau(), clean, geom: geomInfo() });
  worker.postMessage({ type: 'run', running: S.running && active });
  sendIcing();
  worker.postMessage({ type: 'frame' });
}
function tick() {
  if (!active) return;
  requestAnimationFrame(tick);
  if (!fresh) return;
  fresh = false;
  advectParticles(field.batch);
  renderField(); draw();
  if (++frames % 3 === 0) { updateGauges(); drawHist(); updateProbe(); }
  const now = performance.now();
  if (now - lastRateT > 1000) { stepRate = (field.steps - lastRateSteps) / ((now - lastRateT) / 1000); lastRateSteps = field.steps; lastRateT = now; $('w-perf').textContent = S.running ? Math.max(0, Math.round(stepRate)) + ' steps/s' : 'paused'; }
  worker.postMessage({ type: 'frame' });
}

// ---------- probe ----------
function cellAt(ev) { const r = view.getBoundingClientRect(); return { x: Math.floor((ev.clientX - r.left) / r.width * NX), y: Math.floor((1 - (ev.clientY - r.top) / r.height) * NY) }; }
function updateProbe() {
  if (!probe || !field) return;
  if (probe.x < 1 || probe.x >= NX - 1 || probe.y < 1 || probe.y >= NY - 1) return;
  const k = probe.y * NX + probe.x;
  if (clean[k]) { $('w-probe').textContent = 'Inside the wing'; return; }
  if (iceM[k]) { $('w-probe').textContent = 'Ice'; return; }
  const { ux, uy, rho } = field, v = Math.hypot(ux[k], uy[k]) / U0, cp = (rho[k] - 1) / (1.5 * U0 * U0);
  $('w-probe').textContent = `Speed ${v.toFixed(2)}× · pressure Cp ${fmt(cp)}`;
}
view.addEventListener('pointermove', ev => { probe = cellAt(ev); updateProbe(); });
view.addEventListener('pointerleave', () => { probe = null; $('w-probe').textContent = 'Point at the flow to read it'; });

// ---------- controls ----------
function nacaCode() { return [S.m, S.p, S.t].every(Number.isInteger) ? `NACA ${S.m}${S.p}${String(S.t).padStart(2, '0')}` : 'Custom airfoil'; }
function syncLabels() {
  $('w-m-o').textContent = S.m + '%'; $('w-p-o').textContent = S.p * 10 + '% back'; $('w-t-o').textContent = S.t + '%';
  $('w-flap-o').textContent = (S.flap > 0 ? '+' : '') + S.flap + '°'; $('w-aoa-o').textContent = fmt(S.aoa, 1) + '°';
  $('w-re-o').textContent = SL.int(S.re);
  $('w-temp-o').textContent = fmt(S.temp, 1) + ' °C'; $('w-lwc-o').textContent = S.lwc.toFixed(2) + ' g/m³';
  $('w-mvd-o').textContent = S.mvd + ' µm'; $('w-vel-o').textContent = `${S.vel} m/s · ${Math.round(S.vel * 1.944)} kt`;
  $('w-code').textContent = S.shape === 'airfoil' ? nacaCode() + (S.flap ? ' + flap' : '') : S.shape === 'plate' ? 'Flat plate' : 'Cylinder';
  for (const id of ['m', 'p', 't', 'flap', 'aoa']) $('w-' + id).value = S[id];
  $('w-re').value = S.reSlider; $('w-temp').value = S.temp; $('w-lwc').value = S.lwc; $('w-mvd').value = S.mvd; $('w-vel').value = S.vel;
  $('w-icing').checked = S.icing;
  $('w-airfoil').style.display = S.shape === 'airfoil' ? 'flex' : 'none';
}
function shareSection() { if (S.shape === 'airfoil') { SL.section = { m: S.m, p: S.p, t: S.t }; SL.emit('section', SL.section); } }
function geomChanged() { if (iceStats.count) baseline = null; sendGeom(); syncLabels(); shareSection(); }

SL.seg('w-shape', v => { S.shape = v; worker.postMessage({ type: 'deice' }); clearHist(); baseline = null; sendGeom(); worker.postMessage({ type: 'reset' }); syncLabels(); });
SL.seg('w-viewseg', v => { S.view = v; drawLegend(); renderField(); draw(); });
$('w-preset').onchange = e => { const c = e.target.value; S.m = +c[0]; S.p = +c[1] || 4; S.t = +c.slice(2); geomChanged(); };
for (const id of ['m', 'p', 't', 'flap', 'aoa']) $('w-' + id).addEventListener('input', e => { S[id] = +e.target.value; geomChanged(); });
$('w-re').addEventListener('input', e => { S.reSlider = +e.target.value; S.re = reFromSlider(S.reSlider); worker.postMessage({ type: 'tau', tau: tau() }); syncLabels(); });
for (const id of ['temp', 'lwc', 'mvd', 'vel']) $('w-' + id).addEventListener('input', e => { S[id] = +e.target.value; sendIcing(); syncLabels(); });
$('w-icing').onchange = e => { S.icing = e.target.checked; sendIcing(); };
$('w-deice').onclick = () => { worker.postMessage({ type: 'deice' }); baseline = null; };
$('w-parts').onchange = e => { S.particles = e.target.checked; tctx.clearRect(0, 0, W, H); };
$('w-res').onchange = e => { S.res = e.target.value; start(S.res); };
$('w-play').onclick = () => { S.running = !S.running; $('w-play').textContent = S.running ? 'Pause' : 'Resume'; worker.postMessage({ type: 'run', running: S.running }); };
$('w-reset').onclick = () => { clearHist(); worker.postMessage({ type: 'reset' }); };

const PRESETS = {
  cruise: { shape: 'airfoil', m: 2, p: 4, t: 12, flap: 0, aoa: 4, re: 3000, view: 'pressure', icing: false },
  stall: { shape: 'airfoil', m: 0, p: 4, t: 12, flap: 0, aoa: 18, re: 6000, view: 'vorticity', icing: false },
  rime: { shape: 'airfoil', m: 2, p: 4, t: 12, flap: 0, aoa: 4, re: 3000, view: 'speed', icing: true, temp: -18, lwc: 0.5, mvd: 20, vel: 120 },
  glaze: { shape: 'airfoil', m: 2, p: 4, t: 12, flap: 0, aoa: 3, re: 3000, view: 'speed', icing: true, temp: -3, lwc: 1.0, mvd: 22, vel: 130 },
  sld: { shape: 'airfoil', m: 2, p: 4, t: 12, flap: 0, aoa: 4, re: 3000, view: 'speed', icing: true, temp: -8, lwc: 0.4, mvd: 160, vel: 110 },
  street: { shape: 'cylinder', aoa: 0, re: 150, view: 'vorticity', icing: false },
};
$('w-try').addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b) return;
  const p = PRESETS[b.dataset.p];
  Object.assign(S, p); S.reSlider = sliderFromRe(p.re); S.re = reFromSlider(S.reSlider);
  SL.syncSeg('w-shape', S.shape); SL.syncSeg('w-viewseg', S.view); drawLegend();
  if (S.shape === 'airfoil') { const code = `${S.m}${S.p}${String(S.t).padStart(2, '0')}`; if ([...$('w-preset').options].some(o => o.value === code)) $('w-preset').value = code; }
  worker.postMessage({ type: 'deice' }); baseline = null; clearHist();
  sendGeom(); worker.postMessage({ type: 'reset' });
  tctx.clearRect(0, 0, W, H); syncLabels(); shareSection();
});

S.reSlider = sliderFromRe(3000); S.re = reFromSlider(S.reSlider);
syncLabels(); drawLegend();
SL.views.wing = {
  activate() {
    active = true;
    if (!started) { started = true; start(S.res); } else { sizeCanvas(); worker.postMessage({ type: 'run', running: S.running }); worker.postMessage({ type: 'frame' }); }
    requestAnimationFrame(tick);
  },
  deactivate() { active = false; worker.postMessage({ type: 'run', running: false }); },
};
document.addEventListener('visibilitychange', () => { if (active) worker.postMessage({ type: 'run', running: S.running && !document.hidden }); });
})();
