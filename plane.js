// Aircraft-in-flight view: ISA atmosphere + point-mass longitudinal dynamics + icing degradation
(() => {
const { $, fmt, clamp, D2R } = SL;
const G = 9.80665;
// Generic aircraft classes (not specific types). a0: section lift slope per rad.
const AC = {
  light: { name: 'Light single', desc: 'Four-seat piston single. 1,100 kg, 11 m span. Most have no ice protection and are not certified to fly into known icing.',
    m: 1100, S: 16.2, b: 11.0, c: 1.5, CD0: 0.028, e: 0.78, CLmax: 1.6, a0: 5.9, alpha0: -2.1 * D2R, prop: true, P: 134e3, eta: 0.8, Tstatic: 3200, lapse: 1.0, Mcrit: 0.6, v: [35, 80], vt: 60, ht: 2500, deice: false, high: false },
  turboprop: { name: 'Regional turboprop', desc: 'Seventy-seat twin turboprop. 21,500 kg, 27 m span, pneumatic de-icing boots. Flies in the 3–7 km band where icing is most common.',
    m: 21500, S: 61, b: 27.0, c: 2.3, CD0: 0.027, e: 0.8, CLmax: 1.55, a0: 5.9, alpha0: -2.1 * D2R, prop: true, P: 2 * 2300e3, eta: 0.82, Tstatic: 2 * 45e3, lapse: 0.75, Mcrit: 0.62, v: [70, 160], vt: 140, ht: 5500, deice: true, high: true },
  jet: { name: 'Narrow-body jet', desc: '180-seat twinjet. 66,000 kg, 34 m span, heated wing leading edges. Cruises near 11 km at Mach 0.78.',
    m: 66000, S: 122.6, b: 34.1, c: 4.2, CD0: 0.021, e: 0.8, CLmax: 1.45, a0: 6.0, alpha0: -2.1 * D2R, prop: false, T0: 2 * 117e3, Mcrit: 0.74, v: [120, 260], vt: 230, ht: 11000, deice: true, high: false },
};
const P = { ac: 'turboprop', ht: 5500, vt: 140, cloud: false, lwc: 0.4, mvd: 20, ts: 30, running: true };
let st, log = [], active = false, lastT = 0, fromWing = false, frames = 0;

function aero(ac, k, kind) {
  const AR = ac.b * ac.b / ac.S, kc = k / ac.c, s = Math.sqrt(Math.max(0, kc));
  const hf = kind === 'glaze' ? 1 : kind === 'mixed' ? 0.82 : 0.65;
  const dCLmax = Math.min(0.6, 3.0 * s * hf), dCD0 = 0.25 * s * hf;
  const a2d = ac.a0 * (1 - 0.5 * s), aw = a2d / (1 + a2d / (Math.PI * ac.e * AR));
  const CLmax = ac.CLmax * (1 - dCLmax);
  return { AR, aw, CLmax, alphaS: ac.alpha0 + 1.2 * CLmax / aw, dCD0, dCLmax, kc };
}
function CLof(al, A, ac) {
  const x = (al - ac.alpha0) * A.aw / A.CLmax; let n;
  if (x < 0.8) n = x; else if (x < 1.2) n = x - (x - 0.8) ** 2 / 0.8; else n = Math.max(0.55, 1 - 0.9 * (x - 1.2));
  return n * A.CLmax;
}
function CDof(CL, al, M, A, ac) {
  const x = (al - ac.alpha0) * A.aw / A.CLmax, ps = x > 1.2 ? 0.6 * (x - 1.2) : 0, wave = M > ac.Mcrit ? 20 * (M - ac.Mcrit) ** 4 : 0;
  return ac.CD0 + A.dCD0 + CL * CL / (Math.PI * ac.e * A.AR) + ps + wave;
}
function Tmax(ac, rho, V, M) {
  const sig = rho / 1.225;
  if (ac.prop) return Math.min(ac.Tstatic * sig ** ac.lapse, ac.eta * ac.P * sig ** ac.lapse / Math.max(V, 20));
  return ac.T0 * sig ** 0.85 * (1 - 0.25 * M);
}
function collection(ac, V) {  // Langmuir–Blodgett fit, leading edge treated as a cylinder of diameter 0.2c
  const d = P.mvd * 1e-6, mu = 1.7e-5, D = 0.2 * ac.c, K = 1000 * d * d * V / (9 * mu * D);
  if (K <= 0.125) return 0;
  const z = 1.4 * Math.pow(K - 0.125, 0.84); return 0.85 * z / (1 + z);
}
const liquidFraction = Tc => Tc >= 0 ? 0 : clamp((Tc + 40) / 25, 0, 1);
function kindAt(Tc) { const k = SL.iceKind(Tc, P.lwc).key; return k === 'none' ? 'rime' : k; }
const altStr = h => `${SL.int(h)} m`;
const clock = t => { const m = Math.floor(t / 60), s = Math.floor(t % 60); return m >= 60 ? `${Math.floor(m / 60)}h ${String(m % 60).padStart(2, '0')}m` : `${m}:${String(s).padStart(2, '0')}`; };
function addLog(msg, cls = '') { log.unshift({ t: st.t, msg, cls }); log = log.slice(0, 8); renderLog(); }
function renderLog() {
  $('p-log').innerHTML = log.length ? log.map(l => `<li class="${l.cls}"><time>${clock(l.t)}</time><span>${l.msg}</span></li>`).join('') : '<li class="empty">Events such as stalls, ice build-up and de-icing will appear here.</li>';
}

function resetSim() {
  const ac = AC[P.ac];
  st = { h: P.ht, V: P.vt, gam: 0, alpha: 3 * D2R, k: 0, kind: 'rime', t: 0, mode: 'hold', recT: 0, stallH: 0, warned: { light: false, mod: false, sev: false } };
  const atm = SL.isa(st.h), A = aero(ac, 0, 'rime'), q = 0.5 * atm.rho * st.V * st.V;
  st.alpha = clamp(ac.alpha0 + ac.m * G / (q * ac.S) / A.aw, -4 * D2R, 14 * D2R);
  log = []; renderLog(); fromWing = false;
  computeOutputs();
}
function stepSim(dt) {
  const ac = AC[P.ac], atm = SL.isa(st.h), A = aero(ac, st.k, st.kind);
  const W = ac.m * G, q = 0.5 * atm.rho * st.V * st.V, M = st.V / atm.a, Tm = Tmax(ac, atm.rho, st.V, M);
  let alCmd;
  if (st.mode === 'recover') alCmd = ac.alpha0 + 0.7 * (A.alphaS - ac.alpha0);
  else {
    const CLlvl = W / (q * ac.S), D0 = q * ac.S * CDof(CLlvl, ac.alpha0 + CLlvl / A.aw, M, A, ac);
    const gmax = Math.max(0, (Tm - D0) / W * 0.7);
    let gc = clamp(0.002 * (P.ht - st.h), -0.08, 0.08); if (gc > 0) gc = Math.min(gc, gmax);
    const CLc = (W * Math.cos(st.gam) + ac.m * st.V * 0.8 * (gc - st.gam)) / (q * ac.S);
    alCmd = ac.alpha0 + CLc / A.aw;
  }
  alCmd = clamp(alCmd, -6 * D2R, 24 * D2R);
  st.alpha += (alCmd - st.alpha) * Math.min(1, dt * 2.5);
  const CL = CLof(st.alpha, A, ac), CD = CDof(CL, st.alpha, M, A, ac), L = q * ac.S * CL, D = q * ac.S * CD;
  const Tc = st.mode === 'recover' ? Tm : D + W * Math.sin(st.gam) + ac.m * 0.25 * (P.vt - st.V);
  const T = clamp(Tc, 0, Tm);
  st.V = Math.max(15, st.V + (T - D - W * Math.sin(st.gam)) / ac.m * dt);
  st.gam = clamp(st.gam + (L - W * Math.cos(st.gam)) / (ac.m * st.V) * dt, -0.6, 0.4);
  st.h += st.V * Math.sin(st.gam) * dt;
  if (st.h < 30) { st.h = 30; st.gam = Math.max(0, st.gam); }
  st.t += dt;
  const Vs = Math.sqrt(2 * W / (atm.rho * ac.S * A.CLmax));
  if (st.mode === 'hold' && st.alpha > A.alphaS) {
    st.mode = 'recover'; st.stallH = st.h; st.recT = 0;
    addLog(`Stall at ${altStr(st.h)}. The ${st.k > 0.002 ? 'iced ' : ''}wing can't make enough lift. Autopilot disconnects and the nose is lowered to recover.`, 'crit');
  }
  if (st.mode === 'recover') {
    st.recT += dt;
    if (st.recT > 4 && st.V > 1.3 * Vs && st.alpha < A.alphaS) {
      st.mode = 'hold'; const lost = st.stallH - st.h;
      P.ht = Math.max(0, Math.round(st.h / 100) * 100); syncInputs();
      addLog(`Recovered at ${altStr(st.h)} after losing ${SL.int(lost)} m. The autopilot now holds the new altitude.`, 'warn');
    }
  }
  // ice accretion
  const lf = P.cloud ? liquidFraction(atm.Tc) : 0;
  if (lf > 0) {
    st.kind = kindAt(atm.Tc);
    const rhoI = st.kind === 'rime' ? 880 : 917;
    st.k += collection(ac, st.V) * P.lwc * 1e-3 * st.V / rhoI * lf * dt;
    fromWing = false;
  }
  const mm = st.k * 1000;
  if (mm > 6 && !st.warned.light) { st.warned.light = true; addLog(`Ice on the leading edges has reached 6 mm (${st.kind}).`, 'warn'); }
  if (mm > 25 && !st.warned.mod) { st.warned.mod = true; addLog(`Ice at 25 mm. Stall speed is up ${Math.round((1 / Math.sqrt(1 - A.dCLmax) - 1) * 100)}%.`, 'warn'); }
  if (mm > 60 && !st.warned.sev) { st.warned.sev = true; addLog(`Severe ice: 60 mm. Maximum lift is down ${Math.round(A.dCLmax * 100)}%.`, 'crit'); }
}
let out = {};
function computeOutputs() {
  const ac = AC[P.ac], atm = SL.isa(st.h), A = aero(ac, st.k, st.kind), A0 = aero(ac, 0, 'rime');
  const W = ac.m * G, q = 0.5 * atm.rho * st.V * st.V, M = st.V / atm.a;
  const CL = CLof(st.alpha, A, ac), CD = CDof(CL, st.alpha, M, A, ac);
  out = { ac, atm, A, A0, W, q, M, CL, CD, L: q * ac.S * CL, D: q * ac.S * CD, Tm: Tmax(ac, atm.rho, st.V, M),
    Vs: Math.sqrt(2 * W / (atm.rho * ac.S * A.CLmax)), Vs0: Math.sqrt(2 * W / (atm.rho * ac.S * A0.CLmax)),
    Re: atm.rho * st.V * ac.c / atm.mu, vs: st.V * Math.sin(st.gam) };
  out.T = clamp(st.mode === 'recover' ? out.Tm : out.D + W * Math.sin(st.gam) + ac.m * 0.25 * (P.vt - st.V), 0, out.Tm);
}

// ---------- drawing ----------
const view = $('p-view'), ctx = view.getContext('2d');
let W = 0, H = 0, dpr = 1;
function size() {
  dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = view.parentElement.clientWidth; if (!w) return;
  const h = Math.round(Math.min(560, Math.max(340, w * 0.5)));
  W = Math.round(w * dpr); H = Math.round(h * dpr); view.width = W; view.height = H; view.style.height = h + 'px';
}
new ResizeObserver(() => { if (active) size(); }).observe($('p-stage'));
const clouds = Array.from({ length: 14 }, () => ({ x: Math.random(), y: Math.random(), s: 0.5 + Math.random(), z: 0.3 + Math.random() * 0.7 }));
let scroll = 0;
const mix = (a, b, t) => a.map((v, i) => Math.round(v + (b[i] - v) * t));
const rgb = a => `rgb(${a[0]},${a[1]},${a[2]})`;
function drawScene(dtReal) {
  if (!W) return;
  const ac = out.ac, f = clamp(st.h / 13000, 0, 1);
  const top = mix([62, 128, 196], [9, 26, 64], f), bot = mix([196, 222, 240], [74, 120, 180], f);
  const g = ctx.createLinearGradient(0, 0, 0, H); g.addColorStop(0, rgb(top)); g.addColorStop(1, rgb(bot));
  ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);
  scroll += dtReal * st.V * 0.004;
  // ground when low
  if (st.h < 3000) {
    const gy = H * (0.62 + 0.38 * st.h / 3000);
    ctx.fillStyle = 'rgba(62,92,70,0.9)'; ctx.fillRect(0, gy, W, H - gy);
    ctx.fillStyle = 'rgba(40,62,48,0.9)';
    for (let i = 0; i < 12; i++) { const x = ((i * 0.13 - scroll * 0.6) % 1 + 1) % 1 * W * 1.2 - W * 0.1; ctx.fillRect(x, gy, W * 0.05, 3 * dpr); }
  }
  // clouds
  const inCloud = P.cloud;
  for (const c of clouds) {
    const x = ((c.x - scroll * c.z * 0.35) % 1 + 1) % 1 * W * 1.3 - W * 0.15, y = (inCloud ? c.y : 0.7 + c.y * 0.3) * H;
    ctx.fillStyle = `rgba(240,244,248,${inCloud ? 0.35 : 0.5 * (1 - f)})`;
    ctx.beginPath(); ctx.ellipse(x, y, 90 * c.s * dpr, 22 * c.s * dpr, 0, 0, 7); ctx.fill();
    ctx.beginPath(); ctx.ellipse(x + 40 * c.s * dpr, y - 12 * c.s * dpr, 50 * c.s * dpr, 22 * c.s * dpr, 0, 0, 7); ctx.fill();
  }
  if (inCloud) { ctx.fillStyle = 'rgba(205,214,224,0.42)'; ctx.fillRect(0, 0, W, H); }
  // aircraft
  const cx = W * 0.5, cy = H * 0.46, Lpx = Math.min(W * 0.46, H * 0.95), theta = st.alpha + st.gam;
  ctx.save(); ctx.translate(cx, cy); ctx.rotate(-theta);
  drawAircraft(P.ac, Lpx, st.k / ac.c);
  ctx.restore();
  // forces
  const s = 0.2 * H / out.W, sT = s * 5;
  const arrow = (ang, len, col, label) => {
    const ex = cx + Math.cos(ang) * len, ey = cy - Math.sin(ang) * len;
    ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 3 * dpr; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke();
    const hs = 9 * dpr; ctx.beginPath(); ctx.moveTo(ex, ey);
    ctx.lineTo(ex - hs * Math.cos(ang - 0.4), ey + hs * Math.sin(ang - 0.4)); ctx.lineTo(ex - hs * Math.cos(ang + 0.4), ey + hs * Math.sin(ang + 0.4)); ctx.fill();
    ctx.font = `700 ${12 * dpr}px ${SL.css('--f-body')}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    const lx = cx + Math.cos(ang) * (len + 16 * dpr), ly = cy - Math.sin(ang) * (len + 16 * dpr);
    ctx.lineWidth = 3 * dpr; ctx.strokeStyle = 'rgba(5,15,25,0.7)'; ctx.strokeText(label, lx, ly); ctx.fillStyle = '#fff'; ctx.fillText(label, lx, ly);
  };
  arrow(st.gam + Math.PI / 2, out.L * s, '#7fe0a0', 'Lift');
  arrow(-Math.PI / 2, out.W * s, '#ffd27a', 'Weight');
  arrow(st.gam + Math.PI, out.D * sT, '#ff8f7a', 'Drag ×5');
  arrow(theta, out.T * sT, '#9fd2ff', 'Thrust ×5');
  // relative wind & alpha
  ctx.setLineDash([6 * dpr, 6 * dpr]); ctx.strokeStyle = 'rgba(255,255,255,0.7)'; ctx.lineWidth = 1.5 * dpr;
  const rw = Lpx * 0.75;
  ctx.beginPath(); ctx.moveTo(cx - Math.cos(st.gam) * rw, cy + Math.sin(st.gam) * rw); ctx.lineTo(cx + Math.cos(st.gam) * rw, cy - Math.sin(st.gam) * rw); ctx.stroke();
  ctx.setLineDash([]);
  ctx.font = `${12 * dpr}px ${SL.css('--f-mono')}`; ctx.fillStyle = 'rgba(255,255,255,0.85)'; ctx.textAlign = 'left';
  ctx.fillText('flight path', cx + Math.cos(st.gam) * rw - 80 * dpr, cy - Math.sin(st.gam) * rw - 10 * dpr);
  drawInset();
}
function drawAircraft(type, L, kc) {
  const hf = type === 'light' ? 0.12 : type === 'jet' ? 0.085 : 0.095, lw = 1.4 * dpr;
  ctx.lineJoin = 'round';
  const P2 = (x, y) => [x * L, -y * L];
  const path = pts => { ctx.beginPath(); pts.forEach(([x, y], i) => { const [X, Y] = P2(x, y); i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); }); ctx.closePath(); };
  const body = '#eef2f6', edge = '#26384a', accent = SL.css('--accent') || '#0b7ca8';
  const iceCol = 'rgba(200,240,255,0.95)', iceW = Math.max(0, Math.min(0.03, kc * 0.9));  // exaggerated ×~3 for visibility
  // far wing (behind fuselage), shown as darker section
  const wingY = type === 'turboprop' ? hf * 0.42 : -hf * 0.38;
  // fin
  ctx.fillStyle = body; ctx.strokeStyle = edge; ctx.lineWidth = lw;
  path([[-0.33, hf * 0.45], [-0.43, hf * 0.45 + 0.17], [-0.5, hf * 0.45 + 0.17], [-0.5, hf * 0.3]]); ctx.fill(); ctx.stroke();
  ctx.fillStyle = accent; path([[-0.4, hf * 0.45 + 0.1], [-0.43, hf * 0.45 + 0.17], [-0.5, hf * 0.45 + 0.17], [-0.5, hf * 0.45 + 0.1]]); ctx.fill();
  if (iceW > 0.001) { ctx.strokeStyle = iceCol; ctx.lineWidth = iceW * L; ctx.beginPath(); ctx.moveTo(...P2(-0.335, hf * 0.47)); ctx.lineTo(...P2(-0.43, hf * 0.45 + 0.168)); ctx.stroke(); }
  // fuselage
  ctx.fillStyle = body; ctx.strokeStyle = edge; ctx.lineWidth = lw;
  ctx.beginPath();
  ctx.moveTo(...P2(-0.5, hf * 0.32)); ctx.lineTo(...P2(-0.28, hf * 0.5)); ctx.lineTo(...P2(0.33, hf * 0.5));
  ctx.quadraticCurveTo(...P2(0.5, hf * 0.45), ...P2(0.5, 0)); ctx.quadraticCurveTo(...P2(0.49, -hf * 0.5), ...P2(0.33, -hf * 0.5));
  ctx.lineTo(...P2(-0.22, -hf * 0.5)); ctx.lineTo(...P2(-0.5, hf * 0.12)); ctx.closePath(); ctx.fill(); ctx.stroke();
  // windows & cockpit
  if (type !== 'light') { ctx.fillStyle = '#2c3d50'; for (let x = -0.22; x < 0.3; x += 0.022) { ctx.beginPath(); ctx.arc(...P2(x, hf * 0.12), 0.0045 * L, 0, 7); ctx.fill(); } }
  ctx.fillStyle = '#1d2c3c'; path(type === 'light' ? [[0.12, hf * 0.5], [0.2, hf * 0.95], [0.3, hf * 0.5]] : [[0.4, hf * 0.3], [0.455, hf * 0.28], [0.47, hf * 0.12], [0.43, hf * 0.15]]); ctx.fill();
  if (type === 'light') { ctx.fillStyle = body; ctx.strokeStyle = edge; path([[0.1, hf * 0.5], [0.2, hf * 0.95], [0.32, hf * 0.5]]); ctx.stroke(); }
  // stabiliser
  const sy = type === 'turboprop' ? hf * 0.45 + 0.16 : hf * 0.2;
  ctx.fillStyle = '#dfe5eb'; ctx.strokeStyle = edge; ctx.beginPath(); ctx.ellipse(...P2(-0.44, sy), 0.06 * L, 0.008 * L, 0, 0, 7); ctx.fill(); ctx.stroke();
  if (iceW > 0.001) { ctx.fillStyle = iceCol; ctx.beginPath(); ctx.ellipse(...P2(-0.385, sy), iceW * L * 0.9 + 1, 0.007 * L, 0, 0, 7); ctx.fill(); }
  // wing root section
  ctx.fillStyle = '#d5dce4'; ctx.strokeStyle = edge;
  ctx.beginPath(); ctx.ellipse(...P2(0.02, wingY), 0.11 * L, 0.013 * L, 0, 0, 7); ctx.fill(); ctx.stroke();
  if (iceW > 0.001) { ctx.fillStyle = iceCol; ctx.beginPath(); ctx.ellipse(...P2(0.13 + iceW * 0.3, wingY), iceW * L + 1.5, 0.012 * L, 0, 0, 7); ctx.fill(); }
  // engines
  ctx.fillStyle = '#cfd7df'; ctx.strokeStyle = edge;
  if (type === 'jet') { ctx.beginPath(); ctx.ellipse(...P2(0.07, wingY - 0.04), 0.07 * L, 0.022 * L, 0, 0, 7); ctx.fill(); ctx.stroke(); }
  else if (type === 'turboprop') {
    ctx.beginPath(); ctx.ellipse(...P2(0.07, wingY - 0.012), 0.07 * L, 0.02 * L, 0, 0, 7); ctx.fill(); ctx.stroke();
    ctx.fillStyle = 'rgba(30,40,50,0.35)'; ctx.beginPath(); ctx.ellipse(...P2(0.145, wingY - 0.012), 0.004 * L, 0.075 * L, 0, 0, 7); ctx.fill();
  } else { ctx.fillStyle = 'rgba(30,40,50,0.4)'; ctx.beginPath(); ctx.ellipse(...P2(0.505, 0), 0.004 * L, 0.1 * L, 0, 0, 7); ctx.fill(); }
}
function drawInset() {
  // true-scale close-up of the wing's leading edge
  const r = Math.min(110 * dpr, H * 0.24), cx = r + 14 * dpr, cy = H - r - 60 * dpr;
  ctx.save();
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, 7); ctx.fillStyle = 'rgba(6,16,26,0.82)'; ctx.fill();
  ctx.strokeStyle = 'rgba(255,255,255,0.3)'; ctx.lineWidth = dpr; ctx.stroke(); ctx.clip();
  const ac = out.ac, kc = st.k / ac.c, sc = r * 7, ox = cx - r * 0.45, oy = cy;   // 1 chord = sc px; view ~ first 15% of chord
  const sec = SL.section, m = sec.m / 100, p = sec.p / 10, t = sec.t / 100;
  const tf = (x, y) => [ox + x * sc, oy - y * sc];
  // ice
  if (kc > 1e-4) {
    ctx.fillStyle = 'rgba(210,240,255,0.95)';
    if (fromWing && SL.ice.points && SL.ice.points.length) {
      const pts = SL.ice.points, scale = kc / Math.max(1e-6, SL.ice.kc || kc), cs = Math.max(1.5, SL.ice.cell * sc * scale);
      for (let i = 0; i < pts.length; i += 2) {
        const nx = pts[i], ny = pts[i + 1];
        const [X, Y] = tf(nx, ny); ctx.fillRect(X - cs / 2, Y - cs / 2, cs, cs);
      }
    } else {
      const horn = st.kind === 'glaze' ? 1 : st.kind === 'mixed' ? 0.5 : 0;
      ctx.beginPath();
      const n = 80;
      for (let i = 0; i <= n; i++) {
        const s = -1 + 2 * i / n;                          // −1 lower … +1 upper, 0 = stagnation
        const x = 0.09 * s * s, up = s >= 0;
        const yt = SL.naca.yt(t, x), yc = SL.naca.yc(m, p, x);
        const y = yc + (up ? yt : -yt);
        const dxs = 1e-3, x2 = Math.max(0, x + dxs), y2 = SL.naca.yc(m, p, x2) + (up ? 1 : -1) * SL.naca.yt(t, x2);
        let nx = -(y2 - y), ny = x2 - x; if (!up) { nx = -nx; ny = -ny; }
        const nl = Math.hypot(nx, ny) || 1; nx /= nl; ny /= nl;
        const a = Math.abs(s);
        const th = kc * ((1 - horn) * Math.exp(-((a / 0.55) ** 2)) + horn * (0.35 * Math.exp(-((a / 0.4) ** 2)) + 1.1 * Math.exp(-(((a - 0.55) / 0.18) ** 2))));
        const [X, Y] = tf(x - nx * th * 0 + nx * th, y + ny * th);
        i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y);
      }
      for (let i = n; i >= 0; i--) { const s = -1 + 2 * i / n, x = 0.09 * s * s, up = s >= 0, y = SL.naca.yc(m, p, x) + (up ? 1 : -1) * SL.naca.yt(t, x); ctx.lineTo(...tf(x, y)); }
      ctx.closePath(); ctx.fill();
    }
  }
  // airfoil
  const pts = SL.naca.points(m, p, t, 0, 120);
  ctx.beginPath(); pts.forEach(([x, y], i) => { const [X, Y] = tf(x, y); i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); }); ctx.closePath();
  ctx.fillStyle = '#3a4552'; ctx.fill(); ctx.strokeStyle = 'rgba(235,240,245,0.85)'; ctx.lineWidth = 1.2 * dpr; ctx.stroke();
  ctx.restore();
  ctx.font = `700 ${11 * dpr}px ${SL.css('--f-body')}`; ctx.fillStyle = '#fff'; ctx.textAlign = 'center';
  ctx.fillText('Leading edge, true scale', cx, cy + r + 16 * dpr);
  ctx.font = `${11 * dpr}px ${SL.css('--f-mono')}`; ctx.fillStyle = 'rgba(255,255,255,0.75)';
  ctx.fillText(`${(ac.c * 2 * r / sc * 100).toFixed(0)} cm across · ice ${Math.round(st.k * 1000)} mm`, cx, cy + r + 31 * dpr);
}

function drawLift() {
  const ac = out.ac, A = out.A, A0 = out.A0;
  const ch = SL.chart($('p-lift'), { x0: -4, x1: 24, y0: -0.4, y1: Math.ceil((A0.CLmax * 1.15) * 5) / 5, dy: 0.4, dx: 4, xLabel: 'angle of attack (°)', xFmt: v => v + '°', padL: 36 });
  if (!ch) return;
  const { c, X, Y } = ch;
  const curve = (AA, col, dash) => {
    c.strokeStyle = col; c.lineWidth = 2; c.setLineDash(dash ? [5, 4] : []); c.beginPath();
    for (let a = -4; a <= 24; a += 0.25) { const v = CLof(a * D2R, AA, ac); a === -4 ? c.moveTo(X(a), Y(v)) : c.lineTo(X(a), Y(v)); }
    c.stroke(); c.setLineDash([]);
  };
  curve(A0, SL.css('--muted'), true); curve(A, SL.css('--accent'), false);
  const as = A.alphaS / D2R; c.strokeStyle = SL.css('--crit'); c.globalAlpha = 0.5; c.beginPath(); c.moveTo(X(as), Y(-0.4)); c.lineTo(X(as), Y(A.CLmax)); c.stroke(); c.globalAlpha = 1;
  c.fillStyle = SL.css('--crit'); c.font = `11px ${SL.css('--f-mono')}`; c.textAlign = 'left'; c.fillText(`stall ${fmt(as, 1)}°`, X(as) + 4, Y(-0.25));
  const a = st.alpha / D2R; c.fillStyle = st.mode === 'recover' ? SL.css('--crit') : SL.css('--ink');
  c.beginPath(); c.arc(X(clamp(a, -4, 24)), Y(out.CL), 5, 0, 7); c.fill();
}
function drawDrag() {
  const ac = out.ac, atm = out.atm, A = out.A, A0 = out.A0, W = out.W;
  const v0 = Math.max(10, out.Vs0 * 0.8), v1 = Math.max(ac.v[1] * 1.1, st.V * 1.15);
  const dragAt = (V, AA) => { const q = 0.5 * atm.rho * V * V, CL = W / (q * ac.S); if (CL > AA.CLmax) return null; const al = ac.alpha0 + CL / AA.aw; return q * ac.S * CDof(CL, al, V / atm.a, AA, ac); };
  let ymax = 0; const samples = [];
  for (let i = 0; i <= 80; i++) { const V = v0 + (v1 - v0) * i / 80, d0 = dragAt(V, A0), d1 = dragAt(V, A), t = Tmax(ac, atm.rho, V, V / atm.a); samples.push([V, d0, d1, t]); ymax = Math.max(ymax, d0 || 0, d1 || 0, t); }
  ymax = Math.min(ymax, W * 0.25) / 1000 * 1.1;
  const dy = [0.5, 1, 2, 5, 10, 20, 50].find(s => ymax / s <= 5) || 50;
  const ch = SL.chart($('p-drag'), { x0: v0, x1: v1, y0: 0, y1: Math.ceil(ymax / dy) * dy, dy, dx: v1 - v0 > 120 ? 40 : 20, xLabel: 'true airspeed (m/s)', yFmt: v => v + ' kN', padL: 54 });
  if (!ch) return;
  const { c, X, Y } = ch;
  const line = (idx, col, dash) => {
    c.strokeStyle = col; c.lineWidth = 2; c.setLineDash(dash ? [5, 4] : []); c.beginPath(); let pen = false;
    for (const s of samples) { const v = s[idx]; if (v == null) { pen = false; continue; } const y = Y(Math.min(v / 1000, ch.h)); pen ? c.lineTo(X(s[0]), y) : c.moveTo(X(s[0]), y); pen = true; }
    c.stroke(); c.setLineDash([]);
  };
  line(1, SL.css('--muted'), true); line(2, SL.css('--accent'), false); line(3, SL.css('--warm'), false);
  c.strokeStyle = SL.css('--crit'); c.globalAlpha = 0.6; c.beginPath(); c.moveTo(X(out.Vs), Y(0)); c.lineTo(X(out.Vs), ch.pt); c.stroke(); c.globalAlpha = 1;
  c.fillStyle = SL.css('--crit'); c.font = `11px ${SL.css('--f-mono')}`; c.textAlign = 'left'; c.fillText('stall', X(out.Vs) + 4, ch.pt + 10);
  c.fillStyle = SL.css('--ink'); c.beginPath(); c.arc(X(clamp(st.V, v0, v1)), Y(out.D / 1000), 5, 0, 7); c.fill();
}

function updateHUD() {
  const { atm, A, M, Re } = out, ac = out.ac;
  $('p-h').innerHTML = `${SL.int(st.h)} m <small>FL${String(Math.round(st.h * 3.28084 / 100)).padStart(3, '0')}</small>`;
  $('p-v').innerHTML = `${Math.round(st.V * 1.944)} kt <small>M${M.toFixed(2)}</small>`;
  const vs = out.vs; $('p-vs').innerHTML = `${vs >= 0 ? '+' : '−'}${SL.int(Math.abs(vs * 196.85))} <small>ft/min</small>`;
  $('p-t').textContent = `${fmt(atm.Tc, 0)} °C · ρ ${atm.rho.toFixed(3)}`;
  $('p-re').textContent = (Re / 1e6).toFixed(1) + ' million';
  $('p-a').innerHTML = `${fmt(st.alpha / D2R, 1)}° <small>stall ${fmt(A.alphaS / D2R, 1)}°</small>`;
  $('p-vstall').innerHTML = `${Math.round(out.Vs * 1.944)} kt <small>${out.Vs > out.Vs0 * 1.005 ? '+' + Math.round((out.Vs / out.Vs0 - 1) * 100) + '% from ice' : 'clean'}</small>`;
  $('p-ice').innerHTML = st.k > 0.0005 ? `${Math.round(st.k * 1000)} mm <small>${st.kind}</small>` : 'none';
  $('p-clock').textContent = clock(st.t);
  const margin = st.V / out.Vs;
  const banner = $('p-banner');
  if (st.mode === 'recover') { banner.hidden = false; banner.className = 'banner crit'; banner.textContent = 'Stall · recovering'; }
  else if (margin < 1.2) { banner.hidden = false; banner.className = 'banner warn'; banner.textContent = 'Stall warning'; }
  else banner.hidden = true;
  const chip = $('p-chip');
  if (st.mode === 'recover') { chip.textContent = 'Stalled'; chip.className = 'chip crit'; }
  else if (out.T >= out.Tm * 0.995 && st.V < P.vt - 3) { chip.textContent = 'Full power, losing speed'; chip.className = 'chip warn'; }
  else if (Math.abs(out.vs) > 1.5) { chip.textContent = out.vs > 0 ? 'Climbing' : 'Descending'; chip.className = 'chip neutral'; }
  else { chip.textContent = 'Level flight'; chip.className = 'chip good'; }
  const lf = P.cloud ? liquidFraction(atm.Tc) : 0, ic = $('p-icechip');
  ic.hidden = !(lf > 0); ic.textContent = `${st.kind[0].toUpperCase() + st.kind.slice(1)} ice building`;
  $('p-summary').innerHTML = `L/D <b>${fmt(out.L / Math.max(1, out.D), 1)}</b> · C<sub>L</sub> <b>${fmt(out.CL)}</b> · thrust used <b>${Math.round(out.T / out.Tm * 100)}%</b> · margin above stall <b>${Math.round((margin - 1) * 100)}%</b>`;
  // weather card
  const atT = SL.isa(P.ht);
  $('p-oat').textContent = `${fmt(atT.Tc, 0)} °C at ${SL.int(P.ht)} m`;
  const kindT = SL.iceKind(atT.Tc, P.lwc), lfT = liquidFraction(atT.Tc);
  let msg;
  if (atT.Tc >= 0) msg = `<b>No icing</b><p>Above freezing at this altitude. Rain, not ice.</p>`;
  else if (lfT <= 0) msg = `<b>Too cold to ice</b><p>At ${fmt(atT.Tc, 0)} °C almost all cloud particles are ice crystals, which bounce off the airframe.</p>`;
  else msg = `<b>${kindT.name} likely in cloud</b><p>${kindT.desc} About ${Math.round(lfT * 100)}% of the cloud is still supercooled liquid at this temperature. Collection efficiency β ≈ ${collection(ac, P.vt).toFixed(2)}.</p>`;
  $('p-kind').innerHTML = msg;
}

// ---------- controls ----------
function syncInputs() {
  const ac = AC[P.ac];
  $('p-spd').min = ac.v[0]; $('p-spd').max = ac.v[1];
  $('p-alt').value = P.ht; $('p-spd').value = P.vt; $('p-cloud').checked = P.cloud; $('p-lwc').value = P.lwc; $('p-mvd').value = P.mvd;
  $('p-alt-o').textContent = `${SL.int(P.ht)} m · ${SL.int(P.ht * 3.28084)} ft`;
  $('p-spd-o').textContent = `${P.vt} m/s · ${Math.round(P.vt * 1.944)} kt`;
  $('p-lwc-o').textContent = P.lwc.toFixed(2) + ' g/m³'; $('p-mvd-o').textContent = P.mvd + ' µm';
  $('p-ts-o').textContent = P.ts === 1 ? '1 s = 1 s' : `1 s = ${P.ts} s`;
  $('p-acdesc').textContent = ac.desc;
  $('p-boots').disabled = !ac.deice; $('p-boots').textContent = P.ac === 'jet' ? 'Heat leading edges' : 'Inflate de-icing boots';
  $('p-boots').title = ac.deice ? '' : 'This aircraft has no ice protection';
}
function setAircraft(k) { P.ac = k; const ac = AC[k]; P.vt = ac.vt; P.ht = ac.ht; SL.syncSeg('p-ac', k); syncInputs(); resetSim(); }
SL.seg('p-ac', setAircraft);
SL.on('design', d => { AC.design = d; $('p-ac-design').hidden = false; setAircraft('design'); addLog('Loaded your design from the Designer.', 'good'); });
SL.seg('p-ts', v => { P.ts = +v; syncInputs(); });
$('p-alt').addEventListener('input', e => { P.ht = +e.target.value; syncInputs(); });
$('p-spd').addEventListener('input', e => { P.vt = +e.target.value; syncInputs(); });
$('p-cloud').onchange = e => { P.cloud = e.target.checked; addLog(P.cloud ? `Entered cloud at ${altStr(st.h)}, ${fmt(SL.isa(st.h).Tc, 0)} °C.` : 'Left the cloud.', P.cloud ? 'warn' : 'good'); };
$('p-lwc').addEventListener('input', e => { P.lwc = +e.target.value; syncInputs(); });
$('p-mvd').addEventListener('input', e => { P.mvd = +e.target.value; syncInputs(); });
$('p-boots').onclick = () => {
  if (!AC[P.ac].deice) return;
  const before = st.k * 1000;
  if (before < 0.5) { addLog(P.ac === 'jet' ? 'Wing anti-ice on: no ice to remove.' : 'Boots cycled. No ice to shed.', 'good'); return; }
  st.k *= P.mvd > 50 ? 0.55 : 0.12; fromWing = false;
  addLog(P.ac === 'jet' ? `Leading edges heated: ${Math.round(before)} mm → ${Math.round(st.k * 1000)} mm.` : `Boots inflated and cracked off ice: ${Math.round(before)} mm → ${Math.round(st.k * 1000)} mm${P.mvd > 50 ? '. Large droplets left ice behind the boots.' : ' (some residual ice always remains).'}`, 'good');
};
$('p-clear').onclick = () => { st.k = 0; fromWing = false; addLog('Ice cleared.', 'good'); };
$('p-fromwing').onclick = () => {
  if (!SL.ice.kc || !SL.ice.points || SL.ice.shape !== 'airfoil') { SL.toast($('p-toast'), 'No ice yet. Open the Wing view, switch on the icing cloud, and let some ice build up first.'); return; }
  st.k = SL.ice.kc * AC[P.ac].c; st.kind = SL.ice.kind === 'none' ? 'rime' : SL.ice.kind; fromWing = true;
  addLog(`Loaded ice from the Wing view: ${(SL.ice.kc * 100).toFixed(1)}% of chord, ${Math.round(st.k * 1000)} mm on this wing.`, 'warn');
};
$('p-play').onclick = () => { P.running = !P.running; $('p-play').textContent = P.running ? 'Pause' : 'Resume'; };
$('p-restart').onclick = () => resetSim();
const PRESETS = {
  cruise: { ac: 'jet', ht: 11000, vt: 230, cloud: true, lwc: 0.3, mvd: 20, ts: 30 },
  hold: { ac: 'turboprop', ht: 3500, vt: 92, cloud: true, lwc: 0.55, mvd: 25, ts: 60 },
  light: { ac: 'light', ht: 2000, vt: 55, cloud: true, lwc: 0.35, mvd: 20, ts: 30 },
  sld: { ac: 'turboprop', ht: 2500, vt: 105, cloud: true, lwc: 0.3, mvd: 150, ts: 60 },
};
$('p-try').addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b) return;
  const p = PRESETS[b.dataset.p];
  P.ac = p.ac; SL.syncSeg('p-ac', p.ac);
  Object.assign(P, p); SL.syncSeg('p-ts', String(P.ts)); syncInputs(); resetSim();
  addLog(P.cloud ? `Flying in cloud at ${altStr(P.ht)}, ${fmt(SL.isa(P.ht).Tc, 0)} °C.` : 'Clear air.', 'warn');
});

function loop(now) {
  if (!active) return;
  requestAnimationFrame(loop);
  const dtReal = Math.min(0.1, (now - lastT) / 1000 || 0); lastT = now;
  if (P.running) {
    const simDt = dtReal * P.ts, n = Math.max(1, Math.ceil(simDt / 0.02));
    for (let i = 0; i < n; i++) stepSim(simDt / n);
  }
  computeOutputs();
  drawScene(P.running ? dtReal * Math.min(P.ts, 10) / 10 : 0);
  if (++frames % 4 === 0) { updateHUD(); drawLift(); drawDrag(); }
}
syncInputs(); resetSim();
SL.views.aircraft = {
  activate() { active = true; size(); lastT = performance.now(); requestAnimationFrame(loop); },
  deactivate() { active = false; },
};
})();
