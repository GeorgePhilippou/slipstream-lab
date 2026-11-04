// Flight and icing module: ISA atmosphere + point-mass longitudinal dynamics, aircraft from the VLM solve.
(() => {
const { $, fmt, int, clamp, D2R } = SL;
const G = 9.80665;
const AP = { ht: 2500, vt: 60, cloud: false, lwc: 0.4, mvd: 20, ts: 30, running: true };
let ac = null, st = null, log = [], active = false, lastT = 0, frames = 0, fromAirfoil = false, acVersion = -1;

// ---------- physics ----------
function aeroOf(k, kind) {
  const AR = ac.b * ac.b / ac.S, kc = k / ac.c, s = Math.sqrt(Math.max(0, kc)), hf = kind === 'glaze' ? 1 : kind === 'mixed' ? 0.82 : 0.65;
  const dCLmax = Math.min(0.6, 3.0 * s * hf), dCD0 = 0.25 * s * hf, a2d = ac.a0 * (1 - 0.5 * s), aw = a2d / (1 + a2d / (Math.PI * ac.e * AR));
  const CLmax = ac.CLmax * (1 - dCLmax);
  return { AR, aw, CLmax, alphaS: ac.alpha0 + 1.2 * CLmax / aw, dCD0, dCLmax, kc };
}
function CLof(al, A) { const x = (al - ac.alpha0) * A.aw / A.CLmax; const n = x < 0.8 ? x : x < 1.2 ? x - (x - 0.8) ** 2 / 0.8 : Math.max(0.55, 1 - 0.9 * (x - 1.2)); return n * A.CLmax; }
function CDof(CL, al, M, A) { const x = (al - ac.alpha0) * A.aw / A.CLmax; return ac.CD0 + A.dCD0 + CL * CL / (Math.PI * ac.e * A.AR) + (x > 1.2 ? 0.6 * (x - 1.2) : 0) + (M > ac.Mcrit ? 20 * (M - ac.Mcrit) ** 4 : 0); }
function Tmax(rho, V, M) { const sig = rho / 1.225; if (ac.prop) return Math.min(ac.Tstatic * sig, ac.eta * ac.P * sig / Math.max(V, 20)); return ac.T0 * sig ** 0.85 * (1 - 0.25 * M); }
function collection(V) { const d = AP.mvd * 1e-6, K = 1000 * d * d * V / (9 * 1.7e-5 * 0.2 * ac.c); if (K <= 0.125) return 0; const z = 1.4 * Math.pow(K - 0.125, 0.84); return 0.85 * z / (1 + z); }
const liquid = Tc => Tc >= 0 ? 0 : clamp((Tc + 40) / 25, 0, 1);
const kindAt = Tc => { const k = SL.iceKind(Tc, AP.lwc).key; return k === 'none' ? 'rime' : k; };
const clock = t => { const m = Math.floor(t / 60), s = Math.floor(t % 60); return m >= 60 ? `${Math.floor(m / 60)}h${String(m % 60).padStart(2, '0')}` : `${m}:${String(s).padStart(2, '0')}`; };
function addLog(msg, cls = '') { log.unshift({ t: st ? st.t : 0, msg, cls }); log = log.slice(0, 10); }

function buildAircraft(resetState) {
  ac = SL.aircraftFromProject(); acVersion = SL.version;
  if (resetState || !st) {
    AP.ht = SL.P.flight.h; AP.vt = SL.P.flight.V;
    st = { h: AP.ht, V: AP.vt, gam: 0, alpha: 3 * D2R, k: 0, kind: 'rime', t: 0, mode: 'hold', recT: 0, stallH: 0, warned: {} };
    const atm = SL.isa(st.h), A = aeroOf(0, 'rime'), q = 0.5 * atm.rho * st.V * st.V;
    st.alpha = clamp(ac.alpha0 + ac.m * G / (q * ac.S) / A.aw, -4 * D2R, 14 * D2R);
    log = []; fromAirfoil = false;
    addLog(`Start: ${int(st.h)} m, ${Math.round(st.V * 1.944)} kt, ${fmt(atm.Tc, 0)} °C.`);
  }
}
function step(dt) {
  const atm = SL.isa(st.h), A = aeroOf(st.k, st.kind), W = ac.m * G, q = 0.5 * atm.rho * st.V * st.V, M = st.V / atm.a, Tm = Tmax(atm.rho, st.V, M);
  let alCmd;
  if (st.mode === 'recover') alCmd = ac.alpha0 + 0.7 * (A.alphaS - ac.alpha0);
  else {
    const CLl = W / (q * ac.S), D0 = q * ac.S * CDof(CLl, ac.alpha0 + CLl / A.aw, M, A), gmax = Math.max(0, (Tm - D0) / W * 0.7);
    let gc = clamp(0.002 * (AP.ht - st.h), -0.08, 0.08); if (gc > 0) gc = Math.min(gc, gmax);
    alCmd = ac.alpha0 + (W * Math.cos(st.gam) + ac.m * st.V * 0.8 * (gc - st.gam)) / (q * ac.S) / A.aw;
  }
  st.alpha += (clamp(alCmd, -6 * D2R, 24 * D2R) - st.alpha) * Math.min(1, dt * 2.5);
  const CL = CLof(st.alpha, A), CD = CDof(CL, st.alpha, M, A), L = q * ac.S * CL, D = q * ac.S * CD;
  const T = clamp(st.mode === 'recover' ? Tm : D + W * Math.sin(st.gam) + ac.m * 0.25 * (AP.vt - st.V), 0, Tm);
  st.V = Math.max(12, st.V + (T - D - W * Math.sin(st.gam)) / ac.m * dt);
  st.gam = clamp(st.gam + (L - W * Math.cos(st.gam)) / (ac.m * st.V) * dt, -0.6, 0.4);
  st.h += st.V * Math.sin(st.gam) * dt; if (st.h < 30) { st.h = 30; st.gam = Math.max(0, st.gam); }
  st.t += dt;
  const Vs = Math.sqrt(2 * W / (atm.rho * ac.S * A.CLmax));
  if (st.mode === 'hold' && st.alpha > A.alphaS) { st.mode = 'recover'; st.stallH = st.h; st.recT = 0; addLog(`Stall at ${int(st.h)} m${st.k > 0.002 ? ' with ' + Math.round(st.k * 1000) + ' mm of ice' : ''}. Autopilot off; nose lowered.`, 'crit'); }
  if (st.mode === 'recover') { st.recT += dt; if (st.recT > 4 && st.V > 1.3 * Vs && st.alpha < A.alphaS) { st.mode = 'hold'; AP.ht = Math.max(0, Math.round(st.h / 100) * 100); addLog(`Recovered at ${int(st.h)} m, ${int(st.stallH - st.h)} m lost.`, 'warn'); SL.refreshInspector(); } }
  const lf = AP.cloud ? liquid(atm.Tc) : 0;
  if (lf > 0) { st.kind = kindAt(atm.Tc); st.k += collection(st.V) * AP.lwc * 1e-3 * st.V / (st.kind === 'rime' ? 880 : 917) * lf * dt; fromAirfoil = false; }
  const mm = st.k * 1000;
  for (const [lim, msg, cls] of [[6, 'Ice on the leading edges: 6 mm.', 'warn'], [25, `Ice 25 mm: stall speed up ${Math.round((1 / Math.sqrt(1 - A.dCLmax) - 1) * 100)}%.`, 'warn'], [60, `Ice 60 mm: maximum lift down ${Math.round(A.dCLmax * 100)}%.`, 'crit']]) if (mm > lim && !st.warned[lim]) { st.warned[lim] = 1; addLog(msg, cls); }
}
let O = {};
function outputs() {
  const atm = SL.isa(st.h), A = aeroOf(st.k, st.kind), A0 = aeroOf(0, 'rime'), W = ac.m * G, q = 0.5 * atm.rho * st.V * st.V, M = st.V / atm.a;
  const CL = CLof(st.alpha, A), CD = CDof(CL, st.alpha, M, A), Tm = Tmax(atm.rho, st.V, M), D = q * ac.S * CD;
  O = { atm, A, A0, W, q, M, CL, CD, L: q * ac.S * CL, D, Tm, T: clamp(st.mode === 'recover' ? Tm : D + W * Math.sin(st.gam) + ac.m * 0.25 * (AP.vt - st.V), 0, Tm),
    Vs: Math.sqrt(2 * W / (atm.rho * ac.S * A.CLmax)), Vs0: Math.sqrt(2 * W / (atm.rho * ac.S * A0.CLmax)), Re: atm.rho * st.V * ac.c / atm.mu, vs: st.V * Math.sin(st.gam) };
}

// ---------- drawing (schematic) ----------
const cv = () => $('cv-flight');
function draw() {
  const c = cv(), dpr = Math.min(2, window.devicePixelRatio || 1), Wc = c.clientWidth, Hc = c.clientHeight; if (!Wc || !Hc) return;
  if (c.width !== Math.round(Wc * dpr)) { c.width = Math.round(Wc * dpr); c.height = Math.round(Hc * dpr); }
  const x = c.getContext('2d'), W = c.width, H = c.height, P = SL.P;
  x.fillStyle = SL.css('--vp'); x.fillRect(0, 0, W, H);
  const cx = W * 0.5, cy = H * 0.48;
  // horizon reference and pitch ladder
  x.strokeStyle = SL.css('--grid-strong'); x.lineWidth = dpr; x.beginPath(); x.moveTo(0, cy + 0.5); x.lineTo(W, cy + 0.5); x.stroke();
  x.font = `${10 * dpr}px ${SL.css('--mono')}`; x.fillStyle = SL.css('--faint'); x.textAlign = 'left';
  for (let p = -20; p <= 20; p += 5) { if (!p) continue; const y = cy - Math.tan(p * D2R) * W * 0.35; x.strokeStyle = SL.css('--grid'); x.beginPath(); x.moveTo(W * 0.06, y); x.lineTo(W * 0.11, y); x.stroke(); x.fillText(`${p > 0 ? '+' : ''}${p}°`, W * 0.115, y + 3 * dpr); }
  x.fillText('horizon', W * 0.06, cy - 5 * dpr);
  if (AP.cloud) { x.fillStyle = 'rgba(200,210,220,0.05)'; x.fillRect(0, 0, W, H); x.fillStyle = SL.css('--faint'); x.fillText('in cloud', W - 70 * dpr, 16 * dpr); }
  // aircraft schematic, side view, in metres
  const B = P.body, span = B.lf * 1.1, sc = Math.min(W * 0.52, H * 1.1) / span, th = st.alpha + st.gam;
  x.save(); x.translate(cx, cy); x.rotate(-th); x.scale(sc, sc);
  const xm = B.xnose + B.lf * 0.45, R = B.df / 2, lw = 1.2 * dpr / sc;
  const X = v => xm - v;   // nose (small x) drawn to the right, flying right
  x.lineJoin = 'round';
  x.fillStyle = '#3a4048'; x.strokeStyle = '#aab2bb'; x.lineWidth = lw;
  if (P.tail.on) { const T = P.tail, h = Math.max(0.38 * T.bt, T.zt + 0.3 * T.bt), a0 = T.lt - 0.15 * T.ct, c0 = T.ct * 1.25; x.beginPath(); x.moveTo(X(a0), -0.1 * R); x.lineTo(X(a0 + 0.6 * c0), -h); x.lineTo(X(a0 + 1.2 * c0), -h); x.lineTo(X(a0 + c0), -0.1 * R); x.closePath(); x.fill(); x.stroke(); }
  x.beginPath(); const n = 40;
  for (let i = 0; i <= n; i++) { const u = i / n, r = u < 0.18 ? R * Math.sqrt(Math.max(0, 1 - ((0.18 - u) / 0.18) ** 2)) : u < 0.6 ? R : R * (1 - 0.82 * ((u - 0.6) / 0.4) ** 1.3), zc = (u > 0.6 ? 0.3 * R * ((u - 0.6) / 0.4) ** 2 : 0) - 0.2 * R; x.lineTo(X(B.xnose + u * B.lf), -(zc + r)); }
  for (let i = n; i >= 0; i--) { const u = i / n, r = u < 0.18 ? R * Math.sqrt(Math.max(0, 1 - ((0.18 - u) / 0.18) ** 2)) : u < 0.6 ? R : R * (1 - 0.82 * ((u - 0.6) / 0.4) ** 1.3), zc = (u > 0.6 ? 0.3 * R * ((u - 0.6) / 0.4) ** 2 : 0) - 0.2 * R; x.lineTo(X(B.xnose + u * B.lf), -(zc - r)); }
  x.closePath(); x.fillStyle = '#4a515a'; x.fill(); x.stroke();
  // wing root section and tailplane as true airfoils
  const sec = SL.section(), wcr = P.wing.cr, airfoil = (x0, z0, c, m, p, t, inc) => { const pts = SL.naca.points(m, p, t, 0, 30), ci = Math.cos(inc * D2R), si = Math.sin(inc * D2R); x.beginPath(); pts.forEach(([u, v], i) => { const px = X(x0) - c * (u * ci + v * si), pz = -(z0 + c * (-u * si + v * ci)); i ? x.lineTo(px, pz) : x.moveTo(px, pz); }); x.closePath(); x.fillStyle = '#626a74'; x.fill(); x.stroke(); };
  airfoil(0, -0.15 * R, wcr, sec.m / 100, sec.p / 10, sec.t / 100, P.wing.inc);
  if (P.tail.on) airfoil(P.tail.lt, P.tail.zt, P.tail.ct, 0, 0, 0.1, P.tail.it);
  if (st.k > 0.0005) { const kk = st.k * 6; x.strokeStyle = '#e2eef6'; x.lineWidth = Math.max(lw * 2, kk); x.beginPath(); x.arc(X(0) - 0.05 * wcr, 0.15 * R, 0.055 * wcr, -Math.PI * 0.4, Math.PI * 0.4); x.stroke(); }
  x.restore();
  // force vectors (lift/weight to scale; thrust/drag ×5)
  const s = 0.18 * H / O.W, arrow = (ang, len, col, lab) => {
    const ex = cx + Math.cos(ang) * len, ey = cy - Math.sin(ang) * len, hs = 7 * dpr;
    x.strokeStyle = col; x.fillStyle = col; x.lineWidth = 1.6 * dpr; x.beginPath(); x.moveTo(cx, cy); x.lineTo(ex, ey); x.stroke();
    x.beginPath(); x.moveTo(ex, ey); x.lineTo(ex - hs * Math.cos(ang - 0.35), ey + hs * Math.sin(ang - 0.35)); x.lineTo(ex - hs * Math.cos(ang + 0.35), ey + hs * Math.sin(ang + 0.35)); x.fill();
    x.font = `${10.5 * dpr}px ${SL.css('--mono')}`; x.textAlign = 'center'; x.fillText(lab, cx + Math.cos(ang) * (len + 14 * dpr), cy - Math.sin(ang) * (len + 14 * dpr) + 3 * dpr);
  };
  arrow(st.gam + Math.PI / 2, O.L * s, '#5bb07c', 'L'); arrow(-Math.PI / 2, O.W * s, '#d6a24a', 'W');
  arrow(st.gam + Math.PI, O.D * s * 5, '#e0625c', 'D×5'); arrow(th, O.T * s * 5, '#5b9cf2', 'T×5');
  x.setLineDash([5 * dpr, 4 * dpr]); x.strokeStyle = SL.css('--muted'); x.lineWidth = dpr; const rw = W * 0.4;
  x.beginPath(); x.moveTo(cx - Math.cos(st.gam) * rw, cy + Math.sin(st.gam) * rw); x.lineTo(cx + Math.cos(st.gam) * rw, cy - Math.sin(st.gam) * rw); x.stroke(); x.setLineDash([]);
  x.fillStyle = SL.css('--muted'); x.textAlign = 'right'; x.fillText('flight path', cx + Math.cos(st.gam) * rw, cy - Math.sin(st.gam) * rw - 6 * dpr);
  // leading-edge close-up at true scale
  const r = Math.min(90 * dpr, H * 0.2), ix = W - r - 14 * dpr, iy = H - r - 14 * dpr;
  x.save(); x.fillStyle = SL.css('--chrome'); x.strokeStyle = SL.css('--line'); x.lineWidth = dpr; x.fillRect(ix - r, iy - r, 2 * r, 2 * r); x.strokeRect(ix - r + 0.5, iy - r + 0.5, 2 * r - 1, 2 * r - 1);
  x.beginPath(); x.rect(ix - r, iy - r, 2 * r, 2 * r); x.clip();
  const scL = r * 8, ox = ix - r * 0.6, tf = (u, v) => [ox + u * scL, iy - v * scL], m = sec.m / 100, p = sec.p / 10, t = sec.t / 100, kc = st.k / ac.c;
  if (kc > 1e-4) {
    x.fillStyle = '#e2eef6';
    if (fromAirfoil && SL.ice.points && SL.ice.points.length) { const pts = SL.ice.points, scale = kc / Math.max(1e-6, SL.ice.kc || kc), cs = Math.max(1.5, SL.ice.cell * scL * scale); for (let i = 0; i < pts.length; i += 2) { const [X2, Y2] = tf(pts[i], pts[i + 1]); x.fillRect(X2 - cs / 2, Y2 - cs / 2, cs, cs); } }
    else {
      const horn = st.kind === 'glaze' ? 1 : st.kind === 'mixed' ? 0.5 : 0; x.beginPath();
      for (let i = 0; i <= 80; i++) { const s2 = -1 + i / 40, u = 0.09 * s2 * s2, up = s2 >= 0, y = SL.naca.yc(m, p, u) + (up ? 1 : -1) * SL.naca.yt(t, u), u2 = u + 1e-3, y2 = SL.naca.yc(m, p, u2) + (up ? 1 : -1) * SL.naca.yt(t, u2); let nx = -(y2 - y), ny = u2 - u; if (!up) { nx = -nx; ny = -ny; } const nl = Math.hypot(nx, ny) || 1, a = Math.abs(s2); const thk = kc * ((1 - horn) * Math.exp(-((a / 0.55) ** 2)) + horn * (0.35 * Math.exp(-((a / 0.4) ** 2)) + 1.1 * Math.exp(-(((a - 0.55) / 0.18) ** 2)))); const [X2, Y2] = tf(u + nx / nl * thk, y + ny / nl * thk); i ? x.lineTo(X2, Y2) : x.moveTo(X2, Y2); }
      for (let i = 80; i >= 0; i--) { const s2 = -1 + i / 40, u = 0.09 * s2 * s2, y = SL.naca.yc(m, p, u) + (s2 >= 0 ? 1 : -1) * SL.naca.yt(t, u); x.lineTo(...tf(u, y)); }
      x.closePath(); x.fill();
    }
  }
  const pts = SL.naca.points(m, p, t, 0, 120); x.beginPath(); pts.forEach(([u, v], i) => { const [X2, Y2] = tf(u, v); i ? x.lineTo(X2, Y2) : x.moveTo(X2, Y2); }); x.closePath(); x.fillStyle = '#545b64'; x.fill(); x.strokeStyle = '#c9cfd6'; x.stroke();
  x.restore();
  x.font = `${10 * dpr}px ${SL.css('--mono')}`; x.fillStyle = SL.css('--muted'); x.textAlign = 'left';
  x.fillText(`leading edge, 1:1 · ${(ac.c * 2 * r / scL * 100).toFixed(0)} cm`, ix - r, iy - r - 5 * dpr);
  // HUD
  const hud = $('vp-hud'); hud.hidden = false;
  hud.innerHTML = `ALT <b>${int(st.h)} m</b>  FL${String(Math.round(st.h * 3.28084 / 100)).padStart(3, '0')}<br>TAS <b>${Math.round(st.V * 1.944)} kt</b>  M ${fmt(O.M, 2)}<br>V/S <b>${O.vs >= 0 ? '+' : '−'}${int(Math.abs(O.vs * 196.85))}</b> ft/min<br>α <b>${fmt(st.alpha / D2R, 1)}°</b>  stall ${fmt(O.A.alphaS / D2R, 1)}°<br>OAT <b>${fmt(O.atm.Tc, 0)} °C</b>${st.mode === 'recover' ? '<br><span class="crit">STALL</span>' : st.V / O.Vs < 1.2 ? '<br><span class="warn">STALL WARNING</span>' : ''}`;
}

function results() {
  const A = O.A, margin = st.V / O.Vs;
  const state = st.mode === 'recover' ? ['Stalled, recovering', 'crit'] : margin < 1.2 ? ['Stall warning', 'warn'] : (O.T >= O.Tm * 0.995 && st.V < AP.vt - 3) ? ['Full power, slowing', 'warn'] : Math.abs(O.vs) > 1.5 ? [O.vs > 0 ? 'Climbing' : 'Descending', 'neutral'] : ['Level flight', 'ok'];
  const atT = SL.isa(AP.ht), lfT = liquid(atT.Tc), kT = SL.iceKind(atT.Tc, AP.lwc);
  const risk = atT.Tc >= 0 ? 'none: above freezing' : lfT <= 0 ? 'none: cloud is ice crystals' : `${kT.name.toLowerCase()} likely (${Math.round(lfT * 100)}% liquid)`;
  $('results').innerHTML = `
    <div class="badges">${SL.badge(state[0], state[1])}${AP.cloud && liquid(O.atm.Tc) > 0 ? SL.badge(st.kind[0].toUpperCase() + st.kind.slice(1) + ' ice building', 'warn') : ''}</div>
    <h4>Flight state</h4>${SL.kv([['Altitude', `${int(st.h)} m`, 'big'], ['True airspeed', `${fmt(st.V, 1)} m/s · ${Math.round(st.V * 1.944)} kt`], ['Equivalent airspeed', `${Math.round(st.V * Math.sqrt(O.atm.sigma) * 1.944)} kt`], ['Vertical speed', `${fmt(O.vs, 2)} m/s`], ['Mach', fmt(O.M, 3)], ['Air density', `${fmt(O.atm.rho, 4)} kg/m³`], ['Reynolds (c̄)', `${fmt(O.Re / 1e6, 2)} × 10⁶`], ['Flight time', clock(st.t)]])}
    <h4>Aerodynamics</h4>${SL.kv([['α', `${fmt(st.alpha / D2R, 2)}°`], ['C<sub>L</sub>', fmt(O.CL, 3)], ['C<sub>D</sub>', fmt(O.CD, 4)], ['L/D', fmt(O.L / Math.max(1, O.D), 1)], ['Thrust used', `${Math.round(O.T / Math.max(1, O.Tm) * 100)}%`], ['Stall speed', `${Math.round(O.Vs * 1.944)} kt${O.Vs > O.Vs0 * 1.005 ? ` (+${Math.round((O.Vs / O.Vs0 - 1) * 100)}% ice)` : ''}`], ['Margin above stall', `${Math.round((margin - 1) * 100)}%`, margin < 1.2 ? 'crit' : margin < 1.3 ? 'warn' : '']])}
    <h4>Ice</h4>${SL.kv([['Thickness', st.k > 0.0005 ? `${Math.round(st.k * 1000)} mm · ${(st.k / ac.c * 100).toFixed(2)}% c̄` : 'none'], ['Type', st.k > 0.0005 ? st.kind : '—'], ['Max lift lost', `${Math.round(A.dCLmax * 100)}%`], ['Extra parasite drag', `+${fmt(A.dCD0, 4)}`], ['At target altitude', risk]])}
    <h4>Lift curve</h4><canvas class="plot" id="f-p1"></canvas><div class="leg"><span style="color:var(--muted)"><i class="dash"></i>clean</span><span><i style="background:var(--accent)"></i>now</span></div>
    <h4>Drag and thrust vs airspeed</h4><canvas class="plot" id="f-p2"></canvas><div class="leg"><span style="color:var(--muted)"><i class="dash"></i>drag, clean</span><span><i style="background:var(--accent)"></i>drag now</span><span><i style="background:var(--warn)"></i>max thrust</span></div>
    <h4>Event log</h4><ul class="log">${log.map(l => `<li class="${l.cls}"><time>${clock(l.t)}</time><span>${l.msg}</span></li>`).join('')}</ul>
    <h4>Model</h4><p class="ref">Aircraft from the project's vortex-lattice solve: C<sub>Lα</sub>, e, C<sub>L,max</sub>, C<sub>D0</sub>. ISA atmosphere; point-mass equations with an altitude and speed autopilot; Lock's law for wave drag. Ice grows at β·LWC·V/ρ<sub>ice</sub> (Langmuir–Blodgett β); lift and drag penalties scale with √(k/c), following icing-tunnel trends.</p>`;
  const mut = SL.css('--muted'), acc = SL.css('--accent'), txt = SL.css('--text'), crit = SL.css('--crit'), warn = SL.css('--warn');
  { const ymax = Math.ceil(O.A0.CLmax * 1.15 * 5) / 5, pl = SL.plot($('f-p1'), { x0: -4, x1: 24, y0: -0.4, y1: ymax, dy: SL.niceStep(ymax + 0.4, 4), dx: 4, xLabel: 'α (deg)', xFmt: v => v.toFixed(0) });
    if (pl) { const cur = (AA) => { const a = []; for (let d = -4; d <= 24; d += 0.25) a.push([d, CLof(d * D2R, AA)]); return a; }; pl.line(cur(O.A0), mut, 1.2, [4, 3]); pl.line(cur(A), acc, 1.8); pl.vline(A.alphaS / D2R, crit); pl.dot(clamp(st.alpha / D2R, -4, 24), O.CL, st.mode === 'recover' ? crit : txt, 4); } }
  { const atm = O.atm, W = O.W, v0 = Math.max(10, O.Vs0 * 0.8), v1 = Math.max(ac.vmax, st.V * 1.15);
    const drag = (V, AA) => { const q = 0.5 * atm.rho * V * V, CL = W / (q * ac.S); if (CL > AA.CLmax) return null; return q * ac.S * CDof(CL, ac.alpha0 + CL / AA.aw, V / atm.a, AA) / 1000; };
    const s0 = [], s1 = [], tt = []; let ym = 0;
    for (let i = 0; i <= 80; i++) { const V = v0 + (v1 - v0) * i / 80, a = drag(V, O.A0), b = drag(V, A), t = Tmax(atm.rho, V, V / atm.a) / 1000; s0.push([V, a]); s1.push([V, b]); tt.push([V, t]); ym = Math.max(ym, a || 0, b || 0, t); }
    ym = Math.min(ym, W * 0.25 / 1000) * 1.1;
    const pl = SL.plot($('f-p2'), { x0: v0, x1: v1, y0: 0, y1: ym, dy: SL.niceStep(ym, 4), dx: SL.niceStep(v1 - v0, 5), xLabel: 'TAS (m/s)', yFmt: v => +v.toFixed(2) + '', padL: 42 });
    if (pl) { pl.line(s0, mut, 1.2, [4, 3]); pl.line(s1, acc, 1.8); if (ac.prop || ac.T0) pl.line(tt, warn, 1.4); pl.vline(O.Vs, crit); pl.dot(clamp(st.V, v0, v1), O.D / 1000, txt, 4); } }
}

function loop(now) {
  if (!active) return; requestAnimationFrame(loop);
  const dt = Math.min(0.1, (now - lastT) / 1000 || 0); lastT = now;
  if (acVersion !== SL.version) buildAircraft(false);
  if (AP.running) { const sim = dt * AP.ts, n = Math.max(1, Math.ceil(sim / 0.02)); for (let i = 0; i < n; i++) step(sim / n); }
  outputs(); draw();
  if (++frames % 6 === 0) { results(); SL.status(`Flight · ${SL.P.name} · ×${AP.ts}${AP.running ? '' : ' · paused'}`, `t = ${clock(st.t)}`); }
}
SL.register('flight', {
  title: 'Flight and icing',
  inspector: () => [
    { section: 'Autopilot', fields: [
      { type: 'num', label: 'Hold altitude', get: () => AP.ht, set: v => AP.ht = v, unit: 'm', min: 0, max: 13000, step: 100, dp: 0 },
      { type: 'num', label: 'Hold airspeed', get: () => AP.vt, set: v => AP.vt = v, unit: 'm/s', min: ac ? ac.vmin : 20, max: ac ? ac.vmax : 260, step: 1, dp: 0 },
    ] },
    { section: 'Weather', fields: [
      { type: 'toggle', label: 'In cloud', get: () => AP.cloud, set: v => { AP.cloud = v; addLog(v ? `Entered cloud at ${int(st.h)} m, ${fmt(SL.isa(st.h).Tc, 0)} °C.` : 'Left cloud.', v ? 'warn' : 'ok'); } },
      { type: 'num', label: 'Liquid water', get: () => AP.lwc, set: v => AP.lwc = v, unit: 'g/m³', min: 0.1, max: 1.5, step: 0.05, dp: 2 },
      { type: 'num', label: 'Droplet MVD', get: () => AP.mvd, set: v => AP.mvd = v, unit: 'µm', min: 10, max: 200, step: 1, dp: 0 },
      { type: 'readout', label: 'OAT at target', get: () => `${fmt(SL.isa(AP.ht).Tc, 0)} °C` },
    ] },
    { section: 'Ice', fields: [
      { type: 'button', buttons: [
        { label: SL.P.engine.type === 'jet' ? 'Heat leading edges' : 'Inflate boots', onclick: () => { const b = st.k * 1000; if (b < 0.5) { addLog('De-ice cycled: no ice to remove.', 'ok'); return; } st.k *= AP.mvd > 50 ? 0.55 : 0.12; fromAirfoil = false; addLog(`De-iced: ${Math.round(b)} → ${Math.round(st.k * 1000)} mm.`, 'ok'); } },
        { label: 'Load Airfoil ice', onclick: () => { if (!SL.ice.kc || !SL.ice.points) { SL.toast('No ice yet. Grow some in the Airfoil module first.'); return; } st.k = SL.ice.kc * ac.c; st.kind = SL.ice.kind === 'none' ? 'rime' : SL.ice.kind; fromAirfoil = true; addLog(`Loaded ice from Airfoil: ${(SL.ice.kc * 100).toFixed(1)}% c, ${Math.round(st.k * 1000)} mm.`, 'warn'); } },
        { label: 'Clear', onclick: () => { st.k = 0; addLog('Ice cleared.', 'ok'); } },
      ] },
    ] },
    { section: 'Simulation', fields: [
      { type: 'seg', label: 'Time scale', get: () => String(AP.ts), set: v => AP.ts = +v, options: [['1', '×1'], ['10', '×10'], ['30', '×30'], ['60', '×60']] },
      { type: 'button', buttons: [{ label: 'Pause / run', onclick: () => AP.running = !AP.running }, { label: 'Restart', onclick: () => buildAircraft(true) }] },
    ] },
    { section: 'Scenarios', open: false, fields: [
      { type: 'note', text: 'These set the weather and autopilot for the current project.' },
      { type: 'button', buttons: [
        { label: 'Hold in icing cloud', onclick: () => { AP.cloud = true; AP.lwc = 0.55; AP.mvd = 25; AP.ts = 60; AP.ht = 3500; AP.vt = Math.round(ac.vmin * 1.45); addLog('Scenario: holding at 3,500 m in supercooled cloud.', 'warn'); SL.refreshInspector(); } },
        { label: 'Freezing drizzle', onclick: () => { AP.cloud = true; AP.lwc = 0.3; AP.mvd = 150; AP.ts = 60; AP.ht = 2500; addLog('Scenario: freezing drizzle at 2,500 m.', 'warn'); SL.refreshInspector(); } },
        { label: 'Cold cruise', onclick: () => { AP.cloud = true; AP.ht = 11000; AP.ts = 30; addLog('Scenario: in cloud at 11,000 m (−56 °C).', 'warn'); SL.refreshInspector(); } },
      ] },
    ] },
  ],
  toolbar(bar) { bar.innerHTML = '<span class="lab">Side view · lift and weight to scale, thrust and drag ×5</span>'; },
  activate() { active = true; if (!ac || acVersion !== SL.version) buildAircraft(!st); SL.remountInspector(); lastT = performance.now(); requestAnimationFrame(loop); },
  deactivate() { active = false; },
  onProject(d) { if (d.full) { buildAircraft(true); SL.remountInspector(); } },
});
})();
