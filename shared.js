// Shared helpers for Slipstream Lab
(() => {
const SL = window.SL = {};
const $ = SL.$ = id => document.getElementById(id);
SL.clamp = (v, a, b) => Math.min(b, Math.max(a, v));
SL.fmt = (v, d = 2) => {
  if (!isFinite(v)) return '—';
  const s = Math.abs(v).toFixed(d);
  return (v < 0 && +s !== 0 ? '−' : '') + s;
};
SL.int = v => isFinite(v) ? Math.round(v).toLocaleString('en-US') : '—';
SL.D2R = Math.PI / 180;

// ---------- colour maps ----------
SL.STOPS = {
  speed: ['#07111f', '#123a6b', '#1f6fa3', '#33a7b5', '#8fd3b8', '#f1e7a0', '#fff7e6'],
  pressure: ['#162f78', '#2f62b8', '#7fa9da', '#e9edf0', '#eeab8b', '#c8502f', '#7a1612'],
  vorticity: ['#0d4f57', '#2c8f8a', '#9fd6cc', '#eef0ee', '#e2a3c4', '#b04682', '#5c0f45'],
};
SL.hexRGB = h => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];
SL.lut = stops => {
  const out = new Uint8Array(256 * 3), rgb = stops.map(SL.hexRGB);
  for (let i = 0; i < 256; i++) {
    const t = i / 255 * (rgb.length - 1), j = Math.min(rgb.length - 2, Math.floor(t)), u = t - j;
    for (let c = 0; c < 3; c++) out[i * 3 + c] = Math.round(rgb[j][c] * (1 - u) + rgb[j + 1][c] * u);
  }
  return out;
};
SL.MAPS = { speed: SL.lut(SL.STOPS.speed), pressure: SL.lut(SL.STOPS.pressure), vorticity: SL.lut(SL.STOPS.vorticity) };
SL.drawBar = (canvas, map) => {
  const x = canvas.getContext('2d'), w = canvas.width, h = canvas.height;
  for (let i = 0; i < w; i++) { const j = Math.round(i / (w - 1) * 255) * 3; x.fillStyle = `rgb(${map[j]},${map[j + 1]},${map[j + 2]})`; x.fillRect(i, 0, 1, h); }
};

// ---------- NACA 4-digit geometry (m, p, t as fractions) ----------
SL.naca = {
  yc(m, p, x) { if (m === 0 || p === 0) return 0; return x < p ? m / (p * p) * (2 * p * x - x * x) : m / ((1 - p) * (1 - p)) * (1 - 2 * p + 2 * p * x - x * x); },
  dyc(m, p, x) { if (m === 0 || p === 0) return 0; return x < p ? 2 * m / (p * p) * (p - x) : 2 * m / ((1 - p) * (1 - p)) * (p - x); },
  yt(t, x) { x = Math.max(0, x); return 5 * t * (0.2969 * Math.sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4); },
  points(m, p, t, flapDeg = 0, n = 90) {
    const up = [], lo = [], hinge = 0.75, d = -flapDeg * SL.D2R, hy = this.yc(m, p, hinge);
    const rot = (x, y, xc) => {
      if (xc <= hinge || !flapDeg) return [x, y];
      const dx = x - hinge, dy = y - hy;
      return [hinge + dx * Math.cos(d) - dy * Math.sin(d), hy + dx * Math.sin(d) + dy * Math.cos(d)];
    };
    for (let i = 0; i <= n; i++) {
      const x = 0.5 * (1 - Math.cos(Math.PI * i / n)), yt = this.yt(t, x), th = Math.atan(this.dyc(m, p, x)), c = this.yc(m, p, x);
      up.push(rot(x - yt * Math.sin(th), c + yt * Math.cos(th), x));
      lo.push(rot(x + yt * Math.sin(th), c - yt * Math.cos(th), x));
    }
    return up.concat(lo.reverse().slice(1, -1));
  },
};
// zero-lift angle from thin-airfoil theory (radians)
SL.alpha0 = (m, p, flapRad = 0) => {
  const dz = x => {
    let s = 0;
    if (m > 0 && p > 0) s = x < p ? 2 * m / (p * p) * (p - x) : 2 * m / ((1 - p) * (1 - p)) * (p - x);
    if (x > 0.75) s -= Math.tan(flapRad);
    return s;
  };
  let a = 0; const K = 400;
  for (let i = 0; i < K; i++) { const th = (i + 0.5) / K * Math.PI; a += dz(0.5 * (1 - Math.cos(th))) * (Math.cos(th) - 1); }
  return -a / K;
};

// ---------- International Standard Atmosphere ----------
SL.isa = h => {
  let T, p;
  if (h < 11000) { T = 288.15 - 0.0065 * h; p = 101325 * Math.pow(T / 288.15, 5.25588); }
  else { T = 216.65; p = 22632.1 * Math.exp(-(h - 11000) / 6341.62); }
  const rho = p / (287.053 * T), a = Math.sqrt(1.4 * 287.053 * T), mu = 1.458e-6 * Math.pow(T, 1.5) / (T + 110.4);
  return { T, Tc: T - 273.15, p, rho, a, mu, sigma: rho / 1.225 };
};

// ---------- ice type from temperature and cloud water ----------
SL.iceKind = (Tc, lwc) => {
  if (Tc >= 0) return { key: 'none', n0: 0, name: 'No ice', desc: 'Above freezing: droplets stay liquid and run off.' };
  if (Tc < -40) return { key: 'none', n0: 1, name: 'No ice', desc: 'Below −40 °C clouds are ice crystals. They bounce off instead of sticking.' };
  const n0 = SL.clamp(-Tc * 0.08 / Math.sqrt(Math.max(0.05, lwc) / 0.5), 0.05, 1);
  if (n0 >= 0.9) return { key: 'rime', n0, name: 'Rime ice', desc: 'Droplets freeze the instant they hit. Milky, rough and brittle, it builds forward into the wind.' };
  if (n0 >= 0.4) return { key: 'mixed', n0, name: 'Mixed ice', desc: 'Most of each droplet freezes on impact. Some water runs back first.' };
  return { key: 'glaze', n0, name: 'Glaze ice', desc: 'Only part of each droplet freezes on impact. The rest runs back and freezes later, so it can build horns.' };
};

// ---------- shared state between views ----------
SL.section = { m: 2, p: 4, t: 12 };        // NACA digits: camber %, position tenths, thickness %
SL.ice = { points: null, kc: 0, kmm: 0, kind: 'none', cell: 0.012, version: 0, shape: 'airfoil' };
SL.bus = new EventTarget();
SL.emit = (n, d) => SL.bus.dispatchEvent(new CustomEvent(n, { detail: d }));
SL.on = (n, fn) => SL.bus.addEventListener(n, e => fn(e.detail));
SL.views = {};

// ---------- UI helpers ----------
SL.syncSeg = (id, v) => document.querySelectorAll(`#${id} button`).forEach(b => b.setAttribute('aria-pressed', String(b.dataset.v === String(v))));
SL.seg = (id, fn) => $(id).addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b || b.disabled) return;
  SL.syncSeg(id, b.dataset.v); fn(b.dataset.v);
});
SL.css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
SL.toast = (el, msg, ms = 4500) => {
  el.textContent = msg; el.hidden = false;
  clearTimeout(el._t); el._t = setTimeout(() => el.hidden = true, ms);
};
SL.mean = (a, n) => { const s = a.slice(-n); return s.length ? s.reduce((p, q) => p + q, 0) / s.length : 0; };
SL.std = (a, n) => { const s = a.slice(-n), m = SL.mean(s, n); return s.length ? Math.sqrt(s.reduce((p, q) => p + (q - m) ** 2, 0) / s.length) : 0; };

// small 2D chart helper: draws axes, returns mapping functions
SL.chart = (canvas, o) => {
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return null;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  const c = canvas.getContext('2d');
  c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, w, h);
  const ink = SL.css('--muted'), line = SL.css('--line'), mono = SL.css('--f-mono');
  const pl = o.padL ?? 40, pr = 10, pt = 8, pb = o.xLabel ? 30 : 8;
  const iw = w - pl - pr, ih = h - pt - pb;
  const X = v => pl + iw * (v - o.x0) / (o.x1 - o.x0), Y = v => pt + ih * (1 - (v - o.y0) / (o.y1 - o.y0));
  c.font = `11px ${mono}`; c.fillStyle = ink; c.lineWidth = 1;
  c.textAlign = 'right'; c.textBaseline = 'middle';
  for (let v = Math.ceil(o.y0 / o.dy) * o.dy; v <= o.y1 + 1e-9; v += o.dy) {
    const y = Y(v); c.strokeStyle = line; c.lineWidth = Math.abs(v) < 1e-9 ? 1.5 : 1;
    c.beginPath(); c.moveTo(pl, y); c.lineTo(w - pr, y); c.stroke();
    c.fillText(o.yFmt ? o.yFmt(v) : SL.fmt(v, o.dy < 0.5 ? 2 : o.dy < 1 ? 1 : 0), pl - 6, y);
  }
  if (o.xLabel) {
    c.textAlign = 'center'; c.textBaseline = 'top';
    for (let v = Math.ceil(o.x0 / o.dx) * o.dx; v <= o.x1 + 1e-9; v += o.dx) c.fillText(o.xFmt ? o.xFmt(v) : String(Math.round(v)), X(v), pt + ih + 5);
    c.fillText(o.xLabel, pl + iw / 2, pt + ih + 17);
  }
  return { c, X, Y, w, h, pl, pr, pt, pb, iw, ih };
};
})();
