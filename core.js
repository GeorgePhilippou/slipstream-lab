// Slipstream: core utilities, project model, module framework and inspector builder.
(() => {
const SL = window.SL = {};
// surface any failure instead of failing silently
const showErr = msg => { let e = document.getElementById('err'); if (!e) { e = document.createElement('div'); e.id = 'err'; e.setAttribute('role', 'alert'); document.body.appendChild(e); } e.textContent = 'Error: ' + msg + ' (try reloading the page with Ctrl/Cmd+Shift+R)'; const s = document.getElementById('st-left'); if (s) s.textContent = 'Error: ' + msg; };
// only report failures from this site's own files: browser extensions inject scripts into every page,
// and their errors reach us as a bare "Script error." with no file (or a file from another origin)
const ours = f => !!f && (f.startsWith(location.origin) || f.startsWith('blob:' + location.origin));
addEventListener('error', e => {
  if (e.target && e.target !== window && (e.target.src || e.target.href)) { const u = e.target.src || e.target.href; if (ours(u)) showErr('could not load ' + u); return; }
  if (!ours(e.filename) || /^Script error\.?$/.test(e.message || '')) { console.warn('Ignored error from outside Slipstream:', e.message, e.filename); return; }
  showErr(e.message || 'unknown error');
}, true);
addEventListener('unhandledrejection', e => showErr((e.reason && e.reason.message) || String(e.reason)));
const $ = SL.$ = id => document.getElementById(id);
const clamp = SL.clamp = (v, a, b) => Math.min(b, Math.max(a, v));
SL.D2R = Math.PI / 180;
SL.fmt = (v, d = 2) => { if (!isFinite(v)) return '—'; const s = Math.abs(v).toFixed(d); return (v < 0 && +s !== 0 ? '−' : '') + s; };
SL.int = v => isFinite(v) ? Math.round(v).toLocaleString('en-US') : '—';
SL.css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
SL.el = (tag, attrs = {}, html = '') => { const e = document.createElement(tag); for (const [k, v] of Object.entries(attrs)) { if (k === 'class') e.className = v; else if (k.startsWith('on')) e.addEventListener(k.slice(2), v); else e.setAttribute(k, v); } if (html) e.innerHTML = html; return e; };
SL.mean = (a, n) => { const s = a.slice(-n); return s.length ? s.reduce((p, q) => p + q, 0) / s.length : 0; };
SL.std = (a, n) => { const s = a.slice(-n), m = SL.mean(s, n); return s.length ? Math.sqrt(s.reduce((p, q) => p + (q - m) ** 2, 0) / s.length) : 0; };

// ---------- colour maps (perceptually uniform) ----------
SL.STOPS = {
  viridis: ['#440154', '#443983', '#31688e', '#21918c', '#35b779', '#90d743', '#fde725'],
  coolwarm: ['#3b4cc0', '#6f92f3', '#aac7fd', '#dddddd', '#f7b89c', '#e7745b', '#b40426'],
  grey: ['#0e1013', '#f2f3f5'],
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
SL.MAPS = { viridis: SL.lut(SL.STOPS.viridis), coolwarm: SL.lut(SL.STOPS.coolwarm), grey: SL.lut(SL.STOPS.grey) };
SL.mapRGB = (name, t) => { const m = SL.MAPS[name], i = Math.round(clamp(t, 0, 1) * 255) * 3; return [m[i], m[i + 1], m[i + 2]]; };

// ---------- NACA 4-digit (m, p, t as fractions) ----------
SL.naca = {
  yc(m, p, x) { if (m === 0 || p === 0) return 0; return x < p ? m / (p * p) * (2 * p * x - x * x) : m / ((1 - p) * (1 - p)) * (1 - 2 * p + 2 * p * x - x * x); },
  dyc(m, p, x) { if (m === 0 || p === 0) return 0; return x < p ? 2 * m / (p * p) * (p - x) : 2 * m / ((1 - p) * (1 - p)) * (p - x); },
  yt(t, x) { x = Math.max(0, x); return 5 * t * (0.2969 * Math.sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4); },
  points(m, p, t, flapDeg = 0, n = 90) {
    const up = [], lo = [], hinge = 0.75, d = -flapDeg * SL.D2R, hy = this.yc(m, p, hinge);
    const rot = (x, y, xc) => { if (xc <= hinge || !flapDeg) return [x, y]; const dx = x - hinge, dy = y - hy; return [hinge + dx * Math.cos(d) - dy * Math.sin(d), hy + dx * Math.sin(d) + dy * Math.cos(d)]; };
    for (let i = 0; i <= n; i++) {
      const x = 0.5 * (1 - Math.cos(Math.PI * i / n)), yt = this.yt(t, x), th = Math.atan(this.dyc(m, p, x)), c = this.yc(m, p, x);
      up.push(rot(x - yt * Math.sin(th), c + yt * Math.cos(th), x)); lo.push(rot(x + yt * Math.sin(th), c - yt * Math.cos(th), x));
    }
    return up.concat(lo.reverse().slice(1, -1));
  },
};
SL.alpha0 = (m, p, flapRad = 0) => {
  const dz = x => { let s = 0; if (m > 0 && p > 0) s = x < p ? 2 * m / (p * p) * (p - x) : 2 * m / ((1 - p) * (1 - p)) * (p - x); if (x > 0.75) s -= Math.tan(flapRad); return s; };
  let a = 0; const K = 400; for (let i = 0; i < K; i++) { const th = (i + 0.5) / K * Math.PI; a += dz(0.5 * (1 - Math.cos(th))) * (Math.cos(th) - 1); }
  return -a / K;
};
SL.isa = h => {
  let T, p;
  if (h < 11000) { T = 288.15 - 0.0065 * h; p = 101325 * Math.pow(T / 288.15, 5.25588); } else { T = 216.65; p = 22632.1 * Math.exp(-(h - 11000) / 6341.62); }
  const rho = p / (287.053 * T);
  return { T, Tc: T - 273.15, p, rho, a: Math.sqrt(1.4 * 287.053 * T), mu: 1.458e-6 * Math.pow(T, 1.5) / (T + 110.4), sigma: rho / 1.225 };
};
SL.iceKind = (Tc, lwc) => {
  if (Tc >= 0) return { key: 'none', n0: 0, name: 'No ice', desc: 'Above freezing. Droplets stay liquid and run off.' };
  if (Tc < -40) return { key: 'none', n0: 1, name: 'No ice', desc: 'Below −40 °C clouds are ice crystals, which bounce off.' };
  const n0 = clamp(-Tc * 0.08 / Math.sqrt(Math.max(0.05, lwc) / 0.5), 0.05, 1);
  if (n0 >= 0.9) return { key: 'rime', n0, name: 'Rime', desc: 'Freezes on impact. Rough, opaque, grows into the wind.' };
  if (n0 >= 0.4) return { key: 'mixed', n0, name: 'Mixed', desc: 'Mostly freezes on impact; some water runs back first.' };
  return { key: 'glaze', n0, name: 'Glaze', desc: 'Partly freezes on impact; the rest runs back and can form horns.' };
};

// ---------- events ----------
SL.bus = new EventTarget();
SL.emit = (n, d) => SL.bus.dispatchEvent(new CustomEvent(n, { detail: d }));
SL.on = (n, fn) => SL.bus.addEventListener(n, e => fn(e.detail));

// ---------- project (one aircraft shared by all modules) ----------
const BASE = {
  name: 'Light trainer',
  wing: { b: 11, cr: 1.65, taper: 0.65, sweep: 0, dihedral: 5, twist: -2, inc: 2, sec: '2412', flapSpan: 0.6, flap: 0, wlH: 0, wlCant: 75 },
  tail: { on: true, bt: 3.6, ct: 0.85, lt: 4.6, zt: 0.3, it: -2, tsweep: 5 },
  body: { lf: 7.8, df: 1.2, xnose: -1.9 },
  mass: 1100, cg: 25,
  engine: { type: 'prop', power: 134, thrust: 20 },
  flight: { h: 2500, V: 60 },
};
const deep = o => JSON.parse(JSON.stringify(o));
const merge = (a, b) => { const o = deep(a); for (const k in b) o[k] = (b[k] && typeof b[k] === 'object' && !Array.isArray(b[k])) ? { ...o[k], ...b[k] } : b[k]; return o; };
SL.PROJECTS = {
  trainer: BASE,
  glider: merge(BASE, { name: 'Sailplane', wing: { b: 18, cr: 0.95, taper: 0.45, dihedral: 3, twist: -2.5, sec: '4415' }, tail: { bt: 2.6, ct: 0.6, lt: 4.6, zt: 1.0, it: -1.5 }, body: { lf: 7, df: 0.7, xnose: -1.6 }, mass: 420, engine: { type: 'none' }, flight: { h: 1500, V: 28 } }),
  turboprop: merge(BASE, { name: 'Regional turboprop', wing: { b: 27, cr: 2.6, taper: 0.55, sweep: 3, dihedral: 3, twist: -2.5, sec: '4415' }, tail: { bt: 7.3, ct: 1.9, lt: 12.5, zt: 2.2, it: -2, tsweep: 10 }, body: { lf: 27, df: 2.8, xnose: -10.5 }, mass: 21500, cg: 22, engine: { type: 'prop', power: 4600 }, flight: { h: 5500, V: 140 } }),
  airliner: merge(BASE, { name: 'Airliner on approach', wing: { b: 34, cr: 7.0, taper: 0.24, sweep: 25, dihedral: 6, twist: -3, flapSpan: 0.6, flap: 15 }, tail: { bt: 12.5, ct: 3.6, lt: 17.5, zt: 0.8, it: -2, tsweep: 30 }, body: { lf: 38, df: 4, xnose: -13 }, mass: 64000, engine: { type: 'jet', thrust: 240 }, flight: { h: 300, V: 75 } }),
  swept: merge(BASE, { name: 'Swept wing, no twist', wing: { sweep: 35, taper: 0.3, twist: 0, dihedral: 0, cr: 2.3 } }),
  forward: merge(BASE, { name: 'Forward-swept wing', wing: { sweep: -25, taper: 0.5, twist: 0, dihedral: 0, cr: 2.0 } }),
};
SL.AIRFOILS = { '2213': [2, 2, 13], '0009': [0, 0, 9], '0012': [0, 0, 12], '2412': [2, 4, 12], '2415': [2, 4, 15], '4412': [4, 4, 12], '4415': [4, 4, 15], '6409': [6, 4, 9] };
SL.P = deep(BASE);
SL.version = 0;
SL.loadProject = key => { SL.P = deep(SL.PROJECTS[key]); SL.version++; SL.emit('project', { full: true }); };
SL.changed = (what = 'geometry') => { SL.version++; SL.emit('project', { what }); };
SL.section = () => { const [m, p, t] = SL.AIRFOILS[SL.P.wing.sec]; return { m, p, t }; };

// ---------- shared ice state (grown in the Airfoil module) ----------
SL.ice = { points: null, kc: 0, kmm: 0, kind: 'none', cell: 0.012, version: 0 };

// ---------- module framework ----------
SL.modules = {};
SL.order = [];
SL.register = (key, mod) => { SL.modules[key] = mod; SL.order.push(key); };
SL.status = (left, right) => { if (left != null) $('st-left').textContent = left; if (right != null) $('st-right').textContent = right; };
SL.toast = (msg, ms = 4500) => { const t = $('vp-toast'); t.textContent = msg; t.hidden = false; clearTimeout(t._t); t._t = setTimeout(() => t.hidden = true, ms); };

// ---------- inspector builder ----------
// spec: [{section:'Wing', open:true, fields:[{key, label, type:'num'|'select'|'toggle'|'seg'|'button'|'note', ...}]}]
SL.inspector = (root, spec) => {
  root.innerHTML = '';
  const refreshers = [];
  for (const sec of spec) {
    const d = SL.el('details', { class: 'sec' }); if (sec.open !== false) d.open = true;
    d.appendChild(SL.el('summary', {}, sec.section));
    const body = SL.el('div', { class: 'sec-body' }); d.appendChild(body);
    for (const f of sec.fields) {
      const id = 'f-' + Math.random().toString(36).slice(2, 9);
      if (f.type === 'note') { const n = SL.el('p', { class: 'note' }, f.text); body.appendChild(n); if (f.refresh) refreshers.push(() => n.innerHTML = f.refresh()); continue; }
      if (f.type === 'button') { const r = SL.el('div', { class: 'row-btns' }); (f.buttons || [f]).forEach(b => r.appendChild(SL.el('button', { class: 'btn' + (b.primary ? ' primary' : ''), type: 'button', onclick: b.onclick, ...(b.id ? { id: b.id } : {}) }, b.label))); body.appendChild(r); continue; }
      const row = SL.el('div', { class: 'field ' + f.type });
      if (f.hidden) refreshers.push(() => row.hidden = f.hidden());
      const lab = SL.el('label', { for: id, title: f.hint || '' }, f.label); row.appendChild(lab);
      if (f.type === 'num') {
        const sl = SL.el('input', { type: 'range', id, min: f.min, max: f.max, step: f.step, 'aria-label': f.label });
        const box = SL.el('input', { type: 'text', inputmode: 'decimal', class: 'val', 'aria-label': f.label + ' value' });
        const unit = SL.el('span', { class: 'unit' }, f.unit || '');
        const show = () => { const v = f.get(); sl.value = v; if (document.activeElement !== box) box.value = (f.fmt ? f.fmt(v) : String(+(+v).toFixed(f.dp ?? 2))); };
        sl.addEventListener('input', () => { f.set(+sl.value); show(); });
        box.addEventListener('change', () => { const raw = parseFloat(box.value.replace('−', '-').replace(/,/g, '')); const v = f.parse ? f.parse(raw) : raw; if (isFinite(v)) f.set(clamp(v, f.hardMin ?? -1e9, f.hardMax ?? 1e9)); show(); });
        box.addEventListener('keydown', e => { if (e.key === 'Enter') box.blur(); });
        row.append(sl, box, unit); refreshers.push(show);
      } else if (f.type === 'select') {
        const s = SL.el('select', { id }); for (const [v, t] of f.options) s.appendChild(SL.el('option', { value: v }, t));
        s.addEventListener('change', () => f.set(s.value)); row.appendChild(s); refreshers.push(() => { s.value = f.get(); });
      } else if (f.type === 'toggle') {
        const c = SL.el('input', { type: 'checkbox', id, class: 'tog' }); c.addEventListener('change', () => f.set(c.checked)); row.appendChild(c); refreshers.push(() => c.checked = !!f.get());
      } else if (f.type === 'seg') {
        const g = SL.el('div', { class: 'segm', role: 'group', 'aria-label': f.label });
        for (const [v, t] of f.options) g.appendChild(SL.el('button', { type: 'button', 'data-v': v, onclick: () => { f.set(v); refresh(); } }, t));
        row.appendChild(g); refreshers.push(() => g.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.v === String(f.get())))));
      } else if (f.type === 'readout') {
        const o = SL.el('output', { class: 'ro' }); row.appendChild(o); refreshers.push(() => o.innerHTML = f.get());
      }
      body.appendChild(row);
    }
    root.appendChild(d);
  }
  const refresh = () => refreshers.forEach(r => r());
  refresh();
  return refresh;
};

// ---------- results panel helpers ----------
SL.kv = (rows) => '<dl class="kv">' + rows.map(r => r ? `<dt>${r[0]}</dt><dd${r[2] ? ` class="${r[2]}"` : ''}>${r[1]}</dd>` : '<div class="kv-gap"></div>').join('') + '</dl>';
SL.badge = (text, cls) => `<span class="badge ${cls}">${text}</span>`;

// ---------- plot helper (canvas 2D, restrained styling) ----------
SL.plot = (canvas, o) => {
  const dpr = Math.min(2, window.devicePixelRatio || 1), w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return null;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  const c = canvas.getContext('2d'); c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, w, h);
  const ink = SL.css('--muted'), grid = SL.css('--grid'), mono = SL.css('--mono');
  const pl = o.padL ?? 38, pr = 8, pt = 8, pb = o.xLabel ? 28 : 8, iw = w - pl - pr, ih = h - pt - pb;
  const X = v => pl + iw * (v - o.x0) / (o.x1 - o.x0), Y = v => pt + ih * (1 - (v - o.y0) / (o.y1 - o.y0));
  c.font = `10.5px ${mono}`; c.fillStyle = ink; c.lineWidth = 1;
  c.textAlign = 'right'; c.textBaseline = 'middle';
  for (let v = Math.ceil(o.y0 / o.dy - 1e-9) * o.dy; v <= o.y1 + 1e-9; v += o.dy) {
    const y = Math.round(Y(v)) + 0.5; c.strokeStyle = Math.abs(v) < 1e-9 ? SL.css('--grid-strong') : grid;
    c.beginPath(); c.moveTo(pl, y); c.lineTo(w - pr, y); c.stroke();
    c.fillText(o.yFmt ? o.yFmt(v) : SL.fmt(v, o.dy < 0.1 ? 3 : o.dy < 1 ? (o.dy < 0.5 ? 2 : 1) : 0), pl - 5, y);
  }
  c.textAlign = 'center'; c.textBaseline = 'top';
  if (o.dx) for (let v = Math.ceil(o.x0 / o.dx - 1e-9) * o.dx; v <= o.x1 + 1e-9; v += o.dx) {
    const x = Math.round(X(v)) + 0.5; c.strokeStyle = grid; c.beginPath(); c.moveTo(x, pt); c.lineTo(x, pt + ih); c.stroke();
    if (o.xLabel) c.fillText(o.xFmt ? o.xFmt(v) : String(+v.toFixed(3)), x, pt + ih + 4);
  }
  if (o.xLabel) { c.fillStyle = ink; c.fillText(o.xLabel, pl + iw / 2, pt + ih + 15); }
  c.strokeStyle = SL.css('--grid-strong'); c.strokeRect(pl + 0.5, pt + 0.5, iw, ih);
  const line = (pts, col, wdt = 1.5, dash) => { c.save(); c.beginPath(); c.rect(pl, pt, iw, ih); c.clip(); c.strokeStyle = col; c.lineWidth = wdt; c.setLineDash(dash || []); c.beginPath(); let pen = false; for (const p of pts) { if (!p || !isFinite(p[1])) { pen = false; continue; } pen ? c.lineTo(X(p[0]), Y(p[1])) : c.moveTo(X(p[0]), Y(p[1])); pen = true; } c.stroke(); c.restore(); };
  const dot = (x, y, col, r = 3.5) => { c.fillStyle = col; c.beginPath(); c.arc(X(x), Y(y), r, 0, 7); c.fill(); };
  const vline = (x, col, dash = [3, 3]) => { c.save(); c.strokeStyle = col; c.setLineDash(dash); c.beginPath(); c.moveTo(Math.round(X(x)) + 0.5, pt); c.lineTo(Math.round(X(x)) + 0.5, pt + ih); c.stroke(); c.restore(); };
  const label = (x, y, text, col, align = 'left') => { c.fillStyle = col; c.font = `10.5px ${mono}`; c.textAlign = align; c.textBaseline = 'bottom'; c.fillText(text, X(x) + (align === 'left' ? 4 : -4), Y(y) - 3); };
  return { c, X, Y, line, dot, vline, label, w, h };
};
SL.niceStep = (span, n = 5) => { const raw = span / n, p = Math.pow(10, Math.floor(Math.log10(raw))), m = raw / p; return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p; };

// ---------- colour bar ----------
SL.colorbar = (map, lo, hi, title) => {
  const cv = $('vp-cbar'), x = cv.getContext('2d'), w = cv.width, h = cv.height;
  const show = !!map; $('vp-legend').hidden = !show; if (!show) return;
  for (let i = 0; i < w; i++) { const [r, g, b] = SL.mapRGB(map, i / (w - 1)); x.fillStyle = `rgb(${r},${g},${b})`; x.fillRect(i, 0, 1, h); }
  $('vp-lt').textContent = title; $('vp-lmin').textContent = lo; $('vp-lmax').textContent = hi;
};
})();
