// Wing and tail module: vortex-lattice analysis of the project aircraft.
(() => {
const { $, fmt, int, D2R, clamp } = SL;
let view = null, active = false, pending = false, mode = 'load', wake = true, A = null;
const kt = v => Math.round(v * 1.944);
function build() {
  const P = SL.P; A = SL.aero();
  const g = A.g, al = A.alReq, items = [];
  let dmax = 1e-9; g.panels.forEach(p => dmax = Math.max(dmax, p.dcp));
  const hideR = P.body.df * 0.45;
  for (const p of g.panels) {
    if (p.corners.every(c => Math.abs(c[1]) < hideR)) continue;
    const s = p.strip, r = s.surf === 'wing' ? (s.cl0 + s.cla * al) / s.clmax : 0.3;
    const col = mode === 'stall' ? SL.mapRGB('coolwarm', clamp(r, 0, 1)) : SL.mapRGB('viridis', clamp(p.dcp / dmax, 0, 1));
    items.push({ pts: p.corners, col, edge: 'rgba(14,16,19,0.35)' });
  }
  const M = SL.geom.mesh(P);
  const body = { pos: M.pos, idx: [], part: [] };
  for (let t = 0; t < M.part.length; t++) if (M.part[t] === 'body' || M.part[t] === 'fin') { body.idx.push(M.idx[t * 3], M.idx[t * 3 + 1], M.idx[t * 3 + 2]); body.part.push(M.part[t]); }
  items.push(...SL.geom.meshItems(body, { body: [120, 127, 136], fin: [112, 119, 128] }));
  view.items = items;
  view.lines = [];
  if (wake) {
    let gmax = 1e-9;
    for (const t of g.tre) for (let k = 0; k <= t.ss.length; k++) gmax = Math.max(gmax, Math.abs((k > 0 ? t.ss[k - 1].G : 0) - (k < t.ss.length ? t.ss[k].G : 0)));
    for (const t of g.tre) for (let k = 0; k <= t.ss.length; k++) {
      const str = Math.abs((k > 0 ? t.ss[k - 1].G : 0) - (k < t.ss.length ? t.ss[k].G : 0)) / gmax; if (str < 0.05) continue;
      const e = k < t.ss.length ? t.ss[k].te[0] : t.ss[k - 1].te[1];
      view.lines.push({ a: e, b: [e[0] + P.wing.b * 0.9, e[1], e[2]], col: `rgba(220,226,232,${(0.1 + 0.5 * str).toFixed(2)})`, w: 0.6 + 1.4 * str });
    }
  }
  view.markers = [{ p: [A.ref.xcg, 0, P.body.df * 0.62], col: '#d6a24a', label: 'CG', dy: -8 }];
  if (P.tail.on && isFinite(A.xnp)) view.markers.push({ p: [A.xnp, 0, P.body.df * 0.62], col: '#5b9cf2', label: 'NP', dy: 8 });
  view.groundZ = -P.body.df * 1.1;
  view.cam.target = [(2 * P.body.xnose + P.body.lf) / 2, 0, 0]; view.cam.size = Math.max(P.wing.b, P.body.lf) * 0.55;
  view.draw();
  if (mode === 'stall') SL.colorbar('coolwarm', '0', '1.0 stall', 'Section c_l ÷ c_l,max');
  else SL.colorbar('viridis', '0', 'max', 'Panel loading ΔC_p');
  SL.status(`Vortex lattice · α = ${fmt(al / D2R, 2)}° for level flight`, `${A.panels} panels · ${A.ms.toFixed(0)} ms`);
  SL.refreshInspector();
}
function plots() {
  const P = SL.P, g = A.g, s = P.wing.b / 2, al = A.alReq, acc = SL.css('--accent'), mut = SL.css('--muted'), crit = SL.css('--crit'), warn = SL.css('--warn'), ok = SL.css('--ok'), txt = SL.css('--text');
  const wing = g.strips.filter(st => st.surf === 'wing' && st.side === 1);
  { // span loading
    const cbar = A.ref.cbar, CL = Math.max(1e-3, A.op.CL), pts = wing.map(st => [st.mid[1] / s, st.c * (st.cl0 + st.cla * al) / (cbar * CL)]);
    const ymax = Math.max(1.6, ...pts.map(p => p[1])) * 1.1, x1 = Math.max(1, ...pts.map(p => p[0]));
    const pl = SL.plot($('w-p1'), { x0: 0, x1, y0: 0, y1: ymax, dy: SL.niceStep(ymax, 4), dx: 0.25, xLabel: '2y/b', xFmt: v => v.toFixed(2) });
    if (pl) { const ell = []; for (let e = 0; e <= 1.0001; e += 0.01) ell.push([e, 4 / Math.PI * Math.sqrt(Math.max(0, 1 - e * e)) * A.ref.S / (P.wing.b * cbar)]); pl.line(ell, mut, 1.2, [4, 3]); pl.line(pts, acc, 1.8); }
  }
  { // section lift vs limit
    const now = wing.map(st => [st.mid[1] / s, st.cl0 + st.cla * al]), atS = wing.map(st => [st.mid[1] / s, st.cl0 + st.cla * A.aStall]), lim = wing.map(st => [st.mid[1] / s, st.clmax]);
    const ymax = Math.max(...lim.map(p => p[1]), ...atS.map(p => p[1])) * 1.12;
    const pl = SL.plot($('w-p2'), { x0: 0, x1: Math.max(1, ...now.map(p => p[0])), y0: 0, y1: ymax, dy: SL.niceStep(ymax, 4), dx: 0.25, xLabel: '2y/b', xFmt: v => v.toFixed(2) });
    if (pl) { pl.line(lim, crit, 1.2, [4, 3]); pl.line(atS, warn, 1.4); pl.line(now, acc, 1.8); if (A.stallStrip) pl.dot(A.stallStrip.mid[1] / s, A.stallStrip.clmax, crit); }
  }
  { // drag polar
    const Pp = A.polar, xmax = Math.max(...Pp.map(p => p.CD)) * 1.08, ymin = Math.min(0, ...Pp.map(p => p.CL)), ymax = Math.max(...Pp.map(p => p.CL)) * 1.1;
    const dx = SL.niceStep(xmax, 4);
    const pl = SL.plot($('w-p3'), { x0: 0, x1: xmax, y0: Math.floor(ymin * 5) / 5, y1: ymax, dy: SL.niceStep(ymax - ymin, 4), dx, xLabel: 'C_D', xFmt: v => v.toFixed(dx < 0.01 ? 3 : 2) });
    if (pl) {
      pl.line(Pp.map(p => [p.CDi, p.CL]), mut, 1.2, [4, 3]); pl.line(Pp.map(p => [p.CD, p.CL]), acc, 1.8);
      const b = A.best; pl.line([[0, 0], [b.CD * 1.5, b.CL * 1.5]], ok, 1); pl.dot(b.CD, b.CL, ok); pl.dot(A.op.CD, A.op.CL, txt, 4);
    }
  }
  { // Cm vs alpha
    const a0 = -4, a1 = Math.max(4, isFinite(A.aStall) ? A.aStall / D2R : 16), cm = a => A.Cm0 + A.Cma * a * D2R;
    const lo = Math.min(-0.2, cm(a0), cm(a1)), hi = Math.max(0.2, cm(a0), cm(a1)), dy = SL.niceStep(hi - lo, 4);
    const pl = SL.plot($('w-p4'), { x0: a0, x1: a1, y0: Math.floor(lo / dy) * dy, y1: Math.ceil(hi / dy) * dy, dy, dx: SL.niceStep(a1 - a0, 5), xLabel: 'α (deg)', xFmt: v => v.toFixed(0) });
    if (pl) { pl.line([[a0, cm(a0)], [a1, cm(a1)]], A.Cma < 0 ? acc : crit, 1.8); const ar = al / D2R; if (ar >= a0 && ar <= a1) pl.dot(ar, cm(ar), txt, 4); }
  }
}
function results() {
  const P = SL.P, r = A.ref, smv = A.SM * 100;
  const fly = A.CLreq > A.CLmax ? ['Below stall speed', 'crit'] : (P.engine.type !== 'none' && A.Dn > A.avail) ? [`Thrust short: ${Math.round(A.Dn / A.avail * 100)}% needed`, 'crit'] : P.flight.V / A.Vs < 1.3 ? [`Near stall: ${fmt(P.flight.V / A.Vs, 2)} V_s`, 'warn'] : ['Level flight OK', 'ok'];
  const stab = !P.tail.on ? ['No tail', 'neutral'] : smv < 0 ? ['Unstable in pitch', 'crit'] : smv < 5 ? ['Marginally stable', 'warn'] : smv > 35 ? ['Very stable, heavy', 'warn'] : ['Stable in pitch', 'ok'];
  const frac = A.stallStrip ? A.stallStrip.mid[1] / (P.wing.b / 2) : 0;
  const stl = frac > 0.6 ? [`Tip stall first (${Math.round(frac * 100)}%)`, 'crit'] : frac > 0.35 ? [`Mid-span stall (${Math.round(frac * 100)}%)`, 'warn'] : [`Root stall first (${Math.round(frac * 100)}%)`, 'ok'];
  const trimTxt = A.trim ? `${A.trim.dit >= 0 ? '+' : '−'}${fmt(Math.abs(A.trim.dit / D2R), 2)}° → ${fmt(P.tail.it + A.trim.dit / D2R, 2)}°` : '—';
  $('results').innerHTML = `
    <div class="badges">${SL.badge(fly[0], fly[1])}${SL.badge(stab[0], stab[1])}${SL.badge(stl[0], stl[1])}</div>
    ${A.M > 0.5 ? `<p class="ref warn">Mach ${fmt(A.M, 2)}: this method is incompressible, so results above about Mach 0.5 are optimistic.</p>` : ''}
    <h4>Coefficients <span>at level flight</span></h4>
    ${SL.kv([['C<sub>L</sub>', fmt(A.op.CL, 3), 'big'], ['C<sub>D</sub> total', fmt(A.op.CD, 4), 'big'], ['L/D', fmt(A.op.CL / A.op.CD, 2), 'big'], null,
      ['C<sub>Di</sub> induced', `${fmt(A.op.CDi, 4)} · ${Math.round(A.op.CDi / A.op.CD * 100)}%`], ['C<sub>Dp</sub> profile + body', `${fmt(A.op.CDp, 4)} · ${Math.round(A.op.CDp / A.op.CD * 100)}%`],
      ['Span efficiency e', fmt(A.e, 3)], ['C<sub>Lα</sub>', `${fmt(A.CLa, 3)} /rad · ${fmt(A.CLa * D2R, 4)} /°`], ['α zero lift', `${fmt(-A.CL0 / A.CLa / D2R, 2)}°`], ['α required', `${fmt(A.alReq / D2R, 2)}°`]])}
    <h4>Stall</h4>${SL.kv([['C<sub>L,max</sub>', fmt(A.CLmax, 3)], ['α stall', `${fmt(A.aStall / D2R, 1)}°`], ['Stall speed (TAS)', `${fmt(A.Vs, 1)} m/s · ${kt(A.Vs)} kt`]])}
    <h4>Stability and trim</h4>${SL.kv([['C<sub>mα</sub>', `${fmt(A.Cma, 3)} /rad`], ['Neutral point', P.tail.on ? `${fmt((A.xnp - r.xmac) / r.cbar * 100, 1)}% c̄` : '—'], ['CG', `${P.cg}% c̄`], ['Static margin', P.tail.on ? `${fmt(smv, 1)}% c̄` : '—', stab[1] === 'neutral' ? '' : stab[1]], ['Tail incidence to trim', trimTxt]])}
    <h4>Performance</h4>${SL.kv([['Best L/D', `${fmt(A.best.CL / A.best.CD, 1)} at ${kt(Math.sqrt(2 * A.W / (A.atm.rho * r.S * Math.max(0.05, A.best.CL))))} kt`], ['Drag', `${fmt(A.Dn / 1000, 3)} kN`], ['Power required', `${fmt(A.Preq / 1000, 1)} kW`], ['Thrust available', P.engine.type === 'none' ? 'glider' : `${fmt(A.avail / 1000, 3)} kN`]])}
    <h4>Span loading</h4><canvas class="plot" id="w-p1"></canvas><div class="leg"><span><i style="background:var(--accent)"></i>c·c<sub>l</sub>/(c̄·C<sub>L</sub>)</span><span style="color:var(--muted)"><i class="dash"></i>elliptic</span></div>
    <h4>Section lift and stall limit</h4><canvas class="plot" id="w-p2"></canvas><div class="leg"><span><i style="background:var(--accent)"></i>now</span><span><i style="background:var(--warn)"></i>at stall</span><span style="color:var(--crit)"><i class="dash"></i>c<sub>l,max</sub></span></div>
    <h4>Drag polar</h4><canvas class="plot" id="w-p3"></canvas><div class="leg"><span><i style="background:var(--accent)"></i>total</span><span style="color:var(--muted)"><i class="dash"></i>induced</span><span><i style="background:var(--ok)"></i>best L/D</span></div>
    <h4>Pitching moment about CG</h4><canvas class="plot" id="w-p4"></canvas><div class="leg"><span>negative slope = stable</span></div>
    <h4>Method</h4><p class="ref">Vortex lattice: horseshoe vortices on the cambered mean surface, Kutta–Joukowski forces, induced drag in the Trefftz plane. Profile drag uses turbulent skin friction with a form factor, plus the fuselage and fin. Stall uses the critical-section method. Validation: a rectangular wing of aspect ratio 6 gives C<sub>Lα</sub> = 4.29/rad and e = 0.96.</p>`;
  plots();
}
function schedule() { if (pending) return; pending = true; requestAnimationFrame(() => { pending = false; if (!active) return; try { build(); results(); } catch (e) { console.error(e); SL.status('Analysis failed: ' + e.message); } }); }
SL.register('wing', {
  title: 'Wing and tail',
  inspector: () => [
    { section: 'Analysis', fields: [
      { type: 'readout', label: 'α for level', get: () => A ? `${fmt(A.alReq / D2R, 2)}°` : '…' },
      { type: 'button', buttons: [{ label: 'Trim with tail incidence', primary: true, onclick: () => { const a = SL.aero(); if (a.trim) { SL.P.tail.it = +(SL.P.tail.it + a.trim.dit / D2R).toFixed(2); SL.changed(); } } }] },
    ] },
    SL.specs.flight(), SL.specs.mass(), SL.specs.wing(), SL.specs.highlift(), SL.specs.tail(),
  ],
  toolbar(bar) {
    bar.innerHTML = '<span class="lab">Colour</span>';
    const g = SL.el('div', { class: 'segm' });
    for (const [k, t] of [['load', 'Loading'], ['stall', 'Stall margin']]) g.appendChild(SL.el('button', { type: 'button', 'aria-pressed': String(k === mode), onclick: e => { mode = k; g.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', String(b === e.target))); schedule(); } }, t));
    bar.appendChild(g);
    const ck = SL.el('label', { class: 'chk' }); const c = SL.el('input', { type: 'checkbox', class: 'tog' }); c.checked = wake; c.onchange = () => { wake = c.checked; schedule(); }; ck.append(c, document.createTextNode('Trailing vortices')); bar.appendChild(ck);
    bar.appendChild(SL.el('span', { class: 'grow' }));
    const v = SL.el('div', { class: 'segm' });
    for (const [k, t] of [['iso', 'Iso'], ['top', 'Top'], ['front', 'Front'], ['side', 'Side']]) v.appendChild(SL.el('button', { type: 'button', 'aria-pressed': String(k === 'iso'), onclick: e => { v.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', String(b === e.target))); view.preset(k); } }, t));
    bar.appendChild(v);
  },
  activate() {
    active = true;
    if (!view) { view = new SL.geom.View3D($('cv-wing')); new ResizeObserver(() => { if (active) { view.draw(); if (A) plots(); } }).observe($('vp')); }
    schedule();
  },
  deactivate() { active = false; },
  onProject: schedule,
});
})();
