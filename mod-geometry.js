// Geometry module: edit the shared aircraft and inspect it.
(() => {
const { $, fmt, int } = SL;
const big = () => SL.P.wing.b > 20 || SL.P.body.lf > 20;
const num = (label, get, set, unit, min, max, step, dp = 2, hint) => ({ type: 'num', label, get, set: v => { set(v); SL.changed(); }, unit, min, max, step, dp, hint });
SL.specs = {
  wing: () => { const P = SL.P, w = P.wing; return { section: 'Wing', fields: [
    num('Span', () => w.b, v => w.b = v, 'm', 3, big() ? 70 : 25, 0.1, 1),
    num('Root chord', () => w.cr, v => w.cr = v, 'm', 0.3, big() ? 12 : 4, 0.05),
    num('Taper ratio', () => w.taper, v => w.taper = v, '', 0.1, 1, 0.01, 2, 'Tip chord ÷ root chord'),
    num('Sweep c/4', () => w.sweep, v => w.sweep = v, '°', -35, 50, 1, 0, 'Quarter-chord sweep'),
    num('Dihedral', () => w.dihedral, v => w.dihedral = v, '°', -8, 12, 0.5, 1),
    num('Tip twist', () => w.twist, v => w.twist = v, '°', -8, 4, 0.5, 1, 'Negative = washout'),
    num('Incidence', () => w.inc, v => w.inc = v, '°', -4, 8, 0.5, 1),
    { type: 'select', label: 'Airfoil', get: () => w.sec, set: v => { w.sec = v; SL.changed(); }, options: [['0009', 'NACA 0009'], ['0012', 'NACA 0012'], ['2412', 'NACA 2412'], ['2415', 'NACA 2415'], ['4412', 'NACA 4412'], ['4415', 'NACA 4415'], ['6409', 'NACA 6409']] },
  ] }; },
  highlift: () => { const w = SL.P.wing; return { section: 'Flaps and winglets', open: false, fields: [
    num('Flap span', () => w.flapSpan, v => w.flapSpan = v, '', 0, 1, 0.05, 2, 'Fraction of semispan from the root'),
    num('Flap angle', () => w.flap, v => w.flap = v, '°', 0, 40, 1, 0),
    num('Winglet height', () => w.wlH, v => w.wlH = v, '', 0, 0.3, 0.01, 2, 'Fraction of semispan; 0 = none'),
    num('Winglet cant', () => w.wlCant, v => w.wlCant = v, '°', 30, 90, 1, 0, '90° = vertical'),
  ] }; },
  tail: () => { const T = SL.P.tail; return { section: 'Horizontal tail', open: false, fields: [
    { type: 'toggle', label: 'Fitted', get: () => T.on, set: v => { T.on = v; SL.changed(); } },
    num('Span', () => T.bt, v => T.bt = v, 'm', 0.5, big() ? 25 : 8, 0.1, 1),
    num('Root chord', () => T.ct, v => T.ct = v, 'm', 0.2, big() ? 8 : 2.5, 0.05),
    num('Arm', () => T.lt, v => T.lt = v, 'm', 1, big() ? 40 : 12, 0.1, 1, 'Wing root LE to tail root LE'),
    num('Height', () => T.zt, v => T.zt = v, 'm', -1, big() ? 8 : 2.5, 0.05, 2, 'Above the wing plane'),
    num('Incidence', () => T.it, v => T.it = v, '°', -8, 4, 0.25, 2),
    num('Sweep c/4', () => T.tsweep, v => T.tsweep = v, '°', 0, 45, 1, 0),
  ] }; },
  body: () => { const B = SL.P.body; return { section: 'Fuselage', open: false, fields: [
    num('Length', () => B.lf, v => B.lf = v, 'm', 2, 75, 0.5, 1),
    num('Diameter', () => B.df, v => B.df = v, 'm', 0.3, 7, 0.05),
    num('Nose position', () => B.xnose, v => B.xnose = v, 'm', -30, 2, 0.1, 1, 'Relative to the wing root leading edge'),
  ] }; },
  mass: () => { const P = SL.P; return { section: 'Mass and propulsion', fields: [
    num('Mass', () => P.mass, v => P.mass = v, 'kg', big() ? 2000 : 100, big() ? 400000 : 6000, big() ? 500 : 10, 0),
    num('CG position', () => P.cg, v => P.cg = v, '%c̄', -10, 80, 1, 0, 'Centre of gravity, % of mean aerodynamic chord'),
    { type: 'seg', label: 'Propulsion', get: () => P.engine.type, set: v => { P.engine.type = v; SL.changed(); SL.refreshInspector(); }, options: [['prop', 'Prop'], ['jet', 'Jet'], ['none', 'Glider']] },
    { ...num('Power', () => P.engine.power, v => P.engine.power = v, 'kW', 0, 6000, 5, 0), hidden: () => P.engine.type !== 'prop' },
    { ...num('Thrust', () => P.engine.thrust, v => P.engine.thrust = v, 'kN', 0, 600, 1, 0), hidden: () => P.engine.type !== 'jet' },
  ] }; },
  flight: () => { const F = SL.P.flight; return { section: 'Flight condition', fields: [
    num('Altitude', () => F.h, v => F.h = v, 'm', 0, 13000, 100, 0),
    num('Airspeed', () => F.V, v => F.V = v, 'm/s', 15, 260, 1, 0, 'True airspeed'),
  ] }; },
};

const COL = { wing: [186, 193, 201], tail: [176, 183, 191], body: [150, 158, 167], fin: [140, 148, 157] };
let view = null, active = false, pending = false;
function build() {
  const P = SL.P, M = SL.geom.mesh(P);
  view.items = SL.geom.meshItems(M, COL, P.body.df * 0.45);
  const ref = SL.geom.reference(P);
  view.markers = [{ p: [ref.xcg, 0, P.body.df * 0.62], col: '#d6a24a', label: 'CG' }];
  view.lines = [];
  view.groundZ = -P.body.df * 1.1;
  view.cam.target = [(P.body.xnose + P.body.xnose + P.body.lf) / 2, 0, 0];
  view.cam.size = Math.max(P.wing.b, P.body.lf) * 0.55;
  view.draw();
}
function results() {
  const P = SL.P, r = SL.geom.reference(P), atm = SL.isa(P.flight.h);
  const St = P.tail.on ? P.tail.bt * P.tail.ct * 0.8 : 0, lt = P.tail.lt + 0.25 * P.tail.ct - (r.xmac + 0.25 * r.cbar);
  const Vh = St * lt / (r.S * r.cbar), WS = P.mass * 9.80665 / r.S;
  const Re = atm.rho * P.flight.V * r.cbar / atm.mu;
  const pw = P.engine.type === 'prop' ? `${fmt(P.engine.power * 1000 / (P.mass * 9.80665), 1)} W/N` : P.engine.type === 'jet' ? `${fmt(P.engine.thrust * 1000 / (P.mass * 9.80665), 2)}` : '—';
  const vhCls = !P.tail.on ? '' : Vh < 0.35 ? 'warn' : Vh > 1.1 ? 'warn' : 'ok';
  $('results').innerHTML = `
    <h4>Wing</h4>${SL.kv([['Reference area S', `${fmt(r.S, r.S < 50 ? 2 : 1)} m²`], ['Span b', `${fmt(P.wing.b, 2)} m`], ['Aspect ratio', fmt(r.AR, 2), 'big'], ['Mean aerodynamic chord', `${fmt(r.cbar, 3)} m`], ['MAC leading edge x', `${fmt(r.xmac, 3)} m`], ['Tip chord', `${fmt(P.wing.cr * P.wing.taper, 3)} m`]])}
    <h4>Tail and balance</h4>${SL.kv([['Tail area', P.tail.on ? `${fmt(St, 2)} m²` : 'none'], ['Tail arm (c̄/4 to c̄/4)', P.tail.on ? `${fmt(lt, 2)} m` : '—'], ['Tail volume V<sub>H</sub>', P.tail.on ? fmt(Vh, 3) : '—', vhCls], ['CG', `${fmt(r.xcg, 3)} m · ${P.cg}% c̄`]])}
    <p class="ref">Tail volume coefficient V<sub>H</sub> = S<sub>t</sub>·l<sub>t</sub>/(S·c̄). Typical values: 0.5–0.7 for light aircraft, about 1.0 for airliners.</p>
    <h4>Loading</h4>${SL.kv([['Wing loading', `${fmt(WS, 0)} N/m²`], [P.engine.type === 'jet' ? 'Thrust-to-weight' : 'Power loading', pw], ['Fuselage fineness', fmt(P.body.lf / P.body.df, 1)], ['Reynolds number (c̄)', `${fmt(Re / 1e6, 2)} × 10⁶`], ['Mach number', fmt(P.flight.V / atm.a, 3)]])}
    <h4>Next</h4><p class="ref">Open <b>Wing and tail</b> for lift, drag, stall and stability, or <b>Airfoil</b> to see the section in a 2D flow simulation.</p>`;
}
function schedule() { if (pending) return; pending = true; requestAnimationFrame(() => { pending = false; if (active) { build(); results(); } }); }
SL.register('geometry', {
  title: 'Geometry',
  inspector: () => [SL.specs.wing(), SL.specs.highlift(), SL.specs.tail(), SL.specs.body(), SL.specs.mass(), SL.specs.flight()],
  toolbar(bar) {
    bar.innerHTML = '<span class="lab">View</span>';
    const g = SL.el('div', { class: 'segm' });
    for (const [k, t] of [['iso', 'Iso'], ['top', 'Top'], ['front', 'Front'], ['side', 'Side']]) g.appendChild(SL.el('button', { type: 'button', 'data-v': k, 'aria-pressed': k === 'iso' ? 'true' : 'false', onclick: e => { g.querySelectorAll('button').forEach(b => b.setAttribute('aria-pressed', String(b === e.target))); view.preset(k); } }, t));
    bar.appendChild(g);
    bar.appendChild(SL.el('span', { class: 'grow' }));
    bar.appendChild(SL.el('span', { class: 'lab' }, 'Drag to orbit · scroll to zoom'));
  },
  activate() {
    active = true;
    if (!view) { view = new SL.geom.View3D($('cv-geometry')); new ResizeObserver(() => active && view.draw()).observe($('vp')); }
    build(); results();
    SL.status('Geometry editor', `${SL.int(view.items.length)} triangles`);
  },
  deactivate() { active = false; },
  onProject: schedule,
});
})();
