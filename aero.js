// Slipstream: whole-aircraft aerodynamics from the vortex-lattice method plus a drag build-up.
// Cached by project version, so every module that needs it shares one solve.
(() => {
const { D2R, clamp } = SL;
const G0 = 9.80665;
const cfTurb = Re => 0.455 / Math.pow(Math.log10(Math.max(Re, 1e4)), 2.58);
function clmaxSection(tPct, mPct, Re, flap) {
  let c = 1.6 - 0.0194 * Math.max(0, 12 - tPct) ** 2 - 0.006 * Math.max(0, tPct - 15) ** 2 + 0.05 * Math.min(mPct, 4);
  c *= clamp(Math.pow(Re / 6e6, 0.08), 0.75, 1.05);
  if (flap) c += 0.9 * clamp(flap / 40, 0, 1);
  return c;
}
let cache = { v: -1, A: null };
SL.aero = (force = false) => {
  if (!force && cache.v === SL.version && cache.A) return cache.A;
  const P = SL.P, t0 = performance.now();
  const ref = SL.geom.reference(P), atm = SL.isa(P.flight.h);
  const g = VLM.analyse(SL.geom.surfaces(P), ref);
  const gT = P.tail.on ? VLM.analyse(SL.geom.surfaces(P, 1), ref) : null;
  const r0 = VLM.evaluate(g, 0), cl0 = g.strips.map(s => s.cl), r1 = VLM.evaluate(g, D2R);
  const cla = g.strips.map((s, i) => (s.cl - cl0[i]) / D2R);
  const CL0 = r0.CL, CLa = (r1.CL - r0.CL) / D2R, Cm0 = r0.Cm, Cma = (r1.Cm - r0.Cm) / D2R;
  let CLit = 0, Cmit = 0; if (gT) { const t = VLM.evaluate(gT, 0); CLit = (t.CL - CL0) / D2R; Cmit = (t.Cm - Cm0) / D2R; }
  const [mPct, , tPct] = SL.AIRFOILS[P.wing.sec], V = P.flight.V, rho = atm.rho, mu = atm.mu;
  g.strips.forEach((s, i) => {
    const Re = rho * V * s.c / mu, wing = s.surf === 'wing', tt = wing ? tPct / 100 : 0.1;
    Object.assign(s, { Re, cl0: cl0[i], cla: cla[i], clmax: wing ? clmaxSection(tPct, mPct, Re, s.flap ? P.wing.flap : 0) : 1.2,
      cd0: 2 * cfTurb(Re) * (1 + 2 * tt + 100 * tt ** 4) * 1.02, clmd: wing ? 12 * mPct / 100 : 0 });
  });
  let aStall = Infinity, stallStrip = null;
  g.strips.forEach(s => { if (s.surf !== 'wing' || s.side !== 1 || s.cla <= 0) return; const a = (s.clmax - s.cl0) / s.cla; if (a < aStall) { aStall = a; stallStrip = s; } });
  const CLmax = CL0 + CLa * aStall;
  const B = P.body, f = B.lf / B.df;
  const CDf = cfTurb(rho * V * B.lf / mu) * (1 + 60 / f ** 3 + f / 400) * 0.75 * Math.PI * B.df * B.lf / ref.S;
  const Svt = P.tail.on ? 0.35 * P.tail.bt * P.tail.ct : 0, CDvt = P.tail.on ? 2 * cfTurb(rho * V * P.tail.ct / mu) * 1.25 * Svt / ref.S : 0;
  const profile = al => { let s = 0; for (const st of g.strips) { const cl = st.cl0 + st.cla * al; s += (st.cd0 + 0.006 * (cl - st.clmd) ** 2) * st.c * st.dyProj; } return s / ref.S; };
  const polarAt = al => { const r = VLM.evaluate(g, al), CDp = (profile(al) + CDf + CDvt) * 1.05; return { al, CL: r.CL, CDi: r.CDi, CDp, CD: r.CDi + CDp, Cm: r.Cm }; };
  const W = P.mass * G0, q = 0.5 * rho * V * V, CLreq = W / (q * ref.S), alReq = (CLreq - CL0) / CLa;
  const op = polarAt(alReq);
  const polar = []; const aTop = isFinite(aStall) ? aStall / D2R : 16;
  for (let a = -4; a <= aTop + 0.01; a += Math.max(0.5, (aTop + 4) / 30)) polar.push(polarAt(a * D2R));
  let best = polar[0]; polar.forEach(pt => { if (pt.CL > 0 && pt.CL / pt.CD > best.CL / best.CD) best = pt; });
  let trim = null;
  if (gT) { const det = CLa * Cmit - CLit * Cma; if (Math.abs(det) > 1e-9) trim = { alpha: ((CLreq - CL0) * Cmit - CLit * (-Cm0)) / det, dit: (CLa * (-Cm0) - Cma * (CLreq - CL0)) / det }; }
  const CD0 = polarAt(-CL0 / CLa).CDp;
  VLM.evaluate(g, alReq);   // leave the lattice at the operating point for drawing
  const SM = -Cma / CLa, xnp = ref.xcg + SM * ref.cbar, Vs = Math.sqrt(2 * W / (rho * ref.S * Math.max(0.1, CLmax)));
  const e = op.CL ** 2 / (Math.PI * ref.AR * Math.max(1e-6, op.CDi)), Dn = q * ref.S * op.CD;
  const E = P.engine;
  const avail = E.type === 'prop' ? Math.min(E.power * 800 * atm.sigma / V, 30 * E.power * atm.sigma) : E.type === 'jet' ? E.thrust * 1000 * atm.sigma ** 0.85 * (1 - 0.25 * V / atm.a) : 0;
  const A = { ref, atm, g, CL0, CLa, Cm0, Cma, CLit, Cmit, aStall, stallStrip, CLmax, CDf, CDvt, CD0, W, q, CLreq, alReq, op, polar, best, trim, SM, xnp, Vs, e, Dn, Preq: Dn * V, avail, M: V / atm.a, ms: performance.now() - t0, panels: g.n };
  cache = { v: SL.version, A };
  return A;
};
// aircraft description for the flight model, derived from the VLM solve
SL.aircraftFromProject = () => {
  const A = SL.aero(), P = SL.P, r = A.ref, e = clamp(A.e, 0.5, 1.1);
  const a2d = A.CLa / Math.max(0.15, 1 - A.CLa / (Math.PI * e * r.AR));
  const E = P.engine;
  return { name: P.name, m: P.mass, S: r.S, b: P.wing.b, c: r.cbar, CD0: A.CD0, e, CLmax: A.CLmax, a0: a2d, alpha0: -A.CL0 / A.CLa,
    prop: E.type !== 'jet', P: E.type === 'prop' ? E.power * 1000 : 0, eta: 0.8, Tstatic: E.type === 'prop' ? 30 * E.power : 0, lapse: 1.0, T0: (E.thrust || 0) * 1000,
    Mcrit: E.type === 'jet' ? 0.72 : 0.62, vmin: Math.max(10, Math.round(A.Vs * 1.05)), vmax: Math.max(Math.round(A.Vs * 1.3) + 10, Math.round(P.flight.V * 1.8)) };
};
})();
