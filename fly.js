// Slipstream explore view. Real aircraft models (FlightGear, GPL-2.0) flying through weather.
// Physics: vortex-lattice wing/tail on each aircraft's measured planform (vlm.js) → potential-flow field
// (field-worker.js) for smoke and wingtip-vortex trails; point-mass flight dynamics with ISA; ice accretion
// on the actual surfaces (local impingement × Langmuir–Blodgett collection, rime/glaze with runback);
// snow and frost contamination; D2Q9 lattice Boltzmann for the live wing section (wing-worker.js).
(() => {
const { $, fmt, int, clamp, D2R } = SL;
const G0 = 9.80665, VER = 10;   // VER busts browser caches of the model files
const S = { key: 'c172', mode: 'fly', aoa: 4, view: 'chase', wx: 'clear', dT: 0, ts: 1, smoke: false, trails: true, liftmap: false, icehl: false };
const WX = {
  clear: { name: 'Clear', lwc: 0, mvd: 0, precip: null },
  cloud: { name: 'Icing cloud', lwc: 0.5, mvd: 22, precip: 'cloud' },
  frz: { name: 'Freezing rain', lwc: 0.35, mvd: 900, precip: 'rain' },
  snow: { name: 'Snow', lwc: 0, mvd: 0, precip: 'snow' },
  frost: { name: 'Frost', lwc: 0, mvd: 0, precip: null },
};
const AP = { ht: 2500, vt: 60 };
let CAT = null, ac = null, acVer = -1, st = null, A = null, O = {}, base = null;
const tweens = [];
const toast = (msg, ms = 4500) => { const t = $('toast'); t.textContent = msg; t.hidden = false; clearTimeout(t._t); t._t = setTimeout(() => t.hidden = true, ms); };
const OAT = h => SL.isa(h).Tc + S.dT;
const hNow = () => S.mode === 'park' ? 0 : st.h;   // parked aircraft sit at sea level

// ====================================================== flight model (ISA, point mass, icing and contamination)
function aeroOf(k, kind, c) {
  const AR = ac.b * ac.b / ac.S, kc = k / ac.c, s = Math.sqrt(Math.max(0, kc)), hf = kind === 'glaze' ? 1 : kind === 'mixed' ? 0.82 : 0.65;
  const dCLmax = Math.min(0.6, 3.0 * s * hf + 0.3 * c), dCD0 = 0.25 * s * hf + 0.4 * c * ac.CD0, a2d = ac.a0 * (1 - 0.5 * s), aw = a2d / (1 + a2d / (Math.PI * ac.e * AR));
  const CLmax = ac.CLmax * (1 - dCLmax);
  return { AR, aw, CLmax, alphaS: ac.alpha0 + 1.2 * CLmax / aw, dCD0, dCLmax, kc };
}
const CLof = (al, X) => { const x = (al - ac.alpha0) * X.aw / X.CLmax; return (x < 0.8 ? x : x < 1.2 ? x - (x - 0.8) ** 2 / 0.8 : Math.max(0.55, 1 - 0.9 * (x - 1.2))) * X.CLmax; };
const CDof = (CL, al, M, X) => { const x = (al - ac.alpha0) * X.aw / X.CLmax; return ac.CD0 + X.dCD0 + CL * CL / (Math.PI * ac.e * X.AR) + (x > 1.2 ? 0.6 * (x - 1.2) : 0) + (M > ac.Mcrit ? 20 * (M - ac.Mcrit) ** 4 : 0); };
const Tmax = (rho, V, M) => { const sig = rho / 1.225; return ac.prop ? Math.min(ac.Tstatic * sig, ac.eta * ac.P * sig / Math.max(V, 20)) : ac.T0 * sig ** 0.85 * (1 - 0.25 * M); };
const beta0 = V => { const w = WX[S.wx]; if (!w.mvd) return 0; const d = w.mvd * 1e-6, K = 1000 * d * d * Math.max(V, 1) / (9 * 1.7e-5 * 0.2 * ac.c); if (K <= 0.125) return 0; const z = 1.4 * Math.pow(K - 0.125, 0.84); return 0.85 * z / (1 + z); };
const liquid = Tc => Tc >= 0 ? 0 : clamp((Tc + 40) / 25, 0, 1);
const liquidFor = (wx, Tc) => wx === 'frz' ? (Tc < 0 && Tc > -12 ? 1 : 0) : WX[wx].lwc ? liquid(Tc) : 0;
const iceRate = () => { const w = WX[S.wx], Tc = OAT(hNow()), lf = liquidFor(S.wx, Tc); if (lf <= 0 || st.V < 5 || S.mode === 'park') return 0;
  const kd = SL.iceKind(Tc, w.lwc).key, kind = kd === 'none' ? 'rime' : (S.wx === 'frz' ? 'glaze' : kd); return beta0(st.V) * w.lwc * 1e-3 * st.V / (kind === 'rime' ? 880 : 917) * lf; };   // m/s of sim time
function newState() {
  st = { h: AP.ht, V: AP.vt, gam: 0, alpha: 3 * D2R, k: 0, kind: 'rime', t: 0, mode: 'hold', recT: 0, stallH: 0, contam: 0, cType: 'snow', dk: 0 };
  const atm = SL.isa(st.h), X = aeroOf(0, 'rime', 0), q = 0.5 * atm.rho * st.V * st.V;
  st.alpha = clamp(ac.alpha0 + ac.m * G0 / (q * ac.S) / X.aw, -4 * D2R, 14 * D2R);
}
function rebuildAircraft() { ac = SL.aircraftFromProject(); acVer = SL.version; A = SL.aero(); }
function accrete(dt) {   // stagnation-line ice growth (m), shared by the flight model and the surface ice
  const Tc = OAT(hNow()), w = WX[S.wx], lf = liquidFor(S.wx, Tc);
  if (lf <= 0 || st.V < 5) return;
  const kd = SL.iceKind(Tc, w.lwc).key; st.kind = kd === 'none' ? 'rime' : (S.wx === 'frz' ? 'glaze' : kd);
  const d = beta0(st.V) * w.lwc * 1e-3 * st.V / (st.kind === 'rime' ? 880 : 917) * lf * dt;
  st.k += d; st.dk += d;
}
function flightStep(dt) {
  const atm = SL.isa(st.h), X = aeroOf(st.k, st.kind, st.contam), W = ac.m * G0, q = 0.5 * atm.rho * st.V * st.V, M = st.V / atm.a, Tm = Tmax(atm.rho, st.V, M);
  let alCmd;
  if (st.mode === 'recover') alCmd = ac.alpha0 + 0.7 * (X.alphaS - ac.alpha0);
  else {
    const CLl = W / (q * ac.S), D0 = q * ac.S * CDof(CLl, ac.alpha0 + CLl / X.aw, M, X), gmax = Math.max(0, (Tm - D0) / W * 0.7);
    let gc = clamp(0.002 * (AP.ht - st.h), -0.08, 0.08); if (gc > 0) gc = Math.min(gc, gmax);
    alCmd = ac.alpha0 + (W * Math.cos(st.gam) + ac.m * st.V * 0.8 * (gc - st.gam)) / (q * ac.S) / X.aw;
  }
  st.alpha += (clamp(alCmd, -6 * D2R, 24 * D2R) - st.alpha) * Math.min(1, dt * 2.5);
  const CL = CLof(st.alpha, X), CD = CDof(CL, st.alpha, M, X), L = q * ac.S * CL, D = q * ac.S * CD;
  const T = clamp(st.mode === 'recover' ? Tm : D + W * Math.sin(st.gam) + ac.m * 0.25 * (AP.vt - st.V), 0, Tm);
  st.V = Math.max(12, st.V + (T - D - W * Math.sin(st.gam)) / ac.m * dt);
  st.gam = clamp(st.gam + (L - W * Math.cos(st.gam)) / (ac.m * st.V) * dt, -0.6, 0.4);
  st.h = Math.max(30, st.h + st.V * Math.sin(st.gam) * dt); st.t += dt;
  const Vs = Math.sqrt(2 * W / (atm.rho * ac.S * X.CLmax));
  if (st.mode === 'hold' && st.alpha > X.alphaS) { st.mode = 'recover'; st.stallH = st.h; st.recT = 0; toast(`Stall at ${int(st.h)} m${st.k > 0.002 ? ` with ${Math.round(st.k * 1000)} mm of ice` : st.contam > 0.2 ? ' with a contaminated wing' : ''}. The nose drops to recover.`, 5500); }
  if (st.mode === 'recover') { st.recT += dt; if (st.recT > 4 && st.V > 1.3 * Vs && st.alpha < X.alphaS) { st.mode = 'hold'; AP.ht = Math.round(st.h / 100) * 100; syncControls(); toast(`Recovered, ${int(st.stallH - st.h)} m lower.`); } }
  accrete(dt);
}
function outputs() {
  const atm = SL.isa(st.h), X = aeroOf(st.k, st.kind, st.contam), W = ac.m * G0, q = 0.5 * atm.rho * st.V * st.V, M = st.V / atm.a;
  const CL = CLof(st.alpha, X), CD = CDof(CL, st.alpha, M, X);
  O = { atm, X, W, q, M, CL, CD, L: q * ac.S * CL, D: q * ac.S * CD, Vs: Math.sqrt(2 * W / (atm.rho * ac.S * X.CLmax)), vs: st.V * Math.sin(st.gam), stalled: S.mode !== 'park' && (st.alpha > X.alphaS || st.mode === 'recover') };
}

// ====================================================== renderer, sky, environment
const canvas = $('scene');
const R = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
R.setPixelRatio(Math.min(1.5, window.devicePixelRatio || 1));   // retina at 2× costs ~80% more GPU energy for little gain
R.outputEncoding = THREE.sRGBEncoding; R.toneMapping = THREE.ACESFilmicToneMapping; R.toneMappingExposure = 0.95;
R.physicallyCorrectLights = true;
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 40000);
scene.fog = new THREE.Fog(0xb9d4ea, 1200, 12000);
const hemi = new THREE.HemisphereLight(0xdcefff, 0x56634f, 1.4); scene.add(hemi);
const sun = new THREE.DirectionalLight(0xfff0db, 3.2); sun.position.set(-0.45, 1, 0.4).multiplyScalar(100); scene.add(sun);
const skyU = { top: { value: new THREE.Color(0x2a6fc0) }, hor: { value: new THREE.Color(0xcfe3f3) }, sunDir: { value: new THREE.Vector3(-0.45, 0.55, 0.4).normalize() }, overcast: { value: 0 } };
const skyMat = new THREE.ShaderMaterial({ uniforms: skyU, side: THREE.BackSide, depthWrite: false, fog: false,
  vertexShader: 'varying vec3 vD; void main(){ vD = normalize(position); gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
  fragmentShader: `uniform vec3 top; uniform vec3 hor; uniform vec3 sunDir; uniform float overcast; varying vec3 vD;
    void main(){ float h = clamp(vD.y, -0.3, 1.0); vec3 c = mix(hor, top, pow(max(h, 0.0), 0.5)); if (h < 0.0) c = mix(hor, hor * 0.8, -h * 3.0);
      float s = max(dot(normalize(vD), sunDir), 0.0); c += vec3(1.0, 0.88, 0.7) * (pow(s, 900.0) * 2.0 + pow(s, 10.0) * 0.16) * (1.0 - overcast);
      c = mix(c, vec3(0.74, 0.77, 0.8) * (0.85 + 0.15 * h), overcast); gl_FragColor = vec4(c, 1.0); }` });
const sky = new THREE.Mesh(new THREE.SphereGeometry(20000, 48, 24), skyMat); scene.add(sky);
const pmrem = new THREE.PMREMGenerator(R);
let envRT = null;
function refreshEnv() {
  const es = new THREE.Scene(); es.add(new THREE.Mesh(new THREE.SphereGeometry(100, 32, 16), skyMat));
  const g = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), new THREE.MeshBasicMaterial({ color: 0x55664d })); g.rotation.x = -Math.PI / 2; g.position.y = -20; es.add(g);
  const rt = pmrem.fromScene(es, 0.02); if (envRT) envRT.dispose(); envRT = rt; scene.environment = rt.texture;
}
// ground and runway
const gTex = (() => { const c = document.createElement('canvas'); c.width = c.height = 1024; const x = c.getContext('2d'); x.fillStyle = '#6f8a5e'; x.fillRect(0, 0, 1024, 1024);
  const cols = ['#7d9866', '#647f55', '#8aa071', '#6c8457', '#93a678', '#5f7a4f', '#a3a57a'];
  for (let i = 0; i < 260; i++) { x.fillStyle = cols[i % cols.length]; x.save(); x.translate(Math.random() * 1024, Math.random() * 1024); x.rotate((Math.random() - 0.5) * 0.4); x.fillRect(0, 0, 40 + Math.random() * 120, 30 + Math.random() * 90); x.restore(); }
  x.strokeStyle = 'rgba(225,215,190,.4)'; x.lineWidth = 3; for (let i = 0; i < 10; i++) { x.beginPath(); x.moveTo(0, Math.random() * 1024); x.bezierCurveTo(340, Math.random() * 1024, 680, Math.random() * 1024, 1024, Math.random() * 1024); x.stroke(); }
  const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(30, 30); t.encoding = THREE.sRGBEncoding; t.anisotropy = 8; return t; })();
const groundMat = new THREE.MeshStandardMaterial({ map: gTex, roughness: 1 });
const ground = new THREE.Mesh(new THREE.PlaneGeometry(80000, 80000), groundMat); ground.rotation.x = -Math.PI / 2; scene.add(ground);
const rwTex = (() => { const c = document.createElement('canvas'); c.width = 256; c.height = 1024; const x = c.getContext('2d'); x.fillStyle = '#3c3f44'; x.fillRect(0, 0, 256, 1024);
  for (let i = 0; i < 4000; i++) { x.fillStyle = `rgba(${Math.random() > 0.5 ? 255 : 0},${Math.random() > 0.5 ? 255 : 0},${Math.random() > 0.5 ? 255 : 0},0.04)`; x.fillRect(Math.random() * 256, Math.random() * 1024, 2, 2); }
  x.fillStyle = '#e8e8e2'; for (let y = 0; y < 1024; y += 128) x.fillRect(124, y, 8, 64); x.fillRect(8, 0, 6, 1024); x.fillRect(242, 0, 6, 1024);
  const t = new THREE.CanvasTexture(c); t.wrapT = THREE.RepeatWrapping; t.repeat.set(1, 6); t.encoding = THREE.sRGBEncoding; t.anisotropy = 8; return t; })();
const runwayMat = new THREE.MeshStandardMaterial({ map: rwTex, roughness: 0.9, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
const runway = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), runwayMat); runway.rotation.x = -Math.PI / 2; rwTex.center.set(0.5, 0.5); rwTex.rotation = Math.PI / 2; scene.add(runway);
// clouds: soft lit puffs
const puffTex = (() => { const c = document.createElement('canvas'); c.width = c.height = 128; const x = c.getContext('2d'); const g = x.createRadialGradient(64, 58, 4, 64, 64, 62); g.addColorStop(0, 'rgba(255,255,255,0.95)'); g.addColorStop(0.55, 'rgba(244,246,250,0.55)'); g.addColorStop(1, 'rgba(235,240,245,0)'); x.fillStyle = g; x.fillRect(0, 0, 128, 128); return new THREE.CanvasTexture(c); })();
const puffs = [];
for (let i = 0; i < 80; i++) {
  const cx = (Math.random() - 0.5) * 12000, cz = (Math.random() - 0.5) * 12000, alt = 1500 + Math.random() * 2200;
  for (let j = 0; j < 8; j++) {
    const m = new THREE.Sprite(new THREE.SpriteMaterial({ map: puffTex, color: 0xffffff, transparent: true, opacity: 0.9, depthWrite: false }));
    const s = 260 + Math.random() * 420; m.scale.set(s * 1.7, s, 1);
    m.userData = { x: cx + (Math.random() - 0.5) * 600, z: cz + (Math.random() - 0.5) * 600, alt: alt + (Math.random() - 0.5) * 200, shade: 0.82 + Math.random() * 0.18 };
    m.material.color.setScalar(m.userData.shade); scene.add(m); puffs.push(m);
  }
}

// ====================================================== aircraft
const plane = new THREE.Group(); scene.add(plane);
const body = new THREE.Group(); body.matrixAutoUpdate = false; plane.add(body);
let L_ref = 10, xcg = 0, model = null, meshes = [], iceGeo = null, snowGeo = null, props = [];
let corner = null, baseCol = null;   // per-corner data for airframe + glass: pos, nrm, wing flag, noise, ice t, snow s
const paint = new THREE.MeshPhysicalMaterial({ vertexColors: true, roughness: 0.38, metalness: 0.05, clearcoat: 0.4, clearcoatRoughness: 0.2, envMapIntensity: 0.65, side: THREE.DoubleSide });
const glassMat = new THREE.MeshPhysicalMaterial({ side: THREE.DoubleSide, color: 0x14202b, roughness: 0.04, metalness: 0.95, envMapIntensity: 1.3 });
const propMat = new THREE.MeshStandardMaterial({ color: 0x24272b, roughness: 0.45, metalness: 0.5, side: THREE.DoubleSide });
const iceMat = new THREE.MeshPhysicalMaterial({ vertexColors: true, transparent: true, roughness: 0.6, metalness: 0, clearcoat: 0.3, clearcoatRoughness: 0.1, sheen: 1, sheenColor: new THREE.Color(0.8, 0.9, 1), sheenRoughness: 0.5, envMapIntensity: 1.2, alphaTest: 0.04 });
const snowMat = new THREE.MeshStandardMaterial({ vertexColors: true, transparent: true, roughness: 0.95, metalness: 0, alphaTest: 0.04 });
const cache = {};
async function loadModel(key) {
  if (cache[key]) return cache[key];
  const m = CAT[key]; let buf, r = await fetch(m.file + '?v=' + VER).catch(() => null);
  if (r && r.ok && !(r.headers.get('content-type') || '').includes('text/html')) buf = await r.arrayBuffer();
  else { r = await fetch(m.file + '.txt?v=' + VER); if (!r.ok) throw new Error('Could not load ' + m.file); const b = atob((await r.text()).trim()), u = new Uint8Array(b.length); for (let i = 0; i < b.length; i++) u[i] = b.charCodeAt(i); buf = u.buffer; }
  const dv = new DataView(buf); if (dv.getUint32(0, true) !== 0x334D4C53) throw new Error('Bad model file ' + m.file);
  const nt = dv.getUint32(4, true);
  return (cache[key] = { nt, pos: new Float32Array(buf, 8, nt * 9), nrm: new Int8Array(buf, 8 + nt * 36, nt * 9), part: new Uint8Array(buf, 8 + nt * 45, nt) });
}
const PAINT = { c172: { top: [0.03, 0.045, 0.085], bottom: [0.4, 0.42, 0.45], wing: [0.035, 0.05, 0.09], dark: [0.08, 0.08, 0.09] },
  spitfire: { top: [0.07, 0.085, 0.04], camo: [0.15, 0.16, 0.17], bottom: [0.3, 0.315, 0.33], wing: [0.07, 0.085, 0.04], dark: [0.05, 0.05, 0.05] },   // RAF dark green / ocean grey over medium sea grey (linear RGB)
  b738: { top: [0.95, 0.95, 0.96], bottom: [0.2, 0.27, 0.42], wing: [0.62, 0.65, 0.69], dark: [0.15, 0.15, 0.16] },
  a320: { top: [0.95, 0.95, 0.96], bottom: [0.78, 0.8, 0.83], wing: [0.62, 0.65, 0.69], dark: [0.15, 0.15, 0.16] } };
function inWing(x, y, z) {
  const P = SL.P, w = P.wing, s = w.b / 2, ay = Math.abs(y); if (ay < 0.55 * P.body.df || ay > s * 1.08) return false;
  const W = SL.geom.wing(P), yy = Math.min(ay, s), c = W.c(yy), xl = W.xle(yy), zw = yy * W.tG;
  return x > xl - 0.1 * c && x < xl + 1.1 * c && Math.abs(z - zw) < 0.2 * c + 0.05;
}
function buildModel(M) {
  for (const m of meshes) { if (m.parent) m.parent.remove(m); m.geometry.dispose(); } meshes = [];
  for (const h of props) body.remove(h);   // propeller holders too, or the previous aircraft's propeller stays behind
  const P = SL.P, pal = PAINT[S.key], ref = SL.geom.reference(P);
  xcg = ref.xcg; L_ref = Math.max(P.wing.b, P.body.lf);
  body.matrix.set(-1, 0, 0, xcg, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1); body.matrixWorldNeedsUpdate = true;
  const groups = { 0: [], 1: [], 2: [] };
  for (let t = 0; t < M.nt; t++) groups[M.part[t] === 3 ? 0 : M.part[t]].push(t);
  const mk = (tris, withColour) => {
    const n = tris.length * 3, p = new Float32Array(n * 3), nr = new Float32Array(n * 3), col = withColour ? new Float32Array(n * 3) : null, dark = [];
    tris.forEach((t, i) => { for (let j = 0; j < 9; j++) { p[i * 9 + j] = M.pos[t * 9 + j]; nr[i * 9 + j] = M.nrm[t * 9 + j] / 127; } dark.push(M.part[t] === 3); });
    if (col) for (let v = 0; v < n; v++) {
      const x = p[v * 3], y = p[v * 3 + 1], z = p[v * 3 + 2], nz = nr[v * 3 + 2];
      const c = dark[(v / 3) | 0] ? pal.dark : inWing(x, y, z) ? pal.wing : nz < -0.35 ? pal.bottom : pal.top;
      if (pal.camo && c !== pal.dark && c !== pal.bottom) {   // disruptive two-tone pattern in plan view
        const w = Math.sin(x * 0.9 + 1.3 * Math.sin(y * 0.55)) + Math.sin(y * 0.8 - 0.7 * x + 2) * 0.8 + Math.sin((x + y) * 1.7) * 0.25;
        if (w > 0.15) { col[v * 3] = pal.camo[0]; col[v * 3 + 1] = pal.camo[1]; col[v * 3 + 2] = pal.camo[2]; continue; }
      }
      col[v * 3] = c[0]; col[v * 3 + 1] = c[1]; col[v * 3 + 2] = c[2];
    }
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(p, 3)); g.setAttribute('normal', new THREE.BufferAttribute(nr, 3)); if (col) g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    return g;
  };
  const gA = mk(groups[0], true), gG = mk(groups[1], false), gP = mk(groups[2], false); baseCol = gA.attributes.color.array.slice();
  const mA = new THREE.Mesh(gA, paint), mG = new THREE.Mesh(gG, glassMat); body.add(mA, mG); meshes.push(mA, mG);
  // each propeller or engine fan spins about its own axis (parallel to body x): split them by spanwise position
  props = [];
  if (groups[2].length) {
    const ys = groups[2].map(t => [(M.pos[t * 9 + 1] + M.pos[t * 9 + 4] + M.pos[t * 9 + 7]) / 3, t]).sort((a, b) => a[0] - b[0]), clusters = [[]];
    ys.forEach(([y, t], i) => { if (i && y - ys[i - 1][0] > 0.4) clusters.push([]); clusters[clusters.length - 1].push(t); });
    for (const cl of clusters) {
      const g = mk(cl, false); g.computeBoundingBox(); const c = new THREE.Vector3(); g.boundingBox.getCenter(c); g.translate(0, -c.y, -c.z);
      const holder = new THREE.Group(); holder.position.set(0, c.y, c.z); const mP = new THREE.Mesh(g, propMat); holder.add(mP); body.add(holder); meshes.push(mP); props.push(holder);
    }
    gP.dispose();
  }
  // surface contamination layers share the airframe + glass corners
  const nA = gA.attributes.position.count, nG = gG.attributes.position.count, n = nA + nG;
  const pos = new Float32Array(n * 3), nrm = new Float32Array(n * 3);
  pos.set(gA.attributes.position.array, 0); pos.set(gG.attributes.position.array, nA * 3); nrm.set(gA.attributes.normal.array, 0); nrm.set(gG.attributes.normal.array, nA * 3);
  const wing = new Uint8Array(n), noise = new Float32Array(n);
  for (let v = 0; v < n; v++) { wing[v] = inWing(pos[v * 3], pos[v * 3 + 1], pos[v * 3 + 2]) ? 1 : 0; const h = Math.sin(pos[v * 3] * 12.9 + pos[v * 3 + 1] * 78.2 + pos[v * 3 + 2] * 37.7) * 43758.5; noise[v] = 0.55 + 0.9 * (h - Math.floor(h)); }
  corner = { n, nA, pos, nrm, wing, noise, t: new Float32Array(n), s: new Float32Array(n), imp: new Float32Array(n) };
  const layer = (mat) => { const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 3), 3)); g.setAttribute('normal', new THREE.BufferAttribute(nrm, 3)); g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(n * 4), 4)); const m = new THREE.Mesh(g, mat); m.visible = false; body.add(m); meshes.push(m); return g; };
  iceGeo = layer(iceMat); snowGeo = layer(snowMat);
  computeImpingement();
}
let impAlpha = 99;
function computeImpingement() {   // fraction of the oncoming airflow each surface element faces
  const c = corner, ca = Math.cos(st.alpha), sa = Math.sin(st.alpha); impAlpha = st.alpha;
  for (let v = 0; v < c.n; v++) c.imp[v] = Math.max(0, -(c.nrm[v * 3] * ca + c.nrm[v * 3 + 2] * sa));
}
function surfaceUpdate(dt) {
  if (!corner) return;
  const c = corner, Tc = OAT(hNow()), b0 = beta0(st.V), p = 1.5 + 4 * (1 - b0), n0 = S.wx === 'frz' ? 0.15 : SL.iceKind(Tc, WX[S.wx].lwc || 0.5).n0 || 1;
  if (Math.abs(st.alpha - impAlpha) > 2 * D2R) computeImpingement();
  // ice: distribute this interval's stagnation growth over the surface by local impingement (rime stays at the front; glaze runs back)
  const dk = st.dk; st.dk = 0;
  if (dk > 0) for (let v = 0; v < c.n; v++) {
    const f = c.imp[v]; if (f <= 0.02) continue;
    let sh = Math.pow(f, p); if (n0 < 0.9) sh = sh * (0.4 + 0.6 * n0) + (1 - n0) * 0.75 * Math.exp(-(((f - 0.42) / 0.16) ** 2));
    c.t[v] += dk * sh * c.noise[v];
  }
  // snow and frost: settle on surfaces facing up while slow or parked; blow off at flying speed
  const th = S.mode === 'fly' ? st.alpha + st.gam : S.mode === 'tunnel' ? st.alpha : 0, cth = Math.cos(th), sth = Math.sin(th);
  const snowing = S.wx === 'snow' && Tc < 2, frosting = S.wx === 'frost' && S.mode === 'park' && Tc < 1;
  const rate = snowing ? (S.mode === 'park' ? 0.4e-3 / 60 : 0.03e-3 / 60) : frosting ? 0.5e-3 / 1800 : 0;
  const shed = st.V > 15 ? Math.exp(-dt * st.V / (Tc > -2 ? 600 : 45)) : 1, cap = frosting ? 0.6e-3 : 0.03;
  let sw = 0, nw = 0;
  for (let v = 0; v < c.n; v++) {
    const up = c.nrm[v * 3 + 2] * cth - c.nrm[v * 3] * sth;
    if (rate && up > 0.25) c.s[v] = Math.min(cap, c.s[v] + rate * dt * (up - 0.25) / 0.75 * c.noise[v]);
    c.s[v] *= shed;
    if (c.wing[v] && up > 0.5) { sw += c.s[v]; nw++; }
  }
  if (frosting) st.cType = 'frost'; else if (snowing) st.cType = 'snow';
  const mean = nw ? sw / nw : 0;
  st.contam = clamp(st.cType === 'frost' ? mean / 0.25e-3 : mean / 2e-3, 0, 1);
  st.snowMean = mean;
  // write the visual layers (ice drawn 2× thicker so it reads at a distance; snow true scale ×1.5)
  const ip = iceGeo.attributes.position.array, icol = iceGeo.attributes.color.array, sp = snowGeo.attributes.position.array, scol = snowGeo.attributes.color.array;
  const glaze = st.kind === 'glaze', hl = S.icehl, iceCol = hl ? [0.2, 0.85, 1.0] : glaze ? [0.62, 0.8, 0.92] : [0.86, 0.92, 0.98];
  iceMat.roughness = glaze ? 0.08 : 0.75; if (hl) iceMat.emissive.setRGB(0.05, 0.42, 0.6); else iceMat.emissive.setRGB(glaze ? 0.04 : 0.17, glaze ? 0.07 : 0.19, glaze ? 0.1 : 0.22); iceMat.clearcoat = glaze ? 1 : 0.15; iceMat.sheen = glaze ? 0 : 1;
  let anyI = false, anyS = false; const iceX = clamp(L_ref / 2.4, 4, 12);   // drawn thicker than real so it reads at viewing distance
  for (let v = 0; v < c.n; v++) {
    const t = c.t[v] * iceX, s = c.s[v] * 1.5, o = v * 3, q = v * 4;
    ip[o] = c.pos[o] + c.nrm[o] * t; ip[o + 1] = c.pos[o + 1] + c.nrm[o + 1] * t; ip[o + 2] = c.pos[o + 2] + c.nrm[o + 2] * t;
    const ai = Math.pow(clamp(c.t[v] / 0.0006, 0, 1), 0.7) * (glaze ? 0.8 : 0.97); const gr = 0.82 + 0.18 * (c.noise[v] - 0.55) / 0.9; icol[q] = iceCol[0] * gr; icol[q + 1] = iceCol[1] * gr; icol[q + 2] = iceCol[2] * (0.9 + 0.1 * gr); icol[q + 3] = ai; if (ai > 0.04) anyI = true;
    const off = t + s + 0.0004;
    sp[o] = c.pos[o] + c.nrm[o] * off; sp[o + 1] = c.pos[o + 1] + c.nrm[o + 1] * off; sp[o + 2] = c.pos[o + 2] + c.nrm[o + 2] * off;
    const as = st.cType === 'frost' ? clamp(c.s[v] / 0.25e-3, 0, 1) * 0.75 : clamp(c.s[v] / 0.8e-3, 0, 1); scol[q] = 0.96; scol[q + 1] = 0.97; scol[q + 2] = 0.99; scol[q + 3] = as; if (as > 0.04) anyS = true;
  }
  iceGeo.attributes.position.needsUpdate = true; iceGeo.attributes.color.needsUpdate = true; snowGeo.attributes.position.needsUpdate = true; snowGeo.attributes.color.needsUpdate = true;
  meshes.find(m => m.geometry === iceGeo).visible = anyI; meshes.find(m => m.geometry === snowGeo).visible = anyS;
  if (S.liftmap) liftMap();
}
function clearSurface(f = 0) { if (!corner) return; for (let v = 0; v < corner.n; v++) { corner.t[v] *= f; corner.s[v] *= f; } st.k *= f; st.contam *= f; }
// lift map: tint the wing by how close each section is to its maximum lift (vortex-lattice strips)
function liftMap(off) {
  const g = meshes[0] && meshes[0].geometry; if (!g || !A) return;
  const col = g.attributes.color.array, pos = g.attributes.position.array, pal = PAINT[S.key];
  const strips = A.g.strips.filter(x => x.surf === 'wing' && x.side === 1).sort((a, b) => a.mid[1] - b.mid[1]);
  const pulse = 0.5 + 0.5 * Math.sin(performance.now() / 160), lossF = O.X ? O.X.CLmax / ac.CLmax : 1;
  for (let v = 0; v < corner.nA; v++) {
    if (!corner.wing[v]) continue;
    let c = [baseCol[v * 3], baseCol[v * 3 + 1], baseCol[v * 3 + 2]];
    if (!off && S.liftmap) {
      const y = Math.abs(pos[v * 3 + 1]); let k = 0; while (k < strips.length - 1 && strips[k + 1].mid[1] < y) k++;
      const sp = strips[k], r = clamp((sp.cl0 + sp.cla * st.alpha) / (sp.clmax * lossF), 0, 1.4);
      c = r < 1 ? SL.mapRGB('viridis', r).map(u => u / 255) : [1, 0.25 + 0.15 * pulse, 0.15];
    }
    col[v * 3] = c[0]; col[v * 3 + 1] = c[1]; col[v * 3 + 2] = c[2];
  }
  g.attributes.color.needsUpdate = true;
}

// ====================================================== flow field, wingtip-vortex trails and smoke
let field = null, fieldBusy = false, fieldWant = null, fieldAlpha = 99, fieldVer = -1, box = null;
const fw = new Worker('field-worker.js?v=10');
fw.onmessage = e => { field = e.data; fieldBusy = false; buildTrails(); if (fieldWant) { const w = fieldWant; fieldWant = null; requestField(w.alpha); } };
function requestField(alpha) {
  if (!A) return;
  if (fieldBusy) { fieldWant = { alpha }; return; }
  const P = SL.P, g = A.g; VLM.evaluate(g, alpha);
  const n = g.panels.length, Aa = new Float32Array(n * 3), Ba = new Float32Array(n * 3), Ga = new Float32Array(n);
  g.panels.forEach((p, i) => { Aa.set(p.A, i * 3); Ba.set(p.B, i * 3); Ga[i] = p.gam; });
  const b = P.wing.b, L = L_ref;
  box = [P.body.xnose - 0.25 * L, P.body.xnose + P.body.lf + 1.6 * L, 0, 0.75 * b, -0.35 * b, 0.35 * b];
  fieldBusy = true; fieldAlpha = alpha; fieldVer = SL.version;
  fw.postMessage({ id: SL.version, A: Aa, B: Ba, G: Ga, alpha, box, dims: [72, 30, 30], rc: 0.05 * SL.geom.reference(P).cbar, far: 200 * b }, [Aa.buffer, Ba.buffer, Ga.buffer]);
}
function vel(x, y, z, out) {
  if (!field) { out[0] = Math.cos(st.alpha); out[1] = 0; out[2] = Math.sin(st.alpha); return; }
  const [x0, x1, y0, y1, z0, z1] = field.box, [NX, NY, NZ] = field.dims, ay = Math.abs(y);
  if (x < x0 || x > x1 || ay > y1 || z < z0 || z > z1) { out[0] = Math.cos(fieldAlpha); out[1] = 0; out[2] = Math.sin(fieldAlpha); return; }
  const fx = (x - x0) / (x1 - x0) * (NX - 1), fy = (ay - y0) / (y1 - y0) * (NY - 1), fz = (z - z0) / (z1 - z0) * (NZ - 1);
  const i = Math.min(NX - 2, fx | 0), j = Math.min(NY - 2, fy | 0), k = Math.min(NZ - 2, fz | 0), u = fx - i, v = fy - j, w = fz - k, F = field.field;
  for (let c = 0; c < 3; c++) {
    const at = (ii, jj, kk) => F[((kk * NY + jj) * NX + ii) * 3 + c];
    out[c] = ((at(i, j, k) * (1 - u) + at(i + 1, j, k) * u) * (1 - v) + (at(i, j + 1, k) * (1 - u) + at(i + 1, j + 1, k) * u) * v) * (1 - w)
           + ((at(i, j, k + 1) * (1 - u) + at(i + 1, j, k + 1) * u) * (1 - v) + (at(i, j + 1, k + 1) * (1 - u) + at(i + 1, j + 1, k + 1) * u) * v) * w;
  }
  if (y < 0) out[1] = -out[1];
}
const trailMat = new THREE.MeshBasicMaterial({ color: 0x2f6bff, transparent: true, opacity: 0.85, depthWrite: false });
let trails = null;
function buildTrails() {
  if (trails) { body.remove(trails); trails.traverse(o => o.geometry && o.geometry.dispose()); trails = null; }
  if (!S.trails || !field) return;
  const P = SL.P, W = SL.geom.wing(P), s = P.wing.b / 2, b = P.wing.b, grp = new THREE.Group(), vv = [0, 0, 0];
  const tipX = W.xle(s * 0.97) + W.c(s * 0.97), tipZ = s * 0.97 * W.tG, r0 = 0.025 * b;
  for (const side of [1, -1]) for (let j = 0; j < 3; j++) {
    const a = j / 3 * 2 * Math.PI; let x = tipX, y = side * (s * 0.97 + r0 * Math.cos(a) * 0.6 - 0.02 * b), z = tipZ + r0 * Math.sin(a) * 0.6;
    const pts = [new THREE.Vector3(x, y, z)], h = 0.018 * b;
    for (let k = 0; k < 220; k++) { vel(x, y, z, vv); const m = Math.hypot(vv[0], vv[1], vv[2]) || 1; x += vv[0] / m * h; y += vv[1] / m * h; z += vv[2] / m * h; if (k % 2) pts.push(new THREE.Vector3(x, y, z)); if (x > box[1]) break; }
    if (pts.length > 4) grp.add(new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), pts.length * 2, (j ? 0.0022 : 0.0035) * L_ref, 6, false), trailMat));
  }
  trails = grp; body.add(grp);
}
// smoke streaklines (optional)
const NP = 1200, HN = 12;
const pX = new Float32Array(NP), pY = new Float32Array(NP), pZ = new Float32Array(NP), pA = new Float32Array(NP);
const hX = new Float32Array(NP * HN), hY = new Float32Array(NP * HN), hZ = new Float32Array(NP * HN), hS = new Float32Array(NP * HN), hHead = new Uint8Array(NP), hCount = new Uint8Array(NP);
const NSEG = NP * (HN - 1), lineGeo = new THREE.BufferGeometry(), lp = new Float32Array(NSEG * 6), lc = new Float32Array(NSEG * 8);
lineGeo.setAttribute('position', new THREE.BufferAttribute(lp, 3).setUsage(THREE.DynamicDrawUsage));
lineGeo.setAttribute('color', new THREE.BufferAttribute(lc, 4).setUsage(THREE.DynamicDrawUsage));
const smoke = new THREE.LineSegments(lineGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false }));
smoke.frustumCulled = false; body.add(smoke);
function seed(i, anywhere) {
  const P = SL.P, b = P.wing.b, bx = box || [P.body.xnose - 3, P.body.xnose + P.body.lf + 10, 0, b, -b / 3, b / 3];
  const lane = (Math.random() * 22) | 0, lev = (Math.random() * 4) | 0;
  pY[i] = (-0.6 + 1.2 * (lane + 0.5) / 22) * b; pZ[i] = (-0.05 + 0.11 * (lev + 0.5) / 4) * b;
  pX[i] = anywhere ? bx[0] + Math.random() * (bx[1] - bx[0]) : bx[0] + Math.random() * 0.02 * b; pA[i] = 0; hCount[i] = 0;
}
for (let i = 0; i < NP; i++) seed(i, true);
const colAt = (s, f, out, o) => { const t = clamp((s - 1) / 0.22, -1, 1); let r, g, b; if (t < 0) { const u = -t; r = 0.16 - 0.08 * u; g = 0.66 - 0.36 * u; b = 0.98 - 0.1 * u; } else { r = 0.16 + 0.84 * t; g = 0.66 - 0.1 * t; b = 0.98 - 0.88 * t; } out[o] = r; out[o + 1] = g; out[o + 2] = b; out[o + 3] = f * 0.9; };
let histT = 0; const vv = [0, 0, 0];
function stepSmoke(dt) {
  smoke.visible = S.smoke; if (!S.smoke) return;
  const P = SL.P, sp = 0.42 * L_ref * dt, xEnd = box ? box[1] : P.body.xnose + P.body.lf + L_ref, b = P.wing.b;
  histT += dt; const push = histT > 0.045; if (push) histT = 0;
  let seg = 0;
  for (let i = 0; i < NP; i++) {
    vel(pX[i], pY[i], pZ[i], vv); const ux = vv[0], uy = vv[1], uz = vv[2];
    pX[i] += ux * sp; pY[i] += uy * sp; pZ[i] += uz * sp; pA[i] += dt;
    if (pX[i] > xEnd || Math.abs(pY[i]) > b || Math.abs(pZ[i]) > b * 0.6) seed(i, false);
    const s = Math.hypot(ux, uy, uz);
    if (push || hCount[i] === 0) { const h = (hHead[i] + 1) % HN; hHead[i] = h; const o = i * HN + h; hX[o] = pX[i]; hY[o] = pY[i]; hZ[o] = pZ[i]; hS[o] = s; if (hCount[i] < HN) hCount[i]++; }
    else { const o = i * HN + hHead[i]; hX[o] = pX[i]; hY[o] = pY[i]; hZ[o] = pZ[i]; hS[o] = s; }
    const n = hCount[i];
    for (let k = 0; k < HN - 1; k++) {
      const q = seg * 6, qc = seg * 8; seg++;
      if (k >= n - 1) { lp[q] = lp[q + 3] = pX[i]; lp[q + 1] = lp[q + 4] = pY[i]; lp[q + 2] = lp[q + 5] = pZ[i]; lc.fill(0, qc, qc + 8); continue; }
      const a = i * HN + (hHead[i] - k + HN) % HN, c = i * HN + (hHead[i] - k - 1 + HN) % HN;
      lp[q] = hX[a]; lp[q + 1] = hY[a]; lp[q + 2] = hZ[a]; lp[q + 3] = hX[c]; lp[q + 4] = hY[c]; lp[q + 5] = hZ[c];
      colAt(hS[a], (1 - k / (HN - 1)) * Math.min(1, pA[i] * 3), lc, qc); colAt(hS[c], (1 - (k + 1) / (HN - 1)) * Math.min(1, pA[i] * 3), lc, qc + 4);
    }
  }
  lineGeo.attributes.position.needsUpdate = true; lineGeo.attributes.color.needsUpdate = true;
}

// ====================================================== precipitation (world space, relative to the aircraft)
const NQ = 3600, qP = new Float32Array(NQ * 3), qGeo = new THREE.BufferGeometry(), qPos = new Float32Array(NQ * 6), qCol = new Float32Array(NQ * 8);
qGeo.setAttribute('position', new THREE.BufferAttribute(qPos, 3).setUsage(THREE.DynamicDrawUsage)); qGeo.setAttribute('color', new THREE.BufferAttribute(qCol, 4).setUsage(THREE.DynamicDrawUsage));
const precip = new THREE.LineSegments(qGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false })); precip.frustumCulled = false; scene.add(precip);
let qBox = 30; for (let i = 0; i < NQ; i++) { qP[i * 3] = (Math.random() - 0.5) * 2; qP[i * 3 + 1] = (Math.random() - 0.5) * 2; qP[i * 3 + 2] = (Math.random() - 0.5) * 2; }
function stepPrecip(dt) {
  const kind = WX[S.wx].precip; precip.visible = !!kind; if (!kind) return;
  const B = qBox = 1.3 * L_ref, g = S.mode === 'fly' ? st.gam : 0, V = S.mode === 'park' ? 0 : st.V;
  const fall = kind === 'snow' ? 1.3 : kind === 'rain' ? 7 : 0.05, wob = kind === 'snow' ? 0.8 : 0;
  const vx = -V * Math.cos(g), vy = -fall - V * Math.sin(g), streak = kind === 'rain' ? 0.045 : kind === 'snow' ? 0.035 : 0.03;
  const col = kind === 'rain' ? [0.78, 0.86, 0.95, 0.55] : kind === 'snow' ? [1, 1, 1, 0.9] : [0.95, 0.97, 1, 0.35];
  const n = kind === 'cloud' ? NQ : kind === 'snow' ? NQ : (NQ * 0.7) | 0;
  const t = performance.now() / 1000;
  for (let i = 0; i < NQ; i++) {
    const o = i * 3, q = i * 6, c = i * 8;
    if (i >= n) { qCol.fill(0, c, c + 8); continue; }
    let x = qP[o] * B, y = qP[o + 1] * B * 0.6, z = qP[o + 2] * B;
    const wx = wob * Math.sin(t * 1.7 + i), wz = wob * Math.cos(t * 1.3 + i * 0.7);
    x += (vx + wx) * dt; y += vy * dt; z += wz * dt;
    if (x < -B) x += 2 * B; if (x > B) x -= 2 * B; if (y < -0.6 * B) y += 1.2 * B; if (y > 0.6 * B) y -= 1.2 * B; if (z < -B) z += 2 * B; if (z > B) z -= 2 * B;
    qP[o] = x / B; qP[o + 1] = y / (B * 0.6); qP[o + 2] = z / B;
    const len = Math.max(kind === 'snow' ? 0.035 : 0.02, streak);
    qPos[q] = x; qPos[q + 1] = y; qPos[q + 2] = z; qPos[q + 3] = x - (vx + wx) * len - (kind === 'snow' ? 0.02 : 0); qPos[q + 4] = y - vy * len; qPos[q + 5] = z - wz * len;
    qCol[c] = col[0]; qCol[c + 1] = col[1]; qCol[c + 2] = col[2]; qCol[c + 3] = col[3]; qCol[c + 4] = col[0]; qCol[c + 5] = col[1]; qCol[c + 6] = col[2]; qCol[c + 7] = 0;
  }
  qGeo.attributes.position.needsUpdate = true; qGeo.attributes.color.needsUpdate = true;
}

// ====================================================== camera
const cam = { yaw: 0, pitch: 0, zoom: 1, shake: 0 };
const glassCentre = () => { const g = meshes[1] && meshes[1].geometry; if (!g || !g.attributes.position.count) return new THREE.Vector3(SL.P.body.xnose + 0.2 * SL.P.body.lf, 0, 0); g.computeBoundingBox(); const c = new THREE.Vector3(); g.boundingBox.getCenter(c); return new THREE.Vector3(-(c.x - xcg), c.z, c.y); };
const PRESET = {
  chase: L => [new THREE.Vector3(-1.25 * L, 0.42 * L, -0.55 * L), new THREE.Vector3(0.05 * L, 0, 0)],
  side: L => [new THREE.Vector3(0.05 * L, 0.04 * L, -1.9 * L), new THREE.Vector3(0.05 * L, 0, 0)],
  front: L => [new THREE.Vector3(1.4 * L, 0.18 * L, 0.5 * L), new THREE.Vector3(0, 0, 0)],
  top: L => [new THREE.Vector3(-0.4 * L, 1.7 * L, 0.01), new THREE.Vector3(-0.1 * L, 0, 0)],
  tip: L => [new THREE.Vector3(-1.2 * L, 0.12 * L, 0.85 * SL.P.wing.b), new THREE.Vector3(-0.5 * L, 0, 0.42 * SL.P.wing.b)],
  close: L => { const g = glassCentre(); const t = g.clone().add(new THREE.Vector3(-0.04 * L, -0.02 * L, 0.06 * L)); return [g.clone().add(new THREE.Vector3(0.34 * L, -0.03 * L, -0.24 * L)), t]; },
};
const camPos = new THREE.Vector3(-30, 8, -10), camTgt = new THREE.Vector3();
let drag = null;
canvas.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, yaw: cam.yaw, pitch: cam.pitch }; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener('pointermove', e => { if (!drag) return; cam.yaw = drag.yaw - (e.clientX - drag.x) * 0.006; cam.pitch = clamp(drag.pitch + (e.clientY - drag.y) * 0.005, -1.2, 1.2); });
canvas.addEventListener('pointerup', () => drag = null); canvas.addEventListener('pointercancel', () => drag = null);
canvas.addEventListener('wheel', e => { e.preventDefault(); cam.zoom = clamp(cam.zoom * Math.exp(e.deltaY * 0.001), 0.25, 3); }, { passive: false });
function setView(v) { S.view = v; cam.yaw = 0; cam.pitch = 0; cam.zoom = 1; document.querySelectorAll('#views button').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.v === v))); }
function updateCamera(dt) {
  const [p, t] = PRESET[S.view](L_ref), off = p.clone().sub(t);
  const sph = new THREE.Spherical().setFromVector3(off); sph.radius *= cam.zoom; sph.theta += cam.yaw; sph.phi = clamp(sph.phi - cam.pitch, 0.05, Math.PI - 0.05);
  const want = new THREE.Vector3().setFromSpherical(sph).add(t);
  if (S.mode === 'park') want.y = Math.max(want.y, groundY + 1.2);
  const k = 1 - Math.exp(-dt * 3.5); camPos.lerp(want, k); camTgt.lerp(t, k);
  camera.position.copy(camPos); camera.lookAt(camTgt);
  if (cam.shake > 0) { camera.position.x += (Math.random() - 0.5) * cam.shake; camera.position.y += (Math.random() - 0.5) * cam.shake; }
}
function resize() { const w = canvas.clientWidth, h = canvas.clientHeight; if (!w || !h) return; if (canvas.width !== Math.round(w * R.getPixelRatio())) { R.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); } }

// ====================================================== world
let travel = 0, groundY = -1, lastEnvH = -1e9;
function updateWorld(dt) {
  const park = S.mode === 'park', h = park ? 0 : st.h, V = park ? 0 : st.V, g = S.mode === 'fly' ? st.gam : 0;
  const th = S.mode === 'fly' ? st.alpha + st.gam : S.mode === 'tunnel' ? st.alpha : 0;
  plane.rotation.z = th;
  const f = clamp(h / 12000, 0, 1), over = S.wx === 'clear' || S.wx === 'frost' ? 0 : S.wx === 'cloud' ? 0.85 : 0.7;
  skyU.top.value.setRGB(0.16 - 0.1 * f, 0.43 - 0.25 * f, 0.75 - 0.3 * f); skyU.hor.value.setRGB(0.81 - 0.2 * f, 0.89 - 0.15 * f, 0.95 - 0.08 * f); skyU.overcast.value = over;
  sun.intensity = 3.2 * (1 - 0.6 * over); hemi.intensity = 1.4 + 0.4 * over;
  const fogC = new THREE.Color().copy(skyU.hor.value).lerp(new THREE.Color(0.78, 0.8, 0.83), over);
  scene.fog.color.copy(fogC);
  scene.fog.near = S.wx === 'cloud' ? 25 : over ? 200 : 1200; scene.fog.far = S.wx === 'cloud' ? 320 : over ? 2500 : 12000;
  if (Math.abs(h - lastEnvH) > 1500 || over !== lastOver) { lastEnvH = h; lastOver = over; refreshEnv(); }
  const adv = S.mode === 'fly' ? V * dt : V * dt * 0.15; travel += adv * Math.cos(g);
  // ground: far below in flight; under the wheels when parked
  groundY = park ? modelMinY() : -h; ground.position.y = groundY - (park ? 0.02 : 0); ground.visible = h < 8000 || park;
  gTex.offset.x = (travel / 1500) % 1;
  const snowy = S.wx === 'snow' || S.wx === 'frost';
  const cover = snowy && park ? clamp((st.snowMean || 0) / 0.6e-3, S.wx === 'frost' ? 0.25 : 0.35, S.wx === 'frost' ? 0.45 : 0.85) : 0;
  groundMat.color.setScalar(1 - 0.7 * cover); groundMat.emissive.setRGB(0.62 * cover, 0.65 * cover, 0.7 * cover);
  runway.visible = park; runway.position.set(0, groundY + 0.01, 0); runway.scale.set(Math.max(60, 3 * L_ref), Math.max(30, 1.6 * SL.P.wing.b), 1);
  runwayMat.color.setScalar(1 - 0.5 * cover); runwayMat.emissive.setRGB(0.45 * cover, 0.47 * cover, 0.5 * cover);
  for (const m of puffs) {
    const u = m.userData; u.x -= adv * Math.cos(g); if (u.x < -6000) u.x += 12000;
    m.position.set(u.x, u.alt - h, u.z);
    const d = Math.hypot(u.x, u.alt - h, u.z); m.material.opacity = 0.9 * clamp((d - 350) / 700, 0, 1) * (park ? 0.6 : 1);
  }
  for (const h of props) h.rotation.x += dt * (S.mode === 'park' ? 0 : ac.prop ? 38 : 12);
}
let lastOver = -1;
let minYcache = { key: '', y: -1 };
function modelMinY() { if (minYcache.key === S.key) return minYcache.y; const g = meshes[0].geometry; g.computeBoundingBox(); minYcache = { key: S.key, y: g.boundingBox.min.z }; return minYcache.y; }

// ====================================================== live 2D wing section (lattice Boltzmann)
// Each aircraft's own wing section, simulated live. Colour is the simulated pressure (from the lattice density),
// white streaks are smoke; overlays mark the airflow direction, lift, the stagnation point and flow separation.
const SECTIONS = {
  c172: { kind: 'naca', m: 0.02, p: 0.4, t: 0.12, name: 'NACA 2412', where: 'Cessna 172 wing' },
  spitfire: { kind: 'naca', m: 0.02, p: 0.2, t: 0.13, name: 'NACA 2213', where: 'Spitfire wing root (thins to 9.4% at the tip)' },
  b738: { kind: 'sc', t: 0.12, name: 'Supercritical section', where: '737 wing, approximate: Boeing’s exact sections are not published' },
  a320: { kind: 'sc', t: 0.12, name: 'Supercritical section', where: 'A320 wing, approximate: Airbus’s exact sections are not published' },
};
function sectionShape() {
  const s = SECTIONS[S.key];
  if (s && s.kind === 'sc') {   // flat-topped, aft-loaded profile with a cusped lower trailing edge (SC(2)-like)
    const t = s.t, yt = x => 5 * t * (0.2969 * Math.sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4) * (1 + 0.18 * Math.sin(Math.PI * x));
    const yc = x => 0.12 * x ** 2.4 * (1 - x) * (1 + x) - 0.004 * Math.sin(Math.PI * x) ** 2;
    return { ...s, up: x => yc(x) + yt(x) * 0.9, lo: x => yc(x) - yt(x) * 1.1 + 0.035 * Math.max(0, x - 0.55) ** 2 * (x - 0.55) / 0.45 * 2 };
  }
  const n = s || (() => { const q = SL.section(); return { kind: 'naca', m: q.m / 100, p: q.p / 10, t: q.t / 100, name: `NACA ${SL.P.wing.sec}`, where: 'wing' }; })();
  return { ...n, up: x => SL.naca.yc(n.m, n.p, x) + SL.naca.yt(n.t, x), lo: x => SL.naca.yc(n.m, n.p, x) - SL.naca.yt(n.t, x) };
}
const secCv = $('sec'), sx = secCv.getContext('2d');
const SNX = 400, SNY = 180, SU0 = 0.12, SL_ = 84, sw = new Worker('wing-worker.js?v=10');
const sFieldC = document.createElement('canvas'); sFieldC.width = SNX; sFieldC.height = SNY; const sfx = sFieldC.getContext('2d'), sImg = sfx.createImageData(SNX, SNY);
let sField = null, sMask = null, sPolys = [], sKey = '', sStarted = false, sHist = [], sShape = null, sAl = 0, sSep = 1, sView = 'p';
const sGeom = al => ({ px: 0.24 * SNX + 0.25 * SL_, py: SNY / 2 + 6, L: SL_, aoa: al, ox: 0.25 });
const secTf = (al) => { const gi = sGeom(al), a = -al * D2R, ca = Math.cos(a), sa = Math.sin(a); return ([x, y]) => { const dx = x - 0.25; return [gi.px + SL_ * (dx * ca - y * sa), gi.py + SL_ * (dx * sa + y * ca)]; }; };
function sectionPolys(al) {
  const sh = sectionShape(), tf = secTf(al), N = 120, up = [], lo = [];
  for (let i = 0; i <= N; i++) { const x = 0.5 * (1 - Math.cos(Math.PI * i / N)); up.push([x, sh.up(x)]); lo.push([x, sh.lo(x)]); }
  const body = up.slice().reverse().concat(lo.slice(1)), polys = [body.map(tf)], kc = st.k / ac.c;
  sShape = { sh, up: up.map(tf), body };
  if (kc > 2e-4) {   // ice shape: rime hugs the leading edge, glaze grows horns
    const horn = st.kind === 'glaze' ? 1 : st.kind === 'mixed' ? 0.5 : 0, outer = [], inner = [];
    for (let i = 0; i <= 60; i++) {
      const s2 = -1 + i / 30, u = 0.1 * s2 * s2, upS = s2 >= 0, f = upS ? sh.up : sh.lo, y = f(u), u2 = u + 1e-3, y2 = f(u2);
      let nx = -(y2 - y), ny = u2 - u; if (!upS) { nx = -nx; ny = -ny; }
      const nl = Math.hypot(nx, ny) || 1, aa = Math.abs(s2), th = Math.min(0.08, kc * 1.4) * ((1 - horn) * Math.exp(-((aa / 0.55) ** 2)) + horn * (0.35 * Math.exp(-((aa / 0.4) ** 2)) + 1.1 * Math.exp(-(((aa - 0.55) / 0.18) ** 2))));
      outer.push(tf([u + nx / nl * th, y + ny / nl * th])); inner.push(tf([u - nx / nl * 0.004, y - ny / nl * 0.004]));
    }
    polys.push(outer.concat(inner.reverse()));
  }
  return polys;
}
function raster(ps) {
  const m = new Uint8Array(SNX * SNY), xs = [];
  for (const poly of ps) for (let y = 0; y < SNY; y++) {
    const yc = y + 0.5; xs.length = 0;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) { const [x1, y1] = poly[i], [x2, y2] = poly[j]; if ((y1 <= yc) !== (y2 <= yc)) xs.push(x1 + (yc - y1) * (x2 - x1) / (y2 - y1)); }
    xs.sort((a, b) => a - b);
    for (let i = 0; i + 1 < xs.length; i += 2) for (let x = Math.max(2, Math.ceil(xs[i] - 0.5)); x <= Math.min(SNX - 3, Math.floor(xs[i + 1] - 0.5)); x++) m[y * SNX + x] = 1;
  }
  return m;
}
function sectionUpdate(force) {
  const al = clamp(Math.round(st.alpha / D2R * 2) / 2, -5, 25), key = `${S.key}|${SL.P.wing.sec}|${al}|${Math.round(st.k * 2000)}|${st.kind}`;
  if (!force && key === sKey) return; sKey = key; sAl = al;
  sPolys = sectionPolys(al); sMask = raster(sPolys);
  const sh = sShape.sh; $('sec-name').textContent = `${sh.name} · ${sh.where}`;
  if (!sStarted) { sStarted = true; sw.postMessage({ type: 'init', nx: SNX, ny: SNY, u0: SU0, tau: 3 * SU0 * SL_ / 4000 + 0.5, clean: sMask, geom: sGeom(al) }); sw.postMessage({ type: 'frame' }); }
  else { sw.postMessage({ type: 'geom', clean: sMask, geom: sGeom(al) }); sHist.length = 0; }
}
sw.onmessage = e => { const m = e.data; if (m.type !== 'field') { if (m.type !== 'blowup') sw.postMessage({ type: 'frame' }); return; } sField = m; if (m.fn && m.steps * SU0 / SL_ > 0.6) { sHist.push(m.Fy / (0.5 * SU0 * SU0 * SL_)); if (sHist.length > 90) sHist.shift(); } setTimeout(() => sw.postMessage({ type: 'frame' }), 30); };
document.querySelectorAll('#sec-view button').forEach(b => b.addEventListener('click', () => { sView = b.dataset.v; document.querySelectorAll('#sec-view button').forEach(x => x.setAttribute('aria-pressed', String(x === b))); }));
// pressure colours: suction (air speeding up) blue → cyan, pressure (air slowing) amber → red
const CP_NEG = [[38, 52, 70], [32, 92, 170], [70, 170, 235], [190, 240, 255]], CP_POS = [[38, 52, 70], [170, 110, 60], [235, 150, 60], [255, 220, 150]];
const ramp = (st_, t) => { t = clamp(t, 0, 1) * (st_.length - 1); const i = Math.min(st_.length - 2, Math.floor(t)), f = t - i, a = st_[i], b = st_[i + 1]; return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f]; };
function arrow(x0, y0, x1, y1, col, w) { const a = Math.atan2(y1 - y0, x1 - x0), h = 4 + 2.2 * w; sx.strokeStyle = sx.fillStyle = col; sx.lineWidth = w; sx.beginPath(); sx.moveTo(x0, y0); sx.lineTo(x1 - Math.cos(a) * h * 0.6, y1 - Math.sin(a) * h * 0.6); sx.stroke(); sx.beginPath(); sx.moveTo(x1, y1); sx.lineTo(x1 - h * Math.cos(a - 0.45), y1 - h * Math.sin(a - 0.45)); sx.lineTo(x1 - h * Math.cos(a + 0.45), y1 - h * Math.sin(a + 0.45)); sx.closePath(); sx.fill(); }
function tag(text, x, y, col, align = 'left') { sx.font = `500 ${11 * dprS}px Figtree, system-ui, sans-serif`; sx.textAlign = align; sx.textBaseline = 'middle'; const w = sx.measureText(text).width, p = 4 * dprS; let x0 = align === 'left' ? x : align === 'right' ? x - w : x - w / 2;
  x0 = clamp(x0, p + 2, secCv.width - w - p - 2); y = clamp(y, 10 * dprS, secCv.height - 10 * dprS); x = align === 'left' ? x0 : align === 'right' ? x0 + w : x0 + w / 2;
  sx.fillStyle = 'rgba(8,14,22,.62)'; sx.beginPath(); sx.roundRect ? sx.roundRect(x0 - p, y - 8 * dprS, w + 2 * p, 16 * dprS, 4 * dprS) : sx.rect(x0 - p, y - 8 * dprS, w + 2 * p, 16 * dprS); sx.fill(); sx.fillStyle = col; sx.fillText(text, x, y); }
let dprS = 1;
const SK = { rows: [], t: 0, key: '' };
function streaks(dt, ux, uy, cx0, cy0, cw, ch) {
  const nRows = 17, y0 = cy0 + 2, y1 = cy0 + ch - 2, speed = cw / 2.4 / SU0;   // free stream crosses the view in 2.4 s
  if (SK.rows.length !== nRows || SK.key !== S.key) { SK.rows = Array.from({ length: nRows }, () => []); SK.key = S.key; }
  const samp = (x, y) => { x -= 0.5; y -= 0.5; const i = Math.floor(x), j = Math.floor(y); if (i < 0 || j < 0 || i >= SNX - 1 || j >= SNY - 1) return null; const a = x - i, b = y - j, k = j * SNX + i;
    if (sMask[k] && sMask[k + 1] && sMask[k + SNX] && sMask[k + SNX + 1]) return null;
    const w00 = (1 - a) * (1 - b), w10 = a * (1 - b), w01 = (1 - a) * b, w11 = a * b; return [ux[k] * w00 + ux[k + 1] * w10 + ux[k + SNX] * w01 + ux[k + SNX + 1] * w11, uy[k] * w00 + uy[k + 1] * w10 + uy[k + SNX] * w01 + uy[k + SNX + 1] * w11]; };
  SK.t += dt; const emit = SK.t > 0.04; if (emit) SK.t = 0;
  for (let r = 0; r < nRows; r++) {
    const row = SK.rows[r]; if (emit) row.push([cx0 - 3, y0 + (y1 - y0) * (r + 0.5) / nRows]);
    for (let i = row.length - 1; i >= 0; i--) {
      const p = row[i]; let dead = false;
      for (let sub = 0; sub < 3 && !dead; sub++) { const u = samp(p[0], p[1]); if (!u) { dead = true; break; } p[0] += u[0] * speed * dt / 3; p[1] += u[1] * speed * dt / 3; }
      if (dead || p[0] > cx0 + cw + 3 || p[1] < cy0 - 6 || p[1] > cy0 + ch + 6) row.splice(i, 1);
    }
    if (row.length > 260) row.splice(0, row.length - 260);
  }
}
function drawSection() {
  if (!sField || !sMask) return;
  const W = secCv.clientWidth, H = secCv.clientHeight, dpr = dprS = Math.min(2, window.devicePixelRatio || 1);
  if (!W || !H) return;
  if (secCv.width !== Math.round(W * dpr) || secCv.height !== Math.round(H * dpr)) { secCv.width = Math.round(W * dpr); secCv.height = Math.round(H * dpr); }
  const { ux, uy, rho } = sField, d = sImg.data, q = 1.5 * SU0 * SU0, now = performance.now(), fdt = Math.min(0.05, (now - (drawSection.t || now)) / 1000); drawSection.t = now;
  if (drawSection.f !== sField || drawSection.v !== sView) { drawSection.f = sField; drawSection.v = sView;
  for (let y = 0; y < SNY; y++) for (let x = 0; x < SNX; x++) {
    let k = y * SNX + x; const o = ((SNY - 1 - y) * SNX + x) * 4;
    if (sMask[k]) { // fill the body with nearby fluid so the smooth outline has no dark halo
      let kk = -1; for (let r = 1; r < 6 && kk < 0; r++) for (const [a, b] of [[0, r], [0, -r], [r, 0], [-r, 0]]) { const j = (y + b) * SNX + x + a; if (j >= 0 && j < SNX * SNY && !sMask[j]) { kk = j; break; } }
      if (kk < 0) { d[o] = 38; d[o + 1] = 52; d[o + 2] = 70; d[o + 3] = 255; continue; } k = kk;
    }
    let c;
    if (sView === 'p') { const cp = (rho[k] - 1) / q; c = cp < 0 ? ramp(CP_NEG, Math.sqrt(-cp / 2)) : ramp(CP_POS, Math.sqrt(cp)); }
    else c = SL.mapRGB('viridis', Math.hypot(ux[k], uy[k]) / SU0 / 1.8);
    d[o] = c[0]; d[o + 1] = c[1]; d[o + 2] = c[2]; d[o + 3] = 255;
  }
  sfx.putImageData(sImg, 0, 0); }
  // viewport in grid cells: about 2 chords wide around the section
  const cw = 1.75 * SL_, ch = cw * secCv.height / secCv.width, cx0 = sGeom(0).px - 0.25 * SL_ - 0.3 * SL_, cy0 = sGeom(0).py - ch * 0.5;
  const toS = ([x, y]) => [(x - cx0) / cw * secCv.width, (1 - (y - cy0) / ch) * secCv.height], sc = secCv.width / cw;
  sx.imageSmoothingEnabled = true; sx.imageSmoothingQuality = 'high'; sx.drawImage(sFieldC, cx0, SNY - cy0 - ch, cw, ch, 0, 0, secCv.width, secCv.height);
  // smoke: streaklines released from fixed points upstream and carried by the simulated velocity field
  streaks(fdt, ux, uy, cx0, cy0, cw, ch);
  sx.lineWidth = 1.25 * dpr; sx.lineJoin = sx.lineCap = 'round'; sx.strokeStyle = sView === 'p' ? 'rgba(255,255,255,.78)' : 'rgba(255,255,255,.7)';
  sx.beginPath();
  for (const row of SK.rows) { let prev = null; for (const pt of row) { const [a, b] = toS(pt); if (prev && Math.hypot(pt[0] - prev[0], pt[1] - prev[1]) < 9) sx.lineTo(a, b); else sx.moveTo(a, b); prev = pt; } }
  sx.stroke();
  const path = poly => { sx.beginPath(); poly.forEach((p, i) => { const [a, b] = toS(p); i ? sx.lineTo(a, b) : sx.moveTo(a, b); }); sx.closePath(); };
  // wing (and ice) as smooth vector shapes
  path(sPolys[0]); const gr = sx.createLinearGradient(0, toS([0, sGeom(0).py + 0.1 * SL_])[1], 0, toS([0, sGeom(0).py - 0.1 * SL_])[1]); gr.addColorStop(0, '#c9d3de'); gr.addColorStop(1, '#8e9aa8'); sx.fillStyle = gr; sx.fill(); sx.lineWidth = 1.2 * dpr; sx.strokeStyle = 'rgba(255,255,255,.85)'; sx.stroke();
  if (sPolys.length > 1) { path(sPolys[1]); sx.fillStyle = st.kind === 'glaze' ? 'rgba(200,235,255,.9)' : '#f2f8ff'; sx.fill(); sx.strokeStyle = 'rgba(160,200,230,.9)'; sx.lineWidth = dpr; sx.stroke(); }
  // chord line and angle of attack
  const tf = secTf(sAl), le = toS(tf([0, 0])), te = toS(tf([1, 0]));
  sx.setLineDash([4 * dpr, 4 * dpr]); sx.strokeStyle = 'rgba(255,255,255,.45)'; sx.lineWidth = dpr; sx.beginPath(); sx.moveTo(le[0] - 0.12 * SL_ * sc * Math.cos(sAl * D2R), le[1] - 0.12 * SL_ * sc * Math.sin(sAl * D2R)); sx.lineTo(te[0], te[1]); sx.stroke(); sx.setLineDash([]);
  const ay = 14 * dpr + 8 * dpr; arrow(10 * dpr, ay, 10 * dpr + 0.34 * SL_ * sc, ay, 'rgba(255,255,255,.9)', 2 * dpr); tag(`airflow  ${fmt(st.alpha / D2R, 1)}°`, 16 * dpr + 0.34 * SL_ * sc, ay, '#e8eef5');
  // simulated lift: arrow at the quarter chord, perpendicular to the airflow
  const cl = SL.mean(sHist, 40), qc = toS(tf([0.25, sShape.sh.up(0.25)]));
  if (sHist.length > 20) { const Lp = clamp(cl, -0.6, 2.2) * 0.32 * SL_ * sc; if (Math.abs(Lp) > 3 * dpr) { arrow(qc[0], qc[1] - 4 * dpr, qc[0], qc[1] - 4 * dpr - Lp, '#ffd36a', 3 * dpr); tag(`lift ${fmt(cl, 2)}`, qc[0] + 7 * dpr, qc[1] - 4 * dpr - Lp + 6 * dpr, '#ffd36a'); } }
  // stagnation point: highest pressure next to the nose
  let best = -1e9, sp = null; for (const p of sPolys[0]) { const [gx, gy] = p; if (Math.hypot(gx - tf([0, 0])[0], gy - tf([0, 0])[1]) > 0.12 * SL_) continue; const k = (Math.round(gy) * SNX + Math.round(gx - 2)); const r = rho[k] || 0; if (!sMask[k] && r > best) { best = r; sp = p; } }
  if (sp && sView === 'p') { const [a, b] = toS(sp); sx.fillStyle = '#ffb05a'; sx.beginPath(); sx.arc(a, b, 3.5 * dpr, 0, 7); sx.fill(); tag('stagnation', a - 4 * dpr, b + 16 * dpr, '#ffcf9a', 'center'); }
  // separation: first point along the upper surface (aft of 5% chord) where the air next to it flows backwards
  const cd = [Math.cos(-sAl * D2R), Math.sin(-sAl * D2R)]; let sepX = 1;
  for (let i = 0; i < sShape.up.length; i++) {
    const x = 0.5 * (1 - Math.cos(Math.PI * i / 120)); if (x < 0.05 || x > 0.97) continue;
    const [gx, gy] = sShape.up[i], nX = -cd[1], nY = cd[0];   // surface normal ≈ chord normal
    let rev = 0; for (const h of [2, 3.5]) { const k = Math.round(gy + nY * h) * SNX + Math.round(gx + nX * h); if (!sMask[k] && ux[k] * cd[0] + uy[k] * cd[1] < -0.02 * SU0) rev++; }
    if (rev === 2) { sepX = x; break; }
  }
  sSep += (sepX - sSep) * 0.15;
  const separated = sSep < 0.75;
  if (separated) { const i = Math.round(Math.acos(1 - 2 * clamp(sSep, 0, 1)) / Math.PI * 120), [a, b] = toS(sShape.up[clamp(i, 0, 120)]); sx.fillStyle = '#ff6b5e'; sx.beginPath(); sx.arc(a, b, 4 * dpr, 0, 7); sx.fill(); tag(sSep < 0.2 ? 'separated from the nose' : `separates at ${Math.round(sSep * 100)}% chord`, a + 6 * dpr, b - 15 * dpr, '#ffa59c'); }
  else if (sView === 'p' && sHist.length > 20 && cl > 0.15) { const p = toS(tf([0.72, sShape.sh.up(0.72) + 0.14])); tag('low pressure', p[0], p[1], '#a9dcff', 'center'); const p2 = toS(tf([0.62, sShape.sh.lo(0.62) - 0.14])); tag('high pressure', p2[0], p2[1], '#ffd2a0', 'center'); }
  // legend
  const lx = secCv.width - 118 * dpr, ly = secCv.height - 14 * dpr, lw = 100 * dpr, lg = sx.createLinearGradient(lx, 0, lx + lw, 0);
  if (sView === 'p') { [[0, CP_NEG[3]], [0.33, CP_NEG[2]], [0.62, CP_NEG[0]], [0.82, CP_POS[2]], [1, CP_POS[3]]].forEach(([t, c]) => lg.addColorStop(t, `rgb(${c.map(Math.round)})`)); }
  else for (let t = 0; t <= 1; t += 0.25) lg.addColorStop(t, `rgb(${SL.mapRGB('viridis', t).join(',')})`);
  sx.fillStyle = 'rgba(8,14,22,.6)'; sx.fillRect(lx - 6 * dpr, ly - 18 * dpr, lw + 12 * dpr, 26 * dpr); sx.fillStyle = lg; sx.fillRect(lx, ly - 2 * dpr, lw, 6 * dpr);
  sx.font = `500 ${9.5 * dpr}px Figtree, system-ui, sans-serif`; sx.fillStyle = '#cfd8e2'; sx.textBaseline = 'alphabetic'; sx.textAlign = 'left'; sx.fillText(sView === 'p' ? 'suction' : 'slow', lx, ly - 6 * dpr); sx.textAlign = 'right'; sx.fillText(sView === 'p' ? 'pressure' : 'fast', lx + lw, ly - 6 * dpr);
  // header note and caption
  $('sec-note').textContent = separated ? (sSep < 0.2 ? 'stalled' : 'flow separating') : 'attached flow'; $('sec-note').style.color = separated ? 'var(--bad)' : 'var(--good)';
  const capKey = separated ? 'sep' : sPolys.length > 1 ? 'ice' : 'ok';
  if (drawSection.cap !== capKey) { drawSection.cap = capKey; $('sec-cap').innerHTML = capKey === 'sep' ? 'The air can no longer follow the top surface. It breaks away into a swirling wake, the suction collapses and lift falls: a stall.'
    : capKey === 'ice' ? 'Ice has reshaped the leading edge. The air trips over it, separates sooner, and the wing stalls at a lower angle.'
    : 'Air speeds up over the top and its pressure drops (blue); underneath it slows and pressure rises (orange). The difference is lift. Simulated live at reduced scale.'; }
}

// ====================================================== controls
const kt = v => Math.round(v * 1.944);
function syncControls() {
  const sp = $('c-speed'); sp.min = Math.max(10, Math.round(ac.vmin * 0.8)); sp.max = Math.round(ac.vmax); sp.step = 1; sp.value = AP.vt;
  $('c-speed-o').textContent = `${kt(AP.vt)} kt`;
  $('c-alt').value = AP.ht; $('c-alt-o').textContent = `${int(AP.ht)} m`;
  $('c-aoa').value = S.aoa; $('c-aoa-o').textContent = S.mode === 'tunnel' ? `${fmt(S.aoa, 1)}°` : 'auto'; $('c-aoa').disabled = S.mode !== 'tunnel';
  const hR = S.mode === 'park' ? 0 : AP.ht; $('c-dt').value = Math.round(OAT(hR)); $('c-dt-o').textContent = `${fmt(OAT(hR), 0)} °C`; wxStatus();
  for (const [id, k] of [['t-trails', 'trails'], ['t-smoke', 'smoke'], ['t-lift', 'liftmap'], ['t-icehl', 'icehl']]) $(id).checked = S[k];
  $('c-takeoff').hidden = S.mode !== 'park';
  const sel = (id, v, attr = 'v') => document.querySelectorAll(`#${id} button`).forEach(b => b.setAttribute('aria-pressed', String(b.dataset[attr] === String(v))));
  sel('mode', S.mode); sel('c-time', S.ts); sel('wx', S.wx); sel('aircraft', S.key, 'k');
}
async function setAircraft(k) {
  S.key = k; SL.loadProject(k); base = JSON.parse(JSON.stringify(SL.P));
  AP.ht = SL.P.flight.h; AP.vt = SL.P.flight.V;
  rebuildAircraft(); newState();
  try { const M = await loadModel(k); if (S.key !== k) return; buildModel(M); } catch (e) { console.error(e); toast(e.message); }
  requestField(st.alpha); sectionUpdate(true); syncControls(); infoCard();
}
function setMode(m) {
  const prev = S.mode; S.mode = m;
  if (m === 'tunnel') { S.aoa = Math.round(st.alpha / D2R * 2) / 2; st.gam = 0; st.mode = 'hold'; }
  if (m === 'park') { st.V = 0; st.gam = 0; st.alpha = 0; setView(S.view === 'tip' ? 'chase' : S.view); }
  if (m === 'fly' && prev === 'park') { st.h = Math.max(st.h, AP.ht); st.V = AP.vt; }
  syncControls();
}
function takeOff() {
  const X0 = aeroOf(0, 'rime', 0), atm = SL.isa(150), Vs0 = Math.sqrt(2 * ac.m * G0 / (atm.rho * ac.S * X0.CLmax));
  S.mode = 'fly'; st.h = 150; st.V = 1.2 * Vs0; st.gam = 0.08; st.mode = 'hold'; AP.vt = Math.round(1.35 * Vs0); AP.ht = 900;
  if (st.contam > 0.15) toast(`Rotating at ${kt(st.V)} kt, the normal take-off speed for a clean wing. With the wing ${st.cType === 'frost' ? 'frosted' : 'covered in snow'}, maximum lift is ${Math.round(30 * st.contam)}% lower.`, 6500);
  syncControls();
}
const on = (id, ev, fn) => $(id).addEventListener(ev, fn);
on('aircraft', 'click', e => { const b = e.target.closest('.chip'); if (b) { stopExp(); setAircraft(b.dataset.k); } });
on('views', 'click', e => { const b = e.target.closest('button'); if (b) setView(b.dataset.v); });
on('mode', 'click', e => { const b = e.target.closest('button'); if (b) setMode(b.dataset.v); });
on('wx', 'click', e => { const b = e.target.closest('button'); if (b) chooseWx(b.dataset.v); });
const setOAT = (T, hR = S.mode === 'park' ? 0 : AP.ht) => { S.dT = T - SL.isa(hR).Tc; };
function chooseWx(w) {
  S.wx = w; const hR = S.mode === 'park' ? 0 : AP.ht, T = OAT(hR), msgs = [];
  if (w === 'cloud' && (T >= -2 || T < -20)) { setOAT(-8); msgs.push('temperature set to −8 °C, where cloud droplets stay liquid below freezing'); }
  if (w === 'frz' && (T >= -1 || T < -10)) { setOAT(-3); msgs.push('temperature set to −3 °C: freezing rain needs a thin sub-zero layer'); }
  if ((w === 'snow' || w === 'frost') && S.mode === 'park' && T > -1) { setOAT(-4, 0); msgs.push('temperature set to −4 °C'); }
  if ((w === 'cloud' || w === 'frz' || ((w === 'snow' || w === 'frost') && S.mode === 'park')) && S.ts < 60) { S.ts = 60; msgs.push('time sped up 60× so you can watch it build'); }
  if (w === 'clear') S.ts = 1;
  if (msgs.length) toast(`${WX[w].name}: ${msgs.join('; ')}.`, 5500);
  syncControls();
}
// plain-language summary of what the chosen weather is doing to the aircraft, with a one-click fix
let wxFix = null;
function wxStatus() {
  if (!st) return;
  const el = $('wx-status'), Tc = OAT(hNow()), r = iceRate() * 60e3, w = S.wx; let txt, cls = '', fix = null;
  const T = `${fmt(Tc, 0)} °C`;
  if (w === 'clear') txt = 'Clear air: nothing to freeze. Pick <b>Icing cloud</b> or <b>Freezing rain</b> to grow ice, or <b>Snow</b> while parked.';
  else if (w === 'cloud' || w === 'frz') {
    if (S.mode === 'park') { txt = 'Icing needs airflow. Switch to <b>Fly</b> or <b>Wind tunnel</b>.'; fix = ['Fly', () => setMode('fly')]; cls = 'warn'; }
    else if (Tc >= 0) { txt = `${T} is above freezing: the water stays liquid, so no ice.`; fix = [w === 'frz' ? 'Set −3 °C' : 'Set −8 °C', () => setOAT(w === 'frz' ? -3 : -8)]; cls = 'warn'; }
    else if (r <= 0) { txt = w === 'frz' ? `${T} is too cold for freezing rain: it would fall as snow.` : `${T}: the cloud has frozen to ice crystals, which bounce off.`; fix = [w === 'frz' ? 'Set −3 °C' : 'Set −8 °C', () => setOAT(w === 'frz' ? -3 : -8)]; cls = 'warn'; }
    else { const kd = st.kind === 'glaze' ? 'clear glaze' : st.kind === 'mixed' ? 'mixed rime and glaze' : 'white rime'; txt = `<b>Ice forming</b> · ${kd} · ${fmt(r, 1)} mm per minute of flight${S.ts > 1 ? ` (≈${fmt(r * S.ts / 60, 1)} mm/s at ${S.ts}×)` : ''} · <b>${Math.round(st.k * 1000)} mm</b> so far`; cls = 'good';
      if (S.ts < 60) fix = ['Speed up 60×', () => { S.ts = 60; }]; else if (st.k > 0.004 && !['close', 'front'].includes(S.view)) fix = ['Look closer', () => setView('front')]; else if (st.k > 0.004 && !S.icehl) fix = ['Highlight ice', () => { S.icehl = true; }]; }
  } else {
    if (S.mode !== 'park') { txt = w === 'snow' ? 'Dry snow blows straight off a flying aircraft. Park it to see snow settle.' : 'Frost forms on a parked aircraft on a cold, clear night.'; fix = ['Park', () => { setMode('park'); if (OAT(0) > -1) setOAT(-4, 0); S.ts = 60; }]; cls = 'warn'; }
    else if (Tc > 1) { txt = `${T} is too warm: ${w} melts.`; fix = ['Set −4 °C', () => setOAT(-4, 0)]; cls = 'warn'; }
    else { txt = st.contam > 0.15 ? `<b>Wing ${w === 'frost' ? 'frosted' : 'covered'}</b>: about ${Math.round(30 * st.contam)}% less lift. Try <b>Take off</b>, or <b>De-ice</b> first.` : `<b>${w === 'frost' ? 'Frost' : 'Snow'} settling</b> on the upper surfaces…`; cls = 'good'; if (S.ts < 60) fix = ['Speed up 60×', () => { S.ts = 60; }]; }
  }
  const key = txt + (fix ? fix[0] : '');
  if (el._k !== key) { el._k = key; el.className = 'wxs ' + cls; el.innerHTML = `<span>${txt}</span>` + (fix ? `<button type="button" class="btn sm">${fix[0]}</button>` : ''); wxFix = fix; const bt = el.querySelector('button'); if (bt) bt.onclick = () => { wxFix[1](); syncControls(); }; }
}
on('c-time', 'click', e => { const b = e.target.closest('button'); if (b) { S.ts = +b.dataset.v; syncControls(); } });
on('c-speed', 'input', e => { AP.vt = +e.target.value; if (S.mode === 'tunnel') st.V = AP.vt; $('c-speed-o').textContent = `${kt(AP.vt)} kt`; });
on('c-alt', 'input', e => { AP.ht = +e.target.value; if (S.mode === 'tunnel') st.h = AP.ht; $('c-alt-o').textContent = `${int(AP.ht)} m`; syncControls(); });
on('c-aoa', 'input', e => { S.aoa = +e.target.value; $('c-aoa-o').textContent = `${fmt(S.aoa, 1)}°`; });
on('c-dt', 'input', e => { const hR = S.mode === 'park' ? 0 : AP.ht; S.dT = +e.target.value - SL.isa(hR).Tc; $('c-dt-o').textContent = `${fmt(OAT(hR), 0)} °C`; wxStatus(); });
on('c-deice', 'click', () => { const b = Math.round(st.k * 1000), s = st.contam; if (b < 1 && s < 0.05) { toast('Nothing to remove.'); return; } clearSurface(S.mode === 'park' ? 0 : 0.12); toast(S.mode === 'park' ? 'De-iced on the ground: the wings are clean.' : `De-iced: leading-edge ice ${b} → ${Math.round(st.k * 1000)} mm.`); });
on('c-takeoff', 'click', takeOff);
on('t-trails', 'change', e => { S.trails = e.target.checked; buildTrails(); });
on('t-smoke', 'change', e => { S.smoke = e.target.checked; });
on('t-icehl', 'change', e => { S.icehl = e.target.checked; });
on('t-lift', 'change', e => { S.liftmap = e.target.checked; liftMap(!S.liftmap); });

// ====================================================== gauges and info
function gauges() {
  const lw = S.mode === 'park' ? 0 : O.L / O.W, X = O.X, ratio = (st.alpha - ac.alpha0) / (X.alphaS - ac.alpha0);
  $('g-lw').textContent = S.mode === 'park' ? '—' : `${Math.round(lw * 100)}%`; $('g-lw-bar').style.width = `${clamp(lw / 2, 0, 1) * 100}%`; $('g-lw-bar').style.background = lw < 0.95 ? 'var(--warn)' : 'var(--air)';
  $('g-st').textContent = O.stalled ? 'STALLED' : S.mode === 'park' ? '—' : `${Math.round(clamp(ratio, 0, 1.5) * 100)}%`;
  $('g-st-bar').style.width = `${S.mode === 'park' ? 0 : clamp(ratio, 0, 1) * 100}%`; $('g-st-bar').style.background = ratio >= 1 || O.stalled ? 'var(--bad)' : ratio > 0.8 ? 'var(--warn)' : 'var(--good)';
  $('g-v').textContent = `${kt(S.mode === 'park' ? 0 : st.V)} kt`; $('g-h').textContent = S.mode === 'park' ? 'ground' : `${int(st.h)} m`;
  $('g-vs').textContent = S.mode === 'fly' ? `${O.vs >= 0 ? '+' : '−'}${int(Math.abs(O.vs * 196.85))} fpm` : '—';
  $('g-a').textContent = `${fmt(st.alpha / D2R, 1)}°`; $('g-ld').textContent = S.mode === 'park' ? '—' : fmt(O.L / Math.max(1, O.D), 1);
  $('g-ice').textContent = st.k > 0.0005 ? `${Math.round(st.k * 1000)} mm ${st.kind}` : 'none';
  $('g-snow').textContent = st.contam > 0.02 ? `${st.cType} · lift −${Math.round(30 * st.contam)}%` : 'clean';
  $('g-oat').textContent = `${fmt(OAT(S.mode === 'park' ? 0 : st.h), 0)} °C`;
  $('g-vstall').textContent = S.mode === 'park' ? '—' : `${kt(O.Vs)} kt`;
  const el = $('g-state'), ban = $('banner');
  let txt, cls = '';
  if (O.stalled) { txt = 'Stalled'; cls = 'bad'; }
  else if (S.mode !== 'park' && ratio > 0.85) { txt = 'Stall warning'; cls = 'warn'; }
  else if (S.mode === 'park') txt = st.contam > 0.15 ? `Parked · wing ${st.cType === 'frost' ? 'frosted' : 'snow-covered'}` : 'Parked';
  else if (S.mode === 'tunnel') txt = lw >= 1 ? 'Wind tunnel · enough lift to fly' : 'Wind tunnel · not enough lift';
  else if (Math.abs(O.vs) > 1.5) txt = O.vs > 0 ? 'Climbing' : 'Descending';
  else txt = iceRate() > 0 ? 'Cruising · ice accreting' : 'Cruising';
  el.textContent = txt; el.className = 'state ' + cls;
  ban.hidden = !cls; ban.className = cls; ban.textContent = O.stalled ? 'Stall' : 'Stall warning';
  cam.shake = O.stalled ? 0.008 * L_ref : 0;
}
function infoCard() { const m = CAT[S.key]; $('ac-name').textContent = m.name; $('ac-desc').textContent = `${m.cls}. Span ${fmt(m.wing.b, 1)} m, wing area ${fmt(SL.geom.reference(SL.P).S, 1)} m², ${int(m.mass)} kg.`; $('ac-src').textContent = `Model: ${m.source}.`; }

// ====================================================== guided experiments
let EXP = null, step = 0;
const tween = (get, set, to, dur) => tweens.push({ from: get(), to, set, t: 0, dur });
const tw = { aoaT: (to, d) => tween(() => S.aoa, v => S.aoa = v, to, d) };
const setWx = w => { S.wx = w; syncControls(); };   // experiments set exact conditions themselves
const EXPERIMENTS = [
  { title: 'How a wing makes lift', blurb: 'Smoke over the wing, lift on the map.', steps: [
    { t: 'Air splits around the wing', b: 'The smoke streams turn <b>amber</b> where the air speeds up over the wing and <b>blue</b> where it slows underneath. Faster air has lower pressure, so the wing is pushed up.', go() { setWx('clear'); setMode('tunnel'); S.aoa = 3; S.smoke = true; S.liftmap = true; setView('side'); syncControls(); } },
    { t: 'Tilt the wing up', b: 'The angle rises to 10°. The lift map on the wing brightens towards yellow as each section works harder, and <b>Lift compared with weight</b> climbs.', go() { tw.aoaT(10, 6); } },
    { t: 'In flight, lift balances weight', b: 'In flight the autopilot holds the angle where lift equals weight.', go() { setMode('fly'); S.smoke = false; setView('chase'); syncControls(); } },
  ] },
  { title: 'Stall', blurb: 'Raise the nose until the air lets go.', steps: [
    { t: 'Smooth flow', b: 'At a moderate angle the smoke in the <b>wing cross-section</b> hugs the top surface.', go() { setWx('clear'); setMode('tunnel'); S.aoa = 5; S.liftmap = true; setView('side'); syncControls(); } },
    { t: 'Keep raising the angle', b: 'The angle climbs towards 22°. Watch the lift map: sections turn <b>red</b> as they reach their limit.', go() { tw.aoaT(22, 14); } },
    { t: 'The wing has stalled', b: 'The flow has <b>separated</b> from the top of the wing and lift has fallen, even though the angle is high.', go() { S.aoa = Math.max(S.aoa, 20); } },
    { t: 'Recover', b: 'Lowering the nose lets the airflow reattach. Pilots recover from a stall by pushing forward.', go() { tw.aoaT(5, 4); } },
  ] },
  { title: 'Wingtip vortices', blurb: 'The spinning wake behind every wing.', steps: [
    { t: 'Air spills round the tips', b: 'High-pressure air under the wing curls up round each tip into a vortex. The <b>blue trails</b> follow the computed airflow from just inside each tip: watch them twist around each other.', go() { setWx('clear'); setMode('fly'); S.trails = true; setView('tip'); syncControls(); buildTrails(); } },
    { t: 'More lift, stronger vortices', b: 'Slowing down needs a higher angle and more circulation, so the vortices strengthen. This is why airliners leave a dangerous wake on approach.', go() { AP.vt = Math.round(ac.vmin * 1.15); syncControls(); } },
  ] },
  { title: 'Flying into icing cloud', blurb: 'Watch ice build on the real airframe.', steps: [
    { t: 'Into supercooled cloud', b: 'At about −8 °C the cloud droplets are still liquid and freeze the instant they hit the aircraft. Time runs 60× faster.', go() { setMode('fly'); AP.ht = 3500; st.h = 3500; S.dT = -8 - SL.isa(3500).Tc; setWx('cloud'); S.ts = 60; setView('chase'); syncControls(); } },
    { t: 'Ice on every leading edge', b: () => `<b>${Math.round(st.k * 1000)} mm</b> of ${st.kind} ice. It forms wherever the airflow strikes the aircraft: the wing and tail leading edges, the nose and the <b>windscreen</b>. Try the <b>Close-up</b> camera.`, go() { setView('close'); } },
    { t: 'Losing performance', b: () => `Stall speed is now <b>${kt(O.Vs)} kt</b>. The autopilot raises the nose to hold height until the wing runs out of lift. Press <b>De-ice</b> to recover.`, go() { setView('chase'); } },
  ] },
  { title: 'Freezing rain', blurb: 'Large drops, clear ice, fast.', steps: [
    { t: 'Rain falling into sub-zero air', b: 'Large raindrops below 0 °C hit everything and only part freezes on impact. The rest runs back and freezes as clear <b>glaze</b> ridges, far behind the leading edge where de-icers do not reach.', go() { setMode('fly'); AP.ht = 1500; st.h = 1500; S.dT = -3 - SL.isa(1500).Tc; setWx('frz'); S.ts = 30; setView('close'); syncControls(); } },
    { t: 'Why pilots exit immediately', b: () => `Glaze ice: <b>${Math.round(st.k * 1000)} mm</b>. Freezing rain is one of the most dangerous conditions; the procedure is to leave it at once.`, go() { setView('chase'); } },
  ] },
  { title: 'Snow and frost before take-off', blurb: 'Why wings must be clean.', steps: [
    { t: 'Parked in falling snow', b: 'The aircraft is parked at −4 °C in steady snow (time ×60). Snow settles on every upward-facing surface.', go() { setMode('park'); S.dT = -4 - SL.isa(0).Tc; setWx('snow'); S.ts = 60; setView('chase'); clearSurface(0); syncControls(); } },
    { t: 'Take off without de-icing', b: () => `Contamination is roughening the wing: maximum lift is down about <b>${Math.round(30 * st.contam)}%</b>. Press <b>Take off</b> to see what happens at the normal rotation speed.`, go() {} },
    { t: 'The rule: a clean wing', b: 'Frost or snow as rough as sandpaper can cut lift by up to 30% and raise drag by 40% (FAA). That is why aircraft are de-iced at the gate.', go() {} },
  ] },
];
function renderExp() {
  const el = $('exp');
  if (!EXP) {
    el.innerHTML = `<div class="ac-card"><b id="ac-name"></b><span id="ac-desc"></span><small id="ac-src"></small></div><h2>Experiments</h2><p class="lede">Pick one and watch it play out, or set the weather and fly it yourself.</p>
      <div class="exp-list">${EXPERIMENTS.map((x, i) => `<button class="exp-item" data-i="${i}"><span class="n">${i + 1}</span><span><b>${x.title}</b><span>${x.blurb}</span></span></button>`).join('')}</div>`;
    el.querySelectorAll('.exp-item').forEach(b => b.onclick = () => startExp(+b.dataset.i)); if (CAT) infoCard(); return;
  }
  const s = EXP.steps[step];
  el.innerHTML = `<div class="step-k">${EXP.title} · step ${step + 1} of ${EXP.steps.length}</div><p class="step-t">${s.t}</p><p class="step-b" id="step-b">${typeof s.b === 'function' ? s.b() : s.b}</p>
    <div class="dots">${EXP.steps.map((_, i) => `<i class="${i <= step ? 'on' : ''}"></i>`).join('')}</div>
    <div class="row"><button class="btn quiet" id="x-back">Back</button><button class="btn primary" id="x-next">${step < EXP.steps.length - 1 ? 'Next' : 'Finish'}</button><button class="btn quiet" id="x-stop">Exit</button></div>`;
  $('x-back').onclick = () => { if (step > 0) goStep(step - 1); }; $('x-next').onclick = () => { if (step < EXP.steps.length - 1) goStep(step + 1); else stopExp(); }; $('x-stop').onclick = stopExp;
}
function startExp(i) { EXP = EXPERIMENTS[i]; goStep(0); }
function goStep(i) { step = i; tweens.length = 0; EXP.steps[i].go(); syncControls(); renderExp(); liftMap(!S.liftmap); }
function stopExp() { if (!EXP) return; EXP = null; tweens.length = 0; S.ts = 1; renderExp(); syncControls(); }

// ====================================================== main loop
let last = performance.now(), frame = 0, slowT = 0;
function loop(now) {
  requestAnimationFrame(loop);
  const dt = Math.min(0.05, (now - last) / 1000); last = now; frame++;
  if (!ac || !corner) return;
  for (let i = tweens.length - 1; i >= 0; i--) { const t = tweens[i]; t.t += dt; const u = t.dur ? clamp(t.t / t.dur, 0, 1) : 1, e = u * u * (3 - 2 * u); t.set(t.from + (t.to - t.from) * e); if (u >= 1) tweens.splice(i, 1); }
  if (S.mode === 'fly') { const sim = dt * S.ts, n = Math.max(1, Math.ceil(sim / 0.02)); for (let i = 0; i < n; i++) flightStep(sim / n); }
  else if (S.mode === 'tunnel') { st.alpha += (S.aoa * D2R - st.alpha) * Math.min(1, dt * 4); st.V = AP.vt; st.h += (AP.ht - st.h) * Math.min(1, dt * 2); st.gam = 0; accrete(dt * S.ts); }
  else { st.alpha = 0; st.gam = 0; }
  outputs();
  if (S.mode !== 'park' && (Math.abs(st.alpha - fieldAlpha) > 0.5 * D2R || fieldVer !== SL.version)) requestField(st.alpha);
  stepSmoke(dt); stepPrecip(dt);
  if (now - slowT > 200) { const d = (now - slowT) / 1000 * (S.mode === 'fly' || S.mode === 'tunnel' || S.mode === 'park' ? S.ts : 1); slowT = now; surfaceUpdate(Math.min(d, 20)); sectionUpdate(false); wxStatus(); }
  if (trails) trails.visible = S.trails && S.mode !== 'park';
  updateWorld(dt); resize(); updateCamera(dt);
  R.render(scene, camera);
  drawSection();
  if (frame % 6 === 0) {
    gauges();
    if (EXP) { const s = EXP.steps[step]; if (typeof s.b === 'function') $('step-b').innerHTML = s.b();
      const T = EXP.title; if (T === 'Stall' && step === 1 && O.stalled) goStep(2); if (T === 'Flying into icing cloud' && step === 0 && st.k > 0.006) goStep(1); if (T === 'Flying into icing cloud' && step === 1 && st.k > 0.035) goStep(2);
      if (T === 'Snow and frost before take-off' && step === 0 && st.contam > 0.6) goStep(1); if (T === 'Snow and frost before take-off' && step === 1 && S.mode === 'fly') goStep(2); }
  }
}
document.addEventListener('visibilitychange', () => sw.postMessage({ type: 'run', running: !document.hidden }));   // no simulation in a hidden tab
window.SLIP = { S, AP, get st() { return st; }, get corner() { return corner; }, setMode, setAircraft, setView };   // console/debug access
(async () => {
  CAT = await (await fetch('models/catalog.json?v=' + VER)).json();
  for (const [k, m] of Object.entries(CAT)) SL.PROJECTS[k] = { name: m.short, wing: m.wing, tail: m.tail, body: m.body, mass: m.mass, cg: m.cg, engine: m.engine, flight: m.flight };
  refreshEnv(); renderExp(); setView('chase');
  await setAircraft('c172');
  requestAnimationFrame(loop);
})().catch(e => { console.error(e); toast('Failed to start: ' + e.message, 10000); });
})();
