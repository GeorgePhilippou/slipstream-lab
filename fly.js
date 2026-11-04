// Slipstream "explore" view: one live scene. Aircraft in the sky, air flowing over it, a live 2D section,
// plain-language gauges, direct controls and guided experiments.
// Physics: vortex-lattice wing/tail (vlm.js) → potential-flow velocity field (field-worker.js) for the particles;
// point-mass flight dynamics with ISA and icing; D2Q9 lattice Boltzmann for the wing section (wing-worker.js).
(() => {
const { $, fmt, int, clamp, D2R } = SL;
const G0 = 9.80665;
const S = { key: 'trainer', mode: 'fly', aoa: 4, view: 'chase', cloud: false, ts: 1, lwc: 0.55, mvd: 22 };
const AP = { ht: 2500, vt: 60 };
let ac = null, acVer = -1, st = null, A = null, O = {}, base = null;
const tweens = [];
const toast = (msg, ms = 4000) => { const t = $('toast'); t.textContent = msg; t.hidden = false; clearTimeout(t._t); t._t = setTimeout(() => t.hidden = true, ms); };

// ====================================================== flight model
function aeroOf(k, kind) {
  const AR = ac.b * ac.b / ac.S, kc = k / ac.c, s = Math.sqrt(Math.max(0, kc)), hf = kind === 'glaze' ? 1 : kind === 'mixed' ? 0.82 : 0.65;
  const dCLmax = Math.min(0.6, 3.0 * s * hf), dCD0 = 0.25 * s * hf, a2d = ac.a0 * (1 - 0.5 * s), aw = a2d / (1 + a2d / (Math.PI * ac.e * AR));
  const CLmax = ac.CLmax * (1 - dCLmax);
  return { AR, aw, CLmax, alphaS: ac.alpha0 + 1.2 * CLmax / aw, dCD0, dCLmax, kc };
}
const CLof = (al, X) => { const x = (al - ac.alpha0) * X.aw / X.CLmax; return (x < 0.8 ? x : x < 1.2 ? x - (x - 0.8) ** 2 / 0.8 : Math.max(0.55, 1 - 0.9 * (x - 1.2))) * X.CLmax; };
const CDof = (CL, al, M, X) => { const x = (al - ac.alpha0) * X.aw / X.CLmax; return ac.CD0 + X.dCD0 + CL * CL / (Math.PI * ac.e * X.AR) + (x > 1.2 ? 0.6 * (x - 1.2) : 0) + (M > ac.Mcrit ? 20 * (M - ac.Mcrit) ** 4 : 0); };
const Tmax = (rho, V, M) => { const sig = rho / 1.225; return ac.prop ? Math.min(ac.Tstatic * sig, ac.eta * ac.P * sig / Math.max(V, 20)) : ac.T0 * sig ** 0.85 * (1 - 0.25 * M); };
const collection = V => { const d = S.mvd * 1e-6, K = 1000 * d * d * V / (9 * 1.7e-5 * 0.2 * ac.c); if (K <= 0.125) return 0; const z = 1.4 * Math.pow(K - 0.125, 0.84); return 0.85 * z / (1 + z); };
const liquid = Tc => Tc >= 0 ? 0 : clamp((Tc + 40) / 25, 0, 1);
function rebuildAircraft(reset) {
  ac = SL.aircraftFromProject(); acVer = SL.version; A = SL.aero();
  if (reset || !st) {
    st = { h: AP.ht, V: AP.vt, gam: 0, alpha: 3 * D2R, k: 0, kind: 'rime', t: 0, mode: 'hold', recT: 0, stallH: 0, said: {} };
    const atm = SL.isa(st.h), X = aeroOf(0, 'rime'), q = 0.5 * atm.rho * st.V * st.V;
    st.alpha = clamp(ac.alpha0 + ac.m * G0 / (q * ac.S) / X.aw, -4 * D2R, 14 * D2R);
  }
}
function flightStep(dt) {
  const atm = SL.isa(st.h), X = aeroOf(st.k, st.kind), W = ac.m * G0, q = 0.5 * atm.rho * st.V * st.V, M = st.V / atm.a, Tm = Tmax(atm.rho, st.V, M);
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
  if (st.mode === 'hold' && st.alpha > X.alphaS) { st.mode = 'recover'; st.stallH = st.h; st.recT = 0; toast(`Stall at ${int(st.h)} m${st.k > 0.002 ? ` with ${Math.round(st.k * 1000)} mm of ice` : ''}. The nose drops to recover.`, 5000); }
  if (st.mode === 'recover') { st.recT += dt; if (st.recT > 4 && st.V > 1.3 * Vs && st.alpha < X.alphaS) { st.mode = 'hold'; AP.ht = Math.round(st.h / 100) * 100; syncControls(); toast(`Recovered, ${int(st.stallH - st.h)} m lower.`); } }
  const lf = S.cloud ? liquid(atm.Tc) : 0;
  if (lf > 0) { const kd = SL.iceKind(atm.Tc, S.lwc).key; st.kind = kd === 'none' ? 'rime' : kd; st.k += collection(st.V) * S.lwc * 1e-3 * st.V / (st.kind === 'rime' ? 880 : 917) * lf * dt; }
}
function outputs() {
  const atm = SL.isa(st.h), X = aeroOf(st.k, st.kind), W = ac.m * G0, q = 0.5 * atm.rho * st.V * st.V, M = st.V / atm.a;
  const CL = CLof(st.alpha, X), CD = CDof(CL, st.alpha, M, X);
  O = { atm, X, W, q, M, CL, CD, L: q * ac.S * CL, D: q * ac.S * CD, Vs: Math.sqrt(2 * W / (atm.rho * ac.S * X.CLmax)), vs: st.V * Math.sin(st.gam), stalled: st.alpha > X.alphaS || st.mode === 'recover' };
}

// ====================================================== 3D scene
const canvas = $('scene');
const R = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
R.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
R.outputEncoding = THREE.sRGBEncoding; R.toneMapping = THREE.ACESFilmicToneMapping; R.toneMappingExposure = 0.92;
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 30000);
scene.fog = new THREE.Fog(0xb9d4ea, 900, 9000);
const hemi = new THREE.HemisphereLight(0xdcefff, 0x6f7f68, 0.85); scene.add(hemi);
const sun = new THREE.DirectionalLight(0xfff1dc, 1.5); sun.position.set(-0.4, 1, 0.35).multiplyScalar(100); scene.add(sun);
// sky dome
const skyU = { top: { value: new THREE.Color(0x2a6fc0) }, hor: { value: new THREE.Color(0xcfe3f3) }, sunDir: { value: new THREE.Vector3(-0.4, 0.45, 0.35).normalize() } };
const sky = new THREE.Mesh(new THREE.SphereGeometry(15000, 32, 16), new THREE.ShaderMaterial({ uniforms: skyU, side: THREE.BackSide, depthWrite: false, fog: false,
  vertexShader: 'varying vec3 vD; void main(){ vD = normalize(position); gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
  fragmentShader: 'uniform vec3 top; uniform vec3 hor; uniform vec3 sunDir; varying vec3 vD; void main(){ float h = clamp(vD.y, -0.2, 1.0); vec3 c = mix(hor, top, pow(max(h, 0.0), 0.55)); if (h < 0.0) c = mix(hor, hor * 0.82, -h * 4.0); float s = max(dot(normalize(vD), sunDir), 0.0); c += vec3(1.0, 0.9, 0.7) * (pow(s, 600.0) * 1.6 + pow(s, 12.0) * 0.18); gl_FragColor = vec4(c, 1.0); }' }));
scene.add(sky);
// ground
const gTex = (() => { const c = document.createElement('canvas'); c.width = c.height = 512; const x = c.getContext('2d'); x.fillStyle = '#6f8a5e'; x.fillRect(0, 0, 512, 512);
  const cols = ['#7d9866', '#647f55', '#8aa071', '#6c8457', '#93a678', '#5f7a4f'];
  for (let i = 0; i < 90; i++) { x.fillStyle = cols[i % cols.length]; x.fillRect(Math.random() * 512, Math.random() * 512, 30 + Math.random() * 90, 30 + Math.random() * 90); }
  x.strokeStyle = 'rgba(220,210,190,.35)'; x.lineWidth = 2; for (let i = 0; i < 6; i++) { x.beginPath(); x.moveTo(0, Math.random() * 512); x.bezierCurveTo(170, Math.random() * 512, 340, Math.random() * 512, 512, Math.random() * 512); x.stroke(); }
  const t = new THREE.CanvasTexture(c); t.wrapS = t.wrapT = THREE.RepeatWrapping; t.repeat.set(40, 40); t.encoding = THREE.sRGBEncoding; return t; })();
const ground = new THREE.Mesh(new THREE.PlaneGeometry(60000, 60000), new THREE.MeshLambertMaterial({ map: gTex })); ground.rotation.x = -Math.PI / 2; scene.add(ground);
// clouds: soft sprites in clusters
const puffTex = (() => { const c = document.createElement('canvas'); c.width = c.height = 128; const x = c.getContext('2d'); const g = x.createRadialGradient(64, 64, 4, 64, 64, 62); g.addColorStop(0, 'rgba(255,255,255,0.95)'); g.addColorStop(0.5, 'rgba(255,255,255,0.55)'); g.addColorStop(1, 'rgba(255,255,255,0)'); x.fillStyle = g; x.fillRect(0, 0, 128, 128); return new THREE.CanvasTexture(c); })();
const clouds = new THREE.Group(); scene.add(clouds);
const puffs = [];
for (let i = 0; i < 70; i++) {
  const cx = (Math.random() - 0.5) * 9000, cz = (Math.random() - 0.5) * 9000, layer = Math.random();
  for (let j = 0; j < 7; j++) {
    const m = new THREE.Sprite(new THREE.SpriteMaterial({ map: puffTex, color: 0xffffff, transparent: true, opacity: 1, depthWrite: false }));
    const s = 220 + Math.random() * 380; m.scale.set(s * 1.6, s, 1);
    m.userData = { x: cx + (Math.random() - 0.5) * 500, z: cz + (Math.random() - 0.5) * 500, alt: 1400 + layer * 1800 + (Math.random() - 0.5) * 220 };
    clouds.add(m); puffs.push(m);
  }
}
// close-in mist for flying inside cloud
const mist = []; for (let i = 0; i < 40; i++) { const m = new THREE.Sprite(new THREE.SpriteMaterial({ map: puffTex, transparent: true, opacity: 0.0, depthWrite: false })); const s = 30 + Math.random() * 60; m.scale.set(s * 1.5, s, 1); m.userData = { x: (Math.random() - 0.5) * 400, y: (Math.random() - 0.5) * 120, z: (Math.random() - 0.5) * 300 }; mist.push(m); scene.add(m); }

// aircraft: plane (pitch) → body (body axes x aft, y right, z up → three: nose +X, up +Y, right +Z)
const plane = new THREE.Group(); scene.add(plane);
const body = new THREE.Group(); plane.add(body);
body.matrixAutoUpdate = false;
let mesh = null, wingIdx = null, iceMesh = null, lastIceK = -1;
const mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.48, metalness: 0.08 });
const iceMat = new THREE.MeshStandardMaterial({ color: 0xeaf7ff, roughness: 0.25, metalness: 0, emissive: 0x16303f, emissiveIntensity: 0.4 });
let L_ref = 10, xcg = 0;
const BASE = { wing: [0.93, 0.94, 0.96], tail: [0.9, 0.91, 0.93], body: [0.95, 0.96, 0.97], fin: [0.16, 0.36, 0.62] };
function buildMesh() {
  const P = SL.P, M = SL.geom.mesh(P), ref = SL.geom.reference(P);
  xcg = ref.xcg; L_ref = Math.max(P.wing.b, P.body.lf);
  body.matrix.set(-1, 0, 0, xcg, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1);
  const n = M.pos.length / 3, col = new Float32Array(n * 3), part = new Array(n).fill('body');
  for (let t = 0; t < M.part.length; t++) for (let j = 0; j < 3; j++) { const v = M.idx[t * 3 + j]; if (part[v] === 'body' || M.part[t] !== 'body') part[v] = M.part[t]; }
  wingIdx = [];
  for (let i = 0; i < n; i++) { const b = BASE[part[i]] || BASE.body; col.set(b, i * 3); if (part[i] === 'wing') wingIdx.push(i); }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(M.pos, 3)); g.setAttribute('color', new THREE.BufferAttribute(col, 3)); g.setIndex(new THREE.BufferAttribute(M.idx, 1));
  g.computeVertexNormals();
  if (mesh) { body.remove(mesh); mesh.geometry.dispose(); }
  mesh = new THREE.Mesh(g, mat); body.add(mesh);
  lastIceK = -1;
}
function colourWing() {
  if (!mesh || !A) return;
  const P = SL.P, s = P.wing.b / 2, al = st.alpha;
  const strips = A.g.strips.filter(x => x.surf === 'wing' && x.side === 1).sort((a, b) => a.mid[1] - b.mid[1]);
  const pos = mesh.geometry.attributes.position.array, col = mesh.geometry.attributes.color.array, pulse = 0.5 + 0.5 * Math.sin(performance.now() / 160);
  const amber = [1.0, 0.62, 0.18], red = [1.0, 0.2, 0.14], X = O.X;
  const iceLoss = X ? X.CLmax / ac.CLmax : 1;     // ice lowers every section's limit
  for (const i of wingIdx) {
    const y = Math.abs(pos[i * 3 + 1]); let k = 0; while (k < strips.length - 1 && strips[k + 1].mid[1] < y) k++;
    const sp = strips[Math.min(k, strips.length - 1)], r = clamp((sp.cl0 + sp.cla * al) / (sp.clmax * iceLoss), 0, 1.4);
    const b = BASE.wing; let c;
    if (r < 1) { const t = Math.pow(r, 1.6) * 0.85; c = [b[0] + (amber[0] - b[0]) * t, b[1] + (amber[1] - b[1]) * t, b[2] + (amber[2] - b[2]) * t]; }
    else { const t = 0.6 + 0.4 * pulse; c = [amber[0] + (red[0] - amber[0]) * t, amber[1] + (red[1] - amber[1]) * t, amber[2] + (red[2] - amber[2]) * t]; }
    col[i * 3] = c[0]; col[i * 3 + 1] = c[1]; col[i * 3 + 2] = c[2];
  }
  mesh.geometry.attributes.color.needsUpdate = true;
}
function buildIce() {
  const kk = Math.round(st.k * 1000); if (kk === lastIceK) return; lastIceK = kk;
  if (iceMesh) { body.remove(iceMesh); iceMesh.geometry.dispose(); iceMesh = null; }
  if (st.k < 0.0008) return;
  const secs = SL.geom.surfaces(SL.P)[0].sections.slice(0, SL.P.wing.wlH > 0.01 ? -1 : undefined), r = Math.min(0.04 * SL.P.wing.cr, st.k * 3);
  for (const side of [1, -1]) {
    const pts = secs.map(s => new THREE.Vector3(s.le[0] - r * 0.4, side * s.le[1], s.le[2]));
    if (side > 0) pts[0].y = SL.P.body.df * 0.45; else pts[0].y = -SL.P.body.df * 0.45;
    const g = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 48, r, 8, false);
    if (!iceMesh) { iceMesh = new THREE.Group(); body.add(iceMesh); }
    iceMesh.add(new THREE.Mesh(g, iceMat));
  }
}

// ====================================================== flow particles (vortex-lattice velocity field)
// Each particle keeps its last HN positions, drawn as a fading streakline.
const NP = 1500, HN = 12;
const pX = new Float32Array(NP), pY = new Float32Array(NP), pZ = new Float32Array(NP), pA = new Float32Array(NP);
const hX = new Float32Array(NP * HN), hY = new Float32Array(NP * HN), hZ = new Float32Array(NP * HN), hS = new Float32Array(NP * HN), hHead = new Uint8Array(NP), hCount = new Uint8Array(NP);
const NSEG = NP * (HN - 1), lineGeo = new THREE.BufferGeometry(), lp = new Float32Array(NSEG * 6), lc = new Float32Array(NSEG * 8);
lineGeo.setAttribute('position', new THREE.BufferAttribute(lp, 3).setUsage(THREE.DynamicDrawUsage));
lineGeo.setAttribute('color', new THREE.BufferAttribute(lc, 4).setUsage(THREE.DynamicDrawUsage));
const lines = new THREE.LineSegments(lineGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 1, depthWrite: false }));
lines.frustumCulled = false; body.add(lines);
let field = null, fieldBusy = false, fieldWant = null, fieldAlpha = 99, fieldVer = -1, box = null;
const fw = new Worker('field-worker.js?v=6');
fw.onmessage = e => { field = e.data; fieldBusy = false; if (fieldWant) { const w = fieldWant; fieldWant = null; requestField(w.alpha); } };
function requestField(alpha) {
  if (!A) return;
  if (fieldBusy) { fieldWant = { alpha }; return; }
  const P = SL.P, g = A.g; VLM.evaluate(g, alpha);
  const n = g.panels.length, Aa = new Float32Array(n * 3), Ba = new Float32Array(n * 3), Ga = new Float32Array(n);
  g.panels.forEach((p, i) => { Aa.set(p.A, i * 3); Ba.set(p.B, i * 3); Ga[i] = p.gam; });
  const b = P.wing.b, L = L_ref;
  box = [P.body.xnose - 0.25 * L, P.body.xnose + P.body.lf + 1.1 * L, 0, 0.72 * b, -0.32 * b, 0.32 * b];
  fieldBusy = true; fieldAlpha = alpha; fieldVer = SL.version;
  fw.postMessage({ id: SL.version, A: Aa, B: Ba, G: Ga, alpha, box, dims: [64, 28, 30], rc: 0.06 * SL.geom.reference(P).cbar, far: 200 * b }, [Aa.buffer, Ba.buffer, Ga.buffer]);
}
function vel(x, y, z, out) {
  const a = st.alpha;
  if (!field) { out[0] = Math.cos(a); out[1] = 0; out[2] = Math.sin(a); return; }
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
function seed(i, anywhere) {
  const P = SL.P, b = P.wing.b, bx = box || [P.body.xnose - 3, P.body.xnose + P.body.lf + 10, 0, b, -b / 3, b / 3];
  const lane = (Math.random() * 22) | 0, lev = (Math.random() * 4) | 0;
  pY[i] = (-0.6 + 1.2 * (lane + 0.5) / 22) * b + (Math.random() - 0.5) * 0.006 * b;
  pZ[i] = (-0.05 + 0.11 * (lev + 0.5) / 4) * b + (Math.random() - 0.5) * 0.005 * b;
  pX[i] = anywhere ? bx[0] + Math.random() * (bx[1] - bx[0]) : bx[0] + Math.random() * 0.02 * b;
  pA[i] = 0; hCount[i] = 0;
}
for (let i = 0; i < NP; i++) seed(i, true);
const insideBody = (x, y, z) => { const B = SL.P.body, R0 = B.df / 2; if (x < B.xnose || x > B.xnose + B.lf) return false; const u = (x - B.xnose) / B.lf; const r = u < 0.18 ? R0 * Math.sqrt(Math.max(0, 1 - ((0.18 - u) / 0.18) ** 2)) : u < 0.6 ? R0 : R0 * (1 - 0.82 * ((u - 0.6) / 0.4) ** 1.3); const zc = (u > 0.6 ? 0.3 * R0 * ((u - 0.6) / 0.4) ** 2 : 0) - 0.2 * R0; return y * y + (z - zc) ** 2 < r * r; };
const vv = [0, 0, 0];
let histT = 0;
// slow air (higher pressure) blue → free stream pale → fast air (lower pressure) amber
const colAt = (s, f, out, o) => { const t = clamp((s - 1) / 0.22, -1, 1); let r, g, b; if (t < 0) { const u = -t; r = 0.16 - 0.08 * u; g = 0.66 - 0.36 * u; b = 0.98 - 0.1 * u; } else { r = 0.16 + 0.84 * t; g = 0.66 - 0.1 * t; b = 0.98 - 0.88 * t; } out[o] = r; out[o + 1] = g; out[o + 2] = b; out[o + 3] = f * 0.95; };
function stepParticles(dt) {
  const P = SL.P, sp = 0.42 * L_ref * dt, xEnd = box ? box[1] : P.body.xnose + P.body.lf + L_ref;
  const stalled = O.stalled, b = P.wing.b, xs = SL.geom.reference(P).xmac;
  histT += dt; const push = histT > 0.045; if (push) histT = 0;
  let seg = 0;
  for (let i = 0; i < NP; i++) {
    vel(pX[i], pY[i], pZ[i], vv);
    let ux = vv[0], uy = vv[1], uz = vv[2];
    if (stalled && pX[i] > xs && pX[i] < xs + 0.5 * L_ref && Math.abs(pY[i]) < 0.48 * b && pZ[i] > -0.01 * b && pZ[i] < 0.1 * b) { ux *= 0.45; uy += (Math.random() - 0.5) * 0.5; uz += (Math.random() - 0.5) * 0.6; }   // separated wake over a stalled wing (illustrative)
    pX[i] += ux * sp; pY[i] += uy * sp; pZ[i] += uz * sp; pA[i] += dt;
    if (pX[i] > xEnd || Math.abs(pY[i]) > b || Math.abs(pZ[i]) > b * 0.6 || insideBody(pX[i], pY[i], pZ[i])) seed(i, false);
    const s = Math.hypot(ux, uy, uz);
    if (push || hCount[i] === 0) { const h = (hHead[i] + 1) % HN; hHead[i] = h; const o = i * HN + h; hX[o] = pX[i]; hY[o] = pY[i]; hZ[o] = pZ[i]; hS[o] = s; if (hCount[i] < HN) hCount[i]++; }
    else { const o = i * HN + hHead[i]; hX[o] = pX[i]; hY[o] = pY[i]; hZ[o] = pZ[i]; hS[o] = s; }
    const n = hCount[i];
    for (let k = 0; k < HN - 1; k++) {
      const q = seg * 6; seg++;
      const qc = (seg - 1) * 8;
      if (k >= n - 1) { lp[q] = lp[q + 3] = pX[i]; lp[q + 1] = lp[q + 4] = pY[i]; lp[q + 2] = lp[q + 5] = pZ[i]; lc.fill(0, qc, qc + 8); continue; }
      const a = i * HN + (hHead[i] - k + HN) % HN, c = i * HN + (hHead[i] - k - 1 + HN) % HN;
      lp[q] = hX[a]; lp[q + 1] = hY[a]; lp[q + 2] = hZ[a]; lp[q + 3] = hX[c]; lp[q + 4] = hY[c]; lp[q + 5] = hZ[c];
      const f0 = (1 - k / (HN - 1)) * Math.min(1, pA[i] * 3), f1 = (1 - (k + 1) / (HN - 1)) * Math.min(1, pA[i] * 3);
      colAt(hS[a], f0, lc, qc); colAt(hS[c], f1, lc, qc + 4);
    }
  }
  lineGeo.attributes.position.needsUpdate = true; lineGeo.attributes.color.needsUpdate = true;
}

// ====================================================== camera
const cam = { yaw: 0, pitch: 0, zoom: 1, shake: 0 };
const PRESET = {
  chase: L => [new THREE.Vector3(-1.45 * L, 0.48 * L, -0.62 * L), new THREE.Vector3(0.05 * L, 0, 0)],
  side: L => [new THREE.Vector3(0.05 * L, 0.05 * L, -2.5 * L), new THREE.Vector3(0.05 * L, 0, 0)],
  front: L => [new THREE.Vector3(1.7 * L, 0.28 * L, 0.45 * L), new THREE.Vector3(0, 0, 0)],
  top: L => [new THREE.Vector3(-0.45 * L, 1.9 * L, 0.01), new THREE.Vector3(-0.15 * L, 0, 0)],
  tip: L => [new THREE.Vector3(-1.5 * L, 0.18 * L, 0.95 * SL.P.wing.b), new THREE.Vector3(-0.5 * L, 0, 0.42 * SL.P.wing.b)],
};
const camPos = new THREE.Vector3(-30, 8, -10), camTgt = new THREE.Vector3();
let drag = null;
canvas.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, yaw: cam.yaw, pitch: cam.pitch }; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener('pointermove', e => { if (!drag) return; cam.yaw = drag.yaw - (e.clientX - drag.x) * 0.006; cam.pitch = clamp(drag.pitch + (e.clientY - drag.y) * 0.005, -1.2, 1.2); });
canvas.addEventListener('pointerup', () => drag = null); canvas.addEventListener('pointercancel', () => drag = null);
canvas.addEventListener('wheel', e => { e.preventDefault(); cam.zoom = clamp(cam.zoom * Math.exp(e.deltaY * 0.001), 0.35, 3); }, { passive: false });
function setView(v) { S.view = v; cam.yaw = 0; cam.pitch = 0; cam.zoom = 1; document.querySelectorAll('#views button').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.v === v))); }
function updateCamera(dt) {
  const [p, t] = PRESET[S.view](L_ref);
  const off = p.clone().sub(t), r = off.length() * cam.zoom;
  const sph = new THREE.Spherical().setFromVector3(off); sph.radius = r; sph.theta += cam.yaw; sph.phi = clamp(sph.phi - cam.pitch, 0.05, Math.PI - 0.05);
  const want = new THREE.Vector3().setFromSpherical(sph).add(t);
  const k = 1 - Math.exp(-dt * 3.5); camPos.lerp(want, k); camTgt.lerp(t, k);
  camera.position.copy(camPos); camera.lookAt(camTgt);
  if (cam.shake > 0) { camera.position.x += (Math.random() - 0.5) * cam.shake; camera.position.y += (Math.random() - 0.5) * cam.shake; }
}
function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight; if (!w || !h) return;
  if (canvas.width !== Math.round(w * R.getPixelRatio()) || canvas.height !== Math.round(h * R.getPixelRatio())) { R.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); }
}

// ====================================================== world motion (sky, clouds, ground)
let travel = 0;
function updateWorld(dt) {
  const h = st.h, V = st.V, g = S.mode === 'fly' ? st.gam : 0, th = S.mode === 'fly' ? st.alpha + st.gam : st.alpha;
  plane.rotation.z = th;
  const f = clamp(h / 12000, 0, 1);
  skyU.top.value.setRGB(0.16 - 0.1 * f, 0.43 - 0.25 * f, 0.75 - 0.3 * f); skyU.hor.value.setRGB(0.81 - 0.2 * f, 0.89 - 0.15 * f, 0.95 - 0.08 * f);
  const inCloud = S.cloud;
  scene.fog.color.copy(inCloud ? new THREE.Color(0xc7ced6) : skyU.hor.value);
  scene.fog.near = inCloud ? 40 : 900; scene.fog.far = inCloud ? 520 : 9000;
  const adv = S.mode === 'fly' ? V * dt : V * dt * 0.2;   // in the tunnel the world drifts slowly
  travel += adv * Math.cos(g);
  ground.position.y = -h; ground.visible = h < 7000;
  gTex.offset.x = (travel / 1500) % 1;
  for (const m of puffs) {
    const u = m.userData; u.x -= adv * Math.cos(g); if (u.x < -4500) u.x += 9000;
    m.position.set(u.x, u.alt - h, u.z);
    const dist = Math.hypot(u.x, u.alt - h, u.z);
    m.material.opacity = inCloud ? 0.85 : 0.85 * clamp((dist - 350) / 700, 0, 1);
  }
  for (const m of mist) {
    const u = m.userData; u.x -= adv * 0.5; if (u.x < -200) { u.x += 400; u.y = (Math.random() - 0.5) * 120; u.z = (Math.random() - 0.5) * 300; }
    m.position.set(u.x, u.y, u.z); m.material.opacity = inCloud ? 0.5 : 0;
  }
}

// ====================================================== live 2D wing section (lattice Boltzmann)
const secCv = $('sec'), sx = secCv.getContext('2d');
const SNX = 300, SNY = 140, SU0 = 0.1, SL_ = Math.round(0.2 * SNX), sw = new Worker('wing-worker.js?v=6');
const sFieldC = document.createElement('canvas'); sFieldC.width = SNX; sFieldC.height = SNY; const sfx = sFieldC.getContext('2d'), sImg = sfx.createImageData(SNX, SNY);
let sField = null, sMask = null, sPolys = [], sKey = '', sStarted = false, sHist = [];
const sGeom = al => ({ px: 0.22 * SNX + 0.25 * SL_, py: SNY / 2, L: SL_, aoa: al, ox: 0.25 });
function sectionPolys(al) {
  const s = SL.section(), m = s.m / 100, p = s.p / 10, t = s.t / 100, gi = sGeom(al), a = -al * D2R, ca = Math.cos(a), sa = Math.sin(a);
  const tf = ([x, y]) => { const dx = x - 0.25; return [gi.px + SL_ * (dx * ca - y * sa), gi.py + SL_ * (dx * sa + y * ca)]; };
  const polys = [SL.naca.points(m, p, t, 0).map(tf)];
  const kc = st ? st.k / (ac ? ac.c : 1.5) : 0;
  if (kc > 2e-4) {   // ice shape on the leading edge (rime cap or glaze horns)
    const horn = st.kind === 'glaze' ? 1 : st.kind === 'mixed' ? 0.5 : 0, outer = [], inner = [];
    for (let i = 0; i <= 60; i++) {
      const s2 = -1 + i / 30, u = 0.1 * s2 * s2, up = s2 >= 0, y = SL.naca.yc(m, p, u) + (up ? 1 : -1) * SL.naca.yt(t, u);
      const u2 = u + 1e-3, y2 = SL.naca.yc(m, p, u2) + (up ? 1 : -1) * SL.naca.yt(t, u2); let nx = -(y2 - y), ny = u2 - u; if (!up) { nx = -nx; ny = -ny; }
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
const sTau = () => 3 * SU0 * SL_ / 3000 + 0.5;
function sectionUpdate(force) {
  const al = clamp(Math.round(st.alpha / D2R * 2) / 2, -5, 25), kk = Math.round(st.k * 2000), key = `${SL.P.wing.sec}|${al}|${kk}|${st.kind}`;
  if (!force && key === sKey) return; sKey = key;
  sPolys = sectionPolys(al); sMask = raster(sPolys);
  if (!sStarted) { sStarted = true; sw.postMessage({ type: 'init', nx: SNX, ny: SNY, u0: SU0, tau: sTau(), clean: sMask, geom: sGeom(al) }); sw.postMessage({ type: 'frame' }); }
  else sw.postMessage({ type: 'geom', clean: sMask, geom: sGeom(al) });
}
sw.onmessage = e => { const m = e.data; if (m.type === 'blowup') return; if (m.type !== 'field') { sw.postMessage({ type: 'frame' }); return; } sField = m; if (m.fn && m.steps * SU0 / SL_ > 0.6) { sHist.push(m.Fy / (0.5 * SU0 * SU0 * SL_)); if (sHist.length > 90) sHist.shift(); } setTimeout(() => sw.postMessage({ type: 'frame' }), 30); };
function drawSection() {
  if (!sField) return;
  const W = secCv.clientWidth, H = secCv.clientHeight, dpr = Math.min(2, window.devicePixelRatio || 1);
  if (secCv.width !== Math.round(W * dpr)) { secCv.width = Math.round(W * dpr); secCv.height = Math.round(H * dpr); }
  const { ux, uy, dye } = sField, d = sImg.data;
  for (let y = 0; y < SNY; y++) for (let x = 0; x < SNX; x++) {
    const k = y * SNX + x, o = ((SNY - 1 - y) * SNX + x) * 4;
    if (sMask[k]) { d[o] = 200; d[o + 1] = 208; d[o + 2] = 218; d[o + 3] = 255; continue; }
    const s = clamp(Math.hypot(ux[k], uy[k]) / SU0 / 1.7, 0, 1), dy = clamp(dye[k] * 1.4, 0, 1);
    const r = 8 + 60 * s * s, g = 22 + 110 * s, b = 40 + 140 * s;
    d[o] = r + (245 - r) * dy; d[o + 1] = g + (248 - g) * dy; d[o + 2] = b + (252 - b) * dy; d[o + 3] = 255;
  }
  sfx.putImageData(sImg, 0, 0);
  // crop to the region around the wing
  const cx0 = 0.22 * SNX - 0.3 * SL_, cw = 1.9 * SL_, ch = cw * secCv.height / secCv.width, cy0 = SNY / 2 - ch / 2;
  sx.imageSmoothingEnabled = true; sx.drawImage(sFieldC, cx0, SNY - cy0 - ch, cw, ch, 0, 0, secCv.width, secCv.height);
  // ice on top in white
  if (sPolys.length > 1) {
    const toS = ([x, y]) => [(x - cx0) / cw * secCv.width, (1 - (y - cy0) / ch) * secCv.height];
    sx.beginPath(); sPolys[1].forEach((p, i) => { const [a, b] = toS(p); i ? sx.lineTo(a, b) : sx.moveTo(a, b); }); sx.closePath(); sx.fillStyle = '#eef8ff'; sx.fill();
  }
  const cl = SL.mean(sHist, 40), s = SL.section(), th = 2 * Math.PI * (st.alpha - SL.alpha0(s.m / 100, s.p / 10, 0)), sep = st.alpha / D2R > 9 && sHist.length > 30 && cl < 0.62 * th;
  $('sec-note').textContent = sep ? 'flow separated' : `angle ${fmt(st.alpha / D2R, 1)}°`;
  $('sec-note').style.color = sep ? 'var(--bad)' : '';
}

// ====================================================== controls
const kt = v => Math.round(v * 1.944);
function syncControls() {
  const sp = $('c-speed'), P = SL.P;
  sp.min = Math.max(10, Math.round(ac.vmin * 0.8)); sp.max = Math.round(ac.vmax); sp.step = 1; sp.value = AP.vt;
  $('c-speed-o').textContent = `${kt(AP.vt)} kt`;
  $('c-alt').value = AP.ht; $('c-alt-o').textContent = `${int(AP.ht)} m`;
  $('c-aoa').value = S.aoa; $('c-aoa-o').textContent = S.mode === 'tunnel' ? `${fmt(S.aoa, 1)}°` : 'auto'; $('c-aoa').disabled = S.mode !== 'tunnel';
  const b0 = base.wing.b; $('c-span').min = +(b0 * 0.6).toFixed(1); $('c-span').max = +(b0 * 1.6).toFixed(1); $('c-span').step = 0.1; $('c-span').value = P.wing.b; $('c-span-o').textContent = `${fmt(P.wing.b, 1)} m`;
  $('c-sweep').value = P.wing.sweep; $('c-sweep-o').textContent = `${fmt(P.wing.sweep, 0)}°`;
  $('c-wl').checked = P.wing.wlH > 0.01; $('c-cloud').checked = S.cloud;
  document.querySelectorAll('#mode button').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.v === S.mode)));
  document.querySelectorAll('#c-time button').forEach(b => b.setAttribute('aria-pressed', String(+b.dataset.v === S.ts)));
  document.querySelectorAll('#aircraft .chip').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.k === S.key)));
}
let geoDirty = false, geoT = 0;
const geomChanged = () => { geoDirty = true; };
function setAircraft(k) {
  S.key = k; SL.loadProject(k); base = JSON.parse(JSON.stringify(SL.P));
  AP.ht = SL.P.flight.h; AP.vt = SL.P.flight.V;
  rebuildAircraft(true); buildMesh(); requestField(st.alpha); sectionUpdate(true); syncControls();
}
function setMode(m) { S.mode = m; if (m === 'tunnel') { S.aoa = Math.round(st.alpha / D2R * 2) / 2; st.gam = 0; st.mode = 'hold'; } else { st.mode = 'hold'; } syncControls(); }
$('aircraft').addEventListener('click', e => { const b = e.target.closest('.chip'); if (b) { stopExp(); setAircraft(b.dataset.k); } });
$('views').addEventListener('click', e => { const b = e.target.closest('button'); if (b) setView(b.dataset.v); });
$('mode').addEventListener('click', e => { const b = e.target.closest('button'); if (b) setMode(b.dataset.v); });
$('c-time').addEventListener('click', e => { const b = e.target.closest('button'); if (b) { S.ts = +b.dataset.v; syncControls(); } });
$('c-speed').addEventListener('input', e => { AP.vt = +e.target.value; if (S.mode === 'tunnel') st.V = AP.vt; $('c-speed-o').textContent = `${kt(AP.vt)} kt`; });
$('c-alt').addEventListener('input', e => { AP.ht = +e.target.value; if (S.mode === 'tunnel') st.h = AP.ht; $('c-alt-o').textContent = `${int(AP.ht)} m`; });
$('c-aoa').addEventListener('input', e => { S.aoa = +e.target.value; $('c-aoa-o').textContent = `${fmt(S.aoa, 1)}°`; });
$('c-span').addEventListener('input', e => { SL.P.wing.b = +e.target.value; $('c-span-o').textContent = `${fmt(SL.P.wing.b, 1)} m`; geomChanged(); });
$('c-sweep').addEventListener('input', e => { SL.P.wing.sweep = +e.target.value; $('c-sweep-o').textContent = `${fmt(SL.P.wing.sweep, 0)}°`; geomChanged(); });
$('c-wl').addEventListener('change', e => { SL.P.wing.wlH = e.target.checked ? 0.14 : 0; SL.P.wing.wlCant = 80; geomChanged(); });
$('c-cloud').addEventListener('change', e => { S.cloud = e.target.checked; const T = SL.isa(st.h).Tc; if (S.cloud) toast(liquid(T) > 0 ? `In cloud at ${fmt(T, 0)} °C: supercooled droplets will freeze on the wing.` : T >= 0 ? `In cloud at ${fmt(T, 0)} °C: above freezing, so no ice.` : `In cloud at ${fmt(T, 0)} °C: too cold, the cloud is ice crystals that don't stick.`); });
$('c-deice').addEventListener('click', () => { const b = Math.round(st.k * 1000); if (b < 1) { toast('No ice to remove.'); return; } st.k *= 0.12; toast(`De-iced: ${b} mm → ${Math.round(st.k * 1000)} mm.`); });
function processGeometry(now) {
  if (!geoDirty || now - geoT < 220) return;
  geoDirty = false; geoT = now; SL.changed('geometry');
  rebuildAircraft(false); buildMesh(); requestField(st.alpha); sectionUpdate(true);
}

// ====================================================== gauges
function gauges() {
  const lw = O.L / O.W, X = O.X, ratio = (st.alpha - ac.alpha0) / (X.alphaS - ac.alpha0);
  $('g-lw').textContent = `${Math.round(lw * 100)}%`; $('g-lw-bar').style.width = `${clamp(lw / 2, 0, 1) * 100}%`;
  $('g-lw-bar').style.background = lw < 0.95 ? 'var(--warn)' : 'var(--air)';
  $('g-st').textContent = O.stalled ? 'STALLED' : `${Math.round(clamp(ratio, 0, 1.5) * 100)}%`;
  $('g-st-bar').style.width = `${clamp(ratio, 0, 1) * 100}%`; $('g-st-bar').style.background = ratio >= 1 || O.stalled ? 'var(--bad)' : ratio > 0.8 ? 'var(--warn)' : 'var(--good)';
  $('g-v').textContent = `${kt(st.V)} kt`; $('g-h').textContent = `${int(st.h)} m`;
  $('g-vs').textContent = S.mode === 'fly' ? `${O.vs >= 0 ? '+' : '−'}${int(Math.abs(O.vs * 196.85))} fpm` : '—';
  $('g-a').textContent = `${fmt(st.alpha / D2R, 1)}°`; $('g-ld').textContent = fmt(O.L / Math.max(1, O.D), 1);
  $('g-ice').textContent = st.k > 0.0005 ? `${Math.round(st.k * 1000)} mm` : 'none';
  const el = $('g-state'), ban = $('banner');
  let txt, cls = '';
  if (O.stalled) { txt = 'Stalled'; cls = 'bad'; }
  else if (ratio > 0.85) { txt = 'Stall warning'; cls = 'warn'; }
  else if (S.mode === 'tunnel') txt = lw >= 1 ? 'Wind tunnel · enough lift to fly' : 'Wind tunnel · not enough lift';
  else if (Math.abs(O.vs) > 1.5) txt = O.vs > 0 ? 'Climbing' : 'Descending';
  else txt = S.cloud && liquid(O.atm.Tc) > 0 ? 'Cruising in icing cloud' : 'Cruising';
  el.textContent = txt; el.className = 'state ' + cls;
  ban.hidden = !cls; ban.className = cls; ban.textContent = O.stalled ? 'Stall' : 'Stall warning';
  cam.shake = O.stalled ? 0.012 * L_ref : 0;
}

// ====================================================== guided experiments
let EXP = null, step = 0, stepT = 0;
const tween = (get, set, to, dur) => tweens.push({ from: get(), to, set, t: 0, dur });
const tw = { aoa: to => tween(() => S.aoa, v => S.aoa = v, to, 0), span: (to, d) => tween(() => SL.P.wing.b, v => { SL.P.wing.b = v; geomChanged(); }, to, d), sweep: (to, d) => tween(() => SL.P.wing.sweep, v => { SL.P.wing.sweep = v; geomChanged(); }, to, d), aoaT: (to, d) => tween(() => S.aoa, v => S.aoa = v, to, d) };
const EXPERIMENTS = [
  { title: 'How a wing makes lift', blurb: 'Watch the air speed up over the top.', steps: [
    { t: 'Air splits around the wing', b: 'Watch the streams of air. Above the wing they bunch together and turn <b>amber</b>: the air is moving faster. Below, they turn <b>blue</b>: slower. Faster air has lower pressure, so the wing is pushed up from below and pulled up from above.', go() { setMode('tunnel'); S.aoa = 3; setView('side'); } },
    { t: 'Tilt the wing up', b: 'The angle now rises from 3° to 10°. More air is turned downward, and <b>Lift compared with weight</b> climbs. The wing also glows amber where it is working hardest.', go() { setMode('tunnel'); tw.aoaT(10, 6); } },
    { t: 'In flight, lift balances weight', b: 'Back in flight, the autopilot holds the angle where lift is exactly <b>100% of weight</b>. That balance is what keeps the aircraft up.', go() { setMode('fly'); setView('chase'); } },
  ] },
  { title: 'Stall', blurb: 'Raise the nose until the air lets go.', steps: [
    { t: 'Smooth flow', b: 'At a moderate angle the white smoke in the <b>wing cross-section</b> (right) hugs the curved top of the wing.', go() { setMode('tunnel'); S.aoa = 5; setView('side'); } },
    { t: 'Keep raising the angle', b: 'The angle now climbs slowly towards 22°. Lift grows… until the air can no longer follow the top surface. Watch the cross-section and the <b>stall</b> gauge.', go() { setMode('tunnel'); tw.aoaT(22, 14); } },
    { t: 'The wing has stalled', b: 'The airflow has <b>separated</b>: the top of the wing sits in a turbulent wake, and lift drops even though the angle is high. The <b>red</b> patch shows where the wing gave up first.', go() { setMode('tunnel'); S.aoa = Math.max(S.aoa, 20); } },
    { t: 'Recover', b: 'Lowering the nose lets the flow reattach, and lift comes back. Pilots recover from a stall by pushing forward, not pulling back.', go() { setMode('tunnel'); tw.aoaT(5, 4); } },
  ] },
  { title: 'Wingtip vortices', blurb: 'Why wings leave a spinning wake.', steps: [
    { t: 'Air spills round the tips', b: 'The air under the wing is at higher pressure than the air above. At the tip it escapes upward and rolls into a spinning <b>vortex</b>. Watch the smoke near the tip curl as it streams behind.', go() { setMode('fly'); setView('tip'); } },
    { t: 'A longer wing', b: 'The span grows by half. The tip vortices now matter less to the whole wing, so less energy is wasted: watch <b>Lift ÷ drag</b> rise. This is why gliders have long wings.', go() { setMode('fly'); tw.span(base.wing.b * 1.5, 5); } },
    { t: 'Back to normal', b: 'Restoring the original span.', go() { tw.span(base.wing.b, 3); setView('chase'); } },
  ] },
  { title: 'Swept wings', blurb: 'Why airliners sweep, and the catch.', steps: [
    { t: 'A straight wing near its limit', b: 'At a high angle, a straight wing works hardest near the middle (amber).', go() { setMode('tunnel'); S.aoa = 13; setView('top'); } },
    { t: 'Sweep it back', b: 'Sweeping the wing back helps an airliner fly close to the speed of sound. But it shifts the load outward: the hot spot moves towards the <b>tips</b>, so the tips stall first, where the ailerons are.', go() { tw.sweep(35, 6); } },
    { t: 'Back to straight', b: 'Restoring the original sweep. Real swept wings add twist and other tricks to tame tip stall.', go() { tw.sweep(base.wing.sweep, 3); } },
  ] },
  { title: 'Flying into ice', blurb: 'Supercooled cloud, ice and a stall.', steps: [
    { t: 'Into a supercooled cloud', b: 'At 3,500 m it is about −8 °C. The cloud droplets are still liquid below freezing, and freeze the moment they hit the wing. Time runs 60× faster.', go() { setMode('fly'); AP.ht = 3500; st.h = 3500; AP.vt = Math.round(ac.vmin * 1.45 / 0.8 * 0.8); S.cloud = true; S.ts = 60; setView('chase'); syncControls(); } },
    { t: 'Ice builds on the leading edge', b: () => `Ice on the wing: <b>${Math.round(st.k * 1000)} mm</b>. It spoils the wing's shape: drag rises and the stall gauge creeps up as the autopilot raises the nose to hold height. Watch the cross-section too.`, go() {} },
    { t: 'Use the de-icer', b: 'Press <b>De-ice</b> in the controls to crack the ice off. If you wait too long, the wing stalls even at normal speed. That is how icing accidents happen.', go() {} },
  ] },
  { title: 'Winglets', blurb: 'Small fins, less wasted energy.', steps: [
    { t: 'Without winglets', b: () => `Note <b>Lift ÷ drag</b>: ${fmt(O.L / Math.max(1, O.D), 1)}.`, go() { setMode('fly'); SL.P.wing.wlH = 0; geomChanged(); setView('front'); } },
    { t: 'Add winglets', b: () => `Winglets block part of the spill around the tips and weaken the vortices. <b>Lift ÷ drag</b> is now ${fmt(O.L / Math.max(1, O.D), 1)}.`, go() { SL.P.wing.wlH = 0.14; SL.P.wing.wlCant = 80; geomChanged(); setView('tip'); } },
  ] },
];
function renderExp() {
  const el = $('exp');
  if (!EXP) {
    el.innerHTML = `<h2>Experiments</h2><p class="lede">Pick one and watch it play out, or simply fly: drag the controls at the bottom and the air reacts instantly.</p>
      <div class="exp-list">${EXPERIMENTS.map((x, i) => `<button class="exp-item" data-i="${i}"><span class="n">${i + 1}</span><span><b>${x.title}</b><span>${x.blurb}</span></span></button>`).join('')}</div>`;
    el.querySelectorAll('.exp-item').forEach(b => b.onclick = () => startExp(+b.dataset.i));
    return;
  }
  const s = EXP.steps[step];
  el.innerHTML = `<div class="step-k">${EXP.title} · step ${step + 1} of ${EXP.steps.length}</div>
    <p class="step-t">${s.t}</p><p class="step-b" id="step-b">${typeof s.b === 'function' ? s.b() : s.b}</p>
    <div class="dots">${EXP.steps.map((_, i) => `<i class="${i <= step ? 'on' : ''}"></i>`).join('')}</div>
    <div class="row"><button class="btn quiet" id="x-back">Back</button><button class="btn primary" id="x-next">${step < EXP.steps.length - 1 ? 'Next' : 'Finish'}</button><button class="btn quiet" id="x-stop">Exit</button></div>`;
  $('x-back').onclick = () => { if (step > 0) goStep(step - 1); };
  $('x-next').onclick = () => { if (step < EXP.steps.length - 1) goStep(step + 1); else stopExp(); };
  $('x-stop').onclick = stopExp;
}
function startExp(i) { EXP = EXPERIMENTS[i]; goStep(0); }
function goStep(i) { step = i; stepT = 0; tweens.length = 0; EXP.steps[i].go(); syncControls(); renderExp(); }
function stopExp() { if (!EXP) return; EXP = null; tweens.length = 0; S.ts = 1; renderExp(); syncControls(); }

// ====================================================== main loop
let last = performance.now(), frame = 0, colourT = 0;
function loop(now) {
  requestAnimationFrame(loop);
  const dt = Math.min(0.05, (now - last) / 1000); last = now; frame++;
  for (let i = tweens.length - 1; i >= 0; i--) { const t = tweens[i]; t.t += dt; const u = t.dur ? clamp(t.t / t.dur, 0, 1) : 1, e = u * u * (3 - 2 * u); t.set(t.from + (t.to - t.from) * e); if (u >= 1) tweens.splice(i, 1); }
  processGeometry(now);
  if (acVer !== SL.version) rebuildAircraft(false);
  if (S.mode === 'fly') { const sim = dt * S.ts, n = Math.max(1, Math.ceil(sim / 0.02)); for (let i = 0; i < n; i++) flightStep(sim / n); }
  else { st.alpha += (S.aoa * D2R - st.alpha) * Math.min(1, dt * 4); st.V = AP.vt; st.h += (AP.ht - st.h) * Math.min(1, dt * 2); st.gam = 0;
    const lf = S.cloud ? liquid(SL.isa(st.h).Tc) : 0; if (lf > 0) { st.k += collection(st.V) * S.lwc * 1e-3 * st.V / 900 * lf * dt * S.ts; } }
  outputs();
  if (Math.abs(st.alpha - fieldAlpha) > 0.35 * D2R || fieldVer !== SL.version) requestField(st.alpha);
  stepParticles(dt);
  if (now - colourT > 90) { colourT = now; colourWing(); buildIce(); sectionUpdate(false); }
  updateWorld(dt); resize(); updateCamera(dt);
  R.render(scene, camera);
  drawSection();
  if (frame % 6 === 0) { gauges(); if (EXP) { const s = EXP.steps[step]; if (typeof s.b === 'function') $('step-b').innerHTML = s.b(); if (EXP.title === 'Flying into ice' && step === 0 && st.k > 0.004) goStep(1); if (EXP.title === 'Flying into ice' && step === 1 && st.k > 0.03) goStep(2); if (EXP.title === 'Stall' && step === 1 && O.stalled) goStep(2); } }
}
setAircraft('trainer'); renderExp(); setView('chase');
requestAnimationFrame(loop);
})();
