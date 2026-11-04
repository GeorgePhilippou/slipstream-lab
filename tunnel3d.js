// 3D tunnel: D3Q19 lattice Boltzmann on WebGPU with a symmetry plane at z = 0 (half model)
(() => {
const { $, fmt, clamp, D2R } = SL;
const U0 = 0.08, CS = 0.17;
const RES3 = { low: [128, 64, 64], med: [160, 80, 80], high: [208, 104, 104] };
const V = { voxview: false, model: 'crm', ar: 5, aoa: 8, re: 0, reSlider: 600, ice: false, slice: 1, spos: 0.62, sq: 2, smoke: true, spf: 6, res: 'med', running: true };
const canvas = $('t-view');
let adapter, device, gctx, format, ready = false, failed = false, active = false, initing = false;
let NX, NY, NZ, N, info = null;
let bufA, bufB, macBuf, solidBuf, forceBuf, stageBuf, paramBuf, partBuf, partUBuf, renderUBuf, volTex, sampler, depthTex;
let lbmPipe, partPipe, rayPipe, linePipe, meshPipe, meshUBuf, bgMesh, bgAB, bgBA, bgPart, bgRay, bgLine, flip = false, lastVP = null, lastEye = [0, 0, 0];
let stepsTotal = 0, stepsSince = 0, reading = false, frameNo = 0, lastRateT = 0, lastRateSteps = 0, PCOUNT = 0;
const hist = { cl: [], cd: [] };
let lastF = null;
const cam = { yaw: 2.35, pitch: 0.42, dist: 1, target: [0, 0, 0] };

// ---------- WGSL ----------
const LBM_WGSL = /* wgsl */`
struct Params { nx:u32, ny:u32, nz:u32, n:u32, u0:f32, tau0:f32, cs2:f32, mode:u32 };
@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read> fin: array<f32>;
@group(0) @binding(2) var<storage, read_write> fout: array<f32>;
@group(0) @binding(3) var<storage, read> solid: array<u32>;
@group(0) @binding(4) var<storage, read_write> mac: array<vec4f>;
@group(0) @binding(5) var<storage, read_write> force: array<atomic<i32>, 4>;
var<private> CX: array<i32,19> = array<i32,19>(0, 1,-1,0,0,0,0, 1,-1,1,-1, 1,-1,1,-1, 0,0,0,0);
var<private> CY: array<i32,19> = array<i32,19>(0, 0,0,1,-1,0,0, 1,-1,-1,1, 0,0,0,0, 1,-1,1,-1);
var<private> CZ: array<i32,19> = array<i32,19>(0, 0,0,0,0,1,-1, 0,0,0,0, 1,-1,-1,1, 1,-1,-1,1);
var<private> OPP: array<u32,19> = array<u32,19>(0u, 2u,1u,4u,3u,6u,5u, 8u,7u,10u,9u, 12u,11u,14u,13u, 16u,15u,18u,17u);
var<private> MIR: array<u32,19> = array<u32,19>(0u, 1u,2u,3u,4u,6u,5u, 7u,8u,9u,10u, 13u,14u,11u,12u, 17u,18u,15u,16u);
var<private> WT: array<f32,19> = array<f32,19>(1.0/3.0,
  1.0/18.0,1.0/18.0,1.0/18.0,1.0/18.0,1.0/18.0,1.0/18.0,
  1.0/36.0,1.0/36.0,1.0/36.0,1.0/36.0,1.0/36.0,1.0/36.0,1.0/36.0,1.0/36.0,1.0/36.0,1.0/36.0,1.0/36.0,1.0/36.0);
fn cvec(i: u32) -> vec3f { return vec3f(f32(CX[i]), f32(CY[i]), f32(CZ[i])); }
fn feq(i: u32, r: f32, u: vec3f, usq: f32) -> f32 { let cu = 3.0 * dot(cvec(i), u); return WT[i] * r * (1.0 + cu + 0.5 * cu * cu - usq); }
@compute @workgroup_size(32, 4, 1)
fn main(@builtin(global_invocation_id) gid: vec3u) {
  let x = gid.x; let y = gid.y; let z = gid.z;
  if (x >= P.nx || y >= P.ny || z >= P.nz) { return; }
  let N = P.n; let k = x + P.nx * (y + P.ny * z);
  let uin = vec3f(P.u0, 0.0, 0.0); let usqin = 1.5 * P.u0 * P.u0;
  if (P.mode == 1u) {
    for (var i = 0u; i < 19u; i++) { fout[i * N + k] = feq(i, 1.0, uin, usqin); }
    mac[k] = vec4f(uin, 1.0); return;
  }
  if (solid[k] != 0u) { mac[k] = vec4f(0.0, 0.0, 0.0, 1.0); return; }
  if (x == 0u || y == 0u || y == P.ny - 1u || z == P.nz - 1u) {
    for (var i = 0u; i < 19u; i++) { fout[i * N + k] = feq(i, 1.0, uin, usqin); }
    mac[k] = vec4f(uin, 1.0); return;
  }
  var f: array<f32, 19>;
  if (x == P.nx - 1u) {
    for (var i = 0u; i < 19u; i++) { f[i] = fin[i * N + k - 1u]; }
  } else {
    var F = vec3f(0.0);
    for (var i = 0u; i < 19u; i++) {
      let sx = i32(x) - CX[i]; let sy = i32(y) - CY[i]; var sz = i32(z) - CZ[i]; var d = i;
      if (sz < 0) { sz = 0; d = MIR[i]; }
      let s = u32(sx) + P.nx * (u32(sy) + P.ny * u32(sz));
      if (solid[s] != 0u) { let v = fin[OPP[i] * N + k]; f[i] = v; F -= 2.0 * v * cvec(i); }
      else { f[i] = fin[d * N + s]; }
    }
    if (F.x != 0.0 || F.y != 0.0) {
      atomicAdd(&force[0], i32(round(F.x * 100000.0)));
      atomicAdd(&force[1], i32(round(F.y * 100000.0)));
    }
  }
  var r = 0.0; var m = vec3f(0.0);
  for (var i = 0u; i < 19u; i++) { r += f[i]; m += f[i] * cvec(i); }
  r = max(r, 0.2);
  var u = m / r; let sp = length(u); if (sp > 0.3) { u = u * (0.3 / sp); }
  let usq = 1.5 * dot(u, u);
  var fe: array<f32, 19>;
  var pxx = 0.0; var pyy = 0.0; var pzz = 0.0; var pxy = 0.0; var pxz = 0.0; var pyz = 0.0;
  for (var i = 0u; i < 19u; i++) {
    fe[i] = feq(i, r, u, usq); let q = f[i] - fe[i]; let c = cvec(i);
    pxx += c.x * c.x * q; pyy += c.y * c.y * q; pzz += c.z * c.z * q; pxy += c.x * c.y * q; pxz += c.x * c.z * q; pyz += c.y * c.z * q;
  }
  let Q = sqrt(pxx * pxx + pyy * pyy + pzz * pzz + 2.0 * (pxy * pxy + pxz * pxz + pyz * pyz));
  let tau = 0.5 * (P.tau0 + sqrt(P.tau0 * P.tau0 + 18.0 * 1.41421356 * P.cs2 * Q / r));
  let om = 1.0 / tau;
  for (var i = 0u; i < 19u; i++) { fout[i * N + k] = f[i] - om * (f[i] - fe[i]); }
  mac[k] = vec4f(u, r);
}`;

const PART_WGSL = /* wgsl */`
struct PU { dims: vec4u, a: vec4f, b: vec4f };
@group(0) @binding(0) var<uniform> U: PU;
@group(0) @binding(1) var<storage, read> mac: array<vec4f>;
@group(0) @binding(2) var<storage, read> solid: array<u32>;
@group(0) @binding(3) var<storage, read_write> parts: array<vec4f>;
fn cidx(i: vec3i) -> u32 { let c = clamp(i, vec3i(0), vec3i(U.dims.xyz) - vec3i(1)); return u32(c.x) + U.dims.x * (u32(c.y) + U.dims.y * u32(c.z)); }
fn vel(p: vec3f) -> vec3f {
  let q = p - vec3f(0.5); let i0 = vec3i(floor(q)); let t = q - floor(q);
  var s = vec3f(0.0);
  for (var dz = 0; dz < 2; dz++) { for (var dy = 0; dy < 2; dy++) { for (var dx = 0; dx < 2; dx++) {
    let w = select(1.0 - t.x, t.x, dx == 1) * select(1.0 - t.y, t.y, dy == 1) * select(1.0 - t.z, t.z, dz == 1);
    s += w * mac[cidx(i0 + vec3i(dx, dy, dz))].xyz;
  } } }
  return s;
}
fn hash(v: u32) -> f32 { var x = v * 747796405u + 2891336453u; x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u; x = (x >> 22u) ^ x; return f32(x) / 4294967295.0; }
@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) g: vec3u) {
  let i = g.x; if (i >= U.dims.w) { return; }
  let p = parts[2u * i]; var pos = p.xyz; var age = p.w;
  let dt = U.a.x;
  let u1 = vel(pos); let u2 = vel(pos + u1 * dt * 0.5);
  pos += u2 * dt; age += dt;
  if (pos.z < 0.0) { pos.z = -pos.z; }
  let n = vec3f(U.dims.xyz);
  var dead = pos.x > n.x - 2.0 || pos.x < 1.0 || pos.y < 1.0 || pos.y > n.y - 2.0 || pos.z > n.z - 2.0 || age > U.b.y;
  if (!dead) { if (solid[cidx(vec3i(floor(pos)))] != 0u) { dead = true; } }
  if (dead) {
    let h = i * 3u + u32(U.b.z) * 7919u;
    let ly = floor(hash(h + 1u) * 7.0) / 6.0; let lz = floor(hash(h + 2u) * 18.0) / 17.0;
    pos = vec3f(U.b.x + hash(h) * 3.0, mix(U.a.y, U.a.z, ly) + (hash(h + 5u) - 0.5) * 0.5, max(0.2, lz * U.a.w + (hash(h + 7u) - 0.5) * 0.5));
    age = 0.0;
  }
  parts[2u * i] = vec4f(pos, age);
  parts[2u * i + 1u] = vec4f(u2, length(u2));
}`;

const ramp = key => {
  const s = SL.STOPS[key].map(SL.hexRGB).map(c => `vec3f(${(c[0] / 255).toFixed(4)},${(c[1] / 255).toFixed(4)},${(c[2] / 255).toFixed(4)})`);
  return `array<vec3f,7>(${s.join(',')})`;
};
const COMMON_WGSL = /* wgsl */`
struct R { vp: mat4x4f, ivp: mat4x4f, cam: vec4f, dims: vec4f, bbmin: vec4f, bbmax: vec4f, slice: vec4f, misc: vec4f, view: vec4f };
@group(0) @binding(0) var<uniform> U: R;
fn ramp(t: f32, which: i32) -> vec3f {
  var s = ${ramp('speed')};
  if (which == 1) { s = ${ramp('pressure')}; } else if (which == 2) { s = ${ramp('vorticity')}; }
  let x = clamp(t, 0.0, 1.0) * 6.0; let i = min(i32(floor(x)), 5); let f = x - f32(i);
  return mix(s[i], s[i + 1], f);
}`;

const RAY_WGSL = COMMON_WGSL + /* wgsl */`
@group(0) @binding(1) var vol: texture_3d<f32>;
@group(0) @binding(2) var smp: sampler;
@group(0) @binding(3) var<storage, read> mac: array<vec4f>;
struct FO { @location(0) col: vec4f, @builtin(frag_depth) depth: f32 };
@vertex fn vs(@builtin(vertex_index) vi: u32) -> @builtin(position) vec4f {
  var p = array<vec2f, 3>(vec2f(-1.0, -1.0), vec2f(3.0, -1.0), vec2f(-1.0, 3.0));
  return vec4f(p[vi], 0.0, 1.0);
}
fn vsamp(p: vec3f) -> vec2f { return textureSampleLevel(vol, smp, vec3f(p.x / U.dims.x, p.y / U.dims.y, abs(p.z) / U.dims.z), 0.0).rg; }
fn boxHit(ro: vec3f, rd: vec3f, bmin: vec3f, bmax: vec3f) -> vec2f {
  let inv = 1.0 / rd; let t0 = (bmin - ro) * inv; let t1 = (bmax - ro) * inv;
  let tmin = min(t0, t1); let tmax = max(t0, t1);
  return vec2f(max(max(tmin.x, tmin.y), tmin.z), min(min(tmax.x, tmax.y), tmax.z));
}
fn macAt(c0: vec3i) -> vec4f {
  var c = c0; var sg = 1.0;
  if (c.z < 0) { c.z = -c.z - 1; sg = -1.0; }
  let d = vec3i(U.dims.xyz);
  c = clamp(c, vec3i(0), d - vec3i(1));
  let m = mac[u32(c.x) + u32(d.x) * (u32(c.y) + u32(d.y) * u32(c.z))];
  return vec4f(m.x, m.y, m.z * sg, m.w);
}
fn sliceCol(p: vec3f, ax: i32) -> vec4f {
  if (vsamp(p).r > 0.5) { return vec4f(0.22, 0.25, 0.29, 0.95); }
  let c = vec3i(floor(p)); let m = macAt(c); let q = i32(U.slice.z); let u0 = U.misc.x;
  if (q == 0) { return vec4f(ramp(length(m.xyz) / u0 / 1.8, 0), 0.9); }
  if (q == 1) { let cp = (m.w - 1.0) / (1.5 * u0 * u0); var t = 0.5 + 0.5 * cp; if (cp < 0.0) { t = 0.5 + 0.5 * cp / 2.5; } return vec4f(ramp(t, 1), clamp(abs(t - 0.5) * 7.0, 0.12, 0.92)); }
  var w = 0.0;
  if (ax == 0) { w = macAt(c + vec3i(0, 1, 0)).z - macAt(c - vec3i(0, 1, 0)).z - macAt(c + vec3i(0, 0, 1)).y + macAt(c - vec3i(0, 0, 1)).y; }
  else if (ax == 1) { w = macAt(c + vec3i(0, 0, 1)).x - macAt(c - vec3i(0, 0, 1)).x - macAt(c + vec3i(1, 0, 0)).z + macAt(c - vec3i(1, 0, 0)).z; }
  else { w = macAt(c + vec3i(1, 0, 0)).y - macAt(c - vec3i(1, 0, 0)).y - macAt(c + vec3i(0, 1, 0)).x + macAt(c - vec3i(0, 1, 0)).x; }
  w = 0.5 * w * U.misc.w / u0;
  let tv = 0.5 + 0.5 * tanh(w / 5.0);
  return vec4f(ramp(tv, 2), clamp(abs(tv - 0.5) * 6.0, 0.06, 0.95));
}
@fragment fn fs(@builtin(position) fc: vec4f) -> FO {
  let ndc = vec2f(fc.x / U.view.x * 2.0 - 1.0, 1.0 - fc.y / U.view.y * 2.0);
  var a = U.ivp * vec4f(ndc, 0.0, 1.0); a = a / a.w;
  var b = U.ivp * vec4f(ndc, 1.0, 1.0); b = b / b.w;
  let ro = a.xyz; let rd = normalize(b.xyz - a.xyz);
  var col = mix(vec3f(0.015, 0.035, 0.06), vec3f(0.07, 0.12, 0.17), clamp(ndc.y * 0.5 + 0.5, 0.0, 1.0));
  var depthT = 1e9;
  let D = U.dims.xyz;
  // floor grid at y = 0
  if (rd.y < 0.0) {
    let t = -ro.y / rd.y; let p = ro + rd * t;
    if (p.x > 0.0 && p.x < D.x && abs(p.z) < D.z) {
      let g = abs(fract(p.xz / 10.0) - 0.5); let line = 1.0 - smoothstep(0.0, 0.04, min(g.x, g.y));
      col = mix(col, vec3f(0.09, 0.15, 0.2), 0.8) + line * vec3f(0.06, 0.1, 0.13);
      depthT = t;
    }
  }
  // body
  var tb = 1e9;
  let hb = boxHit(ro, rd, U.bbmin.xyz, U.bbmax.xyz);
  if (hb.y > max(hb.x, 0.0) && U.bbmax.w > 0.5) {
    var t = max(hb.x, 0.0);
    for (var it = 0; it < 900; it++) {
      if (t > hb.y) { break; }
      if (vsamp(ro + rd * t).r > 0.5) {
        var lo = t - 0.35; var hi = t;
        for (var j = 0; j < 6; j++) { let mid = 0.5 * (lo + hi); if (vsamp(ro + rd * mid).r > 0.5) { hi = mid; } else { lo = mid; } }
        tb = hi; break;
      }
      t += 0.35;
    }
  }
  if (tb < 1e8) {
    let p = ro + rd * tb; let e = 0.6;
    let gr = vec3f(vsamp(p + vec3f(e, 0, 0)).r - vsamp(p - vec3f(e, 0, 0)).r,
                   vsamp(p + vec3f(0, e, 0)).r - vsamp(p - vec3f(0, e, 0)).r,
                   vsamp(p + vec3f(0, 0, e)).r - vsamp(p - vec3f(0, 0, e)).r);
    var n = -normalize(gr + vec3f(1e-6));
    if (dot(n, rd) > 0.0) { n = -n; }
    let ice = vsamp(p - n * 0.6).g;
    let L1 = normalize(vec3f(-0.35, 0.85, 0.4));
    let diff = max(dot(n, L1), 0.0); let hemi = 0.5 + 0.5 * n.y;
    let h = normalize(L1 - rd); let spec = pow(max(dot(n, h), 0.0), 40.0);
    let base = mix(vec3f(0.80, 0.83, 0.87), vec3f(0.78, 0.93, 1.0), ice);
    let rim = pow(1.0 - max(dot(n, -rd), 0.0), 3.0);
    col = base * (0.22 + 0.25 * hemi + 0.6 * diff) + spec * mix(0.25, 0.7, ice) + rim * vec3f(0.12, 0.2, 0.28);
    depthT = tb;
  }
  // slice plane
  if (U.slice.x > 0.5) {
    let ax = i32(U.slice.x) - 1;
    let ts = (U.slice.y - ro[ax]) / rd[ax];
    if (ts > 0.0 && ts < tb) {
      let p = ro + rd * ts;
      if (p.x >= 0.0 && p.x <= D.x && p.y >= 0.0 && p.y <= D.y && abs(p.z) <= D.z) {
        let sc = sliceCol(p, ax); col = mix(col, sc.rgb, sc.a); if (sc.a > 0.5) { depthT = ts; }
      }
    }
  }
  var o: FO; o.col = vec4f(col, 1.0);
  if (depthT < 1e8) { let c = U.vp * vec4f(ro + rd * depthT, 1.0); o.depth = clamp(c.z / c.w, 0.0, 1.0); } else { o.depth = 1.0; }
  return o;
}`;

const LINE_WGSL = COMMON_WGSL + /* wgsl */`
@group(0) @binding(1) var<storage, read> parts: array<vec4f>;
struct VO { @builtin(position) pos: vec4f, @location(0) col: vec4f };
@vertex fn vs(@builtin(vertex_index) vi: u32, @builtin(instance_index) ii: u32) -> VO {
  let pi = vi / 2u; let e = vi % 2u;
  let p = parts[2u * pi]; let v = parts[2u * pi + 1u];
  var w = p.xyz;
  if (e == 1u) { w = w - v.xyz * U.misc.y; }
  if (ii == 1u) { w.z = -w.z; }
  var o: VO; o.pos = U.vp * vec4f(w, 1.0);
  let s = clamp(v.w / U.misc.x / 1.6, 0.0, 1.0);
  let fade = min(1.0, p.w / 30.0);
  let c = mix(vec3f(0.55, 0.75, 1.0), vec3f(1.0, 0.95, 0.8), s);
  o.col = vec4f(c, select(0.0, 0.5, e == 0u) * fade);
  return o;
}
@fragment fn fs(i: VO) -> @location(0) vec4f { return i.col; }`;

// ---------- real geometry ----------
const MODELS = {
  crm: { file: 'models/nasa-crm.slm', name: 'NASA Common Research Model',
    info: 'A modern long-range airliner wing, body and tail, designed by NASA so researchers worldwide could test and compare CFD codes against one shared shape. This is the triangulated surface used in the 4th AIAA Drag Prediction Workshop.',
    facts: 'Full scale: semispan 29.4 m, mean chord 7.0 m, wing area 383.7 m², aspect ratio 9.0, quarter-chord sweep 35°. Wind-tunnel design point: C<sub>L</sub> 0.50 at Mach 0.85.',
    credit: 'Geometry: NASA Langley Research Center (NASA CRM), via the DAFoam DPW-4 tutorial files.',
    ref: { Shalf: 191.84, cref: 7.00532 }, AR: 9.0, sweep: 35, sweepNote: 'quarter-chord sweep' },
  m6: { file: 'models/onera-m6.slm', name: 'ONERA M6 wing',
    info: 'The best-known swept test wing in aerodynamics. ONERA measured its surface pressures in the S2MA wind tunnel in the 1970s, and nearly every CFD code since has been checked against it.',
    facts: 'Semispan 1.196 m, mean chord 0.646 m, aspect ratio 3.8, leading-edge sweep 30°, symmetric ONERA D section. Classic test: Mach 0.84, α 3.06°, Re 11.7 million.',
    credit: 'Geometry: ONERA M6 surface from the SU2 project’s validation meshes.',
    ref: { Shalf: 0.7532, cref: 0.64607 }, AR: 3.8, sweep: 23.3, sweepNote: 'mid-chord sweep' },
  x57: { file: 'models/nasa-x57.slm', name: 'NASA X-57 Maxwell',
    info: 'NASA’s all-electric X-plane. Its very slender cruise wing (aspect ratio about 15) is efficient at speed; twelve small propellers along the leading edge blow extra air over it for take-off and landing.',
    facts: 'Span 9.6 m. Reference area here is estimated from the model’s planform, so treat coefficients as approximate.',
    credit: 'Model: NASA 3D Resources (public domain).' },
  gh: { file: 'models/nasa-global-hawk.slm', name: 'NASA Global Hawk',
    info: 'NASA flew Global Hawks for Earth-science missions above 18 km. The enormous, thin wing is what lets it stay up in such thin air for more than a day.',
    facts: 'Span 39.9 m, length 14.5 m. Reference area here is estimated from the model’s planform. The landing gear is down in this model.',
    credit: 'Model: NASA 3D Resources (public domain).' },
};
const isMesh = () => V.model === 'custom' || !!(MODELS[V.model] && MODELS[V.model].file);
const meshCache = {};
let custom = null;   // {name, tris, o: orientation}
const mw = new Worker('mesh-worker.js');
let ridN = 0; const rpcWait = {};
mw.onmessage = e => { const m = e.data, w = rpcWait[m.rid]; if (!w) return; delete rpcWait[m.rid]; m.type === 'error' ? w.rej(new Error(m.message)) : w.res(m); };
const rpc = (msg, tr = []) => new Promise((res, rej) => { const rid = ++ridN; rpcWait[rid] = { res, rej }; mw.postMessage({ ...msg, rid }, tr); });
const IDENT = [1, 0, 0, 0, 1, 0, 0, 0, 1];
function orientR(o) { const M = [0, 0, 0, 0, 0, 0, 0, 0, 0]; M[o.len] = -o.noseSign; M[3 + o.up] = o.upSign; M[6 + o.span] = 1; return M; }
async function buildMesh(key, R) {
  const g = await rpc({ type: 'geometry', id: key, R });
  const old = meshCache[key]; if (old) { old.vb.destroy(); old.ib.destroy(); }
  const vb = device.createBuffer({ size: g.pos.byteLength, usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST });
  const ib = device.createBuffer({ size: g.idx.byteLength, usage: GPUBufferUsage.INDEX | GPUBufferUsage.COPY_DST });
  device.queue.writeBuffer(vb, 0, g.pos); device.queue.writeBuffer(ib, 0, g.idx);
  return (meshCache[key] = { id: key, vb, ib, n: g.idx.length, lo: g.lo, hi: g.hi, R });
}
async function ensureMesh(key) {
  if (meshCache[key]) return meshCache[key];
  if (key === 'custom') return null;
  const M = MODELS[key];
  $('t-chip').textContent = `Loading ${M.name}…`; $('t-chip').className = 'chip neutral';
  let buf, r = await fetch(M.file).catch(() => null);
  if (r && r.ok && !(r.headers.get('content-type') || '').includes('text/html')) buf = await r.arrayBuffer();
  else {   // fallback: base64 copy (used where the host can't serve binary files)
    r = await fetch(M.file + '.txt'); if (!r.ok) throw new Error(`Could not load ${M.file} (${r.status})`);
    const bin = atob((await r.text()).trim()), u8 = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
    buf = u8.buffer;
  }
  await rpc({ type: 'parse', id: key, name: M.file, buf, orient: 'none' }, [buf]);
  return buildMesh(key, IDENT);
}
function meshT(mc) {
  const lo = mc.lo, hi = mc.hi, L = hi[0] - lo[0], Hh = hi[1] - lo[1], half = (hi[2] - lo[2]) / 2;
  const s = Math.min(0.5 * NX / L, 0.62 * NZ / half, 0.42 * NY / Hh);
  return { s, cx: (lo[0] + hi[0]) / 2, cy: (lo[1] + hi[1]) / 2, cz: (lo[2] + hi[2]) / 2, aoa: V.aoa, px: 0.2 * NX + 0.5 * L * s, py: 0.5 * NY, R: mc.R };
}
function meshInfo(mc, T, r) {
  const M = MODELS[V.model] || {}, s = T.s, a = V.aoa * D2R, ca = Math.cos(a), sa = Math.sin(a), bb = r.bb;
  const half = (mc.hi[2] - T.cz) * s;
  const Shalf = M.ref ? M.ref.Shalf * s * s : r.area, cref = M.ref ? M.ref.cref * s : Shalf / Math.max(1, half);
  const model = [s * ca, -s * sa, 0, 0, s * sa, s * ca, 0, 0, 0, 0, s, 0, T.px - s * (ca * T.cx + sa * T.cy), T.py - s * (-sa * T.cx + ca * T.cy), -s * T.cz, 1];
  return { cref, Shalf, span: half, px: T.px, py: T.py, seedY: [Math.max(2, bb[1] - 3), Math.min(NY - 3, bb[4] + 3)], seedZ: Math.min(NZ - 3, bb[5] * 1.2 + 2),
    bbmin: [bb[0] - 1, bb[1] - 1, -(bb[5] + 2)], bbmax: [bb[3] + 2, bb[4] + 2, bb[5] + 2], count: r.count, iceCells: 0, model, mesh: mc, estimated: !M.ref };
}
const MESH_WGSL = `
struct MU { vp: mat4x4f, model: mat4x4f, eye: vec4f };
@group(0) @binding(0) var<uniform> M: MU;
struct VO { @builtin(position) pos: vec4f, @location(0) wp: vec3f };
@vertex fn vs(@location(0) p: vec3f) -> VO { var o: VO; let w = M.model * vec4f(p, 1.0); o.wp = w.xyz; o.pos = M.vp * w; return o; }
@fragment fn fs(i: VO) -> @location(0) vec4f {
  var n = normalize(cross(dpdx(i.wp), dpdy(i.wp)));
  let v = normalize(M.eye.xyz - i.wp);
  if (dot(n, v) < 0.0) { n = -n; }
  let L1 = normalize(vec3f(-0.35, 0.85, 0.4));
  let diff = max(dot(n, L1), 0.0); let hemi = 0.5 + 0.5 * n.y;
  let h = normalize(L1 + v); let spec = pow(max(dot(n, h), 0.0), 50.0);
  let rim = pow(1.0 - max(dot(n, v), 0.0), 3.0);
  let base = vec3f(0.84, 0.86, 0.9);
  return vec4f(base * (0.22 + 0.25 * hemi + 0.6 * diff) + spec * 0.3 + rim * vec3f(0.12, 0.2, 0.28), 1.0);
}`;
// ---------- matrices (column-major) ----------
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function persp(fovy, asp, n, f) { const t = 1 / Math.tan(fovy / 2); return [t / asp, 0, 0, 0, 0, t, 0, 0, 0, 0, f / (n - f), -1, 0, 0, n * f / (n - f), 0]; }
function lookAt(e, c, up) {
  const z = norm(sub(e, c)), x = norm(cross(up, z)), y = cross(z, x);
  return [x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot3(x, e), -dot3(y, e), -dot3(z, e), 1];
}
function mul(a, b) { const o = new Array(16); for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) { let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s; } return o; }
function inv(m) {
  const [a00, a01, a02, a03, a10, a11, a12, a13, a20, a21, a22, a23, a30, a31, a32, a33] = m;
  const b00 = a00 * a11 - a01 * a10, b01 = a00 * a12 - a02 * a10, b02 = a00 * a13 - a03 * a10, b03 = a01 * a12 - a02 * a11, b04 = a01 * a13 - a03 * a11, b05 = a02 * a13 - a03 * a12;
  const b06 = a20 * a31 - a21 * a30, b07 = a20 * a32 - a22 * a30, b08 = a20 * a33 - a23 * a30, b09 = a21 * a32 - a22 * a31, b10 = a21 * a33 - a23 * a31, b11 = a22 * a33 - a23 * a32;
  const d = 1 / (b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06);
  return [(a11 * b11 - a12 * b10 + a13 * b09) * d, (a02 * b10 - a01 * b11 - a03 * b09) * d, (a31 * b05 - a32 * b04 + a33 * b03) * d, (a22 * b04 - a21 * b05 - a23 * b03) * d,
    (a12 * b08 - a10 * b11 - a13 * b07) * d, (a00 * b11 - a02 * b08 + a03 * b07) * d, (a32 * b02 - a30 * b05 - a33 * b01) * d, (a20 * b05 - a22 * b02 + a23 * b01) * d,
    (a10 * b10 - a11 * b08 + a13 * b06) * d, (a01 * b08 - a00 * b10 - a03 * b06) * d, (a30 * b04 - a31 * b02 + a33 * b00) * d, (a21 * b02 - a20 * b04 - a23 * b00) * d,
    (a11 * b07 - a10 * b09 - a12 * b06) * d, (a00 * b09 - a01 * b07 + a02 * b06) * d, (a31 * b01 - a30 * b03 - a32 * b00) * d, (a20 * b03 - a21 * b01 + a22 * b00) * d];
}

// ---------- geometry / voxelization ----------
function iceBitmap() {
  const pts = SL.ice.points;
  if (!pts || !pts.length || SL.ice.shape !== 'airfoil') return null;
  const X0 = -0.3, X1 = 1.05, Y0 = -0.4, Y1 = 0.4, res = 0.004, w = Math.ceil((X1 - X0) / res), h = Math.ceil((Y1 - Y0) / res);
  const bm = new Uint8Array(w * h), r = Math.max(1, Math.ceil(SL.ice.cell * 0.7 / res));
  for (let i = 0; i < pts.length; i += 2) {
    const cx = Math.round((pts[i] - X0) / res), cy = Math.round((pts[i + 1] - Y0) / res);
    for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) { const x = cx + dx, y = cy + dy; if (x >= 0 && y >= 0 && x < w && y < h) bm[y * w + x] = 1; }
  }
  return (xc, yy) => { const i = ((xc - X0) / res) | 0, j = ((yy - Y0) / res) | 0; return i >= 0 && j >= 0 && i < w && j < h && bm[j * w + i] === 1; };
}
function voxelize() {
  const solid = new Uint32Array(N), tex = new Uint8Array(N * 2);
  const a = V.aoa * D2R, ca = Math.cos(a), sa = Math.sin(a);
  const sec = SL.section, m = sec.m / 100, p = sec.p / 10, t = sec.t / 100;
  const iceT = V.ice ? iceBitmap() : null;
  const secTest = (xc, yy, mm, pp, tt, useIce) => {
    if (xc >= 0 && xc <= 1) { const yc = SL.naca.yc(mm, pp, xc), ht = SL.naca.yt(tt, xc); if (Math.abs(yy - yc) <= ht) return 1; }
    if (useIce && iceT && xc > -0.3 && xc < 1.05 && iceT(xc, yy)) return 2;
    return 0;
  };
  let test, px, py, ext;
  if (V.model === 'wing') {
    const c = Math.round(Math.min(0.18 * NX, 2 * 0.62 * NZ / V.ar)), s = V.ar * c / 2;
    px = 0.3 * NX; py = NY / 2;
    info = { cref: c, Shalf: c * s, span: s, px, py, seedY: [py - 0.7 * c, py + 0.7 * c], seedZ: Math.min(NZ - 3, s * 1.35) };
    ext = [-0.6 * c, 1.0 * c, -0.6 * c, 0.6 * c, s + 1];
    test = (bx, by, bz) => bz > s ? 0 : secTest(bx / c + 0.25, by / c, m, p, t, true);
  } else {
    const Lf = Math.round(0.55 * NX);
    px = 0.22 * NX + 0.42 * Lf; py = NY * 0.48;
    const t27 = Math.tan(27 * D2R), t5 = Math.tan(5 * D2R), t32 = Math.tan(32 * D2R), t38 = Math.tan(38 * D2R), inc = Math.tan(2 * D2R);
    const cr = 0.2, ct = 0.07, semi = 0.48;
    info = { cref: (cr + ct) / 2 * Lf, Shalf: semi * (cr + ct) / 2 * Lf * Lf, span: semi * Lf, px, py, seedY: [py - 0.25 * Lf, py + 0.3 * Lf], seedZ: Math.min(NZ - 3, 0.62 * Lf) };
    ext = [-0.45 * Lf, 0.62 * Lf, -0.3 * Lf, 0.35 * Lf, 0.56 * Lf];
    const R0 = 0.055;
    const rad = X => X < 0 || X > 1 ? -1 : X < 0.15 ? R0 * Math.sqrt(Math.max(0, 1 - ((0.15 - X) / 0.15) ** 2)) : X < 0.65 ? R0 : R0 * (1 - 0.82 * ((X - 0.65) / 0.35) ** 1.6);
    const ycf = X => X > 0.65 ? 0.035 * ((X - 0.65) / 0.35) ** 2 : 0;
    test = (bx, by, bz) => {
      const X = bx / Lf + 0.42, Y = by / Lf, Z = bz / Lf;
      const R = rad(X); if (R > 0 && (Y - ycf(X)) ** 2 + Z * Z <= R * R) return 1;
      if (Z <= semi) {   // main wing
        const ch = cr + (ct - cr) * Z / semi, xle = 0.34 + Z * t27, xc = (X - xle) / ch;
        if (xc > -0.3 && xc < 1.05) { const yy = (Y - (-0.03 + Z * t5)) / ch + (xc - 0.25) * inc; const r = secTest(xc, yy, m, p, t, true); if (r) return r; }
      }
      if (Z <= 0.17) {   // horizontal tail
        const ch = 0.1 - 0.05 * Z / 0.17, xle = 0.84 + Z * t32, xc = (X - xle) / ch;
        if (xc > -0.3 && xc < 1.05) { const yy = (Y - (0.035 + Z * Math.tan(6 * D2R))) / ch; const r = secTest(xc, yy, 0, 0.4, 0.11, true); if (r) return r; }
      }
      if (Y > 0.03 && Y < 0.21) {   // fin
        const eta = (Y - 0.03) / 0.18, ch = 0.16 - 0.08 * eta, xle = 0.79 + (Y - 0.03) * t38, xc = (X - xle) / ch;
        if (xc >= 0 && xc <= 1 && Z <= SL.naca.yt(0.1, xc) * ch) return 1;
      }
      if (X > 0.29 && X < 0.42) {   // engine nacelle and pylon
        const rn = 0.028 * Math.min(1, Math.sqrt((X - 0.29) / 0.025));
        if ((Y + 0.075) ** 2 + (Z - 0.17) ** 2 <= rn * rn) return 1;
        if (X > 0.32 && X < 0.4 && Y > -0.075 && Y < -0.02 && Math.abs(Z - 0.17) < 0.006) return 1;
      }
      return 0;
    };
  }
  // loop over a bounding region in world space
  const R = Math.hypot(ext[1] - ext[0], ext[3] - ext[2]) * 0.6;
  const x0 = Math.max(1, Math.floor(px - R)), x1 = Math.min(NX - 2, Math.ceil(px + R));
  const y0 = Math.max(1, Math.floor(py - R)), y1 = Math.min(NY - 2, Math.ceil(py + R)), z1 = Math.min(NZ - 2, Math.ceil(ext[4]));
  let bb = [1e9, 1e9, 1e9, -1e9, -1e9, -1e9], count = 0;
  for (let z = 0; z <= z1; z++) for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    const dx = x + 0.5 - px, dy = y + 0.5 - py;
    const bx = dx * ca - dy * sa, by = dx * sa + dy * ca, bz = z + 0.5;
    if (bx < ext[0] || bx > ext[1] || by < ext[2] || by > ext[3]) continue;
    const r = test(bx, by, bz); if (!r) continue;
    const k = x + NX * (y + NY * z);
    solid[k] = 1; tex[k * 2] = 255; if (r === 2) tex[k * 2 + 1] = 255; count++;
    bb[0] = Math.min(bb[0], x); bb[1] = Math.min(bb[1], y); bb[2] = Math.min(bb[2], z); bb[3] = Math.max(bb[3], x); bb[4] = Math.max(bb[4], y); bb[5] = Math.max(bb[5], z);
  }
  info.bbmin = [bb[0] - 1, bb[1] - 1, -(bb[5] + 2)]; info.bbmax = [bb[3] + 2, bb[4] + 2, bb[5] + 2]; info.count = count;
  info.iceCells = 0; for (let k = 1; k < tex.length; k += 2) if (tex[k]) info.iceCells++;
  return { solid, tex };
}

// ---------- GPU set-up ----------
async function initGPU() {
  if (initing || ready || failed) return;
  initing = true;
  try {
    if (!navigator.gpu) throw new Error('no-gpu');
    adapter = await navigator.gpu.requestAdapter({ powerPreference: "high-performance" });
    if (!adapter) throw new Error('no-adapter');
    const lim = adapter.limits;
    device = await adapter.requestDevice({ requiredLimits: { maxStorageBufferBindingSize: lim.maxStorageBufferBindingSize, maxBufferSize: lim.maxBufferSize } });
    device.addEventListener('uncapturederror', ev => console.error('WebGPU error:', ev.error.message));
    device.lost.then(i => { console.error('device lost', i.reason, i.message); ready = false; failed = true; showNoGPU(`The GPU device was lost (${i.message || i.reason}). Reload the page to try again.`); });
    const need = n => 19 * n * 4;
    for (const [k, [a, b, c]] of Object.entries(RES3)) {
      if (need(a * b * c) > device.limits.maxStorageBufferBindingSize || need(a * b * c) > device.limits.maxBufferSize) {
        const o = [...$('t-res').options].find(o => o.value === k); o.disabled = true; o.textContent += ' (too big for this GPU)';
        if (V.res === k) V.res = 'low';
      }
    }
    $('t-res').value = V.res;
    gctx = canvas.getContext('webgpu'); format = navigator.gpu.getPreferredCanvasFormat();
    gctx.configure({ device, format, alphaMode: 'opaque' });
    const mk = (code, label) => { const m = device.createShaderModule({ code, label }); return m; };
    const lbmMod = mk(LBM_WGSL, 'lbm'), partMod = mk(PART_WGSL, 'part'), rayMod = mk(RAY_WGSL, 'ray'), lineMod = mk(LINE_WGSL, 'line');
    for (const [mod, name] of [[lbmMod, 'lbm'], [partMod, 'particles'], [rayMod, 'raymarch'], [lineMod, 'lines']]) {
      const ci = await mod.getCompilationInfo();
      const errs = ci.messages.filter(m => m.type === 'error');
      if (errs.length) throw new Error(`${name} shader: ` + errs.map(e => `${e.lineNum}:${e.linePos} ${e.message}`).join('; '));
    }
    lbmPipe = device.createComputePipeline({ layout: 'auto', compute: { module: lbmMod, entryPoint: 'main' } });
    partPipe = device.createComputePipeline({ layout: 'auto', compute: { module: partMod, entryPoint: 'main' } });
    rayPipe = device.createRenderPipeline({ layout: 'auto', vertex: { module: rayMod, entryPoint: 'vs' }, fragment: { module: rayMod, entryPoint: 'fs', targets: [{ format }] },
      primitive: { topology: 'triangle-list' }, depthStencil: { format: 'depth32float', depthWriteEnabled: true, depthCompare: 'always' } });
    linePipe = device.createRenderPipeline({ layout: 'auto', vertex: { module: lineMod, entryPoint: 'vs' },
      fragment: { module: lineMod, entryPoint: 'fs', targets: [{ format, blend: { color: { srcFactor: 'src-alpha', dstFactor: 'one-minus-src-alpha' }, alpha: { srcFactor: 'one', dstFactor: 'one-minus-src-alpha' } } }] },
      primitive: { topology: 'line-list' }, depthStencil: { format: 'depth32float', depthWriteEnabled: false, depthCompare: 'less' } });
    const meshMod = mk(MESH_WGSL, 'mesh');
    { const ci = await meshMod.getCompilationInfo(); const errs = ci.messages.filter(m => m.type === 'error'); if (errs.length) throw new Error('mesh shader: ' + errs.map(e => `${e.lineNum}:${e.linePos} ${e.message}`).join('; ')); }
    meshPipe = device.createRenderPipeline({ layout: 'auto', vertex: { module: meshMod, entryPoint: 'vs', buffers: [{ arrayStride: 12, attributes: [{ shaderLocation: 0, offset: 0, format: 'float32x3' }] }] },
      fragment: { module: meshMod, entryPoint: 'fs', targets: [{ format }] }, primitive: { topology: 'triangle-list', cullMode: 'none' },
      depthStencil: { format: 'depth32float', depthWriteEnabled: true, depthCompare: 'less' } });
    meshUBuf = device.createBuffer({ size: 144, usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST });
    bgMesh = device.createBindGroup({ layout: meshPipe.getBindGroupLayout(0), entries: [{ binding: 0, resource: { buffer: meshUBuf } }] });
    sampler = device.createSampler({ magFilter: 'linear', minFilter: 'linear', addressModeU: 'clamp-to-edge', addressModeV: 'clamp-to-edge', addressModeW: 'clamp-to-edge' });
    paramBuf = device.createBuffer({ size: 32, usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST });
    partUBuf = device.createBuffer({ size: 48, usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST });
    renderUBuf = device.createBuffer({ size: 240, usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST });
    forceBuf = device.createBuffer({ size: 16, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_SRC | GPUBufferUsage.COPY_DST });
    stageBuf = device.createBuffer({ size: 16, usage: GPUBufferUsage.MAP_READ | GPUBufferUsage.COPY_DST });
    ready = true;
    await allocate();
  } catch (e) {
    failed = true;
    showNoGPU(e.message === 'no-gpu' ? null : e.message === 'no-adapter' ? 'Your browser supports WebGPU, but no suitable graphics adapter was found. Check that hardware acceleration is switched on.' : 'WebGPU failed to start: ' + e.message);
    console.error(e);
  } finally { initing = false; }
}
function showNoGPU(msg) { $('t-nogpu').hidden = false; if (msg) $('t-nogpu-msg').textContent = msg; canvas.style.visibility = 'hidden'; }

async function allocate() {
  for (const b of [bufA, bufB, macBuf, solidBuf, partBuf]) b && b.destroy();
  volTex && volTex.destroy();
  [NX, NY, NZ] = RES3[V.res]; N = NX * NY * NZ;
  const fsz = 19 * N * 4;
  bufA = device.createBuffer({ size: fsz, usage: GPUBufferUsage.STORAGE });
  bufB = device.createBuffer({ size: fsz, usage: GPUBufferUsage.STORAGE });
  macBuf = device.createBuffer({ size: N * 16, usage: GPUBufferUsage.STORAGE });
  solidBuf = device.createBuffer({ size: N * 4, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST });
  volTex = device.createTexture({ size: [NX, NY, NZ], dimension: '3d', format: 'rg8unorm', usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.COPY_DST });
  PCOUNT = V.res === 'low' ? 16000 : V.res === 'med' ? 24000 : 32000;
  partBuf = device.createBuffer({ size: PCOUNT * 32, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST });
  const mkLbm = (a, b) => device.createBindGroup({ layout: lbmPipe.getBindGroupLayout(0), entries: [
    { binding: 0, resource: { buffer: paramBuf } }, { binding: 1, resource: { buffer: a } }, { binding: 2, resource: { buffer: b } },
    { binding: 3, resource: { buffer: solidBuf } }, { binding: 4, resource: { buffer: macBuf } }, { binding: 5, resource: { buffer: forceBuf } }] });
  bgAB = mkLbm(bufA, bufB); bgBA = mkLbm(bufB, bufA);
  bgPart = device.createBindGroup({ layout: partPipe.getBindGroupLayout(0), entries: [
    { binding: 0, resource: { buffer: partUBuf } }, { binding: 1, resource: { buffer: macBuf } }, { binding: 2, resource: { buffer: solidBuf } }, { binding: 3, resource: { buffer: partBuf } }] });
  bgRay = device.createBindGroup({ layout: rayPipe.getBindGroupLayout(0), entries: [
    { binding: 0, resource: { buffer: renderUBuf } }, { binding: 1, resource: volTex.createView() }, { binding: 2, resource: sampler }, { binding: 3, resource: { buffer: macBuf } }] });
  bgLine = device.createBindGroup({ layout: linePipe.getBindGroupLayout(0), entries: [
    { binding: 0, resource: { buffer: renderUBuf } }, { binding: 1, resource: { buffer: partBuf } }] });
  info = null;
  await rebuild({ reset: true, cam: 'persp' });
}
let chain = Promise.resolve();
function rebuild(o = {}) {
  chain = chain.then(async () => {
    await uploadGeometry();
    if (o.reset) { resetFlow(); initParticles(); }
    hist.cl.length = 0; hist.cd.length = 0;
    if (o.cam) fitCamera(o.cam);
  }).catch(e => { console.error(e); SL.toast($('t-toast'), e.message, 7000); });
  return chain;
}
async function uploadGeometry() {
  let solid, tex;
  if (isMesh()) {
    const mc = await ensureMesh(V.model); if (!mc) return;
    const T = meshT(mc);
    const r = await rpc({ type: 'voxelize', id: mc.id, T, NX, NY, NZ });
    solid = r.solid; tex = r.tex; info = meshInfo(mc, T, r);
  } else ({ solid, tex } = voxelize());
  device.queue.writeBuffer(solidBuf, 0, solid);
  device.queue.writeTexture({ texture: volTex }, tex, { bytesPerRow: NX * 2, rowsPerImage: NY }, [NX, NY, NZ]);
  writeParams(0);
  cam.target = [info.px, info.py, 0];
  updateIceNote(); updateModelInfo();
}
function tau() { return Math.max(0.5008, 3 * U0 * info.cref / V.re + 0.5); }
function writeParams(mode) {
  const ab = new ArrayBuffer(32), u = new Uint32Array(ab), f = new Float32Array(ab);
  u[0] = NX; u[1] = NY; u[2] = NZ; u[3] = N; f[4] = U0; f[5] = tau(); f[6] = CS * CS; u[7] = mode;
  device.queue.writeBuffer(paramBuf, 0, ab);
}
function dispatchLBM(pass) { pass.setPipeline(lbmPipe); pass.setBindGroup(0, flip ? bgBA : bgAB); pass.dispatchWorkgroups(Math.ceil(NX / 32), Math.ceil(NY / 4), NZ); flip = !flip; }
function resetFlow() {
  writeParams(1);
  const enc = device.createCommandEncoder(), pass = enc.beginComputePass();
  dispatchLBM(pass); dispatchLBM(pass); pass.end(); device.queue.submit([enc.finish()]);
  writeParams(0);
  device.queue.writeBuffer(forceBuf, 0, new Int32Array(4));
  stepsTotal = 0; stepsSince = 0; hist.cl.length = 0; hist.cd.length = 0; lastF = null;
}
function initParticles() {
  const d = new Float32Array(PCOUNT * 8);
  for (let i = 0; i < PCOUNT; i++) {
    d[i * 8] = 2 + Math.random() * (NX - 4); d[i * 8 + 1] = info.seedY[0] + Math.random() * (info.seedY[1] - info.seedY[0]);
    d[i * 8 + 2] = Math.random() * info.seedZ; d[i * 8 + 3] = 1e9;
  }
  device.queue.writeBuffer(partBuf, 0, d);
}

// ---------- camera ----------
function fitCamera(kind) {
  const span = Math.max(info.span, info.cref * 2.5);
  cam.dist = Math.max(span * 2.6, NX * 0.55);
  const presets = { persp: [2.35, 0.42], front: [-Math.PI / 2, 0.08], side: [0, 0.02], top: [0.0001, 1.5], behind: [Math.PI / 2, 0.18] };
  [cam.yaw, cam.pitch] = presets[kind] || presets.persp;
  if (kind === 'side') cam.yaw = Math.PI * 0.0001;
}
let drag = null, pinch = null;
canvas.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, yaw: cam.yaw, pitch: cam.pitch }; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener('pointermove', e => {
  if (!drag) return;
  cam.yaw = drag.yaw - (e.clientX - drag.x) * 0.008;
  cam.pitch = clamp(drag.pitch + (e.clientY - drag.y) * 0.006, -1.45, 1.5);
});
canvas.addEventListener('pointerup', () => drag = null);
canvas.addEventListener('pointercancel', () => drag = null);
canvas.addEventListener('wheel', e => { e.preventDefault(); cam.dist = clamp(cam.dist * Math.exp(e.deltaY * 0.001), 20, 2000); }, { passive: false });
canvas.addEventListener('touchstart', e => { if (e.touches.length === 2) { pinch = { d: Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY), dist: cam.dist }; drag = null; } }, { passive: true });
canvas.addEventListener('touchmove', e => { if (pinch && e.touches.length === 2) { const d = Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY); cam.dist = clamp(pinch.dist * pinch.d / d, 20, 2000); } }, { passive: true });
canvas.addEventListener('touchend', () => pinch = null);
document.querySelector('#t-stage .camctl').addEventListener('click', e => { const b = e.target.closest('button'); if (b && info) fitCamera(b.dataset.cam); });
canvas.style.touchAction = 'none';

// ---------- frame ----------
let cw = 0, ch = 0;
function resize() {
  const dpr = Math.min(1.5, window.devicePixelRatio || 1), w = canvas.parentElement.clientWidth;
  if (!w) return;
  const h = Math.round(Math.min(620, Math.max(320, w * 0.56)));
  canvas.style.height = h + 'px';
  const W = Math.round(w * dpr), H = Math.round(h * dpr);
  if (W !== cw || H !== ch) {
    cw = W; ch = H; canvas.width = W; canvas.height = H;
    depthTex && depthTex.destroy();
    depthTex = device.createTexture({ size: [W, H], format: 'depth32float', usage: GPUTextureUsage.RENDER_ATTACHMENT });
  }
}
function slicePos() {
  const ax = V.slice; if (!ax) return 0;
  if (ax === 1) return V.spos * NX;
  if (ax === 2) return V.spos * NY;
  return V.spos * NZ * 0.98;
}
function writeRender() {
  const e = [cam.target[0] + cam.dist * Math.sin(cam.yaw) * Math.cos(cam.pitch), cam.target[1] + cam.dist * Math.sin(cam.pitch), cam.target[2] + cam.dist * Math.cos(cam.yaw) * Math.cos(cam.pitch)];
  const up = Math.abs(cam.pitch) > 1.45 ? [0, 0, -1] : [0, 1, 0];
  const vp = mul(persp(40 * D2R, cw / ch, 1, 5000), lookAt(e, cam.target, up)), ivp = inv(vp);
  lastVP = vp; lastEye = e;
  const d = new Float32Array(60);
  d.set(vp, 0); d.set(ivp, 16); d.set([...e, 0], 32); d.set([NX, NY, NZ, 0], 36);
  d.set([...info.bbmin, 0], 40); d.set([...info.bbmax, info.count > 0 && (!info.mesh || V.voxview) ? 1 : 0], 44);
  d.set([V.slice, slicePos(), V.sq, 0.92], 48); d.set([U0, 45, frameNo, info.cref], 52); d.set([cw, ch, 0, 0], 56);
  device.queue.writeBuffer(renderUBuf, 0, d);
}
function frame() {
  if (!active) return;
  requestAnimationFrame(frame);
  if (!ready || !info) return;
  resize(); if (!cw) return;
  frameNo++;
  const enc = device.createCommandEncoder();
  const steps = V.running ? V.spf : 0;
  if (steps) {
    const pass = enc.beginComputePass();
    for (let i = 0; i < steps; i++) dispatchLBM(pass);
    const pu = new ArrayBuffer(48), pui = new Uint32Array(pu), puf = new Float32Array(pu);
    pui[0] = NX; pui[1] = NY; pui[2] = NZ; pui[3] = PCOUNT;
    puf[4] = steps; puf[5] = info.seedY[0]; puf[6] = info.seedY[1]; puf[7] = info.seedZ;
    puf[8] = 3; puf[9] = 2.2 * NX / U0; puf[10] = frameNo;
    device.queue.writeBuffer(partUBuf, 0, pu);
    if (V.smoke) { pass.setPipeline(partPipe); pass.setBindGroup(0, bgPart); pass.dispatchWorkgroups(Math.ceil(PCOUNT / 64)); }
    pass.end();
    stepsTotal += steps; stepsSince += steps;
  }
  writeRender();
  const rp = enc.beginRenderPass({
    colorAttachments: [{ view: gctx.getCurrentTexture().createView(), loadOp: 'clear', storeOp: 'store', clearValue: { r: 0, g: 0, b: 0, a: 1 } }],
    depthStencilAttachment: { view: depthTex.createView(), depthLoadOp: 'clear', depthClearValue: 1, depthStoreOp: 'store' },
  });
  rp.setPipeline(rayPipe); rp.setBindGroup(0, bgRay); rp.draw(3);
  if (info.mesh && !V.voxview) {
    const mu = new Float32Array(36); mu.set(lastVP, 0); mu.set(info.model, 16); mu.set([...lastEye, 1], 32);
    device.queue.writeBuffer(meshUBuf, 0, mu);
    rp.setPipeline(meshPipe); rp.setBindGroup(0, bgMesh); rp.setVertexBuffer(0, info.mesh.vb); rp.setIndexBuffer(info.mesh.ib, 'uint32'); rp.drawIndexed(info.mesh.n);
  }
  if (V.smoke) { rp.setPipeline(linePipe); rp.setBindGroup(0, bgLine); rp.draw(PCOUNT * 2, 2); }
  rp.end();
  let doRead = false;
  if (!reading && stepsSince >= 60 && frameNo % 10 === 0) { enc.copyBufferToBuffer(forceBuf, 0, stageBuf, 0, 16); doRead = true; }
  device.queue.submit([enc.finish()]);
  if (doRead) {
    reading = true; const n = stepsSince; stepsSince = 0;
    device.queue.writeBuffer(forceBuf, 0, new Int32Array(4));
    stageBuf.mapAsync(GPUMapMode.READ).then(() => {
      const v = new Int32Array(stageBuf.getMappedRange().slice(0)); stageBuf.unmap(); reading = false;
      onForces(v[0] / 1e5 / n, v[1] / 1e5 / n);
    }).catch(() => { reading = false; });
  }
  const now = performance.now();
  if (now - lastRateT > 1000) { $('t-perf').textContent = V.running ? `${SL.int((stepsTotal - lastRateSteps) / ((now - lastRateT) / 1000))} steps/s · ${(N / 1e6).toFixed(1)} M cells` : 'paused'; lastRateSteps = stepsTotal; lastRateT = now; }
}
function onForces(fx, fy) {
  const q = 0.5 * U0 * U0 * info.Shalf, CL = fy / q, CD = fx / q;
  if (!isFinite(CL) || Math.abs(CL) > 30) { resetFlow(); SL.toast($('t-toast'), 'The flow went numerically unstable and was restarted. Lower the Reynolds number or use a larger grid.'); return; }
  const tstar = stepsTotal * U0 / info.cref;
  if (tstar < 1.5) { updateReadout(null, tstar); return; }
  hist.cl.push(CL); hist.cd.push(CD); if (hist.cl.length > 60) { hist.cl.shift(); hist.cd.shift(); }
  updateReadout({ cl: SL.mean(hist.cl, 20), cd: SL.mean(hist.cd, 20), sd: SL.std(hist.cl, 20) }, tstar);
}
function updateReadout(r, tstar) {
  if (!r) { $('t-cl').textContent = '…'; $('t-cd').textContent = '…'; $('t-ld').textContent = '…'; $('t-chip').textContent = `Flow settling · ${tstar.toFixed(1)} chord lengths`; $('t-chip').className = 'chip neutral'; return; }
  $('t-cl').textContent = fmt(r.cl); $('t-cd').textContent = fmt(r.cd, 3); $('t-ld').textContent = Math.abs(r.cd) > 1e-3 ? fmt(r.cl / r.cd, 1) : '—';
  const chip = $('t-chip');
  if (r.sd < 0.01) { chip.textContent = 'Steady flow'; chip.className = 'chip good'; } else { chip.textContent = 'Unsteady wake'; chip.className = 'chip warn'; }
  theory(r);
}
function theory(r) {
  const sec = SL.section, a0 = SL.alpha0(sec.m / 100, sec.p / 10, 0), al = V.aoa * D2R;
  if (V.model === 'wing') {
    const AR = V.ar, aw = 2 * Math.PI * AR / (2 + Math.sqrt(AR * AR + 4)), CLt = aw * (al - a0), CL2 = 2 * Math.PI * (al - a0), CDi = CLt * CLt / (Math.PI * 0.9 * AR);
    $('t-theory').innerHTML = `Lifting-line theory (Helmbold), aspect ratio ${AR}: C<sub>L</sub> ≈ <b>${fmt(CLt)}</b> with induced drag <b>${fmt(CDi, 3)}</b>. An infinitely long wing would give ${fmt(CL2)}; the gap is downwash from the tip vortices.${r ? ` Measured lift is lower because Reynolds here is ${SL.int(V.re)}, not millions.` : ''}`;
  } else if (MODELS[V.model] && MODELS[V.model].AR) {
    const M = MODELS[V.model], A = M.AR, t = Math.tan(M.sweep * D2R), aw = 2 * Math.PI * A / (2 + Math.sqrt(A * A * (1 + t * t) + 4));
    $('t-theory').innerHTML = `Swept-wing theory (USAF DATCOM, aspect ratio ${A}, ${M.sweepNote} ${M.sweep}°): lift slope <b>${fmt(aw * D2R, 3)}</b> per degree, so about <b>${fmt(aw * al, 2)}</b> above zero-lift at this tilt. Coefficients use the published reference area and mean chord. This solver is low-speed; the real tests ran near Mach 0.85, where shock waves change the flow.`;
  } else if (isMesh()) $('t-theory').innerHTML = `Coefficients use a reference area estimated from the voxel planform (${SL.int(info ? info.Shalf * 2 : 0)} cell² for the full span), so compare trends rather than exact values. Reynolds ${SL.int(V.re)} on the mean chord.`;
  else $('t-theory').innerHTML = `Coefficients use the half-model wing area and mean chord. Reynolds ${SL.int(V.re)} on the mean chord; a real airliner wing runs at about 30 million.`;
}

// ---------- controls ----------
const reFromSlider = v => 100 * Math.pow(10, v / 1000 * Math.log10(50));   // 100 … 5000
const sliderFromRe = re => Math.round(Math.log10(re / 100) / Math.log10(50) * 1000);
const VIEW_LEG = { 0: ['Speed', '0', '1.8× free stream', 'speed', 'how fast the air moves'], 1: ['Pressure Cp', '−2.5 suction', '+1 stagnation', 'pressure', ''], 2: ['Spin', 'clockwise', 'anticlockwise', 'vorticity', 'look for the tip vortices'] };
function syncUI() {
  $('t-ar-o').textContent = V.ar.toFixed(1); $('t-aoa-o').textContent = fmt(V.aoa, 1) + '°'; $('t-re-o').textContent = SL.int(V.re);
  $('t-spf-o').textContent = V.spf; $('t-spos-o').textContent = Math.round(V.spos * 100) + '%';
  $('t-ar').value = V.ar; $('t-aoa').value = V.aoa; $('t-re').value = V.reSlider; $('t-spf').value = V.spf; $('t-spos').value = V.spos;
  $('t-ar-ctl').style.display = V.model === 'wing' ? '' : 'none'; $('t-ice').checked = V.ice; $('t-smoke').checked = V.smoke;
  $('t-model').value = V.model; $('t-ice-ctl').hidden = isMesh(); $('t-vox-ctl').hidden = !isMesh(); $('t-voxview').checked = V.voxview; $('t-orient').hidden = V.model !== 'custom';
  const L = VIEW_LEG[V.sq]; $('t-lname').textContent = V.slice ? L[0] + ' on slice' : 'Smoke'; $('t-lmin').textContent = V.slice ? L[1] : ''; $('t-lmax').textContent = V.slice ? L[2] : ''; $('t-lnote').textContent = V.slice ? L[4] : 'particles coloured by speed';
  $('t-bar').hidden = !V.slice; if (V.slice) SL.drawBar($('t-bar'), SL.MAPS[L[3]]);
  SL.syncSeg('t-slice', String(V.slice)); SL.syncSeg('t-sq', String(V.sq));
  if (info) theory(null);
}
function updateIceNote() {
  const ok = SL.ice.points && SL.ice.points.length && SL.ice.shape === 'airfoil';
  $('t-ice-note').textContent = ok ? `${(SL.ice.kc * 100).toFixed(1)}% of chord grown in the Wing view${info && V.ice ? ` · ${info.iceCells ? SL.int(info.iceCells) + ' ice voxels' : 'thinner than one 3D cell'}` : ''}` : 'Grow some ice in the Wing view first';
}
let geoTimer = null;
function geomChanged(resetToo) {
  if (!ready) return;
  clearTimeout(geoTimer);
  geoTimer = setTimeout(() => rebuild({ reset: resetToo }), isMesh() ? 150 : 80);
}
$('t-model').onchange = e => { V.model = e.target.value; syncUI(); updateModelInfo(); if (ready) rebuild({ reset: true, cam: 'persp' }); };
$('t-voxview').onchange = e => { V.voxview = e.target.checked; };
function updateModelInfo() {
  const M = MODELS[V.model], el = $('t-info');
  if (M) el.innerHTML = `<b>${M.name}</b><p>${M.info}</p><p>${M.facts}</p><p><i>${M.credit}</i></p>`;
  else if (V.model === 'custom' && custom) el.innerHTML = `<b>${custom.name}</b><p>${SL.int(custom.tris)} triangles. The model is assumed to be left–right symmetric; only one half is simulated. If it faces the wrong way, use the buttons below.</p>`;
  else if (V.model === 'wing') el.innerHTML = '<b>Straight wing</b><p>A rectangular wing built from the airfoil chosen in the Wing view. Change its aspect ratio to see how tip vortices cost lift.</p>';
  else el.innerHTML = '<b>Simple airliner</b><p>A generic low-wing airliner built from simple shapes: fuselage, swept wing, tail and engines.</p>';
}
async function loadCustom(file) {
  if (!ready) { SL.toast($('t-toast'), 'The 3D tunnel needs WebGPU to load models.'); return; }
  try {
    $('t-chip').textContent = `Reading ${file.name}…`; $('t-chip').className = 'chip neutral';
    const buf = await file.arrayBuffer();
    const r = await rpc({ type: 'parse', id: 'custom', name: file.name, buf, orient: 'auto' }, [buf]);
    custom = { name: file.name, tris: r.tris, o: { ...r.guess, upSign: 1 } };
    await buildMesh('custom', orientR(custom.o));
    const opt = $('t-custom-opt'); opt.disabled = false; opt.textContent = file.name;
    V.model = 'custom'; syncUI(); updateModelInfo();
    await rebuild({ reset: true, cam: 'persp' });
  } catch (e) { console.error(e); SL.toast($('t-toast'), e.message, 7000); }
}
$('t-load').onclick = () => $('t-file').click();
$('t-file').onchange = e => { const f = e.target.files[0]; if (f) loadCustom(f); e.target.value = ''; };
$('t-stage').addEventListener('dragover', e => { e.preventDefault(); });
$('t-stage').addEventListener('drop', e => { e.preventDefault(); const f = e.dataTransfer.files[0]; if (f) loadCustom(f); });
$('t-orient').addEventListener('click', async e => {
  const b = e.target.closest('button'); if (!b || !custom) return;
  const o = custom.o;
  if (b.dataset.o === 'turn') [o.span, o.len] = [o.len, o.span];
  if (b.dataset.o === 'nose') o.noseSign *= -1;
  if (b.dataset.o === 'flip') o.upSign *= -1;
  if (b.dataset.o === 'up') { o.up = (o.up + 1) % 3; const rest = [0, 1, 2].filter(a => a !== o.up); o.span = rest[0]; o.len = rest[1]; }
  await buildMesh('custom', orientR(o)); rebuild({ reset: true, cam: 'persp' });
});
SL.seg('t-slice', v => { V.slice = +v; if (V.slice === 1) V.spos = 0.62; if (V.slice === 3) V.spos = 0.3; if (V.slice === 2) V.spos = 0.5; syncUI(); });
SL.seg('t-sq', v => { V.sq = +v; syncUI(); });
$('t-ar').addEventListener('input', e => { V.ar = +e.target.value; syncUI(); geomChanged(true); });
$('t-aoa').addEventListener('input', e => { V.aoa = +e.target.value; syncUI(); geomChanged(false); });
$('t-re').addEventListener('input', e => { V.reSlider = +e.target.value; V.re = reFromSlider(V.reSlider); syncUI(); if (ready) writeParams(0); });
$('t-spf').addEventListener('input', e => { V.spf = +e.target.value; syncUI(); });
$('t-spos').addEventListener('input', e => { V.spos = +e.target.value; syncUI(); });
$('t-smoke').onchange = e => { V.smoke = e.target.checked; };
$('t-ice').onchange = e => {
  V.ice = e.target.checked;
  if (V.ice && !(SL.ice.points && SL.ice.points.length)) SL.toast($('t-toast'), 'No ice yet. In the Wing view, switch on the icing cloud and let ice build up. It will appear here on the leading edges.');
  geomChanged(false);
};
$('t-res').onchange = e => { V.res = e.target.value; if (ready) allocate(); };
$('t-play').onclick = () => { V.running = !V.running; $('t-play').textContent = V.running ? 'Pause' : 'Resume'; };
$('t-reset').onclick = () => { if (ready) { resetFlow(); initParticles(); } };
const PRESETS = {
  crm: { model: 'crm', aoa: 3, re: 1500, slice: 0, sq: 0, smoke: true, cam: 'persp' },
  m6: { model: 'm6', aoa: 6, re: 1500, slice: 1, spos: 0.7, sq: 2, smoke: true, cam: 'behind' },
  tips: { model: 'wing', ar: 5, aoa: 8, re: 1000, slice: 1, spos: 0.62, sq: 2, smoke: true, cam: 'behind' },
  plane: { model: 'plane', aoa: 3, re: 1500, slice: 0, sq: 0, smoke: true, cam: 'persp' },
  iced: { model: 'wing', ar: 5, aoa: 8, re: 1000, slice: 3, spos: 0.3, sq: 0, smoke: true, ice: true, cam: 'persp' },
  stall3: { model: 'wing', ar: 5, aoa: 20, re: 1500, slice: 3, spos: 0.3, sq: 2, smoke: true, cam: 'side' },
};
$('t-try').addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b) return;
  const p = PRESETS[b.dataset.p];
  Object.assign(V, p); V.reSlider = sliderFromRe(p.re); V.re = reFromSlider(V.reSlider);
  if (p.ice && !(SL.ice.points && SL.ice.points.length)) SL.toast($('t-toast'), 'There is no ice yet, so this wing is clean. Grow ice in the Wing view (try Glaze ice horns), then come back.', 6000);
  syncUI();
  updateModelInfo();
  if (ready) rebuild({ reset: true, cam: p.cam });
});
SL.on('ice', () => { updateIceNote(); if (V.ice && active) geomChanged(false); });
SL.on('section', () => { if (ready && active && !isMesh()) geomChanged(false); });
V.reSlider = sliderFromRe(1500); V.re = reFromSlider(V.reSlider); V.aoa = 3; V.slice = 0; V.sq = 0;
syncUI(); updateModelInfo();

let dirtyOnShow = false;
SL.on('section', () => { if (!active) dirtyOnShow = true; });
SL.on('ice', () => { if (!active && V.ice) dirtyOnShow = true; });
SL.views.tunnel3d = {
  async activate() {
    active = true; updateIceNote();
    if (!ready && !failed) await initGPU();
    if (ready && dirtyOnShow) { dirtyOnShow = false; rebuild({}); }
    requestAnimationFrame(frame);
  },
  deactivate() { active = false; },
};
})();
