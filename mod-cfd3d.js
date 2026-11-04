// 3D flow module: D3Q19 lattice Boltzmann on WebGPU (half model with symmetry plane), GPU streamlines, flow slices.
(() => {
const { $, fmt, int, clamp, D2R } = SL;
const U0 = 0.08, CS = 0.17, SEED_Y = 5, SEED_Z = 11, K = 240;
const RES3 = { low: [128, 64, 64], med: [160, 80, 80], high: [208, 104, 104] };
const V = { model: 'project', aoa: 3, re: 1500, slice: 1, spos: 0.62, sq: 2, stream: true, smoke: false, voxview: false, spf: 6, res: 'med', running: true };
const canvas = $('cv-cfd3d');
let adapter, device, gctx, format, ready = false, failed = false, active = false, initing = false;
let NX, NY, NZ, N, info = null;
let bufA, bufB, macBuf, solidBuf, forceBuf, stageBuf, paramBuf, partBuf, partUBuf, renderUBuf, streamBuf, streamUBuf, volTex, sampler, depthTex;
let lbmPipe, partPipe, rayPipe, linePipe, meshPipe, streamPipe, sLinePipe, meshUBuf, bgMesh, bgAB, bgBA, bgPart, bgRay, bgLine, bgStream, bgSLine, flip = false, lastVP = null, lastEye = [0, 0, 0];
let stepsTotal = 0, stepsSince = 0, reading = false, frameNo = 0, rateT = 0, rateS = 0, PCOUNT = 0;
const hist = { cl: [], cd: [] }, NSEED = SEED_Y * SEED_Z;
const cam = { yaw: 2.35, pitch: 0.42, dist: 1, target: [0, 0, 0] };

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
  var s = ${ramp('viridis')};
  if (which != 0) { s = ${ramp('coolwarm')}; }
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
  var col = vec3f(0.055, 0.063, 0.075);
  var depthT = 1e9;
  let D = U.dims.xyz;
  // floor grid at y = 0
  if (rd.y < 0.0) {
    let t = -ro.y / rd.y; let p = ro + rd * t;
    if (p.x > 0.0 && p.x < D.x && abs(p.z) < D.z) {
      let g = abs(fract(p.xz / 10.0) - 0.5); let line = 1.0 - smoothstep(0.0, 0.04, min(g.x, g.y));
      col = mix(col, vec3f(0.07, 0.08, 0.095), 0.9) + line * vec3f(0.05, 0.055, 0.06);
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
    let base = mix(vec3f(0.62, 0.65, 0.69), vec3f(0.85, 0.92, 0.96), ice);
    let rim = pow(1.0 - max(dot(n, -rd), 0.0), 3.0);
    col = base * (0.25 + 0.2 * hemi + 0.55 * diff) + spec * 0.15;
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
  let c = vec3f(0.85, 0.87, 0.9);
  o.col = vec4f(c, select(0.0, 0.35, e == 0u) * fade);
  return o;
}
@fragment fn fs(i: VO) -> @location(0) vec4f { return i.col; }`;

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
  let base = vec3f(0.66, 0.69, 0.73);
  return vec4f(base * (0.25 + 0.2 * hemi + 0.55 * diff) + spec * 0.15 + rim * 0.04, 1.0);
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



const STREAM_WGSL = /* wgsl */`
struct SU { dims: vec4u, seed0: vec4f, seed1: vec4f };
@group(0) @binding(0) var<uniform> U: SU;
@group(0) @binding(1) var<storage, read> mac: array<vec4f>;
@group(0) @binding(2) var<storage, read> solid: array<u32>;
@group(0) @binding(3) var<storage, read_write> lines: array<vec4f>;
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
@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) g: vec3u) {
  let i = g.x; if (i >= U.dims.w) { return; }
  let ny = u32(U.seed1.x); let iy = i % ny; let iz = i / ny;
  var p = vec3f(U.seed0.x, mix(U.seed0.y, U.seed0.z, (f32(iy) + 0.5) / f32(ny)), U.seed0.w * (f32(iz) + 0.5) / U.seed1.y);
  let KK = u32(U.seed1.z); let h = U.seed1.w; let n = vec3f(U.dims.xyz);
  var alive = 1.0;
  for (var j = 0u; j < KK; j++) {
    lines[i * KK + j] = vec4f(p, alive);
    if (alive > 0.0) {
      let u1 = vel(p); let s1 = length(u1);
      if (s1 < 1e-5) { alive = -1.0; } else {
        let u2 = vel(p + u1 / s1 * h * 0.5); p = p + u2 / max(length(u2), 1e-6) * h;
        if (p.z < 0.0) { p.z = -p.z; }
        if (p.x > n.x - 2.0 || p.y < 1.0 || p.y > n.y - 2.0 || p.z > n.z - 2.0) { alive = -1.0; }
        else if (solid[cidx(vec3i(floor(p)))] != 0u) { alive = -1.0; }
      }
    }
  }
}`;
const SLINE_WGSL = COMMON_WGSL + /* wgsl */`
@group(0) @binding(1) var<storage, read> lines: array<vec4f>;
struct VO { @builtin(position) pos: vec4f, @location(0) col: vec4f };
@vertex fn vs(@builtin(vertex_index) vi: u32, @builtin(instance_index) ii: u32) -> VO {
  let KK = u32(U.view.z); let seg = vi / 2u; let e = vi % 2u; let ln = seg / (KK - 1u); let j = seg % (KK - 1u);
  let a = lines[ln * KK + j]; let b = lines[ln * KK + j + 1u];
  var w = select(a.xyz, b.xyz, e == 1u);
  if (ii == 1u) { w.z = -w.z; }
  var o: VO; o.pos = U.vp * vec4f(w, 1.0);
  if (a.w < 0.0 || b.w < 0.0) { o.pos = vec4f(2.0, 2.0, 2.0, 1.0); }
  o.col = vec4f(0.86, 0.88, 0.92, 0.55);
  return o;
}
@fragment fn fs(i: VO) -> @location(0) vec4f { return i.col; }`;

// ---------- models ----------
const MODELS = {
  project: { name: 'Project aircraft', info: 'The aircraft defined in Geometry, meshed with true airfoil thickness, a fuselage and a fin.', facts: '', credit: '' },
  crm: { file: 'models/nasa-crm.slm', name: 'NASA Common Research Model', info: 'Wing, body and tail of NASA\'s open transport-aircraft research geometry (AIAA Drag Prediction Workshop 4 surface).', facts: 'Semispan 29.4 m, c̄ 7.0 m, S 383.7 m², AR 9.0, Λ<sub>c/4</sub> 35°. Wind-tunnel design point C<sub>L</sub> 0.50 at M 0.85.', credit: 'NASA Langley Research Center, via the DAFoam DPW-4 files.', ref: { Shalf: 191.84, cref: 7.00532 }, AR: 9.0, sweep: 35, sweepNote: 'c/4' },
  m6: { file: 'models/onera-m6.slm', name: 'ONERA M6 wing', info: 'The classic swept validation wing; surface pressures measured in the ONERA S2MA tunnel.', facts: 'Semispan 1.196 m, c̄ 0.646 m, AR 3.8, Λ<sub>LE</sub> 30°, ONERA D section. Test: M 0.84, α 3.06°, Re 11.7 × 10⁶.', credit: 'SU2 project validation mesh.', ref: { Shalf: 0.7532, cref: 0.64607 }, AR: 3.8, sweep: 23.3, sweepNote: 'c/2' },
  x57: { file: 'models/nasa-x57.slm', name: 'NASA X-57 Maxwell', info: 'NASA\'s all-electric X-plane with a high-aspect-ratio cruise wing.', facts: 'Span 9.6 m. Reference area estimated from the voxel planform.', credit: 'NASA 3D Resources (public domain).' },
  gh: { file: 'models/nasa-global-hawk.slm', name: 'NASA Global Hawk', info: 'NASA\'s high-altitude Earth-science UAV.', facts: 'Span 39.9 m, length 14.5 m. Reference area estimated from the voxel planform; gear down.', credit: 'NASA 3D Resources (public domain).' },
};
const meshCache = {}; let custom = null, projectMeshV = -1;
const mw = new Worker('mesh-worker.js');
let ridN = 0; const rpcWait = {};
mw.onmessage = e => { const m = e.data, w = rpcWait[m.rid]; if (!w) return; delete rpcWait[m.rid]; m.type === 'error' ? w.rej(new Error(m.message)) : w.res(m); };
const rpc = (msg, tr = []) => new Promise((res, rej) => { const rid = ++ridN; rpcWait[rid] = { res, rej }; mw.postMessage({ ...msg, rid }, tr); });
const IDENT = [1, 0, 0, 0, 1, 0, 0, 0, 1], PROJ_R = [1, 0, 0, 0, 0, 1, 0, 1, 0];
const orientR = o => { const M = [0, 0, 0, 0, 0, 0, 0, 0, 0]; M[o.len] = -o.noseSign; M[3 + o.up] = o.upSign; M[6 + o.span] = 1; return M; };
async function buildMesh(key, R) {
  const g = await rpc({ type: 'geometry', id: key, R });
  const old = meshCache[key]; if (old) { old.vb.destroy(); old.ib.destroy(); }
  const vb = device.createBuffer({ size: g.pos.byteLength, usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST });
  const ib = device.createBuffer({ size: g.idx.byteLength, usage: GPUBufferUsage.INDEX | GPUBufferUsage.COPY_DST });
  device.queue.writeBuffer(vb, 0, g.pos); device.queue.writeBuffer(ib, 0, g.idx);
  return (meshCache[key] = { id: key, vb, ib, n: g.idx.length, lo: g.lo, hi: g.hi, R });
}
async function ensureMesh(key) {
  if (key === 'project') {
    if (meshCache.project && projectMeshV === SL.version) return meshCache.project;
    const M = SL.geom.mesh(SL.P); projectMeshV = SL.version;
    await rpc({ type: 'setMesh', id: 'project', pos: M.pos, idx: M.idx });
    return buildMesh('project', PROJ_R);
  }
  if (meshCache[key]) return meshCache[key];
  if (key === 'custom') return null;
  const M = MODELS[key]; SL.status(`Loading ${M.name}…`);
  let buf, r = await fetch(M.file).catch(() => null);
  if (r && r.ok && !(r.headers.get('content-type') || '').includes('text/html')) buf = await r.arrayBuffer();
  else { r = await fetch(M.file + '.txt'); if (!r.ok) throw new Error(`Could not load ${M.file}`); const bin = atob((await r.text()).trim()), u8 = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i); buf = u8.buffer; }
  await rpc({ type: 'parse', id: key, name: M.file, buf, orient: 'none' }, [buf]);
  return buildMesh(key, IDENT);
}
function meshT(mc) {
  const lo = mc.lo, hi = mc.hi, L = hi[0] - lo[0], Hh = hi[1] - lo[1], half = (hi[2] - lo[2]) / 2;
  const s = Math.min(0.5 * NX / L, 0.62 * NZ / half, 0.42 * NY / Hh);
  return { s, cx: (lo[0] + hi[0]) / 2, cy: (lo[1] + hi[1]) / 2, cz: (lo[2] + hi[2]) / 2, aoa: V.aoa, px: 0.2 * NX + 0.5 * L * s, py: 0.5 * NY, R: mc.R };
}
function refFor(key) { if (key === 'project') { const r = SL.geom.reference(SL.P); return { Shalf: r.S / 2, cref: r.cbar }; } return MODELS[key] && MODELS[key].ref; }
function meshInfo(mc, T, r) {
  const a = V.aoa * D2R, ca = Math.cos(a), sa = Math.sin(a), bb = r.bb, s = T.s, ref = refFor(V.model);
  const half = (mc.hi[2] - T.cz) * s, Shalf = ref ? ref.Shalf * s * s : r.area, cref = ref ? ref.cref * s : Shalf / Math.max(1, half);
  const model = [s * ca, -s * sa, 0, 0, s * sa, s * ca, 0, 0, 0, 0, s, 0, T.px - s * (ca * T.cx + sa * T.cy), T.py - s * (-sa * T.cx + ca * T.cy), -s * T.cz, 1];
  return { cref, Shalf, span: half, px: T.px, py: T.py, seedY: [Math.max(2, bb[1] - 3), Math.min(NY - 3, bb[4] + 3)], seedZ: Math.min(NZ - 3, bb[5] * 1.2 + 2),
    bbmin: [bb[0] - 1, bb[1] - 1, -(bb[5] + 2)], bbmax: [bb[3] + 2, bb[4] + 2, bb[5] + 2], count: r.count, model, mesh: mc, estimated: !ref };
}

// ---------- GPU set-up ----------
async function initGPU() {
  if (initing || ready || failed) return; initing = true;
  try {
    if (!navigator.gpu) throw new Error('no-gpu');
    adapter = await navigator.gpu.requestAdapter({ powerPreference: 'high-performance' });
    if (!adapter) throw new Error('no-adapter');
    const lim = adapter.limits;
    device = await adapter.requestDevice({ requiredLimits: { maxStorageBufferBindingSize: lim.maxStorageBufferBindingSize, maxBufferSize: lim.maxBufferSize } });
    device.addEventListener('uncapturederror', ev => console.error('WebGPU error:', ev.error.message));
    device.lost.then(i => { ready = false; failed = true; noGPU('GPU device lost', i.message || i.reason); });
    for (const [k, [a, b, c]] of Object.entries(RES3)) if (19 * a * b * c * 4 > Math.min(device.limits.maxStorageBufferBindingSize, device.limits.maxBufferSize)) { RES3[k].tooBig = true; if (V.res === k) V.res = 'low'; }
    gctx = canvas.getContext('webgpu'); format = navigator.gpu.getPreferredCanvasFormat(); gctx.configure({ device, format, alphaMode: 'opaque' });
    const mk = async (code, name) => { const m = device.createShaderModule({ code }); const ci = await m.getCompilationInfo(); const er = ci.messages.filter(x => x.type === 'error'); if (er.length) throw new Error(`${name} shader: ` + er.map(e => `${e.lineNum}:${e.linePos} ${e.message}`).join('; ')); return m; };
    const [lbmMod, partMod, rayMod, lineMod, meshMod, strMod, slMod] = await Promise.all([mk(LBM_WGSL, 'lbm'), mk(PART_WGSL, 'particles'), mk(RAY_WGSL, 'raymarch'), mk(LINE_WGSL, 'particle lines'), mk(MESH_WGSL, 'mesh'), mk(STREAM_WGSL, 'streamlines'), mk(SLINE_WGSL, 'streamline lines')]);
    const blend = { color: { srcFactor: 'src-alpha', dstFactor: 'one-minus-src-alpha' }, alpha: { srcFactor: 'one', dstFactor: 'one-minus-src-alpha' } };
    const depthRO = { format: 'depth32float', depthWriteEnabled: false, depthCompare: 'less' };
    lbmPipe = device.createComputePipeline({ layout: 'auto', compute: { module: lbmMod, entryPoint: 'main' } });
    partPipe = device.createComputePipeline({ layout: 'auto', compute: { module: partMod, entryPoint: 'main' } });
    streamPipe = device.createComputePipeline({ layout: 'auto', compute: { module: strMod, entryPoint: 'main' } });
    rayPipe = device.createRenderPipeline({ layout: 'auto', vertex: { module: rayMod, entryPoint: 'vs' }, fragment: { module: rayMod, entryPoint: 'fs', targets: [{ format }] }, primitive: { topology: 'triangle-list' }, depthStencil: { format: 'depth32float', depthWriteEnabled: true, depthCompare: 'always' } });
    linePipe = device.createRenderPipeline({ layout: 'auto', vertex: { module: lineMod, entryPoint: 'vs' }, fragment: { module: lineMod, entryPoint: 'fs', targets: [{ format, blend }] }, primitive: { topology: 'line-list' }, depthStencil: depthRO });
    sLinePipe = device.createRenderPipeline({ layout: 'auto', vertex: { module: slMod, entryPoint: 'vs' }, fragment: { module: slMod, entryPoint: 'fs', targets: [{ format, blend }] }, primitive: { topology: 'line-list' }, depthStencil: depthRO });
    meshPipe = device.createRenderPipeline({ layout: 'auto', vertex: { module: meshMod, entryPoint: 'vs', buffers: [{ arrayStride: 12, attributes: [{ shaderLocation: 0, offset: 0, format: 'float32x3' }] }] }, fragment: { module: meshMod, entryPoint: 'fs', targets: [{ format }] }, primitive: { topology: 'triangle-list', cullMode: 'none' }, depthStencil: { format: 'depth32float', depthWriteEnabled: true, depthCompare: 'less' } });
    sampler = device.createSampler({ magFilter: 'linear', minFilter: 'linear', addressModeU: 'clamp-to-edge', addressModeV: 'clamp-to-edge', addressModeW: 'clamp-to-edge' });
    const U = GPUBufferUsage;
    paramBuf = device.createBuffer({ size: 32, usage: U.UNIFORM | U.COPY_DST });
    partUBuf = device.createBuffer({ size: 48, usage: U.UNIFORM | U.COPY_DST });
    streamUBuf = device.createBuffer({ size: 48, usage: U.UNIFORM | U.COPY_DST });
    renderUBuf = device.createBuffer({ size: 240, usage: U.UNIFORM | U.COPY_DST });
    meshUBuf = device.createBuffer({ size: 144, usage: U.UNIFORM | U.COPY_DST });
    forceBuf = device.createBuffer({ size: 16, usage: U.STORAGE | U.COPY_SRC | U.COPY_DST });
    stageBuf = device.createBuffer({ size: 16, usage: U.MAP_READ | U.COPY_DST });
    streamBuf = device.createBuffer({ size: NSEED * K * 16, usage: U.STORAGE });
    bgMesh = device.createBindGroup({ layout: meshPipe.getBindGroupLayout(0), entries: [{ binding: 0, resource: { buffer: meshUBuf } }] });
    bgSLine = device.createBindGroup({ layout: sLinePipe.getBindGroupLayout(0), entries: [{ binding: 0, resource: { buffer: renderUBuf } }, { binding: 1, resource: { buffer: streamBuf } }] });
    ready = true;
    await allocate();
  } catch (e) {
    failed = true; console.error(e);
    noGPU(e.message === 'no-gpu' ? 'WebGPU is not available in this browser' : e.message === 'no-adapter' ? 'No suitable GPU adapter' : 'WebGPU failed to start',
      e.message === 'no-gpu' ? 'The 3D solver runs on your graphics card through WebGPU. Use Chrome or Edge 113+, or Safari 26. The other modules work in any browser.' : e.message === 'no-adapter' ? 'WebGPU is supported, but no adapter was returned. Check that hardware acceleration is on.' : e.message);
  } finally { initing = false; }
}
function noGPU(t, m) { $('vp-empty').hidden = false; $('vp-empty-t').textContent = t; $('vp-empty-m').textContent = m; canvas.style.visibility = 'hidden'; SL.status('3D flow unavailable'); }
async function allocate() {
  for (const b of [bufA, bufB, macBuf, solidBuf, partBuf]) b && b.destroy(); volTex && volTex.destroy();
  [NX, NY, NZ] = RES3[V.res]; N = NX * NY * NZ;
  const U = GPUBufferUsage, fsz = 19 * N * 4;
  bufA = device.createBuffer({ size: fsz, usage: U.STORAGE }); bufB = device.createBuffer({ size: fsz, usage: U.STORAGE });
  macBuf = device.createBuffer({ size: N * 16, usage: U.STORAGE }); solidBuf = device.createBuffer({ size: N * 4, usage: U.STORAGE | U.COPY_DST });
  volTex = device.createTexture({ size: [NX, NY, NZ], dimension: '3d', format: 'rg8unorm', usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.COPY_DST });
  PCOUNT = V.res === 'low' ? 16000 : V.res === 'med' ? 24000 : 32000;
  partBuf = device.createBuffer({ size: PCOUNT * 32, usage: U.STORAGE | U.COPY_DST });
  const mkLbm = (a, b) => device.createBindGroup({ layout: lbmPipe.getBindGroupLayout(0), entries: [{ binding: 0, resource: { buffer: paramBuf } }, { binding: 1, resource: { buffer: a } }, { binding: 2, resource: { buffer: b } }, { binding: 3, resource: { buffer: solidBuf } }, { binding: 4, resource: { buffer: macBuf } }, { binding: 5, resource: { buffer: forceBuf } }] });
  bgAB = mkLbm(bufA, bufB); bgBA = mkLbm(bufB, bufA);
  bgPart = device.createBindGroup({ layout: partPipe.getBindGroupLayout(0), entries: [{ binding: 0, resource: { buffer: partUBuf } }, { binding: 1, resource: { buffer: macBuf } }, { binding: 2, resource: { buffer: solidBuf } }, { binding: 3, resource: { buffer: partBuf } }] });
  bgStream = device.createBindGroup({ layout: streamPipe.getBindGroupLayout(0), entries: [{ binding: 0, resource: { buffer: streamUBuf } }, { binding: 1, resource: { buffer: macBuf } }, { binding: 2, resource: { buffer: solidBuf } }, { binding: 3, resource: { buffer: streamBuf } }] });
  bgRay = device.createBindGroup({ layout: rayPipe.getBindGroupLayout(0), entries: [{ binding: 0, resource: { buffer: renderUBuf } }, { binding: 1, resource: volTex.createView() }, { binding: 2, resource: sampler }, { binding: 3, resource: { buffer: macBuf } }] });
  bgLine = device.createBindGroup({ layout: linePipe.getBindGroupLayout(0), entries: [{ binding: 0, resource: { buffer: renderUBuf } }, { binding: 1, resource: { buffer: partBuf } }] });
  info = null;
  await rebuild({ reset: true, cam: 'persp' });
}
let chain = Promise.resolve();
function rebuild(o = {}) {
  chain = chain.then(async () => { await upload(); if (o.reset) { resetFlow(); initParticles(); } hist.cl.length = 0; hist.cd.length = 0; if (o.cam) fitCamera(o.cam); results(); })
    .catch(e => { console.error(e); SL.toast(e.message, 7000); });
  return chain;
}
async function upload() {
  const mc = await ensureMesh(V.model); if (!mc) return;
  const T = meshT(mc), r = await rpc({ type: 'voxelize', id: mc.id, T, NX, NY, NZ });
  info = meshInfo(mc, T, r);
  device.queue.writeBuffer(solidBuf, 0, r.solid);
  device.queue.writeTexture({ texture: volTex }, r.tex, { bytesPerRow: NX * 2, rowsPerImage: NY }, [NX, NY, NZ]);
  writeParams(0); cam.target = [info.px, info.py, 0];
}
const tau = () => Math.max(0.5008, 3 * U0 * info.cref / V.re + 0.5);
function writeParams(mode) { const ab = new ArrayBuffer(32), u = new Uint32Array(ab), f = new Float32Array(ab); u[0] = NX; u[1] = NY; u[2] = NZ; u[3] = N; f[4] = U0; f[5] = info ? tau() : 0.55; f[6] = CS * CS; u[7] = mode; device.queue.writeBuffer(paramBuf, 0, ab); }
function dispatchLBM(pass) { pass.setPipeline(lbmPipe); pass.setBindGroup(0, flip ? bgBA : bgAB); pass.dispatchWorkgroups(Math.ceil(NX / 32), Math.ceil(NY / 4), NZ); flip = !flip; }
function resetFlow() {
  writeParams(1); const enc = device.createCommandEncoder(), pass = enc.beginComputePass(); dispatchLBM(pass); dispatchLBM(pass); pass.end(); device.queue.submit([enc.finish()]);
  writeParams(0); device.queue.writeBuffer(forceBuf, 0, new Int32Array(4)); stepsTotal = 0; stepsSince = 0; hist.cl.length = 0; hist.cd.length = 0;
}
function initParticles() { const d = new Float32Array(PCOUNT * 8); for (let i = 0; i < PCOUNT; i++) { d[i * 8] = 2 + Math.random() * (NX - 4); d[i * 8 + 1] = info.seedY[0] + Math.random() * (info.seedY[1] - info.seedY[0]); d[i * 8 + 2] = Math.random() * info.seedZ; d[i * 8 + 3] = 1e9; } device.queue.writeBuffer(partBuf, 0, d); }

// ---------- camera ----------
function fitCamera(kind) {
  const span = Math.max(info.span, info.cref * 2.5); cam.dist = Math.max(span * 2.6, NX * 0.55);
  [cam.yaw, cam.pitch] = ({ persp: [2.35, 0.42], front: [-Math.PI / 2, 0.08], side: [1e-4, 0.02], top: [1e-4, 1.5], behind: [Math.PI / 2, 0.18] })[kind] || [2.35, 0.42];
}
let drag = null, pinch = null;
canvas.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, yaw: cam.yaw, pitch: cam.pitch }; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener('pointermove', e => { if (!drag) return; cam.yaw = drag.yaw - (e.clientX - drag.x) * 0.008; cam.pitch = clamp(drag.pitch + (e.clientY - drag.y) * 0.006, -1.45, 1.5); });
canvas.addEventListener('pointerup', () => drag = null); canvas.addEventListener('pointercancel', () => drag = null);
canvas.addEventListener('wheel', e => { e.preventDefault(); cam.dist = clamp(cam.dist * Math.exp(e.deltaY * 0.001), 20, 2000); }, { passive: false });
canvas.addEventListener('touchstart', e => { if (e.touches.length === 2) { pinch = { d: Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY), dist: cam.dist }; drag = null; } }, { passive: true });
canvas.addEventListener('touchmove', e => { if (pinch && e.touches.length === 2) cam.dist = clamp(pinch.dist * pinch.d / Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY), 20, 2000); }, { passive: true });
canvas.addEventListener('touchend', () => pinch = null);
canvas.style.touchAction = 'none';
canvas.addEventListener('dragover', e => e.preventDefault());
canvas.addEventListener('drop', e => { e.preventDefault(); const f = e.dataTransfer.files[0]; if (f) loadCustom(f); });

// ---------- frame ----------
let cw = 0, ch = 0;
function resize() {
  const dpr = Math.min(1.5, window.devicePixelRatio || 1), w = canvas.clientWidth, h = canvas.clientHeight; if (!w || !h) return;
  const W = Math.round(w * dpr), H = Math.round(h * dpr);
  if (W !== cw || H !== ch) { cw = W; ch = H; canvas.width = W; canvas.height = H; depthTex && depthTex.destroy(); depthTex = device.createTexture({ size: [W, H], format: 'depth32float', usage: GPUTextureUsage.RENDER_ATTACHMENT }); }
}
const slicePos = () => V.slice === 1 ? V.spos * NX : V.slice === 2 ? V.spos * NY : V.slice === 3 ? V.spos * NZ * 0.98 : 0;
function writeRender() {
  const e = [cam.target[0] + cam.dist * Math.sin(cam.yaw) * Math.cos(cam.pitch), cam.target[1] + cam.dist * Math.sin(cam.pitch), cam.target[2] + cam.dist * Math.cos(cam.yaw) * Math.cos(cam.pitch)];
  const up = Math.abs(cam.pitch) > 1.45 ? [0, 0, -1] : [0, 1, 0];
  const vp = mul(persp(40 * D2R, cw / ch, 1, 5000), lookAt(e, cam.target, up)), ivp = inv(vp); lastVP = vp; lastEye = e;
  const d = new Float32Array(60);
  d.set(vp, 0); d.set(ivp, 16); d.set([...e, 0], 32); d.set([NX, NY, NZ, 0], 36);
  d.set([...info.bbmin, 0], 40); d.set([...info.bbmax, info.count > 0 && V.voxview ? 1 : 0], 44);
  d.set([V.slice, slicePos(), V.sq, 0.92], 48); d.set([U0, 45, frameNo, info.cref], 52); d.set([cw, ch, K, 0], 56);
  device.queue.writeBuffer(renderUBuf, 0, d);
}
function frame() {
  if (!active) return; requestAnimationFrame(frame);
  if (!ready || !info) return; resize(); if (!cw) return; frameNo++;
  const enc = device.createCommandEncoder(), steps = V.running ? V.spf : 0;
  const pass = enc.beginComputePass();
  for (let i = 0; i < steps; i++) dispatchLBM(pass);
  if (V.smoke && steps) {
    const pu = new ArrayBuffer(48), pui = new Uint32Array(pu), puf = new Float32Array(pu);
    pui[0] = NX; pui[1] = NY; pui[2] = NZ; pui[3] = PCOUNT; puf[4] = steps; puf[5] = info.seedY[0]; puf[6] = info.seedY[1]; puf[7] = info.seedZ; puf[8] = 3; puf[9] = 2.2 * NX / U0; puf[10] = frameNo;
    device.queue.writeBuffer(partUBuf, 0, pu); pass.setPipeline(partPipe); pass.setBindGroup(0, bgPart); pass.dispatchWorkgroups(Math.ceil(PCOUNT / 64));
  }
  if (V.stream && frameNo % 2 === 0) {
    const su = new ArrayBuffer(48), sui = new Uint32Array(su), suf = new Float32Array(su);
    sui[0] = NX; sui[1] = NY; sui[2] = NZ; sui[3] = NSEED; suf[4] = 3; suf[5] = info.seedY[0]; suf[6] = info.seedY[1]; suf[7] = info.seedZ; suf[8] = SEED_Y; suf[9] = SEED_Z; suf[10] = K; suf[11] = 0.9;
    device.queue.writeBuffer(streamUBuf, 0, su); pass.setPipeline(streamPipe); pass.setBindGroup(0, bgStream); pass.dispatchWorkgroups(Math.ceil(NSEED / 64));
  }
  pass.end();
  stepsTotal += steps; stepsSince += steps;
  writeRender();
  const rp = enc.beginRenderPass({ colorAttachments: [{ view: gctx.getCurrentTexture().createView(), loadOp: 'clear', storeOp: 'store', clearValue: { r: 0, g: 0, b: 0, a: 1 } }], depthStencilAttachment: { view: depthTex.createView(), depthLoadOp: 'clear', depthClearValue: 1, depthStoreOp: 'store' } });
  rp.setPipeline(rayPipe); rp.setBindGroup(0, bgRay); rp.draw(3);
  if (!V.voxview) { const mu = new Float32Array(36); mu.set(lastVP, 0); mu.set(info.model, 16); mu.set([...lastEye, 1], 32); device.queue.writeBuffer(meshUBuf, 0, mu); rp.setPipeline(meshPipe); rp.setBindGroup(0, bgMesh); rp.setVertexBuffer(0, info.mesh.vb); rp.setIndexBuffer(info.mesh.ib, 'uint32'); rp.drawIndexed(info.mesh.n); }
  if (V.stream) { rp.setPipeline(sLinePipe); rp.setBindGroup(0, bgSLine); rp.draw(NSEED * (K - 1) * 2, 2); }
  if (V.smoke) { rp.setPipeline(linePipe); rp.setBindGroup(0, bgLine); rp.draw(PCOUNT * 2, 2); }
  rp.end();
  let doRead = false;
  if (!reading && stepsSince >= 60 && frameNo % 10 === 0) { enc.copyBufferToBuffer(forceBuf, 0, stageBuf, 0, 16); doRead = true; }
  device.queue.submit([enc.finish()]);
  if (doRead) { reading = true; const n = stepsSince; stepsSince = 0; device.queue.writeBuffer(forceBuf, 0, new Int32Array(4)); stageBuf.mapAsync(GPUMapMode.READ).then(() => { const v = new Int32Array(stageBuf.getMappedRange().slice(0)); stageBuf.unmap(); reading = false; onForces(v[0] / 1e5 / n, v[1] / 1e5 / n); }).catch(() => reading = false); }
  const now = performance.now();
  if (now - rateT > 1000) { SL.status(`3D LBM · ${(MODELS[V.model] || { name: custom ? custom.name : '' }).name} · α ${fmt(V.aoa, 1)}° · Re ${int(V.re)}${V.running ? '' : ' · paused'}`, `${int((stepsTotal - rateS) / ((now - rateT) / 1000))} steps/s · ${NX}×${NY}×${NZ} · ${fmt(stepsTotal * U0 / info.cref, 1)} c/U∞`); rateS = stepsTotal; rateT = now; }
}
function onForces(fx, fy) {
  const q = 0.5 * U0 * U0 * info.Shalf, CL = fy / q, CD = fx / q;
  if (!isFinite(CL) || Math.abs(CL) > 30) { resetFlow(); SL.toast('The flow became numerically unstable and was restarted. Lower the Reynolds number or use a larger grid.'); return; }
  if (stepsTotal * U0 / info.cref < 1.5) return;
  hist.cl.push(CL); hist.cd.push(CD); if (hist.cl.length > 80) { hist.cl.shift(); hist.cd.shift(); }
  results();
}
function results() {
  const M = MODELS[V.model] || { name: custom ? custom.name : 'Custom model', info: custom ? `${int(custom.tris)} triangles. Assumed symmetric left to right; only one half is simulated.` : '', facts: 'Reference area estimated from the voxel planform.', credit: 'Your file.' };
  const n = hist.cl.length, cl = SL.mean(hist.cl, 20), cd = SL.mean(hist.cd, 20), sd = SL.std(hist.cl, 20), ok = n > 3;
  let theory = '';
  if (V.model === 'project') { const A = SL.aero(); theory = `Vortex-lattice C<sub>L</sub> at this α: <b>${fmt(A.CL0 + A.CLa * V.aoa * D2R, 3)}</b> (inviscid, flight Reynolds number).`; }
  else if (M.AR) { const t = Math.tan(M.sweep * D2R), aw = 2 * Math.PI * M.AR / (2 + Math.sqrt(M.AR * M.AR * (1 + t * t) + 4)); theory = `DATCOM lift slope (AR ${M.AR}, Λ<sub>${M.sweepNote}</sub> ${M.sweep}°): <b>${fmt(aw * D2R, 4)}</b> /°, so ≈ <b>${fmt(aw * V.aoa * D2R, 3)}</b> above zero lift.`; }
  $('results').innerHTML = `
    <div class="badges">${!ok ? SL.badge('Settling', 'neutral') : sd < 0.01 ? SL.badge('Steady', 'ok') : SL.badge('Unsteady wake', 'warn')}${info && info.estimated ? SL.badge('Estimated S_ref', 'neutral') : ''}</div>
    <h4>Forces <span>whole aircraft</span></h4>${SL.kv([['C<sub>L</sub>', ok ? fmt(cl, 3) : '…', 'big'], ['C<sub>D</sub>', ok ? fmt(cd, 4) : '…', 'big'], ['L/D', ok && Math.abs(cd) > 1e-3 ? fmt(cl / cd, 2) : '…', 'big'], null, ['σ(C<sub>L</sub>)', ok ? fmt(sd, 3) : '…'], ['Reynolds (c̄)', int(V.re)], ['Cells', info ? int(N) : '…'], ['Solid voxels', info ? int(info.count) : '…']])}
    <p class="ref">${theory} The lattice runs at Reynolds numbers in the low thousands, so expect lower lift and higher drag than the flight values.</p>
    <h4>History</h4><canvas class="plot" id="c-p1"></canvas><div class="leg"><span><i style="background:var(--accent)"></i>C<sub>L</sub></span><span><i style="background:var(--warn)"></i>C<sub>D</sub></span></div>
    <h4>Model</h4><p class="ref"><b>${M.name}</b>. ${M.info}</p>${M.facts ? `<p class="ref">${M.facts}</p>` : ''}${M.credit ? `<p class="ref">Source: ${M.credit}</p>` : ''}
    <h4>Method</h4><p class="ref">D3Q19 lattice Boltzmann with Smagorinsky LES as WebGPU compute shaders; symmetry plane at the centreline; bounce-back walls on the voxelised surface; forces by momentum exchange. Streamlines are traced on the GPU through the instantaneous velocity field.</p>`;
  const pl = SL.plot($('c-p1'), (() => { const v = hist.cl.concat(hist.cd), lo = Math.min(0, ...v), hi = Math.max(0.2, ...v); return { x0: 0, x1: 79, y0: lo, y1: hi * 1.1, dy: SL.niceStep(hi * 1.1 - lo, 4) }; })());
  if (pl) { const off = 80 - hist.cl.length; pl.line(hist.cd.map((v, i) => [i + off, v]), SL.css('--warn'), 1.4); pl.line(hist.cl.map((v, i) => [i + off, v]), SL.css('--accent'), 1.4); }
}
function legend() {
  if (!V.slice) { SL.colorbar(null); return; }
  SL.colorbar(...({ 0: ['viridis', '0', '1.8', '|u| / U∞ on slice'], 1: ['coolwarm', '−2.5', '+1', 'C_p on slice'], 2: ['coolwarm', 'cw', 'ccw', 'Vorticity normal to slice'] })[V.sq]);
}
async function loadCustom(file) {
  if (!ready) { SL.toast('The 3D module needs WebGPU to load models.'); return; }
  try {
    SL.status(`Reading ${file.name}…`);
    const buf = await file.arrayBuffer(), r = await rpc({ type: 'parse', id: 'custom', name: file.name, buf, orient: 'auto' }, [buf]);
    custom = { name: file.name, tris: r.tris, o: { ...r.guess, upSign: 1 } };
    await buildMesh('custom', orientR(custom.o)); V.model = 'custom'; SL.remountInspector(); await rebuild({ reset: true, cam: 'persp' });
  } catch (e) { console.error(e); SL.toast(e.message, 7000); }
}
const fileInput = SL.el('input', { type: 'file', accept: '.stl,.obj,.glb,.gltf', hidden: '' }); document.body.appendChild(fileInput);
fileInput.onchange = () => { const f = fileInput.files[0]; if (f) loadCustom(f); fileInput.value = ''; };
const reS = v => Math.round(Math.log10(v / 100) / Math.log10(50) * 1000), reV = s => 100 * Math.pow(10, s / 1000 * Math.log10(50));
let geoTimer = null;
const geomChanged = (reset) => { if (!ready) return; clearTimeout(geoTimer); geoTimer = setTimeout(() => rebuild({ reset }), 150); };
const orientBtn = k => async () => { if (!custom) return; const o = custom.o; if (k === 'turn') [o.span, o.len] = [o.len, o.span]; if (k === 'nose') o.noseSign *= -1; if (k === 'flip') o.upSign *= -1; if (k === 'up') { o.up = (o.up + 1) % 3; const r = [0, 1, 2].filter(a => a !== o.up); o.span = r[0]; o.len = r[1]; } await buildMesh('custom', orientR(o)); rebuild({ reset: true, cam: 'persp' }); };

SL.register('cfd3d', {
  title: '3D flow',
  inspector: () => [
    { section: 'Model', fields: [
      { type: 'select', label: 'Geometry', get: () => V.model, set: v => { V.model = v; SL.remountInspector(); if (ready) rebuild({ reset: true, cam: 'persp' }); }, options: [['project', 'Project aircraft'], ['crm', 'NASA CRM'], ['m6', 'ONERA M6 wing'], ['x57', 'NASA X-57'], ['gh', 'NASA Global Hawk'], ...(custom ? [['custom', custom.name]] : [])] },
      { type: 'button', buttons: [{ label: 'Load STL / OBJ / GLB…', onclick: () => fileInput.click() }] },
      ...(V.model === 'custom' ? [{ type: 'button', buttons: [{ label: 'Turn 90°', onclick: orientBtn('turn') }, { label: 'Flip nose', onclick: orientBtn('nose') }, { label: 'Up axis', onclick: orientBtn('up') }, { label: 'Invert', onclick: orientBtn('flip') }] }] : []),
      { type: 'toggle', label: 'Show voxels', get: () => V.voxview, set: v => V.voxview = v, hint: 'Show the solver\'s voxel surface instead of the smooth mesh' },
    ] },
    { section: 'Flow', fields: [
      { type: 'num', label: 'Angle of attack', get: () => V.aoa, set: v => { V.aoa = v; geomChanged(false); }, unit: '°', min: -6, max: 24, step: 0.5, dp: 1 },
      { type: 'num', label: 'Reynolds number', get: () => reS(V.re), set: v => { V.re = reV(v); if (ready && info) writeParams(0); }, min: 0, max: 1000, step: 1, fmt: () => int(V.re), parse: t => reS(clamp(t, 100, 5000)), hardMin: 0, hardMax: 1000 },
    ] },
    { section: 'Visualisation', fields: [
      { type: 'toggle', label: 'Streamlines', get: () => V.stream, set: v => V.stream = v },
      { type: 'toggle', label: 'Smoke particles', get: () => V.smoke, set: v => { V.smoke = v; if (v && ready && info) initParticles(); } },
      { type: 'seg', label: 'Slice', get: () => String(V.slice), set: v => { V.slice = +v; V.spos = { 1: 0.62, 2: 0.5, 3: 0.3 }[V.slice] || V.spos; legend(); SL.refreshInspector(); }, options: [['0', 'Off'], ['1', 'x'], ['3', 'z'], ['2', 'y']] },
      { type: 'num', label: 'Slice position', get: () => V.spos, set: v => V.spos = v, unit: '', min: 0, max: 1, step: 0.01, dp: 2 },
      { type: 'seg', label: 'Quantity', get: () => String(V.sq), set: v => { V.sq = +v; legend(); }, options: [['0', '|u|'], ['1', 'C_p'], ['2', 'ω']] },
    ] },
    { section: 'Solver', fields: [
      { type: 'select', label: 'Grid', get: () => V.res, set: v => { if (RES3[v].tooBig) { SL.toast('This grid is too large for your GPU.'); SL.refreshInspector(); return; } V.res = v; if (ready) allocate(); }, options: [['low', '128×64×64 · 0.5 M'], ['med', '160×80×80 · 1.0 M'], ['high', '208×104×104 · 2.2 M']] },
      { type: 'num', label: 'Steps per frame', get: () => V.spf, set: v => V.spf = Math.round(v), unit: '', min: 1, max: 20, step: 1, dp: 0 },
      { type: 'button', buttons: [{ label: 'Pause / run', onclick: () => V.running = !V.running }, { label: 'Restart flow', onclick: () => { if (ready && info) { resetFlow(); initParticles(); } } }] },
    ] },
  ],
  toolbar(bar) {
    bar.innerHTML = '<span class="lab">Camera</span>';
    const g = SL.el('div', { class: 'segm' });
    for (const [k, t] of [['persp', 'Iso'], ['front', 'Front'], ['side', 'Side'], ['top', 'Top'], ['behind', 'Rear']]) g.appendChild(SL.el('button', { type: 'button', onclick: () => info && fitCamera(k) }, t));
    bar.appendChild(g); bar.appendChild(SL.el('span', { class: 'grow' })); bar.appendChild(SL.el('span', { class: 'lab' }, 'Drag to orbit · scroll to zoom · drop a model file here'));
  },
  async activate() {
    active = true; legend(); results();
    if (!ready && !failed) await initGPU();
    else if (ready && V.model === 'project' && projectMeshV !== SL.version) rebuild({});
    if (failed) { $('vp-empty').hidden = false; canvas.style.visibility = 'hidden'; }
    requestAnimationFrame(frame);
  },
  deactivate() { active = false; },
  onProject() { if (ready && V.model === 'project') geomChanged(false); },
});
})();
