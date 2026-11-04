// Slipstream Lab: mesh parsing (SLM, STL, OBJ, glTF/GLB) and solid voxelization for the 3D tunnel.
// Voxelization casts rays along x, y and z through every cell column, fills between entry/exit pairs,
// and keeps a cell solid when at least two of the three directions agree (robust to small leaks).

function parseSLM(buf) {
  const dv = new DataView(buf);
  if (dv.getUint32(0, true) !== 0x314D4C53) throw new Error('Not a Slipstream mesh file');
  const nv = dv.getUint32(4, true), nt = dv.getUint32(8, true);
  return { pos: new Float32Array(buf, 12, nv * 3).slice(), idx: new Uint32Array(buf.slice(12 + nv * 12, 12 + nv * 12 + nt * 12)) };
}
function parseSTL(buf) {
  const dv = new DataView(buf);
  if (buf.byteLength >= 84) {
    const n = dv.getUint32(80, true);
    if (84 + n * 50 === buf.byteLength) {
      const pos = new Float32Array(n * 9);
      for (let i = 0; i < n; i++) { const o = 84 + i * 50 + 12; for (let j = 0; j < 9; j++) pos[i * 9 + j] = dv.getFloat32(o + j * 4, true); }
      return soup(pos);
    }
  }
  const txt = new TextDecoder().decode(buf), re = /vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)/g, out = [];
  let m; while ((m = re.exec(txt))) out.push(+m[1], +m[2], +m[3]);
  if (!out.length) throw new Error('No triangles found in this STL file');
  return soup(new Float32Array(out.slice(0, Math.floor(out.length / 9) * 9)));
}
function soup(pos) { const idx = new Uint32Array(pos.length / 3); for (let i = 0; i < idx.length; i++) idx[i] = i; return { pos, idx }; }
function parseOBJ(buf) {
  const lines = new TextDecoder().decode(buf).split('\n'), v = [], f = [];
  for (const l of lines) {
    if (l.startsWith('v ')) { const p = l.trim().split(/\s+/); v.push(+p[1], +p[2], +p[3]); }
    else if (l.startsWith('f ')) {
      const p = l.trim().split(/\s+/).slice(1).map(s => { const i = parseInt(s, 10); return i < 0 ? v.length / 3 + i : i - 1; });
      for (let i = 1; i + 1 < p.length; i++) f.push(p[0], p[i], p[i + 1]);
    }
  }
  if (!f.length) throw new Error('No faces found in this OBJ file');
  return { pos: new Float32Array(v), idx: new Uint32Array(f) };
}
function parseGLB(buf, name) {
  let json, bin;
  const dv = new DataView(buf);
  if (dv.getUint32(0, true) === 0x46546C67) {
    let off = 12;
    while (off < buf.byteLength) {
      const len = dv.getUint32(off, true), type = dv.getUint32(off + 4, true); off += 8;
      if (type === 0x4E4F534A) json = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, off, len)));
      else if (type === 0x004E4942) bin = buf.slice(off, off + len);
      off += len;
    }
  } else {
    json = JSON.parse(new TextDecoder().decode(buf));
    const u = json.buffers && json.buffers[0] && json.buffers[0].uri;
    if (!u || !u.startsWith('data:')) throw new Error('This .gltf file keeps its geometry in a separate .bin file. Export it as a single .glb (or STL) instead.');
    const b64 = u.split(',')[1], s = atob(b64), a = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) a[i] = s.charCodeAt(i); bin = a.buffer;
  }
  if ((json.extensionsUsed || []).includes('KHR_draco_mesh_compression')) throw new Error('This model uses Draco compression, which the page cannot decode. Re-export it without compression, or as STL (Blender: File → Export → STL).');
  const acc = i => {
    const a = json.accessors[i], bv = json.bufferViews[a.bufferView];
    const C = { 5126: [Float32Array, 4], 5125: [Uint32Array, 4], 5123: [Uint16Array, 2], 5121: [Uint8Array, 1] }[a.componentType];
    const nc = { SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4 }[a.type], start = (bv.byteOffset || 0) + (a.byteOffset || 0), stride = bv.byteStride || C[1] * nc;
    const out = new Float64Array(a.count * nc), d = new DataView(bin);
    const rd = { 5126: o => d.getFloat32(o, true), 5125: o => d.getUint32(o, true), 5123: o => d.getUint16(o, true), 5121: o => d.getUint8(o) }[a.componentType];
    for (let k = 0; k < a.count; k++) for (let c = 0; c < nc; c++) out[k * nc + c] = rd(start + k * stride + c * C[1]);
    return out;
  };
  const mat = n => {
    if (n.matrix) return n.matrix.slice();
    const [x, y, z, w] = n.rotation || [0, 0, 0, 1], [sx, sy, sz] = n.scale || [1, 1, 1], [tx, ty, tz] = n.translation || [0, 0, 0];
    return [(1 - 2 * (y * y + z * z)) * sx, (2 * (x * y + z * w)) * sx, (2 * (x * z - y * w)) * sx, 0,
      (2 * (x * y - z * w)) * sy, (1 - 2 * (x * x + z * z)) * sy, (2 * (y * z + x * w)) * sy, 0,
      (2 * (x * z + y * w)) * sz, (2 * (y * z - x * w)) * sz, (1 - 2 * (x * x + y * y)) * sz, 0, tx, ty, tz, 1];
  };
  const mul = (a, b) => { const o = new Array(16); for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) { let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s; } return o; };
  const P = [], I = [];
  const visit = (ni, M) => {
    const n = json.nodes[ni], W = mul(M, mat(n));
    if (n.mesh != null) for (const pr of json.meshes[n.mesh].primitives) {
      if ((pr.mode ?? 4) !== 4) continue;
      const p = acc(pr.attributes.POSITION), base = P.length / 3;
      for (let k = 0; k < p.length; k += 3) { const x = p[k], y = p[k + 1], z = p[k + 2]; P.push(W[0] * x + W[4] * y + W[8] * z + W[12], W[1] * x + W[5] * y + W[9] * z + W[13], W[2] * x + W[6] * y + W[10] * z + W[14]); }
      const ix = pr.indices != null ? acc(pr.indices) : Float64Array.from({ length: p.length / 3 }, (_, i) => i);
      for (let k = 0; k + 2 < ix.length; k += 3) I.push(base + ix[k], base + ix[k + 1], base + ix[k + 2]);
    }
    for (const c of n.children || []) visit(c, W);
  };
  const scene = json.scenes[json.scene || 0];
  for (const r of scene.nodes) visit(r, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
  if (!I.length) throw new Error('No triangle meshes found in ' + name);
  return { pos: new Float32Array(P), idx: new Uint32Array(I) };
}

// ---------- orientation guess for uploaded models ----------
// up = smallest extent; span = the horizontal axis the model is mirror-symmetric about; nose = end away from the tallest point (the fin).
function guessOrientation(pos) {
  const n = pos.length / 3, lo = [1e30, 1e30, 1e30], hi = [-1e30, -1e30, -1e30];
  for (let i = 0; i < n; i++) for (let a = 0; a < 3; a++) { const v = pos[i * 3 + a]; if (v < lo[a]) lo[a] = v; if (v > hi[a]) hi[a] = v; }
  const ext = [0, 1, 2].map(a => hi[a] - lo[a]);
  const up = [0, 1, 2].sort((a, b) => ext[a] - ext[b])[0];
  const hz = [0, 1, 2].filter(a => a !== up);
  const skew = a => { const mid = (lo[a] + hi[a]) / 2, h = ext[a] / 2 || 1; let s = 0; for (let i = 0; i < n; i++) s += ((pos[i * 3 + a] - mid) / h) ** 3; return Math.abs(s / n); };
  const span = skew(hz[0]) < skew(hz[1]) ? hz[0] : hz[1], len = hz[0] === span ? hz[1] : hz[0];
  let top = -1e30, topAt = 0; for (let i = 0; i < n; i++) { const y = pos[i * 3 + up]; if (y > top) { top = y; topAt = pos[i * 3 + len]; } }
  const noseSign = topAt > (lo[len] + hi[len]) / 2 ? -1 : 1;   // fin near +len end → nose toward −len
  return { up, span, len, noseSign };
}
// ---------- voxelization ----------
// T: {s, cx, cy, cz, aoa, px, py, R(3x3 orientation)} maps model coords → grid coords.
function transform(pos, T) {
  const n = pos.length / 3, out = new Float32Array(pos.length), a = T.aoa * Math.PI / 180, ca = Math.cos(a), sa = Math.sin(a), R = T.R;
  for (let i = 0; i < n; i++) {
    const X = pos[i * 3], Y = pos[i * 3 + 1], Z = pos[i * 3 + 2];
    let x = R[0] * X + R[1] * Y + R[2] * Z - T.cx, y = R[3] * X + R[4] * Y + R[5] * Z - T.cy, z = R[6] * X + R[7] * Y + R[8] * Z - T.cz;
    x *= T.s; y *= T.s; z *= T.s;
    out[i * 3] = T.px + x * ca + y * sa; out[i * 3 + 1] = T.py - x * sa + y * ca; out[i * 3 + 2] = z;
  }
  return out;
}
function voxelize(P, idx, NX, NY, NZ) {
  const N = NX * NY * NZ, vote = new Uint8Array(N), thin = new Uint8Array(N), nt = idx.length / 3;
  // axis triples: ray axis r, plane axes (u, v)
  const dims = [NX, NY, NZ];
  for (const [r, u, v] of [[0, 1, 2], [1, 0, 2], [2, 0, 1]]) {
    const NU = dims[u], NV = dims[v], NR = dims[r];
    const vlo = v === 2 ? 0 : 0;   // plane cells cover z >= 0 only (half domain)
    // bin triangles by their (u, v) bounding box
    const cnt = new Uint32Array(NU * NV + 1), tb = new Int32Array(nt * 4);
    for (let t = 0; t < nt; t++) {
      const i0 = idx[t * 3] * 3, i1 = idx[t * 3 + 1] * 3, i2 = idx[t * 3 + 2] * 3;
      const umin = Math.min(P[i0 + u], P[i1 + u], P[i2 + u]), umax = Math.max(P[i0 + u], P[i1 + u], P[i2 + u]);
      const vmin = Math.min(P[i0 + v], P[i1 + v], P[i2 + v]), vmax = Math.max(P[i0 + v], P[i1 + v], P[i2 + v]);
      const a0 = Math.max(0, Math.ceil(umin - 0.5)), a1 = Math.min(NU - 1, Math.floor(umax - 0.5));
      const b0 = Math.max(vlo, Math.ceil(vmin - 0.5)), b1 = Math.min(NV - 1, Math.floor(vmax - 0.5));
      tb[t * 4] = a0; tb[t * 4 + 1] = a1; tb[t * 4 + 2] = b0; tb[t * 4 + 3] = b1;
      if (a0 > a1 || b0 > b1) continue;
      for (let b = b0; b <= b1; b++) for (let aa = a0; aa <= a1; aa++) cnt[b * NU + aa + 1]++;
    }
    for (let i = 1; i <= NU * NV; i++) cnt[i] += cnt[i - 1];
    const list = new Uint32Array(cnt[NU * NV]), fill = cnt.slice(0, NU * NV);
    for (let t = 0; t < nt; t++) {
      const a0 = tb[t * 4], a1 = tb[t * 4 + 1], b0 = tb[t * 4 + 2], b1 = tb[t * 4 + 3];
      if (a0 > a1 || b0 > b1) continue;
      for (let b = b0; b <= b1; b++) for (let aa = a0; aa <= a1; aa++) list[fill[b * NU + aa]++] = t;
    }
    const hits = [];
    const jit = [1.3e-4, 2.7e-4, 4.1e-4][r];
    for (let b = vlo; b < NV; b++) for (let aa = 0; aa < NU; aa++) {
      const c = b * NU + aa, s0 = cnt[c], s1 = cnt[c + 1]; if (s0 === s1) continue;
      const pu = aa + 0.5 + jit, pv = b + 0.5 - jit * 0.7;
      hits.length = 0;
      for (let q = s0; q < s1; q++) {
        const t = list[q], i0 = idx[t * 3] * 3, i1 = idx[t * 3 + 1] * 3, i2 = idx[t * 3 + 2] * 3;
        const u0 = P[i0 + u], v0 = P[i0 + v], u1 = P[i1 + u], v1 = P[i1 + v], u2 = P[i2 + u], v2 = P[i2 + v];
        const d = (v1 - v2) * (u0 - u2) + (u2 - u1) * (v0 - v2); if (d === 0) continue;
        const l0 = ((v1 - v2) * (pu - u2) + (u2 - u1) * (pv - v2)) / d, l1 = ((v2 - v0) * (pu - u2) + (u0 - u2) * (pv - v2)) / d, l2 = 1 - l0 - l1;
        if (l0 < 0 || l1 < 0 || l2 < 0) continue;
        hits.push(l0 * P[i0 + r] + l1 * P[i1 + r] + l2 * P[i2 + r]);
      }
      if (hits.length < 2) continue;
      hits.sort((x, y) => x - y);
      for (let h = 0; h + 1 < hits.length; h += 2) {
        let k0 = Math.max(0, Math.ceil(hits[h] - 0.5)), k1 = Math.min(NR - 1, Math.floor(hits[h + 1] - 0.5));
        let th = false;
        if (k0 > k1) { const k = Math.floor((hits[h] + hits[h + 1]) / 2); if (k >= 0 && k < NR) { k0 = k; k1 = k; th = true; } }   // keep sub-cell-thin parts (thin wings)
        for (let k = k0; k <= k1; k++) {
          const g = [0, 0, 0]; g[r] = k; g[u] = aa; g[v] = b;
          const kk = g[0] + NX * (g[1] + NY * g[2]); vote[kk]++; if (th) thin[kk] = 1;
        }
      }
    }
  }
  const solid = new Uint32Array(N), tex = new Uint8Array(N * 2);
  const bb = [1e9, 1e9, 1e9, -1e9, -1e9, -1e9]; let count = 0;
  const cols = new Uint8Array(NX * NZ);
  for (let z = 0; z < NZ; z++) for (let y = 0; y < NY; y++) for (let x = 0; x < NX; x++) {
    const k = x + NX * (y + NY * z);
    if ((vote[k] < 2 && !thin[k]) || x < 1 || x > NX - 2 || y < 1 || y > NY - 2 || z > NZ - 2) continue;
    solid[k] = 1; tex[k * 2] = 255; count++; cols[x + NX * z] = 1;
    if (x < bb[0]) bb[0] = x; if (y < bb[1]) bb[1] = y; if (z < bb[2]) bb[2] = z; if (x > bb[3]) bb[3] = x; if (y > bb[4]) bb[4] = y; if (z > bb[5]) bb[5] = z;
  }
  let area = 0; for (let i = 0; i < cols.length; i++) area += cols[i];
  return { solid, tex, bb, count, area };
}

const meshes = {};
onmessage = e => {
  const m = e.data;
  try {
    if (m.type === 'parse') {
      const n = m.name.toLowerCase();
      const mesh = n.endsWith('.slm') ? parseSLM(m.buf) : n.endsWith('.stl') ? parseSTL(m.buf) : n.endsWith('.obj') ? parseOBJ(m.buf) : (n.endsWith('.glb') || n.endsWith('.gltf')) ? parseGLB(m.buf, m.name) : null;
      if (!mesh) throw new Error('Unsupported file type. Use STL, OBJ or GLB.');
      const o = m.orient === 'auto' ? guessOrientation(mesh.pos) : null;
      meshes[m.id] = mesh;
      postMessage({ type: 'parsed', rid: m.rid, id: m.id, tris: mesh.idx.length / 3, verts: mesh.pos.length / 3, guess: o });
    } else if (m.type === 'voxelize') {
      const mesh = meshes[m.id]; if (!mesh) throw new Error('Mesh not loaded');
      const P = transform(mesh.pos, m.T);
      const r = voxelize(P, mesh.idx, m.NX, m.NY, m.NZ);
      postMessage({ type: 'voxels', rid: m.rid, id: m.id, ...r }, [r.solid.buffer, r.tex.buffer]);
    } else if (m.type === 'geometry') {
      const mesh = meshes[m.id];
      const R = m.R, pos = new Float32Array(mesh.pos.length);
      const lo = [1e30, 1e30, 1e30], hi = [-1e30, -1e30, -1e30];
      for (let i = 0; i < mesh.pos.length; i += 3) {
        const X = mesh.pos[i], Y = mesh.pos[i + 1], Z = mesh.pos[i + 2];
        const v = [R[0] * X + R[1] * Y + R[2] * Z, R[3] * X + R[4] * Y + R[5] * Z, R[6] * X + R[7] * Y + R[8] * Z];
        for (let a = 0; a < 3; a++) { pos[i + a] = v[a]; if (v[a] < lo[a]) lo[a] = v[a]; if (v[a] > hi[a]) hi[a] = v[a]; }
      }
      postMessage({ type: 'geometry', rid: m.rid, id: m.id, pos, idx: mesh.idx.slice(), lo, hi }, [pos.buffer]);
    }
  } catch (err) { postMessage({ type: 'error', rid: m.rid, id: m.id, message: err.message }); }
};
