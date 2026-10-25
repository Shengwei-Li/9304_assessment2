// 3D view of hydrofoil_sim.slx: live telemetry from sil_bridge.py, or replay of a recorded run.
// Everything that moves comes from the simulation; this file only draws it.
//   boat        true state x = [h v theta q] from the nonlinear model
//   ghost boat  the controller's estimate xh (Kalman filter)
//   water       the same wave components as the model (wave_eval.m), in the boat's frame
//   foils       actual foil angles delta from the actuator model
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const DEG = 180 / Math.PI;
const HC = 0.55;            // CG height above calm water at trim, for drawing only [m]
const WINDOW = 8;           // plot window [s]

// ------------------------------------------------------------------ data
const live = { setup: null, S: [] };
const replay = { setup: null, S: [], name: '' };
const ui = { mode: 'live', playing: true, speed: 1, tp: 0, exag: 1, ghost: true, beam: true, view: 'side' };
const cur = () => (ui.mode === 'live' ? live : replay);

function decode(buf) {                       // big-endian doubles -> Float64Array
  const dv = new DataView(buf), n = buf.byteLength / 8, v = new Float64Array(n);
  for (let i = 0; i < n; i++) v[i] = dv.getFloat64(8 * i, false);
  return v;
}

function parseSetup(v) {
  const n = v[18], A = (o) => Array.from(v.slice(19 + o * n, 19 + (o + 1) * n));
  const s = { Ts: v[1], U: v[2], ls: v[3], x: [v[4], v[5]], z: [v[6], v[7]], d0: v[8], c: v[9],
              u0: [v[10], v[11]], umin: v[12], umax: v[13], waveOn: v[14], noiseOn: v[15],
              ext: v[16], lockstep: v[17], a: A(0), om: A(1), k: A(2), ph: A(3) };
  s.we = s.om.map((om, i) => om + s.k[i] * s.U);          // encounter frequency
  return s;
}

function parseSample(v) {
  return { t: v[1], wall: v[2], x: [v[3], v[4], v[5], v[6]], d: [v[7], v[8]], y: [v[9], v[10], v[11]],
           r: v[12], xh: [v[13], v[14], v[15], v[16], v[17]], st: [v[18], v[19], v[20]],
           u: v.length > 22 ? [v[21], v[22]] : [0, 0] };
}

function addPacket(ds, v) {
  if (v[0] === 0) { ds.setup = parseSetup(v); ds.S = []; }
  else if (v[0] === 1 && ds.setup) ds.S.push(parseSample(v));
}

function parseRecording(buf) {
  const dv = new DataView(buf), ds = { setup: null, S: [] };
  let off = 0;
  while (off + 8 <= buf.byteLength) {
    const n = dv.getFloat64(off, false); off += 8;
    if (off + 8 * n > buf.byteLength) break;
    addPacket(ds, decode(buf.slice(off, off + 8 * n))); off += 8 * n;
  }
  return ds;
}

function indexAt(S, t) {                     // last sample with S[i].t <= t
  let lo = 0, hi = S.length - 1;
  if (hi < 0 || t < S[0].t) return 0;
  if (t >= S[hi].t) return hi;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (S[m].t <= t) lo = m; else hi = m; }
  return lo;
}

function sampleAt(S, t) {                    // continuous states interpolated, the rest held
  const i = indexAt(S, t), a = S[i], b = S[Math.min(i + 1, S.length - 1)];
  const w = b.t > a.t ? Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))) : 0;
  const L = (p, q) => p.map((v, j) => v + (q[j] - v) * w);
  return { ...a, t, wall: a.wall + (b.wall - a.wall) * w, x: L(a.x, b.x), xh: L(a.xh, b.xh), d: L(a.d, b.d) };
}

// ------------------------------------------------------------------ live connection
const connEl = document.getElementById('conn');
function connect() {
  if (location.protocol === 'file:') { showMsg('Opened as a file: no live data. Use “Open .bin” to replay a recorded run, or start viz/sil_bridge.py and open http://localhost:8000.'); return; }
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => { connEl.textContent = '● bridge connected'; connEl.className = 'live'; };
  ws.onmessage = (e) => addPacket(live, decode(e.data));
  ws.onclose = () => { connEl.textContent = '● offline'; connEl.className = 'off'; setTimeout(connect, 2000); };
}

// ------------------------------------------------------------------ scene
const view = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
view.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xdde8f0);
scene.fog = new THREE.Fog(0xdde8f0, 30, 110);
const camera = new THREE.PerspectiveCamera(35, 1, 0.05, 400);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x51657a, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.7);
sun.position.set(6, 12, 8);
scene.add(sun);

// water: long-crested waves, so one row of vertices across is enough
const WX0 = -45, WX1 = 65, NX = 550;
const waterGeo = new THREE.PlaneGeometry(WX1 - WX0, 60, NX, 1);
waterGeo.rotateX(-Math.PI / 2);
waterGeo.translate((WX0 + WX1) / 2, 0, 0);
const wpos = waterGeo.attributes.position;
const wx = Array.from({ length: NX + 1 }, (_, i) => wpos.getX(i));
function streakTexture() {
  const c = document.createElement('canvas'); c.width = 512; c.height = 128;
  const g = c.getContext('2d');
  g.fillStyle = '#3d86ad'; g.fillRect(0, 0, 512, 128);
  for (let i = 0; i < 260; i++) {
    g.strokeStyle = `rgba(255,255,255,${0.05 + 0.12 * Math.random()})`;
    g.lineWidth = 1 + Math.random() * 1.5;
    const x = Math.random() * 512, y = Math.random() * 128, l = 10 + Math.random() * 40;
    g.beginPath(); g.moveTo(x, y); g.lineTo(x + l, y + (Math.random() - 0.5) * 3); g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}
const TILE = 6;                              // texture tile length [m]
const waterTex = streakTexture();
waterTex.repeat.set((WX1 - WX0) / TILE, 60 / (TILE / 4));
const water = new THREE.Mesh(waterGeo, new THREE.MeshStandardMaterial({
  map: waterTex, transparent: true, opacity: 0.62, roughness: 0.35, metalness: 0.05,
  side: THREE.DoubleSide, depthWrite: false }));
water.renderOrder = 2;
scene.add(water);

// boat
function hullGeometry() {
  const xs = -3.0, xb = 3.3, N = 60, st = [];
  for (let i = 0; i <= N; i++) {
    const x = xs + (xb - xs) * i / N;
    const f = Math.max(0, (x - 0.8) / (xb - 0.8));          // 0 aft of 0.8 m, 1 at the stem
    const b = Math.max(0.015, 0.72 * Math.sqrt(Math.max(0, 1 - f * f)));
    const yd = 0.25 + 0.10 * f * f, yk = -0.25 + 0.40 * Math.pow(f, 1.6);
    st.push([x, [[yd, -b], [yk + 0.17, -b], [yk + 0.04, -0.55 * b], [yk, 0], [yk + 0.04, 0.55 * b], [yk + 0.17, b], [yd, b]]]);
  }
  const p = [], tri = (a, b, c) => p.push(...a, ...b, ...c);
  const P = (s, j) => [s[0], s[1][j][0], s[1][j][1]];
  for (let i = 0; i < N; i++) {
    const A = st[i], B = st[i + 1];
    for (let j = 0; j < 6; j++) { tri(P(A, j), P(B, j), P(B, j + 1)); tri(P(A, j), P(B, j + 1), P(A, j + 1)); }
    tri(P(A, 0), P(A, 6), P(B, 6)); tri(P(A, 0), P(B, 6), P(B, 0));            // deck
  }
  for (let j = 1; j < 6; j++) tri(P(st[0], 0), P(st[0], j), P(st[0], j + 1));  // transom
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(p, 3));
  g.computeVertexNormals();
  return g;
}

function foilGeometry(chord, span, thick) {  // simple foil section extruded along the span
  const s = new THREE.Shape();
  s.moveTo(chord / 2, 0);
  s.quadraticCurveTo(chord / 2, thick, 0, thick);
  s.lineTo(-chord / 2, 0);
  s.lineTo(0, -0.5 * thick);
  s.quadraticCurveTo(chord / 2, -0.5 * thick, chord / 2, 0);
  const g = new THREE.ExtrudeGeometry(s, { depth: span, bevelEnabled: false, curveSegments: 8 });
  g.translate(0, 0, -span / 2);
  return g;
}

const hullGeo = hullGeometry();
const boat = new THREE.Group();
scene.add(boat);
const matHull = new THREE.MeshStandardMaterial({ color: 0xf3f5f7, roughness: 0.55, side: THREE.DoubleSide, flatShading: true });
const matDark = new THREE.MeshStandardMaterial({ color: 0x2a3442, roughness: 0.5, metalness: 0.3 });
boat.add(new THREE.Mesh(hullGeo, matHull));
boat.add(new THREE.LineSegments(new THREE.EdgesGeometry(hullGeo, 25), new THREE.LineBasicMaterial({ color: 0x2a3442 })));
function panelTexture() {
  const c = document.createElement('canvas'); c.width = 512; c.height = 192;
  const g = c.getContext('2d');
  g.fillStyle = '#16264a'; g.fillRect(0, 0, 512, 192);
  g.strokeStyle = '#5f7bb3'; g.lineWidth = 2;
  for (let x = 0; x <= 512; x += 512 / 10) { g.beginPath(); g.moveTo(x, 0); g.lineTo(x, 192); g.stroke(); }
  for (let y = 0; y <= 192; y += 192 / 4) { g.beginPath(); g.moveTo(0, y); g.lineTo(512, y); g.stroke(); }
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}
const panel = new THREE.Mesh(new THREE.BoxGeometry(3.3, 0.03, 1.2),
  [matDark, matDark, new THREE.MeshStandardMaterial({ map: panelTexture(), roughness: 0.3, metalness: 0.4 }), matDark, matDark, matDark]);
panel.position.set(-0.9, 0.275, 0);
boat.add(panel);

const foils = [];
let foilDepthBody = -(HC + 0.30);
let rig = new THREE.Group();                 // struts, foils, pod: rebuilt from the model parameters
boat.add(rig);
function strutAndFoil(x, keelY, span, pod) {
  const len = keelY - foilDepthBody;
  const strut = new THREE.Mesh(new THREE.BoxGeometry(0.12, len, 0.025), matDark);
  strut.position.set(x, keelY - len / 2, 0);
  rig.add(strut);
  const g = new THREE.Group();
  g.position.set(x, foilDepthBody, 0);
  // chord drawn 0.16 m (real 0.073 m) so the foil is visible at boat scale
  g.add(new THREE.Mesh(foilGeometry(0.16, span, 0.022), new THREE.MeshStandardMaterial({ color: 0xd6dbe1, roughness: 0.4, metalness: 0.5 })));
  rig.add(g);
  foils.push(g);
  if (pod) {
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.04, 0.55, 20), matDark);
    body.rotation.z = Math.PI / 2; body.position.set(x - 0.12, foilDepthBody + 0.07, 0);
    rig.add(body);
    const prop = new THREE.Group();
    prop.position.set(x - 0.42, foilDepthBody + 0.07, 0);
    for (let k = 0; k < 3; k++) {
      const blade = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.17, 0.05), matDark);
      blade.position.y = 0.085; const h = new THREE.Group(); h.rotation.x = k * 2 * Math.PI / 3; h.add(blade); prop.add(h);
    }
    rig.add(prop);
    return prop;
  }
  return null;
}
let prop = null;

const sensor = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.06, 0.08), new THREE.MeshStandardMaterial({ color: 0x15803d }));
boat.add(sensor);

// ghost boat: the controller's estimate
const ghost = new THREE.Group();
ghost.add(new THREE.Mesh(hullGeo, new THREE.MeshBasicMaterial({ color: 0xe0661b, transparent: true, opacity: 0.16, depthWrite: false, side: THREE.DoubleSide })));
ghost.add(new THREE.LineSegments(new THREE.EdgesGeometry(hullGeo, 25), new THREE.LineBasicMaterial({ color: 0xe0661b, transparent: true, opacity: 0.9 })));
scene.add(ghost);

// ultrasonic beam from the bow sensor to the water surface
const beamGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const beam = new THREE.Line(beamGeo, new THREE.LineBasicMaterial({ color: 0x22c55e }));
const hit = new THREE.Mesh(new THREE.RingGeometry(0.04, 0.07, 24), new THREE.MeshBasicMaterial({ color: 0x22c55e, side: THREE.DoubleSide }));
hit.rotation.x = -Math.PI / 2;
scene.add(beam, hit);

// target height: dashed line at the requested CG height
const refLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(-3.6, 0, 0.9), new THREE.Vector3(3.8, 0, 0.9)]),
  new THREE.LineDashedMaterial({ color: 0x1d2733, dashSize: 0.15, gapSize: 0.1 }));
refLine.computeLineDistances();
scene.add(refLine);

let builtFor = null;
function buildBoat(s) {                       // foil positions from the model parameters
  if (builtFor === s) return;
  builtFor = s;
  boat.remove(rig);
  rig = new THREE.Group();
  boat.add(rig);
  foils.length = 0;
  foilDepthBody = -(HC + s.d0);
  strutAndFoil(s.x[0], -0.22, 1.3, false);
  prop = strutAndFoil(s.x[1], -0.25, 0.75, true);
  sensor.position.set(s.ls, 0.05, 0.45);
}
buildBoat({ x: [1.10, -2.45], d0: 0.30, ls: 3.0 });

const CAM = {
  side:    { p: [0.2, 0.45, 10.5],  t: [0.2, -0.05, 0] },
  quarter: { p: [-7.5, 3.6, 7.8],  t: [0.3, -0.15, 0] },
  bow:     { p: [4.9, 0.25, 2.9],  t: [2.3, -0.3, 0] },
  under:   { p: [0.4, -1.5, 5.2],  t: [-0.4, -0.6, 0] },
};
function setView(v) {                      // presets are framed for a 16:10 view; back off in narrower ones
  ui.view = v;
  const k = Math.max(1, 1.6 / camera.aspect), [px, py, pz] = CAM[v].p, [tx, ty, tz] = CAM[v].t;
  camera.position.set(tx + k * (px - tx), ty + k * (py - ty), tz + k * (pz - tz));
  controls.target.set(tx, ty, tz);
  controls.update();
}

function zeta(s, x, t) {
  if (!s || !s.waveOn) return 0;
  let z = 0;
  for (let i = 0; i < s.a.length; i++) z += s.a[i] * Math.cos(s.k[i] * x + s.we[i] * t + s.ph[i]);
  return z;
}

function updateScene(s, d, dt) {
  const E = ui.exag;
  buildBoat(s);
  // water in the boat's frame: same formula as wave_eval.m
  for (let i = 0; i <= NX; i++) {
    const z = E * zeta(s, wx[i], d.t);
    wpos.setY(i, z); wpos.setY(i + NX + 1, z);
  }
  wpos.needsUpdate = true;
  waterGeo.computeVertexNormals();
  waterTex.offset.x = ((s.U * d.t) / TILE) % 1;           // water streams past at U

  boat.position.set(0, HC + E * d.x[0], 0);
  boat.rotation.z = E * d.x[2];
  for (let i = 0; i < 2; i++) foils[i].rotation.z = s.u0[i] + E * (d.d[i] - s.u0[i]);
  if (prop) prop.rotation.x += dt * 25;

  ghost.visible = ui.ghost && (s.ext || d.xh.some((v) => v !== 0));
  ghost.position.set(0, HC + E * d.xh[0], 0);
  ghost.rotation.z = E * d.xh[2];

  boat.updateMatrixWorld();
  const sp = sensor.getWorldPosition(new THREE.Vector3());
  const zs = E * zeta(s, sp.x, d.t);
  beamGeo.attributes.position.setXYZ(0, sp.x, sp.y - 0.03, sp.z);
  beamGeo.attributes.position.setXYZ(1, sp.x, zs, sp.z);
  beamGeo.attributes.position.needsUpdate = true;
  hit.position.set(sp.x, zs + 0.005, sp.z);
  beam.visible = hit.visible = ui.beam;

  refLine.position.y = HC + E * d.r;
}

// ------------------------------------------------------------------ plots
const css = getComputedStyle(document.documentElement);
const C = (n) => css.getPropertyValue(n).trim();
function niceStep(range) {
  const r = range / 4, p = Math.pow(10, Math.floor(Math.log10(r))), m = r / p;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
}

function drawPlot(id, S, t1, cfg) {
  const cv = document.getElementById(id), dpr = window.devicePixelRatio || 1;
  const W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  const L = 36, R = 6, T = 4, B = 16, t0 = t1 - WINDOW;
  const i0 = indexAt(S, t0), i1 = indexAt(S, t1);
  let lo = Infinity, hi = -Infinity;
  if (cfg.fixed) [lo, hi] = cfg.fixed;
  else {
    for (const sr of cfg.series) for (let i = i0; i <= i1; i++) { const v = sr.get(S[i]); if (v < lo) lo = v; if (v > hi) hi = v; }
    const mid = (lo + hi) / 2, half = Math.max((hi - lo) / 2 * 1.1, cfg.minSpan / 2);
    lo = mid - half; hi = mid + half;
  }
  if (!isFinite(lo)) { lo = -1; hi = 1; }
  const X = (t) => L + (t - t0) / WINDOW * (W - L - R), Y = (v) => T + (hi - v) / (hi - lo) * (H - T - B);
  g.font = '10px system-ui, sans-serif'; g.fillStyle = C('--muted'); g.strokeStyle = C('--line'); g.lineWidth = 1;
  const step = niceStep(hi - lo);
  for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
    g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(W - R, Y(v)); g.stroke();
    g.textAlign = 'right'; g.fillText(+v.toFixed(6) + '', L - 4, Y(v) + 3);
  }
  g.textAlign = 'center';
  for (let t = Math.ceil(t0); t <= t1; t += 2) if (t >= 0) g.fillText(t + ' s', X(t), H - 3);
  for (const h of cfg.hlines || []) {
    g.strokeStyle = h.color; g.setLineDash([4, 3]); g.beginPath(); g.moveTo(L, Y(h.v)); g.lineTo(W - R, Y(h.v)); g.stroke(); g.setLineDash([]);
  }
  g.save(); g.beginPath(); g.rect(L, T, W - L - R, H - T - B); g.clip();
  for (const sr of cfg.series) {
    g.strokeStyle = g.fillStyle = sr.color; g.lineWidth = sr.w || 1.5; g.setLineDash(sr.dash || []);
    if (sr.dots) {
      for (let i = i0; i <= i1; i++) g.fillRect(X(S[i].t) - 1, Y(sr.get(S[i])) - 1, 2, 2);
      continue;
    }
    g.beginPath();
    for (let i = i0; i <= i1; i++) {
      const x = X(S[i].t), y = Y(sr.get(S[i]));
      if (i === i0) g.moveTo(x, y);
      else if (sr.step) { g.lineTo(x, Y(sr.get(S[i - 1]))); g.lineTo(x, y); }
      else g.lineTo(x, y);
    }
    g.stroke();
  }
  g.setLineDash([]);
  g.restore();
  g.strokeStyle = C('--ink'); g.globalAlpha = 0.35; g.beginPath(); g.moveTo(X(t1), T); g.lineTo(X(t1), H - B); g.stroke(); g.globalAlpha = 1;
  if (cfg.note) { g.fillStyle = C('--muted'); g.textAlign = 'center'; g.font = '12px system-ui, sans-serif'; g.fillText(cfg.note, (L + W) / 2, H / 2); }
}

function drawPlots(s, S, t) {
  const ls = s.ls, D = DEG;
  drawPlot('p-h', S, t, { minSpan: 4, series: [
    { get: (q) => 100 * (q.y[0] - ls * Math.sin(q.y[1])), color: C('--meas'), dots: true },
    { get: (q) => 100 * q.r, color: C('--ref'), w: 1, dash: [5, 3] },
    { get: (q) => 100 * q.x[0], color: C('--true'), w: 2 },
    { get: (q) => 100 * q.xh[0], color: C('--est'), w: 1.5 } ] });
  drawPlot('p-th', S, t, { minSpan: 0.6, series: [
    { get: (q) => D * q.y[1], color: C('--meas'), dots: true },
    { get: (q) => D * q.x[2], color: C('--true'), w: 2 },
    { get: (q) => D * q.xh[2], color: C('--est'), w: 1.5 } ] });
  drawPlot('p-d', S, t, { fixed: [D * s.umin - 1, D * s.umax + 1],
    hlines: [{ v: D * s.umin, color: C('--lim') }, { v: D * s.umax, color: C('--lim') }], series: [
    { get: (q) => D * (s.u0[0] + q.u[0]), color: C('--front'), w: 1, step: true },
    { get: (q) => D * (s.u0[1] + q.u[1]), color: C('--rear'), w: 1, step: true },
    { get: (q) => D * q.d[0], color: C('--front'), w: 2.2 },
    { get: (q) => D * q.d[1], color: C('--rear'), w: 2.2 } ] });
  if (s.ext) {
    let mx = 0; for (let i = indexAt(S, t - WINDOW); i <= indexAt(S, t); i++) mx = Math.max(mx, S[i].st[1], S[i].st[2]);
    drawPlot('p-link', S, t, { fixed: [0, Math.max(25, mx * 1.1)], hlines: [{ v: 1000 * s.Ts, color: C('--lim') }], series: [
      { get: (q) => q.st[2], color: C('--meas'), w: 1, step: true },
      { get: (q) => q.st[1], color: C('--proc'), w: 1.8, step: true } ] });
  } else drawPlot('p-link', S, t, { fixed: [0, 25], series: [], note: 'controller runs inside the model (ctrl_ext = 0)' });
}

// ------------------------------------------------------------------ HUD and controls
const $ = (id) => document.getElementById(id);
function showMsg(t) { $('msg').style.display = t ? 'flex' : 'none'; if (t) $('msg-text').textContent = t; }
function hud(s, d) {
  $('hud-t').textContent = `t = ${d.t.toFixed(2)} s`;
  $('hud-mode').textContent = (s.ext ? (s.lockstep ? 'Controller: separate process (TCP), lockstep' : 'Controller: separate process (TCP), real time')
                                     : 'Controller: inside the model') + (ui.mode === 'replay' ? ` · replay ${ui.speed}×` : '');
  $('hud-e').textContent = `${(100 * (d.x[0] - d.r)).toFixed(2)} cm`;
  $('hud-th').textContent = `${(DEG * d.x[2]).toFixed(2)}°`;
  $('hud-d').textContent = `${(DEG * d.d[0]).toFixed(2)}° / ${(DEG * d.d[1]).toFixed(2)}°`;
  $('hud-late').textContent = s.ext ? `${d.st[0]}` : '–';
  const S = cur().S, lag = (d.wall - S[0].wall) - (d.t - S[0].t);   // drift since the first sample
  $('hud-lag').textContent = `${lag >= 0 ? '+' : ''}${(1000 * lag).toFixed(0)} ms`;
}

function setMode(m) {
  ui.mode = m;
  $('b-live').classList.toggle('on', m === 'live');
  $('b-play').disabled = $('scrub').disabled = m === 'live';
  if (m === 'replay') { ui.tp = replay.S.length ? replay.S[0].t : 0; ui.playing = true; $('b-play').textContent = 'Pause'; }
}
$('b-live').onclick = () => { $('rec-list').value = ''; setMode('live'); };
$('b-play').onclick = () => {
  const S = replay.S;
  if (!ui.playing && S.length && ui.tp >= S[S.length - 1].t) ui.tp = S[0].t;
  ui.playing = !ui.playing; $('b-play').textContent = ui.playing ? 'Pause' : 'Play';
};
$('scrub').oninput = (e) => {
  const S = replay.S; if (!S.length) return;
  ui.tp = S[0].t + e.target.value * (S[S.length - 1].t - S[0].t);
};
function group(attr, fn) {
  document.querySelectorAll(`[data-${attr}]`).forEach((b) => b.onclick = () => {
    document.querySelectorAll(`[data-${attr}]`).forEach((o) => o.classList.toggle('on', o === b));
    fn(b.dataset[attr]);
  });
}
group('speed', (v) => { ui.speed = +v; });
group('view', (v) => setView(v));
group('exag', (v) => { ui.exag = +v; });
$('b-ghost').onclick = (e) => { ui.ghost = !ui.ghost; e.target.classList.toggle('on', ui.ghost); };
$('b-beam').onclick = (e) => { ui.beam = !ui.beam; e.target.classList.toggle('on', ui.beam); };

function loadReplay(buf, name) {
  const ds = parseRecording(buf);
  if (!ds.setup || !ds.S.length) { showMsg(`${name}: no samples in this file.`); return; }
  Object.assign(replay, ds, { name });
  setMode('replay');
}
$('b-file').onclick = () => $('file').click();
$('file').onchange = async (e) => { const f = e.target.files[0]; if (f) loadReplay(await f.arrayBuffer(), f.name); };
$('rec-list').onfocus = refreshList;
$('rec-list').onchange = async (e) => {
  const n = e.target.value; if (!n) return;
  const r = await fetch(`recordings/${n}`); loadReplay(await r.arrayBuffer(), n);
};
async function refreshList() {
  if (location.protocol === 'file:') return;
  try {
    const names = await (await fetch('recordings/')).json(), sel = $('rec-list'), keep = sel.value;
    sel.innerHTML = '<option value="">recorded runs…</option>' + names.map((n) => `<option>${n}</option>`).join('');
    sel.value = keep;
  } catch { /* bridge not running */ }
}

// ------------------------------------------------------------------ main loop
function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h; camera.updateProjectionMatrix();
  setView(ui.view);
}
window.addEventListener('resize', resize);
resize();

let last = performance.now();
function frame(now) {
  const dt = Math.min(0.1, (now - last) / 1000); last = now;
  const ds = cur(), S = ds.S;
  if (ds.setup && S.length) {
    let t;
    if (ui.mode === 'live') t = S[S.length - 1].t;
    else {
      const tEnd = S[S.length - 1].t;
      if (ui.playing) { ui.tp += dt * ui.speed; if (ui.tp >= tEnd) { ui.tp = tEnd; ui.playing = false; $('b-play').textContent = 'Play'; } }
      t = ui.tp;
      $('scrub').value = (t - S[0].t) / Math.max(1e-9, tEnd - S[0].t);
    }
    const d = sampleAt(S, t);
    updateScene(ds.setup, d, ui.mode === 'live' || ui.playing ? dt * Math.min(1, ui.speed) : 0);
    drawPlots(ds.setup, S, t);
    hud(ds.setup, d);
    showMsg('');
  } else if (ui.mode === 'live' && location.protocol !== 'file:') {
    showMsg(connEl.className === 'live'
      ? 'Bridge connected. Waiting for a run: in MATLAB set viz_on = 1 and run hydrofoil_sim (see run_sil.m).'
      : 'sil_bridge.py is not reachable. Start it with: python viz/sil_bridge.py');
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
connect();
refreshList();
requestAnimationFrame(frame);
