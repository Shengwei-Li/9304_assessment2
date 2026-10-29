// Viewer for hydrofoil_sim.slx: live telemetry from sil_bridge.py, or replay of a recorded run.
// Data handling, plots and controls; the 3D scene is in scene.js.
import { createScene } from './scene.js';

const DEG = 180 / Math.PI;
const WINDOW = 8;                            // plot window [s]
const C = { ink: '#1f2328', muted: '#6b7078', faint: '#b9bdc3', grid: '#ececed', accent: '#c2410c' };

// ------------------------------------------------------------------ data
const live = { setup: null, S: [] };
const replay = { setup: null, S: [], name: '' };
const ui = { mode: 'live', playing: true, speed: 1, tp: 0, exag: 1, ghost: true, beam: true };
const cur = () => (ui.mode === 'live' ? live : replay);

function decode(buf) {                       // big-endian doubles -> Float64Array
  const dv = new DataView(buf), n = buf.byteLength / 8, v = new Float64Array(n);
  for (let i = 0; i < n; i++) v[i] = dv.getFloat64(8 * i, false);
  return v;
}

function parseSetup(v) {                     // layout: matlab/SilTelemetry.m
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
const $ = (id) => document.getElementById(id);
let bridgeUp = false;
function connect() {
  if (location.protocol === 'file:') return;
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => { bridgeUp = true; $('conn').textContent = 'Bridge: connected'; };
  ws.onmessage = (e) => addPacket(live, decode(e.data));
  ws.onclose = () => { bridgeUp = false; $('conn').textContent = 'Bridge: offline'; setTimeout(connect, 2000); };
}
function showMsg(t) { $('msg').textContent = t; $('msg').style.display = t ? 'block' : 'none'; }

// ------------------------------------------------------------------ plots
function niceStep(range) {
  const r = range / 4, p = Math.pow(10, Math.floor(Math.log10(r))), m = r / p;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
}

function drawPlot(id, S, t1, cfg) {
  const cv = $(id), dpr = window.devicePixelRatio || 1;
  const W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  const L = 34, R = 4, T = 4, B = 15, t0 = t1 - WINDOW;
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
  g.font = '10px "Segoe UI", system-ui, sans-serif'; g.fillStyle = C.muted; g.strokeStyle = C.grid; g.lineWidth = 1;
  const step = niceStep(hi - lo);
  for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
    g.beginPath(); g.moveTo(L, Y(v)); g.lineTo(W - R, Y(v)); g.stroke();
    g.textAlign = 'right'; g.fillText(+v.toFixed(6) + '', L - 4, Y(v) + 3);
  }
  g.textAlign = 'center';
  for (let t = Math.ceil(t0); t <= t1; t += 2) if (t >= 0) g.fillText(t + ' s', X(t), H - 3);
  for (const h of cfg.hlines || []) {
    g.strokeStyle = C.muted; g.setLineDash([2, 3]); g.beginPath(); g.moveTo(L, Y(h)); g.lineTo(W - R, Y(h)); g.stroke(); g.setLineDash([]);
  }
  g.save(); g.beginPath(); g.rect(L, T, W - L - R, H - T - B); g.clip();
  for (const sr of cfg.series) {
    g.strokeStyle = g.fillStyle = sr.color; g.lineWidth = sr.w || 1.4; g.setLineDash(sr.dash || []);
    if (sr.dots) { for (let i = i0; i <= i1; i++) g.fillRect(X(S[i].t) - 1, Y(sr.get(S[i])) - 1, 2, 2); continue; }
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
  if (cfg.note) { g.fillStyle = C.muted; g.textAlign = 'center'; g.font = '11px "Segoe UI", system-ui, sans-serif'; g.fillText(cfg.note, (L + W) / 2, H / 2); }
}

function drawPlots(s, S, t) {
  const D = DEG;
  drawPlot('p-h', S, t, { minSpan: 4, series: [
    { get: (q) => 100 * (q.y[0] - s.ls * Math.sin(q.y[1])), color: C.faint, dots: true },
    { get: (q) => 100 * q.r, color: C.muted, w: 1, dash: [5, 3] },
    { get: (q) => 100 * q.x[0], color: C.ink, w: 1.8 },
    { get: (q) => 100 * q.xh[0], color: C.accent } ] });
  drawPlot('p-th', S, t, { minSpan: 0.6, series: [
    { get: (q) => D * q.y[1], color: C.faint, dots: true },
    { get: (q) => D * q.x[2], color: C.ink, w: 1.8 },
    { get: (q) => D * q.xh[2], color: C.accent } ] });
  drawPlot('p-d', S, t, { fixed: [D * s.umin - 1, D * s.umax + 1], hlines: [D * s.umin, D * s.umax], series: [
    { get: (q) => D * (s.u0[0] + q.u[0]), color: C.ink, w: 0.8, step: true },
    { get: (q) => D * (s.u0[1] + q.u[1]), color: C.muted, w: 0.8, step: true },
    { get: (q) => D * q.d[0], color: C.ink, w: 1.8 },
    { get: (q) => D * q.d[1], color: C.muted, w: 1.8 } ] });
  if (s.ext) {
    let mx = 0; for (let i = indexAt(S, t - WINDOW); i <= indexAt(S, t); i++) mx = Math.max(mx, S[i].st[1], S[i].st[2]);
    drawPlot('p-link', S, t, { fixed: [0, Math.max(25, mx * 1.1)], hlines: [1000 * s.Ts], series: [
      { get: (q) => q.st[2], color: C.faint, w: 1, step: true },
      { get: (q) => q.st[1], color: C.accent, w: 1.6, step: true } ] });
  } else drawPlot('p-link', S, t, { fixed: [0, 25], series: [], note: 'controller inside the model (ctrl_ext = 0)' });
}

// ------------------------------------------------------------------ side panel and controls
function state(s, d) {
  $('s-t').textContent = `t = ${d.t.toFixed(2)} s`;
  $('s-mode').textContent = (s.ext ? (s.lockstep ? 'Controller: separate process, lockstep' : 'Controller: separate process, real time')
                                   : 'Controller: inside the model') + (ui.mode === 'replay' ? ` · replay ${ui.speed}×` : '');
  $('s-e').textContent = `${(100 * (d.x[0] - d.r)).toFixed(2)} cm`;
  $('s-th').textContent = `${(DEG * d.x[2]).toFixed(2)}°`;
  $('s-d').textContent = `${(DEG * d.d[0]).toFixed(2)}° / ${(DEG * d.d[1]).toFixed(2)}°`;
  $('s-late').textContent = s.ext ? `${d.st[0]}` : '–';
  const S = cur().S, lag = (d.wall - S[0].wall) - (d.t - S[0].t);
  $('s-lag').textContent = `${lag >= 0 ? '+' : ''}${(1000 * lag).toFixed(0)} ms`;
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
group('view', (v) => view3d.setView(v));
group('exag', (v) => { ui.exag = +v; });
$('b-ghost').onclick = (e) => { ui.ghost = !ui.ghost; e.target.classList.toggle('on', ui.ghost); };
$('b-beam').onclick = (e) => { ui.beam = !ui.beam; e.target.classList.toggle('on', ui.beam); };

function loadReplay(buf, name) {
  const ds = parseRecording(buf);
  if (!ds.setup || !ds.S.length) { showMsg(`${name}: no samples in this file`); return; }
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
    sel.innerHTML = '<option value="">Recorded runs</option>' + names.map((n) => `<option>${n}</option>`).join('');
    sel.value = keep;
  } catch { /* bridge not running */ }
}

// ------------------------------------------------------------------ main loop
const view3d = createScene($('view'));
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
    view3d.update(ds.setup, d, { exag: ui.exag, ghost: ui.ghost, beam: ui.beam,
                                 dt: ui.mode === 'live' || ui.playing ? dt * ui.speed : 0 });
    drawPlots(ds.setup, S, t);
    state(ds.setup, d);
    showMsg('');
  } else if (ui.mode === 'live') {
    showMsg(location.protocol === 'file:' ? 'Opened as a file: use Open .bin to replay a recorded run.'
          : bridgeUp ? 'Waiting for a run: in MATLAB set viz_on = 1 and run hydrofoil_sim (see run_sil_demo.m).'
          : 'sil_bridge.py is not reachable. Start it with: python viz/sil_bridge.py');
  }
  view3d.render();
  requestAnimationFrame(frame);
}
connect();
refreshList();
requestAnimationFrame(frame);
