// 3D scene: sky, ocean surface, boat. Everything that moves is set from simulation
// data in update(); nothing here is simulated.
//   water   the model's wave components (wave_eval.m), summed per vertex on the GPU in
//           the boat's frame, so the surface the ultrasonic beam hits is the one drawn.
//           Short ripples and the strut wakes only change the shading, not the shape.
//   boat    true state x = [h v theta q]; second hull outline: the controller's estimate
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { Sky } from 'three/addons/objects/Sky.js';

const HC = 0.55;                 // CM height above calm water at trim, for drawing only [m]
const MAXW = 80;                 // wave components the shader accepts
const SUN = new THREE.Vector3().setFromSphericalCoords(1, THREE.MathUtils.degToRad(62), THREE.MathUtils.degToRad(35));

export function createScene(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.6;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.prepend(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(35, 1, 0.05, 5000);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;

  // ---------------------------------------------------------------- sky and light
  const sky = new Sky();
  sky.scale.setScalar(4000);
  const su = sky.material.uniforms;
  su.turbidity.value = 2;  su.rayleigh.value = 1.8;
  su.mieCoefficient.value = 0.004;  su.mieDirectionalG.value = 0.8;
  su.sunPosition.value.copy(SUN);
  scene.add(sky);

  const pmrem = new THREE.PMREMGenerator(renderer);
  const skyScene = new THREE.Scene();  skyScene.add(sky.clone());
  scene.environment = pmrem.fromScene(skyScene).texture;

  const sunLight = new THREE.DirectionalLight(0xfff4e5, 2.2);
  sunLight.position.copy(SUN).multiplyScalar(20);
  sunLight.castShadow = true;
  sunLight.shadow.mapSize.set(2048, 2048);
  Object.assign(sunLight.shadow.camera, { left: -5, right: 5, top: 5, bottom: -5, near: 1, far: 50 });
  scene.add(sunLight, new THREE.HemisphereLight(0xcfe0f0, 0x274050, 0.5));

  // ---------------------------------------------------------------- ocean
  // vertices denser near the boat: x = R sinh(5u)/sinh(5), spacing about 0.28 m at the boat
  const NX = 720, NZ = 120, RX = 1500, RZ = 1500;
  const warp = (u, R) => R * Math.sinh(5 * u) / Math.sinh(5);
  const pos = new Float32Array((NX + 1) * (NZ + 1) * 3), idx = [];
  for (let j = 0; j <= NZ; j++) for (let i = 0; i <= NX; i++) {
    const k = 3 * (j * (NX + 1) + i);
    pos[k] = warp(2 * i / NX - 1, RX);  pos[k + 1] = 0;  pos[k + 2] = warp(2 * j / NZ - 1, RZ);
  }
  for (let j = 0; j < NZ; j++) for (let i = 0; i < NX; i++) {
    const a = j * (NX + 1) + i, b = a + 1, c = a + NX + 1, d = c + 1;
    idx.push(a, c, b, b, c, d);
  }
  const oceanGeo = new THREE.BufferGeometry();
  oceanGeo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  oceanGeo.setIndex(idx);

  const col = (hex) => new THREE.Color(hex);
  const oceanMat = new THREE.ShaderMaterial({
    transparent: true, side: THREE.DoubleSide, depthWrite: false,
    uniforms: {
      uTime: { value: 0 }, uExag: { value: 1 }, uU: { value: 7 }, uN: { value: 0 },
      uA: { value: new Float32Array(MAXW) }, uK: { value: new Float32Array(MAXW) },
      uW: { value: new Float32Array(MAXW) }, uP: { value: new Float32Array(MAXW) },
      uSun: { value: SUN.clone() }, uSunColor: { value: col(0xfff1d6) },
      uDeep: { value: col(0x03202e) }, uShallow: { value: col(0x0f5f6e) },
      uZenith: { value: col(0x5d8dbd) }, uHorizon: { value: col(0xd6e3ec) },
      uStrut: { value: new THREE.Vector2(1.10, -2.45) }, uFogFar: { value: 900 },
    },
    vertexShader: /* glsl */`
      #define MAXW ${MAXW}
      uniform float uTime, uExag; uniform int uN;
      uniform float uA[MAXW], uK[MAXW], uW[MAXW], uP[MAXW];
      varying vec3 vWorld; varying float vSlope; varying float vHeight;
      void main() {
        vec3 p = position; float z = 0.0, dz = 0.0;
        for (int i = 0; i < MAXW; i++) {
          if (i >= uN) break;
          float ph = uK[i] * p.x + uW[i] * uTime + uP[i];
          z += uA[i] * cos(ph);  dz -= uA[i] * uK[i] * sin(ph);
        }
        // beyond 40 m the vertices get too sparse for short waves: fade them out there
        // (the boat and the sensor see the full surface)
        float lod = 1.0 - smoothstep(40.0, 160.0, length(p.xz));
        p.y = uExag * z * lod;  vSlope = uExag * dz * lod;  vHeight = p.y;
        vec4 w = modelMatrix * vec4(p, 1.0);  vWorld = w.xyz;
        gl_Position = projectionMatrix * viewMatrix * w;
      }`,
    fragmentShader: /* glsl */`
      uniform float uTime, uU, uFogFar;
      uniform vec3 uSun, uSunColor, uDeep, uShallow, uZenith, uHorizon;
      uniform vec2 uStrut;
      varying vec3 vWorld; varying float vSlope; varying float vHeight;
      float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
      float noise(vec2 p) {
        vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
        return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
      }
      // short waves, shading only: gradient of value-noise octaves and a few small
      // dispersive waves in scattered directions, all carried past the boat at U
      vec2 noiseGrad(vec2 p) {
        float e = 0.02, n0 = noise(p);
        return vec2(noise(p + vec2(e, 0.0)) - n0, noise(p + vec2(0.0, e)) - n0) / e;
      }
      vec2 ripples(vec2 p, float fade) {
        vec2 q = p + vec2(uU * uTime, 0.0);
        vec2 g = vec2(0.0);
        float amp = 0.03;
        for (int i = 0; i < 6; i++) {
          float fi = float(i);
          float ang = fi * 2.399 + 0.4;                       // golden-angle spread of directions
          vec2 d = vec2(cos(ang), sin(ang));
          float L = 2.6 * pow(0.62, fi), k = 6.2832 / L, w = sqrt(9.81 * k);
          g += d * (amp * cos(k * dot(d, q) + w * uTime + fi * 1.3));
          amp *= 0.78;
        }
        g += 0.020 * noiseGrad(q * 1.6 + vec2(0.0, uTime * 0.2));
        g += 0.010 * noiseGrad(q * 4.3 - vec2(uTime * 0.35, 0.0));
        return g * fade;
      }
      vec3 skyColor(vec3 r) { return mix(uHorizon, uZenith, pow(clamp(r.y, 0.0, 1.0), 0.45)); }
      void main() {
        vec3 V = normalize(cameraPosition - vWorld);
        float dist = length(cameraPosition - vWorld);
        vec2 g = ripples(vWorld.xz, 1.0 - 0.9 * smoothstep(8.0, 60.0, dist));
        vec3 n = normalize(vec3(-(vSlope + g.x), 1.0, -g.y));
        bool below = !gl_FrontFacing;
        if (below) n = -n;
        float cosv = max(dot(n, V), 0.0);
        float F = 0.02 + 0.98 * pow(1.0 - cosv, 5.0);
        vec3 col;
        float alpha;
        if (!below) {
          vec3 R = reflect(-V, n);
          vec3 refl = skyColor(R) + uSunColor * (pow(max(dot(R, uSun), 0.0), 900.0) * 30.0
                                                  + pow(max(dot(R, uSun), 0.0), 80.0) * 0.35);
          float crest = clamp(vHeight * 6.0 + 0.5, 0.0, 1.0);
          // light scattered back out of the water: brighter on crests and on slopes facing the sun
          float sss = 0.35 * crest + 0.45 * pow(max(dot(V, -uSun) * 0.5 + 0.5, 0.0), 3.0) * crest;
          vec3 body = mix(uDeep, uShallow, clamp(0.2 + sss, 0.0, 1.0));
          col = mix(body, refl, F);
          alpha = mix(0.55, 1.0, F);
        } else {
          col = mix(uShallow * 0.9, uHorizon * 0.8, 1.0 - F);
          alpha = 0.7;
        }
        // strut wakes: thin foam streaks behind the two struts, carried past at U
        for (int s = 0; s < 2; s++) {
          float xs = s == 0 ? uStrut.x : uStrut.y;
          float behind = xs - vWorld.x;
          if (behind > 0.0) {
            // turbulent strip behind the strut and two thin diverging edges
            float w = 0.025 + 0.015 * behind;
            float core = exp(-pow(vWorld.z / w, 2.0));
            float edge = exp(-pow((abs(vWorld.z) - 0.05 * behind) / (0.012 + 0.004 * behind), 2.0));
            float br = noise(vec2((vWorld.x + uU * uTime) * 5.0, vWorld.z * 25.0));
            float m = (0.6 * core + 0.25 * edge) * exp(-behind / 3.5) * (0.35 + 0.65 * br);
            col = mix(col, vec3(0.92), clamp(m, 0.0, 1.0) * (below ? 0.15 : 0.5));
          }
        }
        float fog = smoothstep(uFogFar * 0.08, uFogFar, dist);
        col = mix(col, below ? uDeep : uHorizon, fog);
        gl_FragColor = vec4(col, below ? alpha : mix(alpha, 1.0, fog));
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
      }`,
  });
  const ocean = new THREE.Mesh(oceanGeo, oceanMat);
  ocean.frustumCulled = false;
  ocean.renderOrder = 2;
  scene.add(ocean);

  // ---------------------------------------------------------------- boat
  const boat = new THREE.Group();
  scene.add(boat);
  const hullGeo = hullGeometry();
  const gelcoat = new THREE.MeshPhysicalMaterial({ color: 0xf2f2ef, roughness: 0.32, clearcoat: 0.7, clearcoatRoughness: 0.15, side: THREE.DoubleSide, flatShading: true });
  const alu = new THREE.MeshStandardMaterial({ color: 0x5a6068, metalness: 0.75, roughness: 0.35 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x22262b, metalness: 0.2, roughness: 0.6 });
  const hull = new THREE.Mesh(hullGeo, gelcoat);
  hull.castShadow = hull.receiveShadow = true;
  boat.add(hull);
  const panel = new THREE.Mesh(new THREE.BoxGeometry(3.1, 0.025, 1.16),
    [dark, dark, new THREE.MeshStandardMaterial({ map: cellTexture(), roughness: 0.18, metalness: 0.35 }), dark, dark, dark]);
  panel.position.set(-1.05, 0.265, 0);  panel.castShadow = true;
  boat.add(panel);
  const cockpit = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.02, 0.62), dark);
  cockpit.position.set(1.05, 0.255, 0);
  boat.add(cockpit);
  // windscreen: 0.16 m deep, 0.62 m wide, leaning back
  const screenGeo = new THREE.PlaneGeometry(0.16, 0.62);
  screenGeo.rotateX(-Math.PI / 2);  screenGeo.rotateZ(-1.05);
  const screen = new THREE.Mesh(screenGeo,
    new THREE.MeshPhysicalMaterial({ color: 0x9fb4c4, roughness: 0.05, transparent: true, opacity: 0.45, side: THREE.DoubleSide }));
  screen.position.set(1.47, 0.32, 0);
  boat.add(screen);
  const sensor = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.05, 0.07), dark);
  boat.add(sensor);

  let rig = new THREE.Group(), foils = [], prop = null, builtFor = null;
  boat.add(rig);
  function buildRig(s) {
    if (builtFor === s) return;
    builtFor = s;
    boat.remove(rig);  rig = new THREE.Group();  boat.add(rig);  foils = [];
    const yFoil = -(HC + s.d0);
    const strut = (x, keel, span, pod) => {
      const len = keel - yFoil;
      const st = new THREE.Mesh(new THREE.BoxGeometry(0.11, len, 0.022), alu);
      st.position.set(x, keel - len / 2, 0);  st.castShadow = true;  rig.add(st);
      const g = new THREE.Group();  g.position.set(x, yFoil, 0);
      // chord drawn 0.16 m (real 0.073 m) so the foil angle is visible
      const f = new THREE.Mesh(foilGeometry(0.16, span, 0.022), alu);  f.castShadow = true;
      g.add(f);  rig.add(g);  foils.push(g);
      if (!pod) return null;
      const body = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.04, 0.55, 24), dark);
      body.rotation.z = Math.PI / 2;  body.position.set(x - 0.12, yFoil + 0.07, 0);  rig.add(body);
      const pr = new THREE.Group();  pr.position.set(x - 0.42, yFoil + 0.07, 0);
      for (let k = 0; k < 3; k++) {
        const bl = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.16, 0.045), alu);
        bl.position.y = 0.08;  const h = new THREE.Group();  h.rotation.x = k * 2 * Math.PI / 3;  h.add(bl);  pr.add(h);
      }
      rig.add(pr);
      return pr;
    };
    strut(s.x[0], -0.24, 1.3, false);
    prop = strut(s.x[1], -0.26, 0.75, true);
    sensor.position.set(s.ls, 0.0, 0.42);
    oceanMat.uniforms.uStrut.value.set(s.x[0], s.x[1]);
  }

  // controller's estimate: outline only, in the accent colour
  const ghost = new THREE.LineSegments(new THREE.EdgesGeometry(hullGeo, 25),
    new THREE.LineBasicMaterial({ color: 0xc2410c, transparent: true, opacity: 0.85 }));
  scene.add(ghost);

  // ultrasonic beam and target height
  const beam = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
    new THREE.LineBasicMaterial({ color: 0x1f2328 }));
  scene.add(beam);
  const target = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(-3.4, 0, 0.95), new THREE.Vector3(3.6, 0, 0.95)]),
    new THREE.LineDashedMaterial({ color: 0x1f2328, dashSize: 0.14, gapSize: 0.1 }));
  target.computeLineDistances();
  scene.add(target);

  // ---------------------------------------------------------------- cameras
  const CAM = {
    side:    { p: [0.3, 0.55, 10.5], t: [0.3, -0.05, 0], follow: true },
    quarter: { p: [-7.5, 3.4, 7.6],  t: [0.3, -0.15, 0], follow: true },
    chase:   { p: [-9.5, 2.6, 2.2],  t: [1.5, 0.1, 0],   follow: true },
    bow:     { p: [4.9, 0.3, 2.8],   t: [2.4, -0.3, 0],  follow: true },
    under:   { p: [-0.6, -1.2, 7.0], t: [-0.6, -0.7, 0], follow: true },
  };
  let view = 'side';
  function setView(v) {
    view = v;
    const k = Math.max(1, 1.6 / camera.aspect), [px, py, pz] = CAM[v].p, [tx, ty, tz] = CAM[v].t;
    camera.position.set(tx + k * (px - tx), ty + k * (py - ty), tz + k * (pz - tz));
    controls.target.set(tx, ty, tz);
    controls.update();
  }
  function resize() {
    const w = container.clientWidth, h = container.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;  camera.updateProjectionMatrix();
    setView(view);
  }

  // ---------------------------------------------------------------- per frame
  const zeta = (s, x, t) => {
    if (!s || !s.waveOn) return 0;
    let z = 0;
    for (let i = 0; i < s.a.length; i++) z += s.a[i] * Math.cos(s.k[i] * x + s.we[i] * t + s.ph[i]);
    return z;
  };
  let waveFor = null;
  function update(s, d, opt) {
    const E = opt.exag, U = oceanMat.uniforms;
    buildRig(s);
    if (waveFor !== s) {                                     // wave components of this run
      waveFor = s;
      const n = s.waveOn ? Math.min(MAXW, s.a.length) : 0;
      U.uN.value = n;
      for (let i = 0; i < n; i++) { U.uA.value[i] = s.a[i]; U.uK.value[i] = s.k[i]; U.uW.value[i] = s.we[i]; U.uP.value[i] = s.ph[i]; }
      U.uU.value = s.U;
    }
    U.uTime.value = d.t;  U.uExag.value = E;

    boat.position.set(0, HC + E * d.x[0], 0);
    boat.rotation.z = E * d.x[2];
    foils.forEach((f, i) => { f.rotation.z = s.u0[i] + E * (d.d[i] - s.u0[i]); });
    if (prop) prop.rotation.x += opt.dt * 25;

    ghost.visible = opt.ghost;
    ghost.position.set(0, HC + E * d.xh[0], 0);
    ghost.rotation.z = E * d.xh[2];

    boat.updateMatrixWorld();
    const sp = sensor.getWorldPosition(new THREE.Vector3());
    const bp = beam.geometry.attributes.position;
    bp.setXYZ(0, sp.x, sp.y - 0.025, sp.z);  bp.setXYZ(1, sp.x, E * zeta(s, sp.x, d.t), sp.z);
    bp.needsUpdate = true;
    beam.visible = opt.beam;
    target.position.y = HC + E * d.r;
  }

  function render() {
    controls.update();
    const under = camera.position.y < 0;
    sky.visible = !under;
    scene.background = under ? new THREE.Color(0x0c3340) : null;
    scene.fog = under ? new THREE.FogExp2(0x0c3340, 0.11) : null;
    renderer.render(scene, camera);
  }

  // boat at trim in calm water until data arrives
  buildRig({ x: [1.10, -2.45], d0: 0.30, ls: 3.0 });
  boat.position.y = ghost.position.y = HC;
  target.position.y = HC;

  // still image of the 3D view only, at a given size in pixels (PNG data URL)
  function snapshot(w, h) {
    const pr = renderer.getPixelRatio();
    renderer.setPixelRatio(1);  renderer.setSize(w, h, false);
    camera.aspect = w / h;  camera.updateProjectionMatrix();  setView(view);
    render();
    const url = renderer.domElement.toDataURL('image/png');
    renderer.setPixelRatio(pr);  resize();
    return url;
  }

  window.addEventListener('resize', resize);
  resize();
  return { update, render, setView, snapshot };
}

// ------------------------------------------------------------------ geometry helpers
function hullGeometry() {
  const xs = -3.0, xb = 3.3, N = 72, st = [];
  for (let i = 0; i <= N; i++) {
    const x = xs + (xb - xs) * i / N;
    const f = Math.max(0, (x - 0.8) / (xb - 0.8));          // 0 aft of 0.8 m, 1 at the stem
    const b = Math.max(0.012, 0.72 * Math.sqrt(Math.max(0, 1 - f * f)));
    const yd = 0.25 + 0.10 * f * f, yk = -0.25 + 0.40 * Math.pow(f, 1.6);
    st.push([x, [[yd, -b], [yk + 0.17, -b], [yk + 0.04, -0.55 * b], [yk, 0], [yk + 0.04, 0.55 * b], [yk + 0.17, b], [yd, b]]]);
  }
  const p = [], tri = (a, b, c) => p.push(...a, ...b, ...c);
  const P = (s, j) => [s[0], s[1][j][0], s[1][j][1]];
  for (let i = 0; i < N; i++) {
    const A = st[i], B = st[i + 1];
    for (let j = 0; j < 6; j++) { tri(P(A, j), P(B, j), P(B, j + 1)); tri(P(A, j), P(B, j + 1), P(A, j + 1)); }
    tri(P(A, 0), P(A, 6), P(B, 6)); tri(P(A, 0), P(B, 6), P(B, 0));
  }
  for (let j = 1; j < 6; j++) tri(P(st[0], 0), P(st[0], j), P(st[0], j + 1));
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(p, 3));
  g.computeVertexNormals();
  return g;
}

function foilGeometry(chord, span, thick) {
  const s = new THREE.Shape();
  s.moveTo(chord / 2, 0);
  s.quadraticCurveTo(chord / 2, thick, 0, thick);
  s.lineTo(-chord / 2, 0);
  s.lineTo(0, -0.5 * thick);
  s.quadraticCurveTo(chord / 2, -0.5 * thick, chord / 2, 0);
  const g = new THREE.ExtrudeGeometry(s, { depth: span, bevelEnabled: false, curveSegments: 10 });
  g.translate(0, 0, -span / 2);
  return g;
}

function cellTexture() {
  const c = document.createElement('canvas'); c.width = 1024; c.height = 384;
  const g = c.getContext('2d');
  g.fillStyle = '#101a33'; g.fillRect(0, 0, 1024, 384);
  g.strokeStyle = '#3c4f78'; g.lineWidth = 3;
  for (let x = 0; x <= 1024; x += 1024 / 12) { g.beginPath(); g.moveTo(x, 0); g.lineTo(x, 384); g.stroke(); }
  for (let y = 0; y <= 384; y += 384 / 4) { g.beginPath(); g.moveTo(0, y); g.lineTo(1024, y); g.stroke(); }
  const t = new THREE.CanvasTexture(c);  t.colorSpace = THREE.SRGBColorSpace;  t.anisotropy = 8;
  return t;
}
