// Live breadcrumb RTH over Google Photorealistic 3D Tiles: the offset model runs in the browser (onnxruntime-web) on
// nadir frames rendered by a second, independent CesiumWidget (its own WebGL context + tileset, so the main view never
// flickers or loses LOD). Closed loop: nav.js (truth sim + 2D KF + drift + guidance).
// URL: ?mission=mission_012 &ep=wasm|webgpu &sse=2 (nadir tileset maximumScreenSpaceError) &main=0 (no main view,
//      headless speed) &speed=1 &autoplay=ours|stock &wait=4000 (max ms to wait for nadir tiles per frame)
// Test hook: window.__live (see bottom).
(() => {
  'use strict';
  const params = new URLSearchParams(location.search);
  const TOKEN_KEY = 'homeward.cesiumIonToken';
  const $ = (id) => document.getElementById(id);
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  const ORT_DIST = 'https://cdn.jsdelivr.net/npm/onnxruntime-web@1.22.0/dist/';
  const S = { mission: null, running: false, speed: parseFloat(params.get('speed') || '1'), log: [] };
  const SPEEDS = [1, 2, 4, 8, Infinity];

  function readToken() {   // same sources as ../cesium/demo.js; never logged
    const h = new URLSearchParams(location.hash.slice(1));
    if (h.get('token')) {
      try { localStorage.setItem(TOKEN_KEY, h.get('token')); } catch (e) { /* storage blocked */ }
      history.replaceState(null, '', location.pathname + location.search);
      return h.get('token');
    }
    if (typeof window.CESIUM_ION_TOKEN === 'string' && window.CESIUM_ION_TOKEN.trim()) return window.CESIUM_ION_TOKEN.trim();
    try { return localStorage.getItem(TOKEN_KEY) || ''; } catch (e) { return ''; }
  }
  const status = (txt, cls = '') => { const s = $('status'); s.textContent = txt; s.className = 'status ' + cls; };

  // ------------------------------------------------------------------ local frame (tools/export_trajectories.py)
  let FR, M0, U0, N0, ground, drift, thr, missions;
  function nedToCart(n, e, d, out) {
    return Cesium.Matrix4.multiplyByPoint(M0, new Cesium.Cartesian3(e, n, -d + FR.ground0_enu_z), out || new Cesium.Cartesian3());
  }

  // ------------------------------------------------------------------ nadir camera (second widget)
  class NadirCam {
    constructor(el, sse) {
      this.w = new Cesium.CesiumWidget(el, {
        globe: false, skyBox: false, skyAtmosphere: false, baseLayer: false, useDefaultRenderLoop: false,
        contextOptions: { webgl: { preserveDrawingBuffer: true, alpha: false, antialias: true } }, creditContainer: document.createElement('div'),
      });
      const sc = this.w.scene;
      sc.fog.enabled = false; sc.highDynamicRange = false; sc.backgroundColor = Cesium.Color.BLACK;
      if (sc.sun) sc.sun.show = false; if (sc.moon) sc.moon.show = false;
      sc.postProcessStages.fxaa.enabled = false;
      this.sse = sse;
      this.cv = document.createElement('canvas'); this.cv.width = this.cv.height = 224;
      this.ctx = this.cv.getContext('2d', { willReadFrequently: true });
    }
    async init() {
      this.ts = await Cesium.Cesium3DTileset.fromIonAssetId(2275207, { maximumScreenSpaceError: this.sse, showCreditsOnScreen: false });
      this.w.scene.primitives.add(this.ts);
    }
    // level, north-up nadir view at local NED (n, e, d), square FOV fov_deg -> {rgb: Uint8Array(224*224*3), image: ImageData}
    async capture(n, e, d, fovDeg, maxWait = 4000) {
      const t0 = performance.now(), cam = this.w.scene.camera;
      const f = new Cesium.PerspectiveFrustum({ fov: Cesium.Math.toRadians(fovDeg), aspectRatio: 1, near: 1, far: 20000 });
      cam.frustum = f;
      cam.setView({ destination: nedToCart(n, e, d), orientation: { direction: Cesium.Cartesian3.negate(U0, new Cesium.Cartesian3()), up: Cesium.Cartesian3.clone(N0) } });
      let k = 0, loaded = false;
      for (;;) {
        this.w.render(); k++;
        if (k >= 3 && this.ts.tilesLoaded) { loaded = true; break; }
        if (performance.now() - t0 > maxWait) break;
        await sleep(k < 3 ? 0 : 15);
      }
      this.w.render();
      const tRender = performance.now() - t0;
      this.ctx.drawImage(this.w.canvas, 0, 0, 224, 224);
      const im = this.ctx.getImageData(0, 0, 224, 224), src = im.data, rgb = new Uint8Array(224 * 224 * 3);
      for (let i = 0, j = 0; i < src.length; i += 4, j += 3) { rgb[j] = src[i]; rgb[j + 1] = src[i + 1]; rgb[j + 2] = src[i + 2]; }
      return { rgb, image: im, renderMs: tRender, renders: k, loaded, canvas: [this.w.canvas.width, this.w.canvas.height] };
    }
  }

  // ------------------------------------------------------------------ model (onnxruntime-web)
  const Model = {
    async init(ep) {
      ort.env.wasm.wasmPaths = ORT_DIST;
      ort.env.wasm.numThreads = self.crossOriginIsolated ? Math.min(4, navigator.hardwareConcurrency || 1) : 1;
      const eps = ep === 'webgpu' ? ['webgpu', 'wasm'] : ['wasm'];
      this.ep = ep;
      this.enc = await ort.InferenceSession.create('models/encoder.onnx', { executionProviders: eps, graphOptimizationLevel: 'all' });
      this.mat = await ort.InferenceSession.create('models/matcher.onnx', { executionProviders: eps, graphOptimizationLevel: 'all' });
      this.ones = new Float32Array(196).fill(1);
    },
    async encode(rgb) {
      const t0 = performance.now();
      const o = await this.enc.run({ rgb: new ort.Tensor('uint8', rgb, [1, 224, 224, 3]) });
      return { desc: new Float32Array(o.desc.data), ms: performance.now() - t0 };
    },
    async match(q, b) {
      const t0 = performance.now(), T = (a, s) => new ort.Tensor('float32', a, s);
      const o = await this.mat.run({ q: T(q, [1, 16, 14, 14]), qm: T(this.ones, [1, 14, 14]), b: T(b, [1, 16, 14, 14]), bm: T(this.ones, [1, 14, 14]) });
      const mu = o.offset_cells.data, c = o.cov_cells.data, sig = (x) => 1 / (1 + Math.exp(-x));
      return { mu: [mu[0], mu[1]], cov: [[c[0], c[1]], [c[2], c[3]]], pArr: sig(o.arrival_logit.data[0]), pNm: sig(o.nomatch_logit.data[0]), ms: performance.now() - t0 };
    },
  };

  // ------------------------------------------------------------------ main view
  let viewer = null, ents = [];
  const ICON_DRONE = 'data:image/svg+xml,' + encodeURIComponent('<svg xmlns="http://www.w3.org/2000/svg" viewBox="-50 -50 100 100"><g stroke="#202630" stroke-width="16" stroke-linecap="round"><path d="M-30-30L30 30M30-30L-30 30"/></g><g stroke="#fff" stroke-width="8" stroke-linecap="round"><path d="M-30-30L30 30M30-30L-30 30"/></g><g fill="#fff" stroke="#202630" stroke-width="5"><circle cx="-32" cy="-32" r="15"/><circle cx="32" cy="-32" r="15"/><circle cx="-32" cy="32" r="15"/><circle cx="32" cy="32" r="15"/><rect x="-13" y="-16" width="26" height="32" rx="7"/></g></svg>');
  async function initMain() {
    viewer = new Cesium.Viewer('main', {
      baseLayer: false, baseLayerPicker: false, geocoder: false, homeButton: false, sceneModePicker: false, navigationHelpButton: false,
      animation: false, timeline: false, fullscreenButton: false, infoBox: false, selectionIndicator: false, globe: false,
    });
    viewer.scene.skyAtmosphere.show = true;
    viewer.scene.fog.enabled = true;
    const ts = await Cesium.Cesium3DTileset.fromIonAssetId(2275207, { maximumScreenSpaceError: 16, showCreditsOnScreen: true });
    viewer.scene.primitives.add(ts);
    S.mainTs = ts;
  }
  function buildScene(m) {
    if (!viewer) return;
    ents.forEach((e) => viewer.entities.remove(e)); ents = [];
    const C = Cesium.Color, add = (e) => { const x = viewer.entities.add(e); ents.push(x); return x; }, cb = (f) => new Cesium.CallbackProperty(f, false);
    const D = m.D, est = m.vision ? C.fromCssColorString('#34c759') : C.fromCssColorString('#ff3b30');
    const path = (col) => () => { const T = m.traj, pts = []; const s = Math.max(1, Math.floor(T.length / 1500)); for (let i = 0; i < T.length; i += s) pts.push(nedToCart(T[i][col], T[i][col + 1], D)); pts.push(col === 1 ? nedToCart(m.p[0], m.p[1], D) : nedToCart(m.kf.x[0], m.kf.x[1], D)); return pts.length > 1 ? pts : [pts[0], pts[0]]; };
    add({ polyline: { positions: cb(path(1)), width: 3, material: C.WHITE, depthFailMaterial: C.WHITE.withAlpha(0.4) } });
    add({ polyline: { positions: cb(() => (m.tJam === null ? [] : path(3)().slice(Math.floor(m.traj.findIndex((r) => r[5] >= 2) / Math.max(1, Math.floor(m.traj.length / 1500)))))), width: 3, material: est, depthFailMaterial: est.withAlpha(0.4) } });
    add({ position: nedToCart(m.home[0], m.home[1], -m.zHome - 1.5), point: { pixelSize: 14, color: C.WHITE, outlineColor: C.fromCssColorString('#202630'), outlineWidth: 3, disableDepthTestDistance: Infinity },
      label: { text: 'Home', font: '600 14px sans-serif', fillColor: C.fromCssColorString('#202630'), showBackground: true, backgroundColor: C.WHITE, pixelOffset: new Cesium.Cartesian2(0, -24), disableDepthTestDistance: Infinity } });
    S.bcEnts = [];
    add({ position: cb(() => nedToCart(m.kf.x[0], m.kf.x[1], D)), point: { pixelSize: 12, color: est, outlineColor: C.WHITE, outlineWidth: 2, disableDepthTestDistance: Infinity, show: cb(() => m.tJam !== null) } });
    add({ position: cb(() => nedToCart(m.p[0], m.p[1], D)), billboard: { image: ICON_DRONE, width: 38, height: 38, disableDepthTestDistance: Infinity } });
    add({ position: cb(() => (m.k !== null && m.bcs[m.k] ? nedToCart(m.bcs[m.k].pe[0], m.bcs[m.k].pe[1], D) : nedToCart(m.p[0], m.p[1], D))),
      point: { pixelSize: 10, color: C.fromCssColorString('#1a73e8'), outlineColor: C.WHITE, outlineWidth: 2, disableDepthTestDistance: Infinity, show: cb(() => m.k !== null && m.vision) } });
  }
  function addBcEnt(m, bc) {
    if (!viewer) return;
    ents.push(viewer.entities.add({ position: nedToCart(bc.pe[0], bc.pe[1], m.D), point: { pixelSize: 5, color: Cesium.Color.WHITE.withAlpha(0.85), outlineColor: Cesium.Color.fromCssColorString('#202630').withAlpha(0.6), outlineWidth: 1 } }));
  }
  let headingSm = null;
  function updateCam(m, mode = 'chase') {
    if (!viewer || !m) return;
    const tgt = nedToCart(m.p[0], m.p[1], m.D);
    if (mode === 'top') {
      const pts = m.route.concat([[m.p[0], m.p[1]]]).map((q) => nedToCart(q[0], q[1], m.D));
      const bs = Cesium.BoundingSphere.fromPoints(pts);
      viewer.camera.lookAt(bs.center, new Cesium.HeadingPitchRange(0, -Cesium.Math.PI_OVER_TWO + 0.001, Math.max(600, bs.radius * 2.6)));
      return;
    }
    const v = m.v, h = Math.hypot(v[0], v[1]) > 1 ? Math.atan2(v[1], v[0]) : headingSm ?? 0;
    if (headingSm === null) headingSm = h;
    headingSm += Math.atan2(Math.sin(h - headingSm), Math.cos(h - headingSm)) * 0.05;
    viewer.camera.lookAt(tgt, new Cesium.HeadingPitchRange(headingSm + Cesium.Math.toRadians(28), Cesium.Math.toRadians(-24), 320));
  }

  // ------------------------------------------------------------------ HUD
  const fmt = (x, d = 1) => (x === null || x === undefined || !isFinite(x) ? '–' : x.toFixed(d));
  function hud(m) {
    if (!m) return;
    const err = Math.hypot(m.p[0] - m.kf.x[0], m.p[1] - m.kf.x[1]), dh = Math.hypot(m.p[0] - m.home[0], m.p[1] - m.home[1]);
    const fr = m.frames, fused = fr.filter((f) => f.status === 'fused').length, last = fr.length ? fr[fr.length - 1] : null;
    status(m.done ? `Home (${m.end}), ${fmt(dh)} m from take-off` : m.phase === 'out' ? 'GPS OK · laying breadcrumbs' : `GPS JAMMED · ${m.vision ? 'returning on live vision fixes' : 'returning on IMU only'}`,
      m.done ? 'home' : m.phase === 'out' ? '' : 'jam');
    const rows = [['Sim time', `${fmt(m.t, 0)} s`], ['Position error (est − true)', `${fmt(err)} m`], ['Distance to home', `${fmt(dh, 0)} m`],
      ['Breadcrumbs', `${m.bcs.length}${m.k !== null ? ` · target #${m.k}` : ''}`]];
    if (m.vision && fr.length) {
      rows.push(['Fixes fused', `${fused}/${fr.length} (${fmt(100 * fused / fr.length, 0)}%)`]);
      if (last && last.pNm !== undefined) rows.push(['Last fix', `${last.status} · err ${fmt(last.fixErr)} m · σ ${fmt(last.sigma)} m`], ['p_nomatch / p_arrival', `${fmt(last.pNm, 3)} / ${fmt(last.pArr, 3)}`]);
    }
    if (S.lastMs) rows.push(['Model (enc+match)', `${fmt(S.lastMs.enc)} + ${fmt(S.lastMs.mat)} ms (${Model.ep})`], ['Nadir render', `${fmt(S.lastMs.render, 0)} ms`]);
    $('stats').innerHTML = rows.map(([a, b]) => `<dt>${a}</dt><dd>${b}</dd>`).join('');
  }
  function showBc(m, k) {
    const b = m.bcs[k];
    if (!b || !b.img) return;
    $('bcimg').getContext('2d').putImageData(b.img, 0, 0);
    $('bccap').textContent = `breadcrumb #${k} · ${Math.round(b.bucket)} m footprint`;
  }

  // ------------------------------------------------------------------ closed loop driver
  async function fly(id, vision, opts = {}) {
    const spec = missions.find((s) => s.id === id) || missions[0];
    const m = new Nav.Mission(spec, { vision, drift, ground, thr, camHz: opts.camHz });
    S.mission = m; S.running = true; headingSm = null;
    buildScene(m);
    const speed = () => opts.speed ?? S.speed;
    let wall0 = performance.now(), sim0 = 0, nbc = 0;
    const maxWait = parseFloat(params.get('wait') || opts.wait || '4000');
    for (;;) {
      if (S.abort === m) break;
      const r = m.advance(0.1);
      if (r.type === 'done') break;
      if (r.type === 'bc' || r.type === 'cam') {
        const cap = await S.cam.capture(r.n, r.e, r.d, r.fov_deg, maxWait);
        const enc = await Model.encode(cap.rgb);
        if (r.type === 'bc') {
          m.provide({ desc: enc.desc, img: cap.image });
          addBcEnt(m, m.bcs[r.i]); nbc++;
          S.lastMs = { enc: enc.ms, mat: S.lastMs?.mat ?? 0, render: cap.renderMs };
        } else {
          const res = await Model.match(enc.desc, m.bcs[r.k].desc);
          res.ms = enc.ms + res.ms;
          res.encMs = enc.ms;
          m.provide(res);
          const f = m.frames[m.frames.length - 1];
          f.renderMs = cap.renderMs; f.loaded = cap.loaded; f.encMs = enc.ms;
          S.lastMs = { enc: enc.ms, mat: res.ms - enc.ms, render: cap.renderMs };
          showBc(m, r.k);
        }
        if (!cap.loaded) S.log.push(`frame at t=${m.t.toFixed(1)} rendered before all nadir tiles loaded`);
        // do not race to catch up after a slow capture
        wall0 = performance.now(); sim0 = m.t;
        hud(m);
      } else {
        const sp = speed();
        if (isFinite(sp)) {
          const ahead = (m.t - sim0) / sp * 1000 - (performance.now() - wall0);
          if (ahead > 0) await sleep(ahead);
          else if (ahead < -500) { wall0 = performance.now(); sim0 = m.t; }
        } else if (Math.round(m.t * 10) % 20 === 0) await sleep(0);
        hud(m);
      }
    }
    S.running = false;
    hud(m);
    const sum = m.summary();
    const fr = m.frames.filter((f) => f.renderMs !== undefined);
    const med = (a) => { const s = a.slice().sort((x, y) => x - y); return s.length ? s[Math.floor(s.length / 2)] : null; };
    sum.render_ms_median = med(fr.map((f) => f.renderMs));
    sum.enc_ms_median = med(fr.map((f) => f.encMs));
    sum.match_ms_median = med(fr.map((f) => f.ms - f.encMs));
    sum.frames_not_fully_loaded = fr.filter((f) => !f.loaded).length;
    sum.ep = Model.ep; sum.nadir_sse = S.cam ? S.cam.sse : null;
    S.lastSummary = sum;
    return sum;
  }

  // ------------------------------------------------------------------ boot
  async function boot() {
    if (window.__cesiumLoadFailed || typeof Cesium === 'undefined') throw new Error('CesiumJS failed to load');
    if (window.__ortLoadFailed || typeof ort === 'undefined') throw new Error('onnxruntime-web failed to load');
    const token = readToken();
    if (!token) { status('Needs a Cesium ion token (config.local.js, #token=…).', 'jam'); return; }
    Cesium.Ion.defaultAccessToken = token;
    const [fr, gb, dj, tj, mj] = await Promise.all(['data/frame.json', 'data/ground_i16.bin', 'data/drift.json', 'models/model.json', 'data/missions.json'].map((u) => fetch(u)));
    FR = await fr.json();
    ground = new Nav.Ground(FR.ground_grid, new Int16Array(await gb.arrayBuffer()));
    drift = new Nav.Drift(await dj.json());
    thr = await tj.json();
    missions = await mj.json();
    const P0 = Cesium.Cartesian3.fromDegrees(FR.lon0, FR.lat0, FR.ground_h_ellipsoid);
    M0 = Cesium.Transforms.eastNorthUpToFixedFrame(P0);
    N0 = Cesium.Cartesian3.normalize(Cesium.Matrix4.getColumn(M0, 1, new Cesium.Cartesian4()), new Cesium.Cartesian3());
    U0 = Cesium.Cartesian3.normalize(Cesium.Matrix4.getColumn(M0, 2, new Cesium.Cartesian4()), new Cesium.Cartesian3());
    const sel = $('mission');
    missions.forEach((s) => sel.insertAdjacentHTML('beforeend', `<option value="${s.id}">${s.id} · ${(s.legs.reduce((a, l) => a + l[1], 0) / 1000).toFixed(1)} km · AGL ${Math.round(s.agl)} m</option>`));
    if (params.get('mission')) sel.value = params.get('mission');
    status('Loading model + tiles…');
    const ep = params.get('ep') || 'wasm';
    await Model.init(ep);
    S.cam = new NadirCam('nadir', parseFloat(params.get('sse') || '2'));
    await S.cam.init();
    if (params.get('main') !== '0') await initMain();
    const go = (vision) => { if (S.running) S.abort = S.mission; setTimeout(() => fly(sel.value, vision), S.running ? 300 : 0); };
    $('b-ours').onclick = () => go(true);
    $('b-stock').onclick = () => go(false);
    $('b-speed').onclick = () => { const i = (SPEEDS.indexOf(S.speed) + 1) % SPEEDS.length; S.speed = SPEEDS[i]; $('b-speed').textContent = isFinite(S.speed) ? `${S.speed}×` : 'max'; };
    $('b-speed').textContent = isFinite(S.speed) ? `${S.speed}×` : 'max';
    if (viewer) viewer.scene.preRender.addEventListener(() => updateCam(S.mission));
    status('Ready');
    window.__liveReady = true;
    if (params.get('autoplay')) go(params.get('autoplay') !== 'stock');
  }

  window.__live = {
    fly: (id, vision, opts) => fly(id, vision, opts),
    state: () => { const m = S.mission; return m ? { t: m.t, phase: m.phase, k: m.k, bcs: m.bcs.length, frames: m.frames.length, err: Math.hypot(m.p[0] - m.kf.x[0], m.p[1] - m.kf.x[1]), home: Math.hypot(m.p[0] - m.home[0], m.p[1] - m.home[1]), done: m.done, running: S.running } : null; },
    summary: () => S.lastSummary,
    frames: () => (S.mission ? S.mission.frames.map((f) => ({ ...f })) : []),
    traj: () => (S.mission ? S.mission.traj : []),
    log: () => S.log,
    // capture at local NED -> base64 RGB (domain check)
    capture: async (n, e, d, fov, wait = 8000, sse = null) => {
      if (sse !== null) S.cam.ts.maximumScreenSpaceError = sse;
      const c = await S.cam.capture(n, e, d, fov, wait);
      let s = ''; for (let i = 0; i < c.rgb.length; i += 8192) s += String.fromCharCode.apply(null, c.rgb.subarray(i, i + 8192));
      return { b64: btoa(s), renderMs: c.renderMs, loaded: c.loaded, renders: c.renders, canvas: c.canvas };
    },
    // run the browser model on two base64 RGB frames (q = live, b = breadcrumb)
    matchB64: async (q64, b64) => {
      const dec = (s) => Uint8Array.from(atob(s), (ch) => ch.charCodeAt(0));
      const q = await Model.encode(dec(q64)), b = await Model.encode(dec(b64));
      const r = await Model.match(q.desc, b.desc);
      return { ...r, encMs: q.ms };
    },
    bench: async (n = 50) => {
      const rgb = new Uint8Array(224 * 224 * 3).map((_, i) => (i * 37) & 255);
      const a = await Model.encode(rgb), b = await Model.encode(rgb.reverse());
      const te = [], tm = [];
      for (let i = 0; i < n; i++) { te.push((await Model.encode(rgb)).ms); tm.push((await Model.match(a.desc, b.desc)).ms); }
      const med = (x) => x.sort((p, q) => p - q)[Math.floor(x.length / 2)];
      return { ep: Model.ep, enc_ms: med(te), match_ms: med(tm), enc_min: Math.min(...te), match_min: Math.min(...tm), threads: ort.env.wasm.numThreads, webgpu: !!navigator.gpu, crossOriginIsolated: self.crossOriginIsolated };
    },
  };
  // camera mode for the main view (chase | top), used by screenshots
  window.__live.view = (mode) => { S.mainMode = mode; };
  const _uc = updateCam;
  updateCam = (m) => _uc(m, S.mainMode || 'chase');   // eslint-disable-line no-func-assign

  boot().catch((err) => { console.error('boot failed:', err && err.message); status('Failed to start: ' + (err && err.message), 'jam'); });
})();
