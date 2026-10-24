// Live breadcrumb RTH over Google Photorealistic 3D Tiles: the offset model runs in the browser (onnxruntime-web) on frames
// rendered by a second, independent CesiumWidget (its own WebGL context + tileset, so the main view never flickers or
// loses LOD). The second widget IS the drone's body-fixed downward camera (90° FOV, tilted, yawed); every frame is
// rectified here in JS with the estimated attitude to level / north-up / the breadcrumb's footprint, exactly like the
// training data (hf_world.rectify). Closed loop: nav.js (truth sim + 2D KF + drift + guidance).
// URL: ?mission=… &ep=wasm|webgpu &sse=16 (nadir tileset maximumScreenSpaceError) &main=0 (no main view) &speed=1
//      &autoplay=ours|stock &wait=4000 (max ms to wait for nadir tiles) &present=1 &ideal=1 (level camera, no attitude
//      noise, no altitude change) &wind=<m/s>
// Keys: D debug rows · T true position on the breadcrumb · 1 stock · 2 ours · Space fly · C chase/top
(() => {
  'use strict';
  const params = new URLSearchParams(location.search);
  const TOKEN_KEY = 'homeward.cesiumIonToken';
  const $ = (id) => document.getElementById(id);
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  const ORT_DIST = 'https://cdn.jsdelivr.net/npm/onnxruntime-web@1.22.0/dist/';
  const BLUE = '#315c91', GREEN = '#2e7d4f', GREEN_MAP = '#34c759', RED = '#b3261e', TRUTH = '#f29900';
  const S = { mission: null, running: false, speed: parseFloat(params.get('speed') || '1'), log: [], vision: params.get('autoplay') !== 'stock',
    present: params.get('present') === '1', debug: params.get('present') !== '1', showTruth: false, wind: params.has('wind') ? parseFloat(params.get('wind')) : null,
    realism: params.get('ideal') !== '1', mainMode: 'chase' };
  const SPEEDS = [1, 2, 4, 8, Infinity];
  document.body.classList.toggle('present', S.present);

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
  let FR, M0, ground, drift, thr, missions;
  function nedToCart(n, e, d, out) {
    return Cesium.Matrix4.multiplyByPoint(M0, new Cesium.Cartesian3(e, n, -d + FR.ground0_enu_z), out || new Cesium.Cartesian3());
  }
  function nedVec(v) {   // NED direction -> ECEF direction
    return Cesium.Cartesian3.normalize(Cesium.Matrix4.multiplyByPointAsVector(M0, new Cesium.Cartesian3(v[1], v[0], -v[2]), new Cesium.Cartesian3()), new Cesium.Cartesian3());
  }
  const groundCart = (n, e, up = 0.5) => nedToCart(n, e, -(ground.at(n, e) + up));

  // ------------------------------------------------------------------ camera (second widget) + rectifier
  class DroneCam {
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
    // raw body-camera frame at local NED (n, e, d); Rcam = camera->NED (x right, y down, z forward); square FOV
    async raw(n, e, d, Rcam, fovDeg, maxWait) {
      const t0 = performance.now(), cam = this.w.scene.camera, col = (j) => [Rcam[0][j], Rcam[1][j], Rcam[2][j]];
      cam.frustum = new Cesium.PerspectiveFrustum({ fov: Cesium.Math.toRadians(fovDeg), aspectRatio: 1, near: 1, far: 20000 });
      cam.setView({ destination: nedToCart(n, e, d), orientation: { direction: nedVec(col(2)), up: nedVec(col(1).map((x) => -x)) } });
      // tilesLoaded can flip true for a frame while the traversal is still refining after a camera jump: require it on
      // 4 consecutive renders
      let k = 0, loaded = false, streak = 0;
      for (;;) {
        this.w.render(); k++;
        streak = this.ts.tilesLoaded ? streak + 1 : 0;
        if (k >= 3 && streak >= 4) { loaded = true; break; }
        if (performance.now() - t0 > maxWait) break;
        await sleep(k < 3 ? 0 : 15);
      }
      this.w.render();
      this.ctx.drawImage(this.w.canvas, 0, 0, 224, 224);
      return { image: this.ctx.getImageData(0, 0, 224, 224), renderMs: performance.now() - t0, loaded, renders: k };
    }
    async capture(req, maxWait = 4000) {
      const r = await this.raw(req.n, req.e, req.d, req.Rcam, req.rawFov, maxWait);
      const t0 = performance.now();
      const rect = rectify(r.image, req.Rest, req.Rtgt, req.rawFov, req.fovOut);
      return Object.assign(rect, { raw: r.image, renderMs: r.renderMs, rectMs: performance.now() - t0, loaded: r.loaded, renders: r.renders });
    }
  }
  // Pure-rotation homography + scale (hf_world.rectify): output pixel ray in the target (level, north-up) camera ->
  // NED -> raw camera via the ESTIMATED camera attitude; bilinear; pixels outside the raw frame are invalid (0).
  function rectify(raw, Rest, Rtgt, rawFov, fovOut) {
    const N = 224, c = N / 2, fo = c / Math.tan(fovOut * Math.PI / 360), fr = c / Math.tan(rawFov * Math.PI / 360);
    const M = Nav.m3([[Rest[0][0], Rest[1][0], Rest[2][0]], [Rest[0][1], Rest[1][1], Rest[2][1]], [Rest[0][2], Rest[1][2], Rest[2][2]]], Rtgt);   // Rest^T Rtgt
    const src = raw.data, rgb = new Uint8Array(N * N * 3), valid = new Uint8Array(N * N), disp = new ImageData(N, N), dd = disp.data;
    for (let v = 0; v < N; v++) {
      const y = (v + 0.5 - c) / fo;
      for (let u = 0; u < N; u++) {
        const x = (u + 0.5 - c) / fo;
        const X = M[0][0] * x + M[0][1] * y + M[0][2], Y = M[1][0] * x + M[1][1] * y + M[1][2], Z = M[2][0] * x + M[2][1] * y + M[2][2];
        const o = v * N + u, j = o * 3, q = o * 4;
        dd[q + 3] = 255;
        if (Z <= 1e-6) continue;
        const px = fr * X / Z + c - 0.5, py = fr * Y / Z + c - 0.5;
        if (!(px >= 0 && py >= 0 && px <= N - 1 && py <= N - 1)) continue;
        const x0 = Math.min(Math.floor(px), N - 2), y0 = Math.min(Math.floor(py), N - 2), ax = px - x0, ay = py - y0;
        const i00 = (y0 * N + x0) * 4, i01 = i00 + 4, i10 = i00 + N * 4, i11 = i10 + 4;
        for (let ch = 0; ch < 3; ch++) {
          const val = (src[i00 + ch] * (1 - ax) + src[i01 + ch] * ax) * (1 - ay) + (src[i10 + ch] * (1 - ax) + src[i11 + ch] * ax) * ay;
          rgb[j + ch] = val + 0.5; dd[q + ch] = val + 0.5;
        }
        valid[o] = 1;
      }
    }
    const mask14 = new Float32Array(196);
    for (let o = 0; o < N * N; o++) if (valid[o]) mask14[Math.floor(o / N / 16) * 14 + Math.floor((o % N) / 16)] += 1;
    for (let i = 0; i < 196; i++) mask14[i] = Math.round(mask14[i] / 256 * 255) / 255;
    // display copy: invalid border shown as a dim hatch so the mask is visible
    for (let o = 0; o < N * N; o++) if (!valid[o]) { const q = o * 4, h = ((o % N) + Math.floor(o / N)) % 8 < 4 ? 46 : 30; dd[q] = dd[q + 1] = dd[q + 2] = h; }
    return { rgb, mask14, valid, image: disp };
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
    async match(q, qm, b, bm) {
      const t0 = performance.now(), T = (a, s) => new ort.Tensor('float32', a, s);
      const o = await this.mat.run({ q: T(q, [1, 16, 14, 14]), qm: T(qm || this.ones, [1, 14, 14]), b: T(b, [1, 16, 14, 14]), bm: T(bm || this.ones, [1, 14, 14]) });
      const mu = o.offset_cells.data, c = o.cov_cells.data, sig = (x) => 1 / (1 + Math.exp(-x));
      return { mu: [mu[0], mu[1]], cov: [[c[0], c[1]], [c[2], c[3]]], pArr: sig(o.arrival_logit.data[0]), pNm: sig(o.nomatch_logit.data[0]),
        probs: o.shift_probs ? new Float32Array(o.shift_probs.data) : null, ms: performance.now() - t0 };
    },
  };

  // ------------------------------------------------------------------ main view
  let viewer = null, ents = [], fixEnts = [];
  const ICON_DRONE = 'data:image/svg+xml,' + encodeURIComponent('<svg xmlns="http://www.w3.org/2000/svg" viewBox="-50 -50 100 100"><g stroke="#202630" stroke-width="16" stroke-linecap="round"><path d="M-30-30L30 30M30-30L-30 30"/></g><g stroke="#fff" stroke-width="8" stroke-linecap="round"><path d="M-30-30L30 30M30-30L-30 30"/></g><g fill="#fff" stroke="#202630" stroke-width="5"><circle cx="-32" cy="-32" r="15"/><circle cx="32" cy="-32" r="15"/><circle cx="-32" cy="32" r="15"/><circle cx="32" cy="32" r="15"/><rect x="-13" y="-16" width="26" height="32" rx="7"/></g></svg>');
  const ICON_X = 'data:image/svg+xml,' + encodeURIComponent('<svg xmlns="http://www.w3.org/2000/svg" viewBox="-10 -10 20 20"><path d="M-6-6L6 6M6-6L-6 6" stroke="#ffffff" stroke-width="5" stroke-linecap="round"/><path d="M-6-6L6 6M6-6L-6 6" stroke="#80868b" stroke-width="2.6" stroke-linecap="round"/></svg>');
  async function initMain() {
    viewer = new Cesium.Viewer('main', {
      baseLayer: false, baseLayerPicker: false, geocoder: false, homeButton: false, sceneModePicker: false, navigationHelpButton: false,
      animation: false, timeline: false, fullscreenButton: false, infoBox: false, selectionIndicator: false, globe: false,
    });
    viewer.scene.skyAtmosphere.show = true;
    viewer.scene.fog.enabled = true;
    viewer.scene.postProcessStages.fxaa.enabled = true;
    const ts = await Cesium.Cesium3DTileset.fromIonAssetId(2275207, { maximumScreenSpaceError: 16, showCreditsOnScreen: true });
    viewer.scene.primitives.add(ts);
    S.mainTs = ts;
  }
  function buildScene(m) {
    if (!viewer) return;
    ents.forEach((e) => viewer.entities.remove(e)); ents = [];
    fixEnts.forEach((g) => g.forEach((e) => viewer.entities.remove(e))); fixEnts = [];
    const C = Cesium.Color, add = (e) => { const x = viewer.entities.add(e); ents.push(x); return x; }, cb = (f) => new Cesium.CallbackProperty(f, false);
    const est = m.vision ? C.fromCssColorString(GREEN_MAP) : C.fromCssColorString('#ff3b30');
    const path = (col, from = 0) => () => {
      const T = m.traj, pts = [], s = Math.max(1, Math.floor((T.length - from) / 1500));
      for (let i = from; i < T.length; i += s) pts.push(nedToCart(T[i][col], T[i][col + 1], T[i][8]));
      pts.push(col === 1 ? nedToCart(m.p[0], m.p[1], m.D) : nedToCart(m.kf.x[0], m.kf.x[1], m.D));
      return pts.length > 1 ? pts : [pts[0], pts[0]];
    };
    let jamIdx = -1, outCache = null;
    const jamAt = () => { if (m.tJam === null) return -1; if (jamIdx < 0) jamIdx = m.traj.length - 1; return jamIdx; };
    // outbound true path: thin dotted grey once the return starts (white while flying out); return true path: white
    add({ polyline: { positions: cb(() => { const j = jamAt(); if (j < 0) return path(1)(); if (!outCache) outCache = path(1)().slice(0, -1).slice(0, Math.max(2, Math.ceil(j / Math.max(1, Math.floor(m.traj.length / 1500))))); return outCache; }),
      width: cb(() => (m.tJam === null ? 3 : 2)), material: new Cesium.PolylineDashMaterialProperty({ color: cb(() => (m.tJam === null ? C.WHITE : C.fromCssColorString('#c5c8cc').withAlpha(0.8))), dashLength: cb(() => (m.tJam === null ? 1e6 : 10)) }) } });
    add({ polyline: { positions: cb(() => { const j = jamAt(); return j < 0 ? [] : path(1, j)(); }), width: 3, material: C.WHITE, depthFailMaterial: C.WHITE.withAlpha(0.4) } });
    add({ polyline: { positions: cb(() => { const j = jamAt(); return j < 0 ? [] : path(3, j)(); }), width: 3, material: est, depthFailMaterial: est.withAlpha(0.4) } });
    add({ position: groundCart(m.home[0], m.home[1], 1.5), point: { pixelSize: 14, color: C.WHITE, outlineColor: C.fromCssColorString('#202630'), outlineWidth: 3, disableDepthTestDistance: Infinity },
      label: { text: 'Home', font: '600 14px "Google Sans", sans-serif', fillColor: C.fromCssColorString('#202630'), showBackground: true, backgroundColor: C.WHITE, pixelOffset: new Cesium.Cartesian2(0, -24), disableDepthTestDistance: Infinity } });
    add({ position: cb(() => nedToCart(m.kf.x[0], m.kf.x[1], m.D)), point: { pixelSize: 12, color: est, outlineColor: C.WHITE, outlineWidth: 2, disableDepthTestDistance: Infinity, show: cb(() => m.tJam !== null) } });
    add({ polyline: { positions: cb(() => [nedToCart(m.p[0], m.p[1], m.D), groundCart(m.p[0], m.p[1], 0)]), width: 1.5, material: C.WHITE.withAlpha(0.6), depthFailMaterial: C.WHITE.withAlpha(0.25) } });
    add({ position: cb(() => nedToCart(m.p[0], m.p[1], m.D)), billboard: { image: ICON_DRONE, width: 38, height: 38, disableDepthTestDistance: Infinity } });
    // target breadcrumb (blue ring on the ground)
    add({ position: cb(() => (m.k !== null && m.bcs[m.k] ? groundCart(m.bcs[m.k].pe[0], m.bcs[m.k].pe[1], 1) : groundCart(m.p[0], m.p[1]))),
      point: { pixelSize: 11, color: C.fromCssColorString(BLUE).withAlpha(0.25), outlineColor: C.fromCssColorString(BLUE), outlineWidth: 2.5, disableDepthTestDistance: Infinity, show: cb(() => m.k !== null && m.vision) } });
  }
  function addBcEnt(m, bc) {
    if (!viewer) return;
    ents.push(viewer.entities.add({ position: groundCart(bc.pe[0], bc.pe[1], 1), point: { pixelSize: 5, color: Cesium.Color.WHITE.withAlpha(0.85), outlineColor: Cesium.Color.fromCssColorString('#202630').withAlpha(0.6), outlineWidth: 1, disableDepthTestDistance: Infinity } }));
  }
  function ellipseAxes(c) {   // 2x2 cov (N,E) -> semi axes (m) + Cesium rotation
    const a = c[0][0], b = c[0][1], d = c[1][1], tr2 = (a + d) / 2, det = Math.sqrt(Math.max(((a - d) / 2) ** 2 + b * b, 0));
    const l1 = tr2 + det, l2 = Math.max(tr2 - det, 1e-6), th = Math.atan2(l1 - a, b || 1e-12);   // major-axis angle from north towards east
    return { major: Math.sqrt(l1), minor: Math.sqrt(l2), rot: Math.abs(b) < 1e-9 ? (a >= d ? 0 : Math.PI / 2) : th };
  }
  function addFixEnt(m, fr) {   // last ~10 fixes on the map, fading
    if (!viewer || fr.fix === undefined) return;
    const C = Cesium.Color, b = m.bcs[fr.k], g = [];
    const fused = fr.status === 'fused', col = C.fromCssColorString(fused ? GREEN_MAP : '#80868b');
    if (fused) {
      // 1σ ellipse of the fix covariance, as geometry 1.5 m above our bare-earth grid (draping via classification needs the globe)
      const ax = ellipseAxes(fr.cov), pts = [];
      for (let i = 0; i <= 36; i++) {
        const t = (i / 36) * 2 * Math.PI, u = ax.major * Math.cos(t), w = ax.minor * Math.sin(t);
        const dn = u * Math.cos(ax.rot) - w * Math.sin(ax.rot), de = u * Math.sin(ax.rot) + w * Math.cos(ax.rot);
        pts.push(groundCart(fr.fix[0] + dn, fr.fix[1] + de, 1.5));
      }
      g.push(viewer.entities.add({ polygon: { hierarchy: new Cesium.PolygonHierarchy(pts), perPositionHeight: true, material: col.withAlpha(0.2), outline: false } }));
      g.push(viewer.entities.add({ polyline: { positions: pts, width: 1.5, material: col.withAlpha(0.9), depthFailMaterial: col.withAlpha(0.35) } }));
      g.push(viewer.entities.add({ position: groundCart(fr.fix[0], fr.fix[1], 1.5), point: { pixelSize: 6, color: col, outlineColor: C.WHITE, outlineWidth: 1, disableDepthTestDistance: Infinity } }));
      g.push(viewer.entities.add({ polyline: { positions: [groundCart(b.pe[0], b.pe[1], 1.5), groundCart(fr.fix[0], fr.fix[1], 1.5)], width: 1.5,
        material: C.fromCssColorString(BLUE).withAlpha(0.9), depthFailMaterial: C.fromCssColorString(BLUE).withAlpha(0.35) } }));
    } else {
      g.push(viewer.entities.add({ position: groundCart(fr.fix[0], fr.fix[1], 0.5), billboard: { image: ICON_X, width: 14, height: 14, disableDepthTestDistance: Infinity } }));
    }
    fixEnts.push(g);
    while (fixEnts.length > 10) fixEnts.shift().forEach((e) => viewer.entities.remove(e));
    fixEnts.forEach((grp, i) => {   // fade with age
      const a = 0.25 + 0.75 * (i + 1) / fixEnts.length, fusedG = !grp.some((e) => e.billboard);
      grp.forEach((e) => {
        if (e.polygon) e.polygon.material = C.fromCssColorString(GREEN_MAP).withAlpha(0.2 * a);
        if (e.point) e.point.color = C.fromCssColorString(fusedG ? GREEN_MAP : '#80868b').withAlpha(a);
        if (e.polyline) e.polyline.material = C.fromCssColorString(e.polyline.positions.getValue().length > 2 ? GREEN_MAP : BLUE).withAlpha(0.9 * a);
        if (e.billboard) e.billboard.color = C.WHITE.withAlpha(a);
      });
    });
  }
  let headingSm = null;
  function updateCam() {
    const m = S.mission;
    if (!viewer || !m) return;
    const tgt = nedToCart(m.p[0], m.p[1], m.D);
    if (S.mainMode === 'top') {
      const pts = m.route.concat([[m.p[0], m.p[1]], m.home]).map((q) => groundCart(q[0], q[1]));
      const bs = Cesium.BoundingSphere.fromPoints(pts);
      viewer.camera.lookAt(bs.center, new Cesium.HeadingPitchRange(0, -Cesium.Math.PI_OVER_TWO + 0.001, Math.max(700, bs.radius * 2.9)));
      return;
    }
    const v = m.v, h = Math.hypot(v[0], v[1]) > 1 ? Math.atan2(v[1], v[0]) : headingSm ?? 0;
    if (headingSm === null) headingSm = h;
    headingSm += Math.atan2(Math.sin(h - headingSm), Math.cos(h - headingSm)) * 0.05;
    // aim between the drone and the ground below it, so the fixes drawn on the ground stay in view
    const agl = Math.max(-m.D - ground.at(m.p[0], m.p[1]), 30);
    const aim = nedToCart(m.p[0], m.p[1], m.D + 0.6 * agl);
    viewer.camera.lookAt(aim, new Cesium.HeadingPitchRange(headingSm + Cesium.Math.toRadians(28), Cesium.Math.toRadians(-38), 260 + 1.4 * agl));
  }

  // ------------------------------------------------------------------ HUD + inset
  const fmt = (x, d = 1) => (x === null || x === undefined || !isFinite(x) ? '–' : x.toFixed(d));
  const fmtM = (m) => (m < 10 ? `${m.toFixed(1)} m` : m < 1000 ? `${Math.round(m)} m` : `${(m / 1000).toFixed(2)} km`);
  function hud(m) {
    if (!m) return;
    const err = Math.hypot(m.p[0] - m.kf.x[0], m.p[1] - m.kf.x[1]), dh = Math.hypot(m.p[0] - m.home[0], m.p[1] - m.home[1]);
    const fr = m.frames, fused = fr.filter((f) => f.status === 'fused').length, last = [...fr].reverse().find((f) => f.pNm !== undefined);
    const ours = m.vision;
    status(m.done ? (ours ? `Home · ${fmt(dh)} m from take-off` : `Thinks it is home · actually ${fmtM(dh)} away`) : m.phase === 'out' ? 'GPS OK' : 'GPS JAMMED',
      m.done ? (ours || dh < 10 ? 'home' : 'jam') : m.phase === 'out' ? '' : 'jam');
    $('r-err').textContent = fmtM(err); $('r-err').classList.toggle('bad', err > 20);
    $('r-home').textContent = fmtM(dh);
    $('r-mode').textContent = m.phase === 'out' ? 'GPS · laying breadcrumbs' : ours ? 'Vision fixes (live model)' : 'IMU only';
    // wind arrow (points where the wind blows TO, screen-up = north)
    $('wind-arrow').style.transform = `rotate(${m.windFrom + 180}deg)`;   // svg arrow points north at 0°
    $('wind-txt').textContent = `${fmt(m.windSpeed, 1)} m/s`;
    const rows = [['Sim time', `${fmt(m.t, 0)} s`], ['Breadcrumbs', `${m.bcs.length}${m.k !== null ? ` · target #${m.k}` : ''}`],
      ['Tilt / crab', `${fmt(m.tiltDeg(), 1)}° / ${fmt(m.crabDeg(), 1)}°`], ['AGL now', `${fmt(m.aglTrue(), 0)} m${m.tJam !== null ? ` (return ×${fmt(m.retAgl, 2)})` : ''}`]];
    if (ours && fr.length) {
      rows.push(['Fixes fused', `${fused}/${fr.length} (${fmt(100 * fused / fr.length, 0)}%)`]);
      if (last) rows.push(['Last fix', `${last.status} · err ${fmt(last.fixErr)} m · σ ${fmt(last.sigma)} m`], ['p_nomatch / p_arrival', `${fmt(last.pNm, 3)} / ${fmt(last.pArr, 3)}`]);
    }
    if (S.lastMs) rows.push(['Model (enc + match)', `${fmt(S.lastMs.enc)} + ${fmt(S.lastMs.mat)} ms (${Model.ep})`], ['Render / rectify', `${fmt(S.lastMs.render, 0)} / ${fmt(S.lastMs.rect, 0)} ms`]);
    $('stats').innerHTML = rows.map(([a, b]) => `<dt>${a}</dt><dd>${b}</dd>`).join('');
    if (last && ours) {
      $('bar-conf').style.width = `${100 * (1 - last.pNm)}%`; $('bar-arr').style.width = `${100 * last.pArr}%`;
      $('v-conf').textContent = fmt(1 - last.pNm, 2); $('v-arr').textContent = fmt(last.pArr, 2);
    }
    $('bars').hidden = !(ours && last && m.tJam !== null);
  }
  function warm(t) {   // inferno-like ramp: dark purple -> red -> orange -> pale yellow
    const st = [[40, 11, 84], [120, 28, 109], [188, 55, 84], [237, 105, 37], [251, 180, 26], [252, 255, 164]];
    const x = Math.min(0.9999, Math.max(0, t)) * (st.length - 1), i = Math.floor(x), f = x - i;
    return [0, 1, 2].map((c) => Math.round(st[i][c] * (1 - f) + st[i + 1][c] * f));
  }
  function drawRect(cap) { $('rectimg').getContext('2d').putImageData(cap.image, 0, 0); }
  function drawBc(m, fr) {   // breadcrumb image + model heatmap / soft-argmax / 1σ ellipse (+ truth marker)
    const b = m.bcs[fr ? fr.k : m.k ?? m.bcs.length - 1];
    if (!b || !b.img) return;
    const ctx = $('bcimg').getContext('2d');
    ctx.putImageData(b.img, 0, 0);
    $('bccap').textContent = `breadcrumb #${b.i}`;
    if (!fr || !fr.probs) return;
    const p = fr.probs, hm = S.hm || (S.hm = Object.assign(document.createElement('canvas'), { width: 15, height: 15 })), hc = hm.getContext('2d'), id = hc.createImageData(15, 15);
    const pmax = Math.max(...p);
    for (let k = 0; k < 225; k++) { const a = Math.min(1, Math.pow(p[k] / pmax, 0.3)); id.data.set([...warm(a), Math.round(255 * Math.min(1, 1.6 * a))], k * 4); }
    hc.putImageData(id, 0, 0);
    ctx.save();
    ctx.imageSmoothingEnabled = true;
    ctx.globalAlpha = 0.55;
    ctx.filter = 'blur(4px)';
    ctx.drawImage(hm, -8, -8, 240, 240);
    ctx.filter = 'none';   // shift (dy, dx) cell -> pixel 112 + 16·(dy, dx): the 15x15 grid spans [-8, 232]
    ctx.globalAlpha = 1;
    const cx = 112 + 16 * fr.mu[1], cy = 112 + 16 * fr.mu[0];
    // 1σ ellipse of the model covariance (cells -> px; canvas x = col, y = row)
    const c = fr.covCells, sxx = c[1][1] * 256, syy = c[0][0] * 256, sxy = c[0][1] * 256;
    const tr2 = (sxx + syy) / 2, det = Math.sqrt(Math.max(((sxx - syy) / 2) ** 2 + sxy * sxy, 0));
    const l1 = tr2 + det, l2 = Math.max(tr2 - det, 0.25), ang = Math.atan2(l1 - sxx, sxy || 1e-9);
    const col = fr.status === 'fused' ? '#ffffff' : '#c5c8cc';
    ctx.strokeStyle = col; ctx.lineWidth = 2;
    ctx.shadowColor = 'rgba(0,0,0,.6)'; ctx.shadowBlur = 2;
    ctx.beginPath(); ctx.ellipse(cx, cy, Math.max(2, Math.sqrt(l1)), Math.max(2, Math.sqrt(l2)), Math.abs(sxy) < 1e-9 ? (sxx >= syy ? 0 : Math.PI / 2) : ang, 0, 2 * Math.PI); ctx.stroke();
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(cx - 9, cy); ctx.lineTo(cx - 3, cy); ctx.moveTo(cx + 3, cy); ctx.lineTo(cx + 9, cy); ctx.moveTo(cx, cy - 9); ctx.lineTo(cx, cy - 3); ctx.moveTo(cx, cy + 3); ctx.lineTo(cx, cy + 9); ctx.stroke();
    if (S.showTruth && fr.trueCells) {
      const tx = 112 + 16 * fr.trueCells[1], ty = 112 + 16 * fr.trueCells[0];
      ctx.strokeStyle = TRUTH; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(tx, ty, 5, 0, 2 * Math.PI); ctx.stroke();
    }
    ctx.restore();
  }

  // ------------------------------------------------------------------ closed loop driver
  async function fly(id, vision, opts = {}) {
    const spec = missions.find((s) => s.id === id) || missions[0];
    $('mission').value = spec.id;
    S.vision = vision; segOn();
    const wind = opts.wind ?? (S.wind !== null ? { speed: S.wind, fromDeg: spec.wind_from_deg } : undefined);
    const m = new Nav.Mission(spec, { vision, drift, ground, thr, camHz: opts.camHz, realism: opts.realism ?? S.realism, wind, retAgl: opts.retAgl,
      multiBc: opts.multiBc ?? (params.get('multibc') === '0' ? 1 : 3) });
    S.mission = m; S.running = true; headingSm = null; S.abort = null;
    buildScene(m);
    $('bcimg').getContext('2d').clearRect(0, 0, 224, 224); $('rectimg').getContext('2d').clearRect(0, 0, 224, 224);
    const speed = () => opts.speed ?? S.speed;
    let wall0 = performance.now(), sim0 = 0;
    const maxWait = parseFloat(params.get('wait') || opts.wait || '4000');
    // pre-flight: the camera streams the tiles around home before take-off (otherwise the first breadcrumbs are taken from
    // the coarse root tiles while the tileset is still loading)
    if (vision) {
      status('Pre-flight: loading camera tiles…');
      const q0 = m.camReq('bc', {}, 90);
      await S.cam.raw(q0.n, q0.e, q0.d, q0.Rcam, q0.rawFov, 60000);
    }
    const showCam = !vision && viewer;   // stock: keep the camera strip alive at ~2 Hz for the visual
    let nextShow = 0;
    for (;;) {
      if (S.abort === m) break;
      const r = m.advance(0.1);
      if (r.type === 'done') break;
      if (r.type === 'bc' || r.type === 'cam') {
        const cap = await S.cam.capture(r, r.type === 'bc' ? Math.max(maxWait, 30000) : maxWait);   // breadcrumbs must be sharp
        drawRect(cap);
        const enc = await Model.encode(cap.rgb);
        if (r.type === 'bc') {
          m.provide({ desc: enc.desc, mask14: cap.mask14, img: cap.image, renderMs: cap.renderMs, renders: cap.renders, loaded: cap.loaded });
          addBcEnt(m, m.bcs[r.i]);
          drawBc(m, null);
          S.lastMs = { enc: enc.ms, mat: S.lastMs?.mat ?? 0, render: cap.renderMs, rect: cap.rectMs };
        } else {
          const b = m.bcs[r.k];
          const res = await Model.match(enc.desc, cap.mask14, b.desc, b.mask14);
          let matMs = res.ms;
          res.extra = [];
          for (const j of r.extra || []) {   // home phase: same rectified frame against the next breadcrumbs out
            const x = await Model.match(enc.desc, cap.mask14, m.bcs[j].desc, m.bcs[j].mask14);
            matMs += x.ms; delete x.probs; x.j = j; res.extra.push(x);
          }
          res.ms = enc.ms + matMs;
          m.provide(res);
          const f = m.frames[m.frames.length - 1];
          Object.assign(f, { renderMs: cap.renderMs, rectMs: cap.rectMs, loaded: cap.loaded, encMs: enc.ms });
          S.lastMs = { enc: enc.ms, mat: matMs, render: cap.renderMs, rect: cap.rectMs };
          drawBc(m, f); addFixEnt(m, f); S.lastFrame = f;
        }
        if (!cap.loaded) S.log.push(`frame at t=${m.t.toFixed(1)} rendered before all tiles loaded`);
        wall0 = performance.now(); sim0 = m.t;   // do not race to catch up after a slow capture
        hud(m);
      } else {
        if (showCam && m.t >= nextShow) { nextShow = m.t + 0.5; const q = m.camReq('cam', {}, 2 * Math.atan(Nav.bucketFor(m.aglTrue()) / 2 / m.aglTrue()) * 180 / Math.PI); const cap = await S.cam.capture(q, 300); drawRect(cap); wall0 = performance.now(); sim0 = m.t; }
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
    Object.assign(sum, { render_ms_median: med(fr.map((f) => f.renderMs)), rect_ms_median: med(fr.map((f) => f.rectMs)), enc_ms_median: med(fr.map((f) => f.encMs)),
      match_ms_median: med(fr.map((f) => f.ms - f.encMs)), frames_not_fully_loaded: fr.filter((f) => !f.loaded).length,
      bcs_not_fully_loaded: m.bcs.filter((b) => b.loaded === false).length, bc_render_ms_max: Math.max(0, ...m.bcs.map((b) => b.renderMs || 0)), ep: Model.ep, nadir_sse: S.cam.sse });
    S.lastSummary = sum;
    return sum;
  }

  function segOn() { document.querySelectorAll('[data-scn]').forEach((b) => b.classList.toggle('on', (b.dataset.scn === 'ours') === S.vision)); }

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
    M0 = Cesium.Transforms.eastNorthUpToFixedFrame(Cesium.Cartesian3.fromDegrees(FR.lon0, FR.lat0, FR.ground_h_ellipsoid));
    const sel = $('mission');
    missions.forEach((s) => sel.insertAdjacentHTML('beforeend', `<option value="${s.id}">${s.id.replace('mission_', 'M').replace('cx_', 'CX ')} · ${(s.legs.reduce((a, l) => a + l[1], 0) * (s.jam_frac ?? 1) / 1000).toFixed(1)} km${s.shape ? ' ' + s.shape : ''}</option>`));
    if (params.get('mission')) sel.value = params.get('mission');
    status('Loading model + tiles…');
    await Model.init(params.get('ep') || 'wasm');
    S.cam = new DroneCam('nadir', parseFloat(params.get('sse') || '16'));
    await S.cam.init();
    if (params.get('main') !== '0') await initMain();
    const go = (vision) => { if (S.running) S.abort = S.mission; setTimeout(() => fly(sel.value, vision), S.running ? 300 : 0); };
    $('b-fly').onclick = () => go(S.vision);
    document.querySelectorAll('[data-scn]').forEach((b) => (b.onclick = () => { S.vision = b.dataset.scn === 'ours'; segOn(); go(S.vision); }));
    $('b-speed').onclick = () => { const i = (SPEEDS.indexOf(S.speed) + 1) % SPEEDS.length; S.speed = SPEEDS[i]; $('b-speed').textContent = isFinite(S.speed) ? `${S.speed}×` : 'max'; };
    $('b-speed').textContent = isFinite(S.speed) ? `${S.speed}×` : 'max';
    const ws = $('wind'), wl = $('wind-val');
    ws.oninput = () => { S.wind = parseFloat(ws.value); wl.textContent = `${S.wind} m/s (next flight)`; };
    if (S.wind !== null) { ws.value = S.wind; wl.textContent = `${S.wind} m/s`; }
    $('debug').hidden = !S.debug;
    segOn();
    document.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.metaKey || e.ctrlKey || e.altKey) return;
      const k = e.key.toLowerCase();
      if (k === 'd') { S.debug = !S.debug; $('debug').hidden = !S.debug; }
      else if (k === 't') { S.showTruth = !S.showTruth; if (S.mission) drawBc(S.mission, S.lastFrame); }
      else if (k === '1') { S.vision = false; segOn(); go(false); }
      else if (k === '2') { S.vision = true; segOn(); go(true); }
      else if (k === ' ') go(S.vision);
      else if (k === 'c') S.mainMode = S.mainMode === 'top' ? 'chase' : 'top';
      else return;
      e.preventDefault();
    });
    if (viewer) viewer.scene.preRender.addEventListener(updateCam);
    status('Ready');
    window.__liveReady = true;
    if (params.get('autoplay')) go(params.get('autoplay') !== 'stock');
  }

  window.__live = {
    fly: (id, vision, opts) => fly(id, vision, opts),
    state: () => { const m = S.mission; return m ? { t: m.t, phase: m.phase, k: m.k, bcs: m.bcs.length, frames: m.frames.length, err: Math.hypot(m.p[0] - m.kf.x[0], m.p[1] - m.kf.x[1]), home: Math.hypot(m.p[0] - m.home[0], m.p[1] - m.home[1]), done: m.done, running: S.running } : null; },
    summary: () => S.lastSummary,
    frames: () => (S.mission ? S.mission.frames.map((f) => { const { probs, ...rest } = f; return rest; }) : []),
    traj: () => (S.mission ? S.mission.traj : []),
    log: () => S.log,
    view: (mode) => { S.mainMode = mode; },
    bcs: () => (S.mission ? S.mission.bcs.map((b) => ({ i: b.i, t: b.t, pe: b.pe, pt: b.pt, agl: b.agl, agl_s: b.agl_s, bucket: b.bucket, renderMs: b.renderMs, renders: b.renders, loaded: b.loaded })) : []),
    bcImage: (i) => { const b = S.mission.bcs[i]; let s = ''; const d = b.img.data; for (let j = 0; j < d.length; j += 8192) s += String.fromCharCode.apply(null, d.subarray(j, j + 8192)); return btoa(s); },
    truth: (on) => { S.showTruth = !!on; if (S.mission) drawBc(S.mission, S.lastFrame); },
    debug: (on) => { S.debug = !!on; $('debug').hidden = !S.debug; },
    // domain check: capture a (possibly tilted) raw frame and rectify it -> base64 RGB + mask14
    capture: async (req, wait = 8000, sse = null) => {
      if (sse !== null) S.cam.ts.maximumScreenSpaceError = sse;
      const c = await S.cam.capture(Object.assign({ Rtgt: Nav.TGT }, req), wait);
      let s = ''; for (let i = 0; i < c.rgb.length; i += 8192) s += String.fromCharCode.apply(null, c.rgb.subarray(i, i + 8192));
      return { b64: btoa(s), mask14: Array.from(c.mask14), renderMs: c.renderMs, loaded: c.loaded, renders: c.renders };
    },
    matchB64: async (q64, b64) => {
      const dec = (s) => Uint8Array.from(atob(s), (ch) => ch.charCodeAt(0));
      const q = await Model.encode(dec(q64)), b = await Model.encode(dec(b64));
      const r = await Model.match(q.desc, null, b.desc, null);
      delete r.probs;
      return { ...r, encMs: q.ms };
    },
    bench: async (n = 50) => {
      const rgb = new Uint8Array(224 * 224 * 3).map((_, i) => (i * 37) & 255);
      const a = await Model.encode(rgb), b = await Model.encode(rgb.slice().reverse());
      const te = [], tm = [];
      for (let i = 0; i < n; i++) { te.push((await Model.encode(rgb)).ms); tm.push((await Model.match(a.desc, null, b.desc, null)).ms); }
      const med = (x) => x.sort((p, q) => p - q)[Math.floor(x.length / 2)];
      return { ep: Model.ep, enc_ms: med(te), match_ms: med(tm), enc_min: Math.min(...te), match_min: Math.min(...tm), threads: ort.env.wasm.numThreads, webgpu: !!navigator.gpu, crossOriginIsolated: self.crossOriginIsolated };
    },
  };

  boot().catch((err) => { console.error('boot failed:', err && err.message); status('Failed to start: ' + (err && err.message), 'jam'); });
})();
