// Live 3D replay of recorded missions over Google Photorealistic 3D Tiles (Cesium ion asset 2275207).
// Data: data/*.json, exported by tools/export_trajectories.py (lat/lon/ellipsoidal height, 2 Hz).
// Keys: Space play/pause · 1 stock · 2 ours · C chase/orbit · T top-down · S speed · R restart · , . scrub ±10 s
// Embedded (?embed=1): deck navigation keys are forwarded to the parent; the deck can post
//   {type: 'demo-scenario', scenario: 'stock' | 'ours'} to switch scenario.
(() => {
  'use strict';
  const params = new URLSearchParams(location.search);
  const embedded = params.has('embed');
  if (embedded) document.body.classList.add('embed');
  const solo = params.has('solo');                 // ?solo=1: one scenario, no toggle (deck slide 2)
  if (solo) document.body.classList.add('solo');
  const CUT_M = Number(params.get('cut') ?? 1000);
  const TOKEN_KEY = 'homeward.cesiumIonToken';
  // ours = model v3 (Magdeburg held out of training). Fallback videos are pre-rendered (rth video: model v1).
  const SCN = {
    // stock = PX4 EKF2 replay of complex zig-zag cx_001 (2.7 km, jammed at 0.85 of the route)
    stock: { file: 'data/stock_cx.json', video: '../media/jamming_failure.mp4', poster: '../media/jamming_failure_poster.jpg' },
    ours: { file: 'data/ours.json', video: '../media/rth_mission012.mp4', poster: '../media/rth_mission012_poster.jpg' },
  };
  const SPEEDS = ['auto', 1, 5, 10, 20, 40];
  const $ = (id) => document.getElementById(id);

  let scenario = SCN[params.get('scenario')] ? params.get('scenario') : 'stock';
  let tileset = null;
  let viewer = null, D = null, simT = 0, playing = false, speedIdx = 0, cam = 'chase', lastFrame = null;
  let headingSm = null, orbitHeading = 0;
  const cache = {};

  // ---------------------------------------------------------------- token (never logged)
  function readToken() {
    const h = new URLSearchParams(location.hash.slice(1));
    if (h.get('token')) {
      try { localStorage.setItem(TOKEN_KEY, h.get('token')); } catch (e) { /* storage blocked */ }
      history.replaceState(null, '', location.pathname + location.search);   // keep it out of the URL bar
      return h.get('token');
    }
    if (typeof window.CESIUM_ION_TOKEN === 'string' && window.CESIUM_ION_TOKEN.trim()) return window.CESIUM_ION_TOKEN.trim();
    try { return localStorage.getItem(TOKEN_KEY) || ''; } catch (e) { return ''; }
  }

  // Turn a Cesium RequestErrorEvent into the reason ion gave (status + message), with the usual fixes for each case.
  function ionError(err) {
    let msg = '';
    try { msg = JSON.parse(err.response).message || ''; } catch (e) { msg = (err && (err.message || err.response)) || ''; }
    const code = err && err.statusCode;
    const hint = code === 401 ? 'the token is wrong or expired'
      : code === 403 ? `the token isn't allowed here: add ${location.origin} to its Allowed URLs, and make sure asset 2275207 is in the token's assets`
      : code === 404 ? 'asset 2275207 is not in your ion account: add "Google Photorealistic 3D Tiles" from the ion Asset Depot'
      : code === 429 ? 'rate-limited, wait a minute and reload'
      : 'check the token';
    return `${code ? 'HTTP ' + code : 'error'}${msg ? ` (${String(msg).slice(0, 140)})` : ''}, ${hint}`;
  }

  // ---------------------------------------------------------------- fallback video
  function fallback(why) {
    $('hud').hidden = true;
    $('loading').hidden = true;
    $('fallback').hidden = false;
    $('fb-why').textContent = why;
    const v = $('fb-video');
    v.poster = SCN[scenario].poster;
    v.src = SCN[scenario].video;
    v.play().catch(() => {});
  }
  // The deck preloads this page before anyone has entered a token. When one is saved anywhere on this origin
  // (another tab, live/, #token=…), restart if we're still showing the fallback video.
  addEventListener('storage', (e) => { if (e.key === TOKEN_KEY && e.newValue && !window.__demoReady) location.reload(); });
  $('fb-token').addEventListener('submit', (e) => {
    e.preventDefault();
    const t = $('fb-input').value.trim();
    if (!t) return;
    try { localStorage.setItem(TOKEN_KEY, t); } catch (err) { /* ignore */ }
    location.reload();
  });

  // ---------------------------------------------------------------- data
  async function load(name) {
    if (cache[name]) return cache[name];
    const r = await fetch(SCN[name].file);
    if (!r.ok) throw new Error(`data ${SCN[name].file}: HTTP ${r.status}`);
    const d = await r.json();
    // End the replay once the estimate is CUT_M off the true position (stock drifts ~1.7 km by "home"; 1 km makes the
    // point and saves time). ?cut=0 plays the full log.
    const cutAt = CUT_M > 0 ? d.t.findIndex((_, i) => horiz(d.truth[i], d.est[i]) >= CUT_M) : -1;
    if (cutAt > 0) {
      for (const k of ['t', 'truth', 'est']) d[k] = d[k].slice(0, cutAt + 1);
      d.events = d.events.filter((e) => e.t <= d.t[cutAt]).concat({ t: d.t[cutAt], key: 'cut', label: `${CUT_M / 1000} km off` });
    }
    const C = Cesium.Cartesian3;
    d.P = d.truth.map((p) => C.fromDegrees(p[1], p[0], p[2]));
    d.E = d.est.map((p) => C.fromDegrees(p[1], p[0], p[2] + 4));   // +4 m: drawn above the white path it may retrace (no z-fight)
    d.H = C.fromDegrees(d.home[1], d.home[0], d.home[2] + 1.5);
    d.J = C.fromDegrees(d.jam[1], d.jam[0], d.jam[2] + 1.5);
    d.T = d.t[d.t.length - 1];
    d.ev = Object.fromEntries(d.events.map((e) => [e.key, e.t]));
    d.ground = d.home[2];
    const lats = d.truth.concat(d.est, [d.home]).map((p) => p[0]), lons = d.truth.concat(d.est, [d.home]).map((p) => p[1]);
    const [s0, n0, w0, e0] = [Math.min(...lats), Math.max(...lats), Math.min(...lons), Math.max(...lons)];
    const dl = Math.max(n0 - s0, (e0 - w0) * 0.62), mid = (s0 + n0) / 2, mlon = (w0 + e0) / 2;
    // padded overview rectangle; extra room at the bottom for the control bar
    d.overview = Cesium.Rectangle.fromDegrees(mlon - dl * 1.1, mid - dl * 0.85, mlon + dl * 1.1, mid + dl * 0.65);
    cache[name] = d;
    return d;
  }

  function idxAt(t) {   // last sample index with d.t[i] <= t (uniform 2 Hz, but don't assume it)
    const ts = D.t;
    let lo = 0, hi = ts.length - 1;
    if (t <= ts[0]) return [0, 0];
    if (t >= ts[hi]) return [hi, 0];
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (ts[m] <= t) lo = m; else hi = m; }
    return [lo, (t - ts[lo]) / (ts[lo + 1] - ts[lo])];
  }
  const scratch = { a: new Cesium.Cartesian3(), b: new Cesium.Cartesian3() };
  function at(arr, t, out) {
    const [i, f] = idxAt(t);
    if (f === 0 || i >= arr.length - 1) return Cesium.Cartesian3.clone(arr[i], out);
    return Cesium.Cartesian3.lerp(arr[i], arr[i + 1], f, out);
  }
  function trail(arr, from = 0) {
    const [i] = idxAt(simT);
    const pts = arr.slice(Math.min(from, i), i + 1);
    pts.push(at(arr, simT, new Cesium.Cartesian3()));
    return pts.length >= 2 ? pts : [arr[0], arr[0]];
  }
  function geo(arr, t) {   // lat/lon/h of an interpolated sample, for readouts
    const [i, f] = idxAt(t);
    const a = arr[i], b = arr[Math.min(i + 1, arr.length - 1)];
    return a.map((v, k) => v + (b[k] - v) * f);
  }
  function horiz(p, q) {   // metres between two lat/lon points (local flat earth is plenty at this scale)
    const R = 6371008.8, la = (p[0] * Math.PI) / 180;
    const dn = ((q[0] - p[0]) * Math.PI / 180) * R, de = ((q[1] - p[1]) * Math.PI / 180) * R * Math.cos(la);
    return Math.hypot(dn, de);
  }
  function headingAt(t) {
    const p = geo(D.truth, t), q = geo(D.truth, Math.min(t + 4, D.T));
    const la = (p[0] * Math.PI) / 180;
    const dn = q[0] - p[0], de = (q[1] - p[1]) * Math.cos(la);
    return Math.hypot(dn, de) < 1e-7 ? null : Math.atan2(de, dn);
  }

  // ---------------------------------------------------------------- scene
  const svg = (s) => 'data:image/svg+xml,' + encodeURIComponent(s);
  const ICON_DRONE = svg(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="-50 -50 100 100"><g stroke="#202630" stroke-width="16" stroke-linecap="round"><path d="M-30-30L30 30M30-30L-30 30"/></g><g stroke="#fff" stroke-width="8" stroke-linecap="round"><path d="M-30-30L30 30M30-30L-30 30"/></g><g fill="#fff" stroke="#202630" stroke-width="5"><circle cx="-32" cy="-32" r="15"/><circle cx="32" cy="-32" r="15"/><circle cx="-32" cy="32" r="15"/><circle cx="32" cy="32" r="15"/><rect x="-13" y="-16" width="26" height="32" rx="7"/></g></svg>`);
  const ICON_HOME = svg(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="-50 -50 100 100"><circle r="44" fill="#fff" stroke="#202630" stroke-width="7"/><text y="17" text-anchor="middle" font-family="Arial" font-weight="700" font-size="48" fill="#202630">H</text></svg>`);
  const NO_DEPTH = Number.POSITIVE_INFINITY;
  let ents = [];

  function label(text, opts = {}) {
    return {
      text, font: '600 15px "Google Sans", Arial, sans-serif', fillColor: Cesium.Color.fromCssColorString(opts.fg || '#202630'),
      showBackground: true, backgroundColor: Cesium.Color.fromCssColorString(opts.bg || '#ffffff').withAlpha(0.95),
      backgroundPadding: new Cesium.Cartesian2(9, 6), pixelOffset: new Cesium.Cartesian2(opts.dx ?? 0, opts.dy ?? -30),
      disableDepthTestDistance: NO_DEPTH, style: Cesium.LabelStyle.FILL, ...opts.extra,
    };
  }

  function buildScene() {
    ents.forEach((e) => viewer.entities.remove(e));
    ents = [];
    const add = (e) => { const x = viewer.entities.add(e); ents.push(x); return x; };
    const C = Cesium.Color;
    const estCol = D.kind === 'stock' ? C.fromCssColorString('#ff3b30') : C.fromCssColorString('#34c759');
    const cb = (f) => new Cesium.CallbackProperty(f, false);
    const truthNow = () => at(D.P, simT, scratch.a);
    const estNow = () => at(D.E, simT, scratch.b);
    const jammed = () => simT >= D.ev.jam;
    const atHome = () => simT >= D.ev.home;

    // breadcrumbs (ours): small dots along the outbound path
    if (D.breadcrumbs) {
      D.breadcrumbs.forEach((p) => add({
        position: Cesium.Cartesian3.fromDegrees(p[1], p[0], p[2]),
        point: { pixelSize: 4, color: C.WHITE.withAlpha(0.75), outlineColor: C.fromCssColorString('#202630').withAlpha(0.5), outlineWidth: 1 },
      }));
    }
    add({ polyline: { positions: cb(() => trail(D.P)), width: 3, material: C.WHITE, depthFailMaterial: C.WHITE.withAlpha(0.45) } });
    // solid depth-fail material: the stock estimate retraces the white outbound line and must win the z-fight
    add({ polyline: { positions: cb(() => trail(D.E, idxAt(D.ev.jam)[0])), width: 4, material: estCol, depthFailMaterial: estCol, show: cb(jammed) } });   // from the jam on
    // drop line: drone -> ground (the Börde is flat to within ~10 m, so the home ground height is used)
    add({
      polyline: {
        positions: cb(() => { const p = truthNow(); const g = Cesium.Cartographic.fromCartesian(p); return [Cesium.Cartesian3.clone(p), Cesium.Cartesian3.fromRadians(g.longitude, g.latitude, D.ground)]; }),
        width: 1.5, material: C.WHITE.withAlpha(0.6), depthFailMaterial: C.WHITE.withAlpha(0.25),
      },
    });
    add({ position: D.H, billboard: { image: ICON_HOME, width: 30, height: 30, disableDepthTestDistance: NO_DEPTH }, label: label('Home', { dy: 30, extra: { show: cb(() => !(D.kind === 'stock' && atHome())) } }) });
    // home pole: ground marker up to flight altitude, so perspective views read correctly
    add({ polyline: { positions: [D.H, Cesium.Cartesian3.fromDegrees(D.home[1], D.home[0], D.home[2] + D.agl_m)], width: 1.5,
      material: new Cesium.PolylineDashMaterialProperty({ color: C.WHITE.withAlpha(0.8), dashLength: 10 }) } });
    add({ position: D.J, label: label('GPS jammed', { fg: '#ffffff', bg: '#b3261e', dy: -6, extra: { show: cb(jammed) } }) });
    add({
      position: cb(estNow),
      point: { pixelSize: 13, color: estCol, outlineColor: C.WHITE, outlineWidth: 2, disableDepthTestDistance: NO_DEPTH, show: cb(jammed) },
      label: label(atHome() ? 'Thinks it is home' : 'Thinks it is here', {
        fg: '#ffffff', bg: '#b3261e', dy: -28,
        extra: { text: cb(() => (atHome() ? 'Thinks it is home' : 'Thinks it is here')),
          show: cb(() => D.kind === 'stock' && jammed() && errNow() > 25) },
      }),
    });
    add({ position: cb(truthNow), billboard: { image: ICON_DRONE, width: 40, height: 40, disableDepthTestDistance: NO_DEPTH } });
    if (D.kind === 'stock') {
      add({
        polyline: { positions: cb(() => [Cesium.Cartesian3.clone(truthNow()), Cesium.Cartesian3.clone(estNow())]), width: 2,
          material: new Cesium.PolylineDashMaterialProperty({ color: C.fromCssColorString('#ffd60a'), dashLength: 14 }), show: cb(atHome) },
      });
      add({ position: cb(truthNow), label: label('', { dy: 38, extra: { text: cb(() => `Actually ${Math.round(errNow())} m away`), show: cb(atHome) } }) });
    }
  }

  function errNow() { return horiz(geo(D.truth, simT), geo(D.est, simT)); }

  // ---------------------------------------------------------------- HUD
  const fmtM = (m) => (m < 10 ? `${m.toFixed(1)} m` : m < 1000 ? `${Math.round(m)} m` : `${(m / 1000).toFixed(2)} km`);

  function setupHud() {
    const stock = D.kind === 'stock';
    document.querySelectorAll('[data-scn]').forEach((b) => b.classList.toggle('on', b.dataset.scn === scenario));
    $('lg-est').textContent = stock ? 'Stock EKF2 estimate' : 'Our estimate (vision + EKF2)';
    $('sw-est').classList.toggle('ok', !stock);
    const ticks = $('ticks');
    ticks.innerHTML = '';
    for (const [key, txt] of [['jam', 'jam'], ['home', 'home'], ['cut', `${CUT_M / 1000} km off`]]) {
      if (D.ev[key] === undefined) continue;
      const x = (D.ev[key] / D.T) * 100;
      ticks.insertAdjacentHTML('beforeend', `<i class="${key}" style="left:${x}%"></i><span style="left:${x}%">${txt}</span>`);
    }
  }

  function updateHud() {
    const stock = D.kind === 'stock';
    const jam = simT >= D.ev.jam, home = simT >= D.ev.home;
    const st = $('status');
    st.textContent = home ? (stock ? 'Thinks it is home' : 'Home') : jam ? 'GPS jammed' : 'GPS OK';
    st.className = 'status' + (home ? ' home' : jam ? ' jam' : '');
    $('ff').hidden = !fastForward();
    const e = errNow();
    $('r-err').textContent = fmtM(e);
    $('r-err').classList.toggle('bad', e > 20);
    $('r-home').textContent = fmtM(horiz(geo(D.truth, simT), D.home));
    const card = $('card');
    if (D.ev.cut !== undefined && simT >= D.ev.cut) {
      card.hidden = false;
      card.innerHTML = `<h3>${CUT_M / 1000} km off course, and still drifting.</h3>`;
    } else if (home && simT >= D.ev.home + 1) {
      card.hidden = false;
      card.innerHTML = stock
        ? `<h3>Thinks it's home. Actually ${Math.round(e)} m away.</h3>`
        : `<h3>Home, ${fmtM(horiz(geo(D.truth, simT), D.home))} from take-off.</h3>`;   // same number as the HUD
    } else card.hidden = true;
    if (!scrubbing) $('scrub').value = Math.round((simT / D.T) * 1000);
    $('b-play').textContent = playing ? 'Pause' : simT >= D.T ? 'Replay' : 'Play';
    $('b-speed').textContent = SPEEDS[speedIdx] === 'auto' ? 'Auto' : `${SPEEDS[speedIdx]}×`;
  }

  // talk pacing: the outbound (GPS OK) plays FAST_OUT x faster than everything after the jam, with a badge saying so
  const BASE_RATE = 12, FAST_OUT = 3;
  const fastForward = () => SPEEDS[speedIdx] === 'auto' && simT < D.ev.jam;
  function rate() {
    const s = SPEEDS[speedIdx];
    if (s !== 'auto') return s;
    return fastForward() ? BASE_RATE * FAST_OUT : BASE_RATE;
  }

  // ---------------------------------------------------------------- camera
  function setCam(mode) {
    cam = mode;
    document.querySelectorAll('[data-cam]').forEach((b) => b.classList.toggle('on', b.dataset.cam === cam));
    viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    if (cam === 'top') {
      viewer.camera.flyTo({ destination: D.overview, orientation: { heading: 0, pitch: -Cesium.Math.PI_OVER_TWO, roll: 0 }, duration: 1.0 });
    }
    if (cam === 'orbit') orbitHeading = headingSm ?? 0;
  }

  function updateCam(dt) {
    if (cam === 'top') return;
    const h = headingAt(simT);
    if (h !== null) {
      if (headingSm === null) headingSm = h;
      let d = h - headingSm;
      d = Math.atan2(Math.sin(d), Math.cos(d));
      headingSm += d * Math.min(1, dt * 1.5);
    }
    const target = at(D.P, simT, new Cesium.Cartesian3());
    if (cam === 'chase') {
      viewer.camera.lookAt(target, new Cesium.HeadingPitchRange((headingSm ?? 0) + Cesium.Math.toRadians(28), Cesium.Math.toRadians(-20), 200));
    } else {
      orbitHeading += dt * Cesium.Math.toRadians(8);
      viewer.camera.lookAt(target, new Cesium.HeadingPitchRange(orbitHeading, Cesium.Math.toRadians(-32), 650));
    }
  }

  // ---------------------------------------------------------------- loop + controls
  function tick(now) {
    const dt = lastFrame === null ? 0 : Math.min(0.1, (now - lastFrame) / 1000);
    lastFrame = now;
    if (D) {
      if (playing) {
        simT = Math.min(D.T, simT + dt * rate());
        if (simT >= D.T) playing = false;
      }
      updateCam(dt);
      updateHud();
    }
    requestAnimationFrame(tick);
  }

  async function setScenario(name, autoplay = false) {
    if (!SCN[name]) return;
    scenario = name;
    D = await load(name);
    simT = 0; playing = autoplay; headingSm = null;
    buildScene();
    setupHud();
    if (!$('fallback').hidden) { $('fb-video').poster = SCN[name].poster; $('fb-video').src = SCN[name].video; }
    setCam(cam);
  }

  let scrubbing = false;
  function bindControls() {
    $('b-play').onclick = togglePlay;
    $('b-speed').onclick = () => { speedIdx = (speedIdx + 1) % SPEEDS.length; };
    document.querySelectorAll('[data-scn]').forEach((b) => (b.onclick = () => setScenario(b.dataset.scn)));
    document.querySelectorAll('[data-cam]').forEach((b) => (b.onclick = () => setCam(b.dataset.cam)));
    const s = $('scrub');
    s.addEventListener('input', () => { scrubbing = true; simT = (s.value / 1000) * D.T; });
    s.addEventListener('change', () => { scrubbing = false; });
    // keep focus on the page so keys keep working after clicking a control
    document.querySelectorAll('button, input[type=range]').forEach((el) => el.addEventListener('mouseup', () => el.blur()));
  }
  function togglePlay() {
    if (!D) return;
    if (simT >= D.T) simT = 0;
    playing = !playing;
  }

  const DECK_KEYS = ['PageDown', 'PageUp', 'ArrowRight', 'ArrowLeft', 'n', 'N', 'p', 'P', 'f', 'F', 'Home', 'End'];
  document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' && e.target.type !== 'range') return;
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    if (embedded && DECK_KEYS.includes(e.key)) { parent.postMessage({ type: 'deck-key', key: e.key }, '*'); e.preventDefault(); return; }
    if (!D) return;
    const k = e.key;
    if (k === ' ') togglePlay();
    else if (k === '1' && !solo) setScenario('stock');
    else if (k === '2' && !solo) setScenario('ours');
    else if (k === 'c' || k === 'C') setCam(cam === 'chase' ? 'orbit' : 'chase');
    else if (k === 't' || k === 'T') setCam(cam === 'top' ? 'chase' : 'top');
    else if (k === 's' || k === 'S') speedIdx = (speedIdx + 1) % SPEEDS.length;
    else if (k === 'r' || k === 'R') { simT = 0; playing = true; headingSm = null; }
    else if (k === ',') simT = Math.max(0, simT - 10);
    else if (k === '.') simT = Math.min(D.T, simT + 10);
    else return;
    e.preventDefault();
  });
  window.addEventListener('message', (e) => {
    if (e.data?.type !== 'demo-scenario' || !SCN[e.data.scenario]) return;
    if (!window.__demoReady) { scenario = e.data.scenario; if (!$('fallback').hidden) { $('fb-video').poster = SCN[scenario].poster; $('fb-video').src = SCN[scenario].video; } return; }
    setScenario(e.data.scenario, !!e.data.autoplay);
  });

  // ---------------------------------------------------------------- boot
  async function boot() {
    if (window.__cesiumLoadFailed || typeof Cesium === 'undefined') return fallback('CesiumJS could not be loaded (offline?). Showing the pre-rendered video.');
    const token = readToken();
    if (!token) return fallback('Live 3D needs a Cesium ion token (config.local.js, #token=…, or paste one here). Showing the pre-rendered video.');
    Cesium.Ion.defaultAccessToken = token;
    try {
      viewer = new Cesium.Viewer('cesium', {
        baseLayer: false, baseLayerPicker: false, geocoder: false, homeButton: false, sceneModePicker: false,
        navigationHelpButton: false, animation: false, timeline: false, fullscreenButton: false, infoBox: false,
        selectionIndicator: false, globe: false, requestRenderMode: false,
      });
    } catch (err) {
      return fallback('WebGL is not available in this browser. Showing the pre-rendered video.');
    }
    viewer.shadows = false;
    if (viewer.scene.globe) viewer.scene.globe.show = false;
    viewer.scene.skyAtmosphere.show = true;
    viewer.scene.fog.enabled = true;
    viewer.scene.postProcessStages.fxaa.enabled = true;
    try {
      // No skipLevelOfDetail: with Google tiles it leaves holes / mixed-LOD slabs while streaming.
      tileset = await Cesium.Cesium3DTileset.fromIonAssetId(2275207, { maximumScreenSpaceError: 16, showCreditsOnScreen: true });
      viewer.scene.primitives.add(tileset);
    } catch (err) {
      viewer.destroy(); viewer = null;
      return fallback(`Cesium ion refused Google 3D Tiles: ${ionError(err)}. Showing the pre-rendered video.`);
    }
    $('hud').hidden = false;
    $('loading').hidden = true;
    bindControls();
    await setScenario(scenario, params.has('autoplay'));
    window.__demoReady = true;
    requestAnimationFrame(tick);
  }

  // test hook (used by the headless check; harmless otherwise)
  window.__demo = { fps: (n) => { if (viewer) viewer.targetFrameRate = n; }, seek: (t) => { simT = t; }, cam: (m) => setCam(m), scenario: (s) => setScenario(s), state: () => ({ scenario, simT, T: D && D.T, err: D && errNow(), cam, tilesLoaded: !!(tileset && tileset.tilesLoaded) }) };

  boot().catch((err) => { console.error(err && err.message); fallback('The live demo failed to start. Showing the pre-rendered video.'); });
})();
