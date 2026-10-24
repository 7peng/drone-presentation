import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { SCENARIO } from './config.js';
import { Sim, DT } from './sim.js';
import { terrainHeight } from './terrain.js';
import * as W from './world.js';

const params = new URLSearchParams(location.search);
const embedded = params.has('embed');
if (embedded) document.body.classList.add('embed');
const cfg = { ...SCENARIO, seed: Number(params.get('seed') || SCENARIO.seed) };

// ---------- Renderer / scene ----------
const canvas = document.getElementById('scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(55, 1, 0.5, 6000);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.enabled = false;

W.buildSky(scene);
W.buildTerrain(scene, cfg.seed);
W.buildPad(scene, cfg.ours.home, '#39d0ff');
W.buildPad(scene, cfg.baseline.home, '#ff9a3c');
W.buildRoute(scene, cfg);
const jammer = W.buildJammer(scene, cfg.jammer.position, cfg.jammer.radius);
const gps = W.buildGpsLinks(scene);

const vis = {};
for (const id of ['ours', 'baseline']) {
  const color = W.COLORS[id];
  const mesh = W.buildDrone(color);
  const { ghost, link } = W.buildGhost(color);
  const shadow = W.buildShadow();
  const trail = new W.Trail(scene, color);
  scene.add(mesh, ghost, link, shadow);
  vis[id] = { mesh, ghost, link, shadow, trail, marker: null, heading: 0 };
}

// ---------- State ----------
let sim, acc, speed = 1, showBaseline = true, lastEventIdx;
const SPEEDS = [1, 2, 4];
const CAMS = ['director', 'chase', 'orbit', 'top'];
let camMode = 'director';
const camTarget = new THREE.Vector3(), camPos = new THREE.Vector3();

function reset() {
  sim = new Sim(cfg);
  acc = 0; lastEventIdx = 0;
  for (const id of ['ours', 'baseline']) {
    const v = vis[id];
    v.trail.sync([]);
    if (v.marker) { scene.remove(v.marker); v.marker = null; }
  }
  $('start-hint').classList.remove('hidden');
  $('btn-play').textContent = '▶ Launch';
  toast('', null);
  syncVisuals(0);
  snapCamera();
}

// ---------- HUD helpers ----------
const $ = (id) => document.getElementById(id);
let toastTimer;
function toast(text, kind) {
  const el = $('toast');
  clearTimeout(toastTimer);
  if (!text) { el.className = ''; return; }
  el.textContent = text; el.className = `show ${kind || ''}`;
  toastTimer = setTimeout(() => (el.className = kind || ''), 2600);
}

function setCard(cardId, d) {
  const card = $(cardId);
  const q = (k) => card.querySelector(`[data-k="${k}"]`);
  const err = Math.hypot(d.err.x, d.err.z);
  const home = Math.hypot(d.pos.x - d.home[0], d.pos.z - d.home[1]);
  const spd = Math.hypot(d.vel.x, d.vel.z);
  const agl = d.pos.y - terrainHeight(d.home[0], d.home[1]);
  q('mode').textContent = d.modeLabel;
  q('nav').textContent = !sim.jammed ? 'GPS' : d.id === 'ours' ? 'LEARNED (IMU+VISION)' : 'IMU DEAD RECKONING';
  q('nav').className = !sim.jammed ? 'good' : d.id === 'ours' ? '' : 'warn';
  q('err').textContent = `${err.toFixed(1)} m`;
  q('err').className = err < 5 ? 'good' : err < 20 ? 'warn' : 'bad';
  q('home').textContent = d.mode === 'landed' ? `${d.landError.toFixed(1)} m (landed)` : `${home.toFixed(0)} m`;
  q('home').className = d.mode === 'landed' ? (d.landError < 5 ? 'good' : 'bad') : '';
  q('alt').textContent = `${Math.max(0, agl).toFixed(0)} m · ${spd.toFixed(1)} m/s`;
}

function positionTag(el, obj, visible) {
  const p = obj.position.clone(); p.y += 4;
  p.project(camera);
  const on = visible && p.z < 1 && Math.abs(p.x) < 1.1 && Math.abs(p.y) < 1.1;
  el.classList.toggle('hidden', !on);
  if (on) el.style.transform = `translate(${(p.x * 0.5 + 0.5) * innerWidth}px, ${(-p.y * 0.5 + 0.5) * innerHeight}px) translate(-50%, -170%)`;
}

// ---------- Visual sync ----------
function syncVisuals(frameDt) {
  for (const d of sim.drones) {
    const v = vis[d.id];
    const visible = d.id === 'ours' || showBaseline;
    v.mesh.visible = v.shadow.visible = v.trail.line.visible = visible;
    v.mesh.position.set(d.pos.x, d.pos.y + 0.7, d.pos.z);

    const spd = Math.hypot(d.vel.x, d.vel.z);
    if (spd > 0.5) {
      const target = Math.atan2(d.vel.x, d.vel.z);
      let diff = target - v.heading; diff = Math.atan2(Math.sin(diff), Math.cos(diff));
      v.heading += diff * Math.min(1, frameDt * 4);
    }
    v.mesh.rotation.set(0, v.heading, 0);
    // Tilt into the direction of travel.
    v.mesh.rotateX(Math.min(0.35, spd * 0.02));
    const spin = d.mode === 'idle' || d.mode === 'landed' ? 0 : 60;
    for (const r of v.mesh.userData.rotors) r.rotation.y += spin * frameDt;

    const g = terrainHeight(d.pos.x, d.pos.z);
    v.shadow.position.set(d.pos.x, g + 0.1, d.pos.z);
    v.shadow.material.opacity = Math.max(0.05, 0.3 - (d.pos.y - g) * 0.005);

    const est = d.est;
    const show = visible && sim.jammed && d.mode !== 'landed';
    v.ghost.visible = v.link.visible = show;
    if (show) {
      v.ghost.position.set(est.x, est.y + 0.7, est.z);
      const lp = v.link.geometry.attributes.position;
      lp.setXYZ(0, d.pos.x, d.pos.y + 0.7, d.pos.z); lp.setXYZ(1, est.x, est.y + 0.7, est.z); lp.needsUpdate = true;
      v.link.geometry.computeBoundingSphere(); v.link.computeLineDistances();
    }
    v.trail.sync(d.trail);

    if (d.mode === 'landed' && !v.marker) {
      const m = new THREE.Mesh(new THREE.RingGeometry(3.5, 4.5, 40), new THREE.MeshBasicMaterial({ color: d.landError < 5 ? W.COLORS.ok : W.COLORS.jam, side: THREE.DoubleSide }));
      m.rotation.x = -Math.PI / 2; m.position.set(d.pos.x, g + 0.2, d.pos.z);
      scene.add(m); v.marker = m;
    }
    if (v.marker) v.marker.visible = visible;
  }

  const jamAge = sim.jammed ? sim.t - sim.jamTime : null;
  jammer.update(performance.now() / 1000, jamAge);
  gps.update(vis.ours.mesh.position, jammer, sim.jammed);

  setCard('card-ours', sim.ours);
  setCard('card-base', sim.baseline);
  $('card-base').classList.toggle('hidden', !showBaseline);
  const g = $('gps');
  g.className = `pill ${sim.jammed ? 'bad' : 'ok'}`;
  g.textContent = sim.jammed ? '✕ GPS JAMMED' : '● GPS LOCKED';
  const s = Math.floor(sim.t);
  $('clock').textContent = `T+${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
  positionTag($('tag-ours'), vis.ours.mesh, true);
  positionTag($('tag-base'), vis.baseline.mesh, showBaseline);

  // Surface new sim events as toasts.
  while (lastEventIdx < sim.events.length) {
    const e = sim.events[lastEventIdx++];
    if (e.type === 'jam') toast(e.text, 'jam');
    else if (e.type === 'rtl') setTimeout(() => toast(e.text, 'warn'), 1800);
    else if (e.type === 'landed' && (e.drone === 'ours' || showBaseline)) {
      const good = sim[e.drone].landError < 5;
      toast(`${good ? '✓' : '✕'} ${e.text}`, good ? 'good' : 'warn');
      if (e.drone === 'baseline' && camMode === 'director') setCam('top');
    }
  }
}

// ---------- Camera ----------
function setCam(mode) {
  camMode = mode;
  controls.enabled = mode === 'orbit';
  if (mode === 'orbit') controls.target.copy(vis.ours.mesh.position);
  $('cam-name').textContent = mode.toUpperCase();
}

function desiredCamera(out, look) {
  const o = vis.ours.mesh.position, b = vis.baseline.mesh.position;
  if (camMode === 'chase') {
    const h = vis.ours.heading;
    look.copy(o);
    out.set(o.x - Math.sin(h) * 22, o.y + 8, o.z - Math.cos(h) * 22);
  } else if (camMode === 'top') {
    // Frame the whole flight: home, both drones and everywhere they've been.
    const pts = [o, new THREE.Vector3(0, 0, 0)];
    cfg.waypoints.forEach(([x, z]) => pts.push(new THREE.Vector3(x, 0, z)));
    if (showBaseline) { pts.push(b); sim.baseline.trail.forEach((p) => pts.push(new THREE.Vector3(p.x, 0, p.z))); }
    const box = new THREE.Box3().setFromPoints(pts);
    const c = box.getCenter(new THREE.Vector3()), sz = box.getSize(new THREE.Vector3());
    const halfFov = THREE.MathUtils.degToRad(camera.fov / 2);
    const h = (Math.max(sz.z, sz.x / camera.aspect) * 0.62) / Math.tan(halfFov);
    look.set(c.x, 0, c.z);
    out.set(c.x, Math.max(150, h), c.z + 1);
  } else {
    // Director: frame ours (and baseline if shown), slowly orbiting.
    look.copy(o);
    let dist = 45;
    if (showBaseline) { look.lerp(b, 0.5); dist = Math.max(45, o.distanceTo(b) * 1.1 + 25); }
    const az = -2.3 + sim.t * 0.02;
    out.set(look.x + Math.cos(az) * dist, look.y + dist * 0.45 + 6, look.z + Math.sin(az) * dist);
  }
  const floor = terrainHeight(out.x, out.z) + 4;
  if (out.y < floor) out.y = floor;
}

function snapCamera() {
  desiredCamera(camPos, camTarget);
  camera.position.copy(camPos);
  camera.lookAt(camTarget);
}

const look = new THREE.Vector3();
function updateCamera(frameDt) {
  if (camMode === 'orbit') {
    controls.target.lerp(vis.ours.mesh.position, Math.min(1, frameDt * 3));
    controls.update();
    return;
  }
  desiredCamera(camPos, look);
  const k = Math.min(1, frameDt * 2.2);
  camera.position.lerp(camPos, k);
  camTarget.lerp(look, k);
  camera.lookAt(camTarget);
}

// ---------- Controls ----------
function act(a) {
  if (a === 'play') {
    if (sim.ours.mode === 'idle') { sim.launch(); sim.running = true; $('start-hint').classList.add('hidden'); }
    else sim.running = !sim.running;
    $('btn-play').textContent = sim.running ? '❚❚ Pause' : '▶ Resume';
  }
  if (a === 'restart') reset();
  if (a === 'jam') sim.jam();
  if (a === 'speed') { speed = SPEEDS[(SPEEDS.indexOf(speed) + 1) % SPEEDS.length]; $('btn-speed').textContent = `${speed}×`; }
  if (a === 'cam') setCam(CAMS[(CAMS.indexOf(camMode) + 1) % CAMS.length]);
  if (a === 'baseline') { showBaseline = !showBaseline; $('btn-base').textContent = `Baseline: ${showBaseline ? 'on' : 'off'}`; }
}

document.querySelectorAll('[data-act]').forEach((b) => b.addEventListener('click', (e) => { act(b.dataset.act); b.blur(); }));

document.addEventListener('keydown', (e) => {
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  const k = e.key;
  const map = { ' ': 'play', r: 'restart', R: 'restart', j: 'jam', J: 'jam', c: 'cam', C: 'cam', b: 'baseline', B: 'baseline', s: 'speed', S: 'speed' };
  if (map[k]) { act(map[k]); e.preventDefault(); return; }
  if (['1', '2', '3', '4'].includes(k)) { setCam(CAMS[Number(k) - 1]); return; }
  if (k === 'f' || k === 'F') {
    if (embedded) parent.postMessage({ type: 'deck-key', key: k }, '*');
    else document.fullscreenElement ? document.exitFullscreen() : document.documentElement.requestFullscreen();
    return;
  }
  // Let the slide deck handle navigation when embedded.
  if (embedded && ['PageDown', 'PageUp', 'ArrowRight', 'ArrowLeft', 'n', 'N', 'p', 'P'].includes(k)) {
    parent.postMessage({ type: 'deck-key', key: k }, '*');
    e.preventDefault();
  }
});

// ---------- Loop ----------
function resize() {
  renderer.setSize(innerWidth, innerHeight, false);
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  for (const v of Object.values(vis)) v.trail.setResolution(innerWidth, innerHeight);
}
addEventListener('resize', resize);
resize();
reset();
if (params.has('autoplay')) act('play');

let last = performance.now();
renderer.setAnimationLoop(() => {
  const now = performance.now();
  const frameDt = Math.min(0.1, (now - last) / 1000);
  last = now;
  if (sim.running) {
    acc += frameDt * speed;
    while (acc >= DT) { sim.step(); acc -= DT; }
  }
  syncVisuals(sim.running ? frameDt * speed : frameDt);
  updateCamera(frameDt);
  renderer.render(scene, camera);
});

// Debug hook for rehearsals / automated checks.
window.__demo = { get sim() { return sim; }, act, setCam };
