// Minimal keyboard-driven deck. No dependencies, works offline.
//
// Keys: → / Space / PageDown = next   ← / PageUp = back   Home / End
//       F = fullscreen   N = speaker notes   P = presenter window   T = reset timer
// Each <section class="slide" data-steps="n"> has n in-slide steps; reaching
// step k adds .is-step-1 … .is-step-k to the slide, which CSS uses to animate.

const stage = document.getElementById('stage');
const slides = [...document.querySelectorAll('.slide')];
const progress = document.getElementById('progress');
const slideNum = document.getElementById('slide-num');
const notesOverlay = document.getElementById('notes-overlay');
const demoLayer = document.getElementById('demo-layer');
// One full-viewport iframe per demo page (data-demo-src, default the layer's data-src), created on first use and kept
// alive, so switching between the Cesium replay and live/ never reloads either.
const DEFAULT_DEMO = demoLayer.dataset.src || 'cesium/?embed=1';
const frames = new Map();   // absolute src -> iframe
const channel = 'BroadcastChannel' in window ? new BroadcastChannel('deck') : null;

if (new URLSearchParams(location.search).has('draft')) document.body.classList.add('draft');

let idx = 0;
const srcOf = (el) => new URL(el?.dataset.demoSrc || DEFAULT_DEMO, location.href).href;
const post = (f) => f.dataset.scenario && f.contentWindow?.postMessage({ type: 'demo-scenario', scenario: f.dataset.scenario }, '*');
function frameFor(el) {
  const src = srcOf(el);
  if (!frames.has(src)) {
    const f = document.createElement('iframe');
    f.title = 'Live demo';
    f.allow = 'fullscreen';
    f.src = src;
    f.addEventListener('load', () => setTimeout(() => post(f), 50));
    demoLayer.appendChild(f);
    frames.set(src, f);
  }
  return frames.get(src);
}
let step = 0;
let timerStart = null;

const stepsOf = (i) => Number(slides[i].dataset.steps || 0);
const titleOf = (i) => slides[i]?.dataset.title || slides[i]?.querySelector('h1,h2')?.textContent.trim() || '';
const notesOf = (i) => slides[i]?.querySelector('.notes')?.innerHTML.trim() || '';

function fit() {
  const s = Math.min(innerWidth / 1920, innerHeight / 1080);
  stage.style.transform = `scale(${s}) translate(-50%, -50%)`;
}

function render() {
  slides.forEach((el, i) => {
    el.classList.toggle('is-active', i === idx);
    for (let k = 1; k <= 5; k++) el.classList.toggle(`is-step-${k}`, i === idx && k <= step);
  });

  // Demo slides: data-demo="stock" | "ours" picks the scenario (posted to the page), data-demo-src the page.
  const isDemo = slides[idx].hasAttribute('data-demo');
  if (slides[idx + 1]?.hasAttribute('data-demo')) frameFor(slides[idx + 1]);   // preload: WebGL warm on arrival
  const active = isDemo ? frameFor(slides[idx]) : null;
  frames.forEach((f) => f.classList.toggle('is-shown', f === active));
  demoLayer.classList.toggle('is-active', isDemo);
  if (active) {
    const scenario = slides[idx].dataset.demo;
    if (scenario && scenario !== active.dataset.scenario) { active.dataset.scenario = scenario; post(active); }
    setTimeout(() => active.contentWindow?.focus(), 50);
  } else window.focus();

  // Slide videos: load lazily, play only while their slide is showing.
  slides.forEach((el, i) => el.querySelectorAll('video').forEach((v) => {
    if (i === idx) {
      if (!v.src && v.dataset.src) v.src = v.dataset.src;
      v.play().catch(() => {});
    } else if (!v.paused) v.pause();
  }));

  progress.style.width = `${(idx / (slides.length - 1)) * 100}%`;
  slideNum.textContent = `${idx + 1} / ${slides.length}`;
  history.replaceState(null, '', `#${idx + 1}${step ? '.' + step : ''}`);
  renderNotes();
  broadcast();
}

function go(i, s = 0) {
  idx = Math.max(0, Math.min(slides.length - 1, i));
  step = Math.max(0, Math.min(stepsOf(idx), s));
  render();
}

function next() {
  if (timerStart === null) timerStart = Date.now();
  if (step < stepsOf(idx)) { step++; render(); }
  else if (idx < slides.length - 1) go(idx + 1, 0);
}

function prev() {
  if (step > 0) { step--; render(); }
  else if (idx > 0) go(idx - 1, stepsOf(idx - 1));
}

function renderNotes() {
  if (!notesOverlay.classList.contains('is-open')) return;
  const t = slides[idx].dataset.time;
  notesOverlay.innerHTML =
    `<div class="meta"><span>${idx + 1}. ${titleOf(idx)}</span>` +
    `${t ? `<span>target ${t}s</span>` : ''}<span>next: ${titleOf(idx + 1) || '—'}</span></div>` +
    (notesOf(idx) || '<em>No notes.</em>');
}

function broadcast() {
  channel?.postMessage({
    type: 'state', idx, step, total: slides.length, steps: stepsOf(idx),
    title: titleOf(idx), nextTitle: titleOf(idx + 1), notes: notesOf(idx),
    times: slides.map((s) => Number(s.dataset.time || 0)), timerStart,
  });
}

function handleKey(key) {
  switch (key) {
    case 'ArrowRight': case 'ArrowDown': case 'PageDown': case ' ': case 'Enter': next(); return true;
    case 'ArrowLeft': case 'ArrowUp': case 'PageUp': case 'Backspace': prev(); return true;
    case 'Home': go(0); return true;
    case 'End': go(slides.length - 1); return true;
    case 'f': case 'F':
      document.fullscreenElement ? document.exitFullscreen() : document.documentElement.requestFullscreen();
      return true;
    case 'n': case 'N': notesOverlay.classList.toggle('is-open'); renderNotes(); return true;
    case 'p': case 'P': window.open('presenter.html', 'presenter', 'width=1100,height=700'); setTimeout(broadcast, 800); return true;
    case 't': case 'T': timerStart = Date.now(); broadcast(); return true;
  }
  return false;
}

document.addEventListener('keydown', (e) => {
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  if (handleKey(e.key)) e.preventDefault();
});

// The demo iframe forwards navigation/F/N keys it doesn't use.
window.addEventListener('message', (e) => {
  if (e.data?.type === 'deck-key') handleKey(e.data.key);
});

channel?.addEventListener('message', (e) => {
  const m = e.data;
  if (m.type === 'nav') m.dir > 0 ? next() : prev();
  if (m.type === 'hello') broadcast();
  if (m.type === 'reset-timer') { timerStart = Date.now(); broadcast(); }
});

addEventListener('resize', fit);
fit();

const [, hs, hk] = location.hash.match(/^#(\d+)(?:\.(\d+))?/) || [];
go(hs ? Number(hs) - 1 : 0, hk ? Number(hk) : 0);
