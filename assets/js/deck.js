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
const demoFrame = demoLayer.querySelector('iframe');
const channel = 'BroadcastChannel' in window ? new BroadcastChannel('deck') : null;

if (new URLSearchParams(location.search).has('draft')) document.body.classList.add('draft');

let idx = 0;
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

  const isDemo = slides[idx].hasAttribute('data-demo');
  // Preload the demo one slide early so WebGL is warm when we arrive.
  if ((isDemo || slides[idx + 1]?.hasAttribute('data-demo')) && !demoFrame.src) {
    demoFrame.src = demoFrame.dataset.src;
  }
  demoLayer.classList.toggle('is-active', isDemo);
  if (isDemo) setTimeout(() => demoFrame.contentWindow?.focus(), 50);
  else window.focus();

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
