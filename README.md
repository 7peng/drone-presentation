# drone-presentation

5-minute hackathon deck with a live 3D demo of learned return-to-launch under GPS jamming.
Static site, no build step, no network needed (Three.js is in `vendor/`).

- `index.html`: the deck (7 slides, animated SVG diagrams, demo embedded on slide 5)
- `demo/`: the standalone 3D sim (`demo/index.html`), also loaded by the deck
- `presenter.html`: speaker notes + 5:00 timer, synced to the deck

## Run locally

```bash
python3 -m http.server 8765
```

Open http://localhost:8765. You need a server: ES modules don't load from `file://`.

## Presenting

| Deck | |
|---|---|
| → / Space / PageDown | next step or slide (clickers work) |
| ← / PageUp | back |
| F | fullscreen |
| N | notes overlay |
| P | presenter window (notes, timer, pace) |
| T | reset timer |

| Demo (slide 5 or `demo/`) | |
|---|---|
| Space | launch / pause |
| J | jam GPS now (otherwise auto at waypoint 2) |
| C or 1–4 | camera: director / chase / orbit / top-down |
| B | toggle baseline drone |
| S | speed 1× / 2× / 4× |
| R | restart |

The sim is deterministic: same seed, same flight, every rehearsal. Change it with `demo/?seed=3`. `?autoplay=1` starts immediately.

## What to edit before presenting

Open `/?draft` to outline every placeholder in yellow.

- **Slide text and numbers:** `index.html` (team name, title, architecture boxes, training stats, results)
- **Speaker notes:** the `<div class="notes">` in each slide. `data-time` sets per-slide seconds for the pace indicator.
- **Demo scenario:** `demo/js/config.js` (waypoints, speeds, jammer position, error models)

## Deploy to GitHub Pages

Repo → Settings → Pages → Source: *Deploy from a branch* → `main` / `(root)` → Save.
The site goes live at https://7peng.github.io/drone-presentation/ about a minute after each push.
