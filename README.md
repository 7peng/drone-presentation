# drone-presentation

Hackathon deck for **Homeward**: vision breadcrumbs that get a drone home when GPS is jammed. It includes a live
CesiumJS replay of recorded missions over Google Photorealistic 3D Tiles.

It is a static site with no build step.

- `index.html`: the deck (14 slides; the two live-replay slides load `cesium/` full screen).
- `cesium/`: the live 3D replay (`cesium/index.html`), also usable on its own.
- `presenter.html`: speaker notes plus a 5:00 timer, synced to the deck.
- `media/`: web-encoded videos (H.264, ≤1080p, CRF 26–27, `+faststart`). They are committed as plain git files, not LFS, because GitHub Pages does not serve LFS content. Keep each file under ~20 MB.
- `assets/img/`: still figures. `assets/css/tokens.css` holds the shared palette and fonts.
- `tools/export_trajectories.py`: regenerates `cesium/data/*.json` from the simulation logs.

## Run locally

```bash
python3 -m http.server 8765
```

Open http://localhost:8765. You need a server, because the replay fetches JSON.

## Live 3D replay: Cesium ion token

The replay streams Google Photorealistic 3D Tiles through Cesium ion, so it needs an ion access token in the browser.
**Any token used in a web page is public**: anyone who opens the page can read it. So create a dedicated, restricted one:

1. Go to https://ion.cesium.com/tokens and choose **Create token**.
2. Scopes: `assets:read` only.
3. Assets: restrict it to **2275207** (Google Photorealistic 3D Tiles) and **1** (Cesium World Terrain).
4. Allowed URLs: `https://7peng.github.io` (or wherever the deck is hosted) and `http://localhost:8765`.
5. Then use **one** of these:
   - `cp config.example.js config.local.js` and paste the token there. `config.local.js` is git-ignored: **never commit it**.
   - Open `cesium/#token=YOUR_TOKEN` once. The token is stored in this browser's localStorage and removed from the URL.
   - Paste it into the box the replay shows when no token is found.

If there is no token, if CesiumJS can't load (offline), or if WebGL is unavailable, the replay falls back to the
pre-rendered video for that scenario. The Google and Cesium attributions are shown in the viewer's bottom-left
credit bar, as Google's terms require.

On the deployed site, `config.local.js` doesn't exist, so visitors are asked for a token, or see the video. If you
want the public page to work without that, deliberately commit a URL-restricted token in a separate
`config.public.js`, and load it before `demo.js`.

CesiumJS is pinned to `cesium@1.146.0` from jsDelivr.

## Presenting

| Deck | |
|---|---|
| → / Space / PageDown | next step or slide (clickers work) |
| ← / PageUp | back |
| F | fullscreen |
| N | notes overlay |
| P | presenter window (notes, timer, pace) |
| T | reset timer |

| Live replay (slides 3 and 8, or `cesium/`) | |
|---|---|
| Space | play / pause |
| 1 / 2 | stock PX4 / our system |
| C | chase ↔ orbit camera |
| T | top-down overview on/off |
| S | speed: auto, 1×, 5×, 10×, 20×, 40× |
| , and . | scrub −10 s / +10 s |
| R | restart |
| PageDown / PageUp | next / previous slide (forwarded to the deck) |

`cesium/?scenario=ours` opens on our system. `?mission=000` swaps in the second recorded mission. `?autoplay` starts playback.

## Data

`cesium/data/*.json` holds the time, the lat/lon/ellipsoidal height of the truth and of the estimate (at 2 Hz), home,
the jam point and event times. It is exported from:

- **Stock PX4**: `drone-work/demo/mission_rth` plus the PX4 EKF2 replay `ekf/mission_rth/jam_rth` (seed index 4). This is the same closed-loop composition as the pre-rendered video: the estimate follows the plan, and the truth is the plan minus the low-passed EKF2 error.
- **Our system**: `drone-work/eval_cl/sweep/mission_012/log.npz` (the median mission) and `eval_cl/mission_000/log.npz`.

Positions are converted from the area's local NED frame (rural_main10: `lat0 = 52.02`, `lon0 = 11.45`, tangent plane
at 120 m ellipsoidal height, with D measured from the origin ground) to WGS84 through ECEF. The paths were checked
against independent satellite imagery: field edges, roads and the railway line up.

```bash
/scratch/users/wgpeng/drone-work/realworld/venv3d/bin/python tools/export_trajectories.py
```

## What to edit before presenting

Open `/?draft` to outline every placeholder in yellow:

- the team name and track on slide 1;
- the held-out (v3) numbers on slide 12.

A newer horizontal-chase failure video is still rendering. When it's ready, encode it as `media/jamming_failure.mp4`
(see the media note above). The replay's stock-PX4 fallback uses that file.

## Deploy to GitHub Pages

Repo → Settings → Pages → Source: *Deploy from a branch* → `main` / `(root)` → Save.
