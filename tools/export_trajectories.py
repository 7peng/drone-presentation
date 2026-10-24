"""Export recorded missions to compact JSON for the CesiumJS demo (cesium/data/*.json).

Run with the drone-work venv (needs numpy only):
    /scratch/users/wgpeng/drone-work/realworld/venv3d/bin/python tools/export_trajectories.py

Frame: every mission lives in the rural_main10 local frame (g3d_world.py): NED metres from lat0/lon0 on the
ellipsoid tangent plane at P0 = ecef(lat0, lon0, ground_h_ellipsoid); D = -(height above the ORIGIN ground).
The mesh's ENU z is relative to P0, so  z_enu = -D + ground0,  with ground0 = ground grid at (0, 0).
ECEF = P0 + ENU^T [e, n, z_enu] -> WGS84 lat/lon/ellipsoidal height, which is what Cesium and the
Google Photorealistic 3D Tiles use, so no geoid correction is needed.

Scenarios:
  stock  demo/mission_rth + ekf/mission_rth/jam_rth (seed index 4), closed-loop composition as in
         demo/render_demo.py: estimate(t) = planned(t), truth(t) = planned(t) - LPF_2s(e(t)).
  ours   eval_cl/sweep/mission_012/log.npz (median mission) and eval_cl/mission_000/log.npz.
"""
import json
import math
import os
import sys

import numpy as np

WORK = "/scratch/users/wgpeng/drone-work"
AREA = f"{WORK}/g3d/areas/rural_main10"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cesium", "data")
RATE_HZ = 2.0


class Frame:
    def __init__(self, area=AREA):
        meta = json.load(open(f"{area}/index.json"))["meta"]
        self.lat0, self.lon0, self.h0 = meta["lat0"], meta["lon0"], meta["ground_h_ellipsoid"]
        gi = json.load(open(f"{area}/height_grid.json"))
        self.ge0, self.gn0, self.res = float(gi["origin_e"]), float(gi["origin_n"]), float(gi["res"])
        self.grid = np.load(f"{area}/ground_grid.npy", mmap_mode="r")
        self.ground0 = float(self.ground_abs(0.0, 0.0))
        la, lo = math.radians(self.lat0), math.radians(self.lon0)
        self.ENU = np.array([[-math.sin(lo), math.cos(lo), 0],
                             [-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)],
                             [math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)]])
        self.P0 = self.ecef(self.lat0, self.lon0, self.h0)

    @staticmethod
    def ecef(lat, lon, h):
        a, e2 = 6378137.0, 6.69437999014e-3
        la, lo = math.radians(lat), math.radians(lon)
        N = a / math.sqrt(1 - e2 * math.sin(la) ** 2)
        return np.array([(N + h) * math.cos(la) * math.cos(lo), (N + h) * math.cos(la) * math.sin(lo),
                         (N * (1 - e2) + h) * math.sin(la)])

    @staticmethod
    def geodetic(X):
        """ECEF (n,3) -> lat, lon (deg), ellipsoidal h (m). Bowring, then 3 fixed-point iterations."""
        a, e2 = 6378137.0, 6.69437999014e-3
        x, y, z = X[:, 0], X[:, 1], X[:, 2]
        lon = np.arctan2(y, x)
        p = np.hypot(x, y)
        lat = np.arctan2(z, p * (1 - e2))
        for _ in range(5):
            N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
            h = p / np.cos(lat) - N
            lat = np.arctan2(z, p * (1 - e2 * N / (N + h)))
        N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
        h = p / np.cos(lat) - N
        return np.degrees(lat), np.degrees(lon), h

    def ground_abs(self, e, n):
        A = self.grid
        c = (np.asarray(e, float) - self.ge0) / self.res - 0.5
        r = (np.asarray(n, float) - self.gn0) / self.res - 0.5
        c0 = np.clip(np.floor(c).astype(int), 0, A.shape[1] - 2)
        r0 = np.clip(np.floor(r).astype(int), 0, A.shape[0] - 2)
        fc, fr = np.clip(c - c0, 0, 1), np.clip(r - r0, 0, 1)
        return (A[r0, c0] * (1 - fc) + A[r0, c0 + 1] * fc) * (1 - fr) + (A[r0 + 1, c0] * (1 - fc) + A[r0 + 1, c0 + 1] * fc) * fr

    def ned_to_llh(self, ned):
        ned = np.atleast_2d(np.asarray(ned, float))
        enu = np.stack([ned[:, 1], ned[:, 0], -ned[:, 2] + self.ground0], 1)
        X = self.P0[None] + enu @ self.ENU
        return np.stack(self.geodetic(X), 1)

    def ground_llh(self, n, e):
        """Ground point (bare earth) under local north/east, as lat/lon/h."""
        d = -(float(self.ground_abs(e, n)) - self.ground0)
        return self.ned_to_llh([[n, e, d]])[0]


def pack(llh):
    return [[round(float(a), 7), round(float(b), 7), round(float(c), 1)] for a, b, c in llh]


def resample(t, X, t_new):
    return np.stack([np.interp(t_new, t, X[:, i]) for i in range(X.shape[1])], 1)


def export_stock(fr):
    sys.path.insert(0, f"{WORK}/demo")
    import render_demo  # noqa: E402  (numpy only at import time)
    args = render_demo.build_parser().parse_args(["basemap"])
    render_demo.apply_style(args)
    st = render_demo.Story(args)
    m = st.meta
    t0, t1 = 0.0, float(m["t_home_s"]) + 4.0
    tn = np.arange(t0, t1 + 1e-6, 1.0 / RATE_HZ)
    truth = resample(st.t_true, st.pos_true, tn)
    est = resample(st.t_true, st.planned, tn)
    home = np.array(m["home_ne"], float)
    ih = int(np.argmin(np.abs(tn - float(m["t_home_s"]))))
    err_end = float(np.linalg.norm(truth[ih, :2] - est[ih, :2]))     # miss when it "thinks it's home"
    t_jam = float(m["t_jam_s"])
    return {
        "id": "stock_px4_seed4",
        "title": "Stock PX4, GPS jammed",
        "kind": "stock",
        "t": [round(float(x), 2) for x in tn - t0],
        "truth": pack(fr.ned_to_llh(truth)),
        "est": pack(fr.ned_to_llh(est)),
        "home": [round(float(x), 7) for x in fr.ground_llh(*home)],
        "jam": [round(float(x), 7) for x in fr.ground_llh(*m["jam_ne"])],
        "events": [
            {"t": t_jam, "key": "jam", "label": "GPS jammed"},
            {"t": t_jam + float(st.summary["picked"]["t_invalid_after_jam_s"]), "key": "invalid",
             "label": "EKF2 flags position invalid (failsafe disabled for the demo)"},
            {"t": float(m["t_rth_s"]), "key": "rth", "label": "Return home on the inertial estimate"},
            {"t": float(m["t_home_s"]), "key": "home", "label": "Thinks it's home"},
        ],
        "result": {"miss_m": round(err_end, 1), "spread_median_m": round(st.summary["spread_err_at_end"]["median_m"]),
                   "spread_iqr_m": [round(st.summary["spread_err_at_end"]["p25_m"]),
                                    round(st.summary["spread_err_at_end"]["p75_m"])], "seeds": 10},
        "source": "drone-work/demo: mission_rth, ekf/mission_rth/jam_rth (PX4 EKF2 replay, seed index 4), "
                  "closed loop truth = planned - LPF_2s(e)",
        "agl_m": m["agl_m"], "speed_mps": m["speed_mps"],
    }


def export_ours(fr, d, mid, title):
    L = np.load(f"{d}/log.npz")
    S = json.load(open(f"{d}/summary.json"))
    tr = L["traj"]
    phase = tr[:, 11]
    i0 = int(np.argmax(phase >= 1))                      # skip the 25 s alignment hover (EKF not initialised)
    tr = tr[i0:]
    t = tr[:, 0]
    tn = np.arange(t[0], t[-1] + 1e-6, 1.0 / RATE_HZ)
    truth = resample(t, tr[:, 1:4], tn)
    est = resample(t, tr[:, 4:7], tn)
    sig = np.interp(tn, t, tr[:, 13])
    t_home = float(t[np.argmax(tr[:, 11] >= 3)]) if (tr[:, 11] >= 3).any() else float(t[-1])
    home = L["home"]
    return {
        "id": mid,
        "title": title,
        "kind": "ours",
        "t": [round(float(x - tn[0]), 2) for x in tn],
        "truth": pack(fr.ned_to_llh(truth)),
        "est": pack(fr.ned_to_llh(est)),
        "sigma": [round(float(s), 1) for s in sig],
        "home": [round(float(x), 7) for x in fr.ground_llh(*home)],
        "jam": [round(float(x), 7) for x in fr.ground_llh(*np.interp(S["t_jam"], t, tr[:, 1]).reshape(1),
                                                            *np.interp(S["t_jam"], t, tr[:, 2]).reshape(1))],
        "breadcrumbs": pack(fr.ned_to_llh(L["bc_p_true"]))[::2],
        "events": [
            {"t": round(S["t_jam"] - tn[0], 2), "key": "jam", "label": "GPS jammed"},
            {"t": round(S["t_jam"] - tn[0], 2), "key": "rth", "label": "Return home on vision breadcrumbs"},
            {"t": round(t_home - tn[0], 2), "key": "home", "label": "Home"},
        ],
        "result": {"final_m": round(S["final_dist_m"], 1), "ekf_err_median_m": round(S["ekf_err_return"]["median"], 1),
                   "fix_yield": round(S["fix_yield"], 3), "breadcrumbs": S["breadcrumbs"]},
        "source": f"drone-work/eval_cl/{os.path.relpath(d, WORK + '/eval_cl')}/log.npz (rth_sim.py, closed loop)",
        "agl_m": S["spec"]["agl"], "speed_mps": S["spec"]["cruise"],
    }


def main():
    fr = Frame()
    os.makedirs(OUT, exist_ok=True)
    jobs = [
        ("stock.json", lambda: export_stock(fr)),
        ("ours_m012.json", lambda: export_ours(fr, f"{WORK}/eval_cl/sweep/mission_012", "ours_mission_012",
                                               "Our vision system (median mission)")),
        ("ours_m000.json", lambda: export_ours(fr, f"{WORK}/eval_cl/mission_000", "ours_mission_000",
                                               "Our vision system (mission 000)")),
    ]
    origin = {"lat0": fr.lat0, "lon0": fr.lon0, "ground_h_ellipsoid": fr.h0, "ground0_enu_z": round(fr.ground0, 2)}
    for name, fn in jobs:
        d = fn()
        d["frame"] = origin
        s = json.dumps(d, separators=(",", ":"))
        open(os.path.join(OUT, name), "w").write(s)
        print(f"{name}: {len(d['t'])} samples, {len(s) / 1024:.0f} KB, home {d['home']}, result {d['result']}")


if __name__ == "__main__":
    main()
