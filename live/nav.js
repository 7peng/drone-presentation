// Closed-loop breadcrumb return-to-home, ported from drone-work/eval_cl/rth_sim.py to plain JS (no Cesium here).
// Truth: 50 Hz point mass (thrust lag 0.35 s, 25° tilt cap, quadratic drag in the mean wind, OU gusts).
// Nav: 2D Kalman filter [pN, pE, vN, vE] driven by the measured acceleration. GPS (5 Hz) until the jam; after the
// jam the IMU error follows the stock replay (drift.json: a_err(t) = d²/dt² of the PX4 EKF2 inertial-only error), so
// without fixes the estimate drifts away exactly like the stock case. Vision fixes: fix = bc_pos - Delta_NE,
// cov = model cov + breadcrumb KF cov + attitude term, chi-square gated (3σ).
// Guidance: outbound polyline; breadcrumb every 20 m of KF track; at the turnaround GPS is jammed and the drone follows the
// reversed breadcrumb chain on the KF estimate; k -> k-1 on arrival (p_arr >= p95 thr and |p - bc_k| <= 5 m) or pass.
// The core is synchronous: advance() runs until it needs a camera frame or the UI should redraw, and returns a request.
(function (root) {
  'use strict';
  const G = 9.81, DT = 0.02, TILT_MAX = 25 * Math.PI / 180;
  const BUCKETS = Array.from({ length: 14 }, (_, k) => 50 * 1.2 ** k);

  function rng(seed) {   // mulberry32 + Box-Muller
    let a = seed >>> 0;
    const u = () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
    let spare = null;
    const n = (s = 1) => {
      if (spare !== null) { const r = spare; spare = null; return r * s; }
      let x, y, r;
      do { x = 2 * u() - 1; y = 2 * u() - 1; r = x * x + y * y; } while (r >= 1 || r === 0);
      const f = Math.sqrt(-2 * Math.log(r) / r);
      spare = y * f;
      return x * f * s;
    };
    return { u, n };
  }
  const hyp = (a, b) => Math.hypot(a, b);
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  function bucketFor(agl) {
    const f = Math.log(2 * Math.max(agl, 1));
    let best = BUCKETS[0];
    for (const b of BUCKETS) if (Math.abs(f - Math.log(b)) < Math.abs(f - Math.log(best))) best = b;
    return best;
  }

  class Chain {   // polyline follower (rth_sim.Chain)
    constructor(pts) {
      const P = [pts[0]];
      for (let i = 1; i < pts.length; i++) if (hyp(pts[i][0] - P[P.length - 1][0], pts[i][1] - P[P.length - 1][1]) > 1e-6) P.push(pts[i]);
      if (P.length === 1) P.push([P[0][0] + 1e-3, P[0][1]]);
      this.P = P; this.len = []; this.u = []; this.s0 = [0];
      for (let i = 0; i < P.length - 1; i++) {
        const dn = P[i + 1][0] - P[i][0], de = P[i + 1][1] - P[i][1], L = hyp(dn, de);
        this.len.push(L); this.u.push([dn / L, de / L]); this.s0.push(this.s0[i] + L);
      }
      this.total = this.s0[this.s0.length - 1]; this.i = 0;
    }
    project(p) {
      let best = null;
      for (let i = this.i; i < Math.min(this.i + 6, this.len.length); i++) {
        const t = Math.min(Math.max((p[0] - this.P[i][0]) * this.u[i][0] + (p[1] - this.P[i][1]) * this.u[i][1], 0), this.len[i]);
        const d = hyp(p[0] - (this.P[i][0] + t * this.u[i][0]), p[1] - (this.P[i][1] + t * this.u[i][1]));
        if (best === null || d < best[0] - 1e-6) best = [d, i, t];
      }
      if (!best) return this.total;
      this.i = best[1];
      return this.s0[best[1]] + best[2];
    }
    point(s) {
      s = Math.min(Math.max(s, 0), this.total);
      let i = 0;
      while (i < this.len.length - 1 && this.s0[i + 1] <= s) i++;
      return [this.P[i][0] + (s - this.s0[i]) * this.u[i][0], this.P[i][1] + (s - this.s0[i]) * this.u[i][1]];
    }
  }

  // 2D constant-acceleration-input KF
  class KF {
    constructor(p, v, sigA) { this.x = [p[0], p[1], v[0], v[1]]; this.P = diag([1, 1, 0.1, 0.1]); this.q = sigA * sigA; }
    predict(a, dt) {
      const x = this.x;
      x[0] += x[2] * dt + 0.5 * a[0] * dt * dt; x[1] += x[3] * dt + 0.5 * a[1] * dt * dt;
      x[2] += a[0] * dt; x[3] += a[1] * dt;
      const F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]];
      const q = this.q, d3 = dt ** 3 / 3, d2 = dt ** 2 / 2;
      const Q = [[q * d3, 0, q * d2, 0], [0, q * d3, 0, q * d2], [q * d2, 0, q * dt, 0], [0, q * d2, 0, q * dt]];
      this.P = add(mul(mul(F, this.P), tr(F)), Q);
    }
    update(z, R, idx, gate) {   // z (m), R (m x m), idx = state indices; returns [accepted, mahalanobis]
      const m = idx.length, H = idx.map((j) => [0, 1, 2, 3].map((c) => (c === j ? 1 : 0)));
      const y = z.map((zi, i) => zi - this.x[idx[i]]);
      const S = add(mul(mul(H, this.P), tr(H)), R);
      const Si = inv(S);
      let m2 = 0;
      for (let i = 0; i < m; i++) for (let j = 0; j < m; j++) m2 += y[i] * Si[i][j] * y[j];
      const maha = Math.sqrt(Math.max(m2, 0));
      if (gate && maha > gate) return [false, maha];
      const K = mul(mul(this.P, tr(H)), Si);
      for (let r = 0; r < 4; r++) for (let i = 0; i < m; i++) this.x[r] += K[r][i] * y[i];
      const IKH = add(eye(4), scale(mul(K, H), -1));
      this.P = add(mul(mul(IKH, this.P), tr(IKH)), mul(mul(K, R), tr(K)));   // Joseph form
      return [true, maha];
    }
    get p() { return [this.x[0], this.x[1]]; }
    get v() { return [this.x[2], this.x[3]]; }
    get Pp() { return [[this.P[0][0], this.P[0][1]], [this.P[1][0], this.P[1][1]]]; }
  }
  function diag(d) { return d.map((v, i) => d.map((_, j) => (i === j ? v : 0))); }
  function eye(n) { return diag(Array(n).fill(1)); }
  function tr(A) { return A[0].map((_, j) => A.map((r) => r[j])); }
  function mul(A, B) { return A.map((r) => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0))); }
  function add(A, B) { return A.map((r, i) => r.map((v, j) => v + B[i][j])); }
  function scale(A, s) { return A.map((r) => r.map((v) => v * s)); }
  function inv(A) {
    if (A.length === 1) return [[1 / A[0][0]]];
    if (A.length === 2) { const d = A[0][0] * A[1][1] - A[0][1] * A[1][0]; return [[A[1][1] / d, -A[0][1] / d], [-A[1][0] / d, A[0][0] / d]]; }
    const n = A.length, M = A.map((r, i) => r.concat(eye(n)[i]));   // Gauss-Jordan
    for (let c = 0; c < n; c++) {
      let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      [M[c], M[p]] = [M[p], M[c]];
      const d = M[c][c]; for (let j = 0; j < 2 * n; j++) M[c][j] /= d;
      for (let r = 0; r < n; r++) if (r !== c) { const f = M[r][c]; for (let j = 0; j < 2 * n; j++) M[r][j] -= f * M[c][j]; }
    }
    return M.map((r) => r.slice(n));
  }

  class Drift {   // a_err(t since jam) from drift.json
    constructor(d) {
      this.dt = d.dt; this.a = d.a_ne;
      const n = Math.max(1, Math.round(20 / d.dt)), tail = this.a.slice(-n);
      this.tail = [tail.reduce((s, x) => s + x[0], 0) / tail.length, tail.reduce((s, x) => s + x[1], 0) / tail.length];
    }
    at(t) {
      if (t < 0) return [0, 0];
      const f = t / this.dt, i = Math.floor(f);
      if (i >= this.a.length - 1) return this.tail;
      const w = f - i;
      return [this.a[i][0] * (1 - w) + this.a[i + 1][0] * w, this.a[i][1] * (1 - w) + this.a[i + 1][1] * w];
    }
  }

  class Ground {   // coarse bare-earth grid (ENU z relative to the origin ground), bilinear
    constructor(meta, i16) { Object.assign(this, meta); this.g = i16; }
    at(n, e) {
      const c = (e - this.origin_e) / this.res - 0.5, r = (n - this.origin_n) / this.res - 0.5;
      const c0 = Math.min(Math.max(Math.floor(c), 0), this.cols - 2), r0 = Math.min(Math.max(Math.floor(r), 0), this.rows - 2);
      const fc = Math.min(Math.max(c - c0, 0), 1), fr = Math.min(Math.max(r - r0, 0), 1), g = this.g, C = this.cols;
      const v = (g[r0 * C + c0] * (1 - fc) + g[r0 * C + c0 + 1] * fc) * (1 - fr) + (g[(r0 + 1) * C + c0] * (1 - fc) + g[(r0 + 1) * C + c0 + 1] * fc) * fr;
      return v / 10;
    }
  }

  class Mission {
    // spec: rth_sim mission spec; opt: {vision, drift: Drift, ground: Ground, thr: {nomatch_gate, arrival_p95}, camHz, sigA}
    constructor(spec, opt) {
      this.spec = spec; this.opt = opt; this.vision = !!opt.vision;
      const R = rng(spec.seed * 7919 + 17);
      this.r = R;
      this.ground = opt.ground;
      this.home = spec.home_ne.slice();
      const route = [this.home.slice()];
      for (const [hdg, L] of spec.legs) { const h = hdg * Math.PI / 180, q = route[route.length - 1]; route.push([q[0] + L * Math.cos(h), q[1] + L * Math.sin(h)]); }
      this.route = route; this.chain = new Chain(route);
      this.zHome = this.ground.at(this.home[0], this.home[1]);
      this.D = -(this.zHome + spec.agl);
      this.vc = spec.cruise; this.kd = spec.k_drag ?? 0.011;
      const wd = spec.wind_from_deg * Math.PI / 180 + Math.PI;
      this.wind = [spec.wind * Math.cos(wd), spec.wind * Math.sin(wd)];
      this.gustTau = spec.gust_tau ?? 3; this.gustSig = G * Math.tan((spec.gust_deg ?? 2) * Math.PI / 180);
      this.camDt = 1 / (opt.camHz ?? spec.cam_hz ?? 3);
      this.bcSpacing = spec.bc_spacing ?? 20;
      this.sigAttRad = (parseFloat(spec.att_cov) || 0) * Math.PI / 180;
      // truth
      this.p = this.home.slice(); this.v = [0, 0]; this.f = [0, 0]; this.gust = [0, 0];
      this.yaw = spec.legs[0][0] * Math.PI / 180;
      // sensors
      this.gm = [R.n(0.8), R.n(0.8)]; this.baroDrift = R.n(0.3);
      this.kf = new KF(this.p, this.v, opt.sigA ?? 0.25);
      this.t = 0; this.phase = 'out'; this.tJam = null; this.k = null; this.rth = null;
      this.bcs = []; this.lastBc = null; this.pending = null; this.nextCam = null;
      this.Iv = [0, 0]; this.aDes = [0, 0]; this.settle = 0; this.homeT0 = null; this.tMax = Infinity;
      this.frames = []; this.traj = []; this.adv = { arrival: 0, pass: 0 }; this.done = false; this.end = null;
      this.lastFused = null; this.lastUi = -1;
    }
    get pe() { return this.kf.p; }
    altEst() { return -this.D + this.baroDrift + this.r.n(0.2); }
    aglTrue() { return -this.D - this.ground.at(this.p[0], this.p[1]); }

    // Run until a camera frame is needed ({type:'bc'|'cam', n, e, d, fov_deg}), the UI should redraw ({type:'tick'}),
    // or the mission ended ({type:'done'}). Camera requests must be answered with provide() before advancing.
    advance(uiDt = 0.1) {
      if (this.pending) return this.pending.req;
      while (!this.done) {
        this.control();
        if (this.done) break;
        this.physics();
        const req = this.events();
        if (req) return req;
        if (this.t - this.lastUi >= uiDt) { this.lastUi = this.t; return { type: 'tick' }; }
      }
      return { type: 'done', summary: this.summary() };
    }

    control() {
      const pe = this.kf.p, ve = this.kf.v, s0 = hyp(ve[0], ve[1]);
      let vDes = [0, 0];
      if (this.phase === 'out') {
        const s = this.chain.project(pe), remain = this.chain.total - s;
        const L = Math.max(30, 2.5 * s0);
        const c = remain > 1 ? this.chain.point(s + L) : this.route[this.route.length - 1];
        const d = [c[0] - pe[0], c[1] - pe[1]], dn = hyp(d[0], d[1]);
        const spd = Math.min(this.vc, Math.sqrt(2 * 1.2 * Math.max(remain, 0)));
        vDes = dn > 0.3 ? [spd * d[0] / dn, spd * d[1] / dn] : [0, 0];
        if (remain < 3 && s0 < 0.8) {
          this.phase = 'rth'; this.tJam = this.t; this.tMax = this.t + 2.5 * this.t + 120; this.jamPending = true;
        }
      } else if (this.k !== null) {
        const k = this.k, bcs = this.bcs;
        if (!this.rth || this.rth.k !== k) this.rth = { k, c: new Chain([pe].concat(bcs.slice(0, k + 1).map((b) => b.pe).reverse())) };
        const c = this.rth.c, h = bcs[0].pe, dh = hyp(h[0] - pe[0], h[1] - pe[1]);
        let carrot, spd;
        if (k > 0) {
          const s = c.project(pe), remain = c.total - s;
          carrot = c.point(s + Math.max(30, 2.5 * s0));
          spd = Math.min(this.vc, Math.sqrt(2 * 1.2 * Math.max(remain, 0)) + 0.5);
          const bk = bcs[k].pe, b1 = bcs[k - 1].pe, u = [b1[0] - bk[0], b1[1] - bk[1]], un = hyp(u[0], u[1]);
          if (un > 1e-3 && ((pe[0] - bk[0]) * u[0] + (pe[1] - bk[1]) * u[1]) / un >= 0) { this.k -= 1; this.adv.pass++; }
        } else {
          carrot = h; spd = Math.min(this.vc, Math.sqrt(2 * dh));
          if (this.phase === 'rth') { this.phase = 'home'; this.homeT0 = this.t; }
          this.settle = dh < 3 && s0 < 0.7 ? this.settle + DT : 0;   // rth_sim: 2 m / 5 s / 90 s; relaxed for a live demo
          if (this.settle > 3) this.finish('home_settled');
          else if (this.t - this.homeT0 > 40) this.finish('home_timeout');
        }
        const d = [carrot[0] - pe[0], carrot[1] - pe[1]], dn = hyp(d[0], d[1]);
        vDes = dn > 0.3 ? [spd * d[0] / dn, spd * d[1] / dn] : [0, 0];
        if (this.t > this.tMax) this.finish('timeout');
      }
      const ev = [vDes[0] - ve[0], vDes[1] - ve[1]];
      this.Iv = this.Iv.map((x, i) => Math.min(4, Math.max(-4, x + ev[i] * DT)));
      this.aDes = [0, 1].map((i) => 1.2 * ev[i] + 0.25 * this.Iv[i] + this.kd * s0 * ve[i]);
      if (hyp(vDes[0], vDes[1]) > 2) this.yaw += Math.max(-0.785 * DT, Math.min(0.785 * DT, wrap(Math.atan2(vDes[1], vDes[0]) - this.yaw) * 1.5 * DT));
    }

    physics() {
      const R = this.r;
      let fd = this.aDes.slice();
      const cap = Math.tan(TILT_MAX) * G, fn = hyp(fd[0], fd[1]);
      if (fn > cap) fd = fd.map((x) => x * cap / fn);
      this.f = this.f.map((x, i) => x + (fd[i] - x) * (DT / 0.35));
      const a2 = Math.exp(-DT / this.gustTau), sg = this.gustSig * Math.sqrt(1 - a2 * a2);
      this.gust = this.gust.map((g) => g * a2 + sg * R.n());
      const va = [this.v[0] - this.wind[0], this.v[1] - this.wind[1]], vn = hyp(va[0], va[1]);
      const a = [0, 1].map((i) => this.f[i] - this.kd * vn * va[i] + this.gust[i]);
      this.v = this.v.map((x, i) => x + a[i] * DT);
      this.p = this.p.map((x, i) => x + this.v[i] * DT);
      this.t += DT;
      // IMU -> KF (drift only after the jam, as in the replay)
      const ae = this.tJam === null ? [0, 0] : this.opt.drift.at(this.t - this.tJam);
      this.kf.predict([a[0] + ae[0] + R.n(0.05), a[1] + ae[1] + R.n(0.05)], DT);
      this.baroDrift += R.n(0.02 * Math.sqrt(DT / 0.02 / 50));
      const step = Math.round(this.t / DT);
      if (step % 10 === 0) {   // GPS 5 Hz
        this.gm = this.gm.map((g) => g * Math.exp(-0.2 / 60) + R.n(0.8 * Math.sqrt(1 - Math.exp(-0.4 / 60))));
        if (this.phase === 'out') {
          this.kf.update([this.p[0] + this.gm[0] + R.n(0.4), this.p[1] + this.gm[1] + R.n(0.4)], [[0.9 ** 2, 0], [0, 0.9 ** 2]], [0, 1], 0);
          this.kf.update([this.v[0] + R.n(0.1), this.v[1] + R.n(0.1)], [[0.01, 0], [0, 0.01]], [2, 3], 0);
        }
      }
      if (step % 25 === 0) this.traj.push([this.t, this.p[0], this.p[1], this.kf.x[0], this.kf.x[1], this.phase === 'out' ? 1 : this.phase === 'rth' ? 2 : 3, this.k ?? -1,
        Math.sqrt(Math.max(this.kf.P[0][0] + this.kf.P[1][1], 0))]);
    }

    events() {
      if (this.phase === 'out' || this.jamPending) {
        const pe = this.kf.p;
        if (this.lastBc === null || hyp(pe[0] - this.lastBc[0], pe[1] - this.lastBc[1]) >= this.bcSpacing || this.jamPending) {
          const agl = this.aglTrue(), agl_s = agl * (1 + this.r.n(0.05)), bucket = bucketFor(agl_s);
          const bc = { i: this.bcs.length, t: this.t, pe: pe.slice(), P: this.kf.Pp, pt: this.p.slice(), alt: this.altEst(), agl, agl_s, bucket,
            fov_deg: 2 * Math.atan(bucket / 2 / Math.max(agl_s, 1)) * 180 / Math.PI };
          this.lastBc = pe.slice();
          const jam = this.jamPending;
          this.jamPending = false;
          if (jam) { this.k = this.bcs.length; this.nextCam = this.t + this.camDt; }
          this.bcs.push(bc);
          if (this.vision) {
            this.pending = { kind: 'bc', bc, req: { type: 'bc', i: bc.i, n: this.p[0], e: this.p[1], d: this.D, fov_deg: bc.fov_deg } };
            return this.pending.req;
          }
        }
      }
      if ((this.phase === 'rth' || this.phase === 'home') && this.nextCam !== null && this.t >= this.nextCam - 1e-9) {
        this.nextCam += this.camDt;
        const b = this.bcs[this.k];
        const fr = { t: this.t, k: this.k, pt: this.p.slice(), pe: this.kf.p, status: 'none' };
        this.frames.push(fr);
        if (this.vision) {
          const aglq = b.agl_s + (this.altEst() - b.alt);
          fr.aglq = aglq;
          fr.fov_deg = Math.min(2 * Math.atan(b.bucket / 2 / Math.max(aglq, 1)) * 180 / Math.PI, 170);
          this.pending = { kind: 'cam', fr, b, req: { type: 'cam', k: this.k, n: this.p[0], e: this.p[1], d: this.D, fov_deg: fr.fov_deg } };
          return this.pending.req;
        }
      }
      return null;
    }

    // bc: {desc}; cam: {mu: [d_row, d_col], cov: [[..],[..]] (cells²), pArr, pNm, ms}
    provide(res) {
      const P = this.pending;
      this.pending = null;
      if (!P) return;
      if (P.kind === 'bc') { P.bc.desc = res.desc; P.bc.img = res.img; return; }
      const { fr, b } = P, m = b.bucket / 14, thr = this.opt.thr;
      const ne = [m * res.mu[0], -m * res.mu[1]];
      const c = res.cov, sa = Math.max(fr.aglq, 1) * Math.tan(this.sigAttRad) * Math.SQRT2;
      const cov = [[m * m * c[0][0] + b.P[0][0] + sa * sa, -m * m * c[0][1] + b.P[0][1]], [-m * m * c[1][0] + b.P[1][0], m * m * c[1][1] + b.P[1][1] + sa * sa]];
      const fix = [b.pe[0] - ne[0], b.pe[1] - ne[1]];
      Object.assign(fr, { ne, trueNe: [b.pt[0] - fr.pt[0], b.pt[1] - fr.pt[1]], fix, cov, pArr: res.pArr, pNm: res.pNm, ms: res.ms,
        fixErr: hyp(fix[0] - fr.pt[0], fix[1] - fr.pt[1]), sigma: Math.sqrt(Math.max(cov[0][0], cov[1][1])) });
      if (res.pNm >= thr.nomatch_gate) fr.status = 'no-match';
      else {
        const [ok, maha] = this.kf.update(fix, cov, [0, 1], 3.0);
        fr.status = ok ? 'fused' : 'gated'; fr.maha = maha;
        if (ok) this.lastFused = this.t;
      }
      if (this.k > 0 && res.pArr >= thr.arrival_p95 && hyp(this.kf.x[0] - b.pe[0], this.kf.x[1] - b.pe[1]) <= 5) { this.k -= 1; this.adv.arrival++; fr.advance = 'arrival'; }
    }

    finish(reason) { this.done = true; this.end = reason; }

    summary() {
      const ret = this.traj.filter((r) => r[5] >= 2), e = ret.map((r) => hyp(r[1] - r[3], r[2] - r[4])).sort((a, b) => a - b);
      const q = (a, f) => (a.length ? a[Math.min(a.length - 1, Math.floor(f * a.length))] : null);
      const st = (s) => this.frames.filter((f) => f.status === s).length;
      const fe = this.frames.filter((f) => f.status === 'fused').map((f) => f.fixErr).sort((a, b) => a - b);
      const ms = this.frames.filter((f) => f.ms).map((f) => f.ms).sort((a, b) => a - b);
      return {
        id: this.spec.id, vision: this.vision, end: this.end, final_dist_m: hyp(this.p[0] - this.home[0], this.p[1] - this.home[1]),
        final_est_err_m: hyp(this.p[0] - this.kf.x[0], this.p[1] - this.kf.x[1]), t_jam: this.tJam, t_end: this.t,
        breadcrumbs: this.bcs.length, frames: this.frames.length, fused: st('fused'), gated: st('gated'), no_match: st('no-match'),
        fix_rate: this.frames.length ? st('fused') / this.frames.length : null,
        fix_err_fused_median_m: q(fe, 0.5), fix_err_fused_p90_m: q(fe, 0.9),
        est_err_return_median_m: q(e, 0.5), est_err_return_p95_m: q(e, 0.95), advance: this.adv,
        model_ms_median: q(ms, 0.5), model_ms_p90: q(ms, 0.9),
        route_out_m: this.chain.total,
      };
    }
  }

  const Nav = { Mission, Chain, KF, Drift, Ground, bucketFor, BUCKETS, rng };
  if (typeof module !== 'undefined' && module.exports) module.exports = Nav; else root.Nav = Nav;
})(typeof window !== 'undefined' ? window : globalThis);
