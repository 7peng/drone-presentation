// Closed-loop breadcrumb return-to-home, ported from drone-work/eval_cl/rth_sim.py to plain JS (no Cesium here).
// Truth: 50 Hz point mass (3D thrust lag 0.35 s, 25° tilt cap, quadratic drag in the mean wind, OU gusts), body attitude
// from the thrust vector + yaw (rth_sim.attitude), so the camera tilts with every manoeuvre (cruise tilt ~10–15°).
// Nav: 2D Kalman filter [pN, pE, vN, vE] driven by the measured acceleration. GPS (5 Hz) until the jam; after the jam the
// IMU error follows the stock replay (drift.json: a_err(t) = d²/dt² of the PX4 EKF2 inertial-only error), so without
// fixes the estimate drifts away exactly like the stock case. Vision fixes: fix = bc_pos - Delta_NE,
// cov = model cov + breadcrumb KF cov + attitude term, chi-square gated (3σ).
// Camera (realism on, default): Mid-Air downward camera fixed to the body (90° FOV); every frame is rectified in the browser
// with the ESTIMATED attitude (roll/pitch error OU σ 0.5° + 0.3° white, yaw OU σ 2° + 1° white: the q_est model of the
// training data) to level, north-up and the breadcrumb's footprint (scale from the baro altitude difference). Return
// altitude differs from the outbound one by ±10–20% (per mission).
// Guidance: outbound polyline (per-leg AGL, jam at jam_frac of the route for complex missions); breadcrumb every 20 m of
// KF track; after the jam the drone follows the reversed breadcrumb chain on the KF estimate; k -> k-1 on arrival
// (p_arr >= p95 thr and |p - bc_k| <= 5 m) or pass (complex: only within 30 m of bc_k).
// The core is synchronous: advance() runs until it needs a camera frame or the UI should redraw, and returns a request.
(function (root) {
  'use strict';
  const G = 9.81, DT = 0.02, TILT_MAX = 25 * Math.PI / 180, DEG = Math.PI / 180;
  const BUCKETS = Array.from({ length: 14 }, (_, k) => 50 * 1.2 ** k);
  // camera -> body (Mid-Air downward camera: cam x = body right, cam y = -body forward, cam z = body down)
  const CAM = [[0, -1, 0], [1, 0, 0], [0, 0, 1]];
  // level, north-up target camera -> NED (cam x = east, cam y = south, cam z = down)
  const TGT = [[0, -1, 0], [1, 0, 0], [0, 0, 1]];

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
  // 3x3 helpers (row-major nested arrays)
  const m3 = (A, B) => A.map((r) => [0, 1, 2].map((j) => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const norm = (a) => { const n = Math.hypot(a[0], a[1], a[2]); return a.map((x) => x / n); };
  function so3exp(w) {
    const th = Math.hypot(w[0], w[1], w[2]);
    if (th < 1e-12) return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    const k = w.map((x) => x / th), K = [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]], K2 = m3(K, K), s = Math.sin(th), c = 1 - Math.cos(th);
    return [0, 1, 2].map((i) => [0, 1, 2].map((j) => (i === j ? 1 : 0) + s * K[i][j] + c * K2[i][j]));
  }
  function attitude(f, yaw) {   // rth_sim.attitude: body z along -thrust, heading yaw -> body->NED
    const zb = norm(f.map((x) => -x)), xc = [Math.cos(yaw), Math.sin(yaw), 0];
    const yb = norm(cross(zb, xc)), xb = cross(yb, zb);
    return [0, 1, 2].map((i) => [xb[i], yb[i], zb[i]]);
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
    seg(s) { s = Math.min(Math.max(s, 0), this.total); let i = 0; while (i < this.len.length - 1 && this.s0[i + 1] <= s) i++; return [i, s]; }
    point(s) { const [i, ss] = this.seg(s); return [this.P[i][0] + (ss - this.s0[i]) * this.u[i][0], this.P[i][1] + (ss - this.s0[i]) * this.u[i][1]]; }
    dir(s) { return this.u[this.seg(s)[0]]; }
    turnSpeed(s, v) {   // speed cap for the heading change within the lookahead (complex missions)
      const a = this.dir(s), b = this.dir(s + 2.5 * Math.max(v, 4) + 20);
      const ang = Math.acos(Math.min(1, Math.max(-1, a[0] * b[0] + a[1] * b[1])));
      return Math.max(4, v * (1 - 0.7 * ang / Math.PI));
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
    if (A.length === 2) { const d = A[0][0] * A[1][1] - A[0][1] * A[1][0]; return [[A[1][1] / d, -A[0][1] / d], [-A[1][0] / d, A[0][0] / d]]; }
    return [[1 / A[0][0]]];
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
    // spec: rth_sim mission spec (legs [hdg, L(, agl)]); opt: {vision, drift, ground, thr: {nomatch_gate, arrival_p95},
    //   camHz, sigA, realism (default true), retAgl (return AGL factor; default 1 ± U(0.10, 0.20) when realism)}
    constructor(spec, opt) {
      this.spec = spec; this.opt = opt; this.vision = !!opt.vision; this.realism = opt.realism !== false;
      const R = rng(spec.seed * 7919 + 17);
      this.r = R;
      this.ground = opt.ground;
      this.home = spec.home_ne.slice();
      const route = [this.home.slice()];
      for (const leg of spec.legs) { const h = leg[0] * DEG, q = route[route.length - 1]; route.push([q[0] + leg[1] * Math.cos(h), q[1] + leg[1] * Math.sin(h)]); }
      this.route = route; this.chain = new Chain(route);
      this.cx = !!spec.complex;
      this.segAgl = spec.legs.map((l) => (l.length > 2 ? l[2] : spec.agl));
      this.jamFrac = spec.jam_frac ?? 1;
      this.zHome = this.ground.at(this.home[0], this.home[1]);
      this.D = -(this.zHome + (this.cx ? this.segAgl[0] : spec.agl)); this.Dt = this.D; this.vD = 0;
      const sgn = R.u() < 0.5 ? -1 : 1, mag = 0.10 + 0.10 * R.u();
      this.retAgl = opt.retAgl ?? (this.realism ? 1 + sgn * mag : 1);
      this.vc = spec.cruise; this.kd = spec.k_drag ?? 0.011;
      // wind (rth_sim): mean wind blowing FROM wind_from_deg, OU gusts (accelerations, sigma g*tan(gust_deg)); opt.wind overrides
      this.windSpeed = opt.wind?.speed ?? spec.wind; this.windFrom = opt.wind?.fromDeg ?? spec.wind_from_deg;
      const wd = this.windFrom * DEG + Math.PI;
      this.wind = [this.windSpeed * Math.cos(wd), this.windSpeed * Math.sin(wd)];
      this.gustTau = spec.gust_tau ?? 3; this.gustSig = G * Math.tan((spec.gust_deg ?? 2) * DEG);
      this.camDt = 1 / (opt.camHz ?? spec.cam_hz ?? 3);
      this.bcSpacing = spec.bc_spacing ?? 20;
      this.sigAttRad = (parseFloat(spec.att_cov) || 0) * DEG;
      // truth
      this.p = this.home.slice(); this.v = [0, 0]; this.f = [0, 0, -G]; this.gust = [0, 0];
      this.yaw = spec.legs[0][0] * DEG;
      this.R = attitude(this.f, this.yaw);
      // sensors: GPS Gauss-Markov, baro drift, attitude-estimate error (body frame, rad)
      this.gm = [R.n(0.8), R.n(0.8)]; this.baroDrift = R.n(0.3);
      this.attOU = [R.n(0.5 * DEG), R.n(0.5 * DEG), R.n(2 * DEG)];
      // gust-scale attitude jitter of the true body (OU, tau 0.7 s, sigma = gust_deg / 2 per axis, roll/pitch only)
      this.jitSig = this.realism ? 0.5 * (spec.gust_deg ?? 2) * DEG : 0; this.jit = [0, 0];
      this.kf = new KF(this.p, this.v, opt.sigA ?? 1.0);   // sigma_a covers the replay's time-varying IMU error (median 0.15 m/s²)
      this.t = 0; this.phase = 'out'; this.tJam = null; this.k = null; this.rth = null;
      this.bcs = []; this.lastBc = null; this.pending = null; this.nextCam = null;
      this.Iv = [0, 0]; this.aDes = [0, 0, 0]; this.settle = 0; this.homeT0 = null; this.tMax = Infinity;
      this.frames = []; this.traj = []; this.adv = { arrival: 0, pass: 0 }; this.done = false; this.end = null;
      this.lastFused = null; this.lastUi = -1; this.tiltMax = 0;
    }
    get pe() { return this.kf.p; }
    altEst() { return -this.D + this.baroDrift + this.r.n(0.2); }
    aglTrue() { return -this.D - this.ground.at(this.p[0], this.p[1]); }
    tiltDeg() { return Math.acos(Math.min(1, this.R[2][2])) / DEG; }
    crabDeg() { return Math.hypot(this.v[0], this.v[1]) > 2 ? wrap(this.yaw - Math.atan2(this.v[1], this.v[0])) / DEG : 0; }
    // camera request: raw camera (true attitude) and the estimate used to rectify it
    camReq(type, extra, fovOut) {
      const a = this.attOU, w = this.realism ? [a[0] + this.r.n(0.3 * DEG), a[1] + this.r.n(0.3 * DEG), a[2] + this.r.n(1 * DEG)] : [0, 0, 0];
      const Rt = this.realism ? m3(this.R, so3exp([this.jit[0], this.jit[1], 0])) : [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
      const Re = m3(Rt, so3exp(w));
      return Object.assign({ type, n: this.p[0], e: this.p[1], d: this.D, rawFov: this.realism ? 90 : fovOut, fovOut,
        Rcam: m3(Rt, CAM), Rest: m3(Re, CAM), Rtgt: TGT, tilt: this.realism ? this.tiltDeg() : 0, attErrDeg: w.map((x) => x / DEG) }, extra);
    }

    // Run until a camera frame is needed ({type:'bc'|'cam', n, e, d, rawFov, fovOut, Rcam, Rest, Rtgt}), the UI should
    // redraw ({type:'tick'}), or the mission ended ({type:'done'}). Camera requests must be answered with provide().
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
        const s = this.chain.project(pe), sEnd = this.jamFrac * this.chain.total, remain = sEnd - s;
        const L = Math.max(30, 2.5 * s0);
        const c = remain > 1 ? this.chain.point(Math.min(s + L, sEnd)) : this.chain.point(sEnd);
        const d = [c[0] - pe[0], c[1] - pe[1]], dn = hyp(d[0], d[1]);
        let spd = Math.min(this.vc, Math.sqrt(2 * 1.2 * Math.max(remain, 0)));
        if (this.cx) { spd = Math.min(spd, this.chain.turnSpeed(s, this.vc)); this.Dt = -(this.zHome + this.segAgl[Math.min(this.chain.i, this.segAgl.length - 1)]); }
        vDes = dn > 0.3 ? [spd * d[0] / dn, spd * d[1] / dn] : [0, 0];
        if (remain < 3 && s0 < 0.8) {
          this.phase = 'rth'; this.tJam = this.t; this.tMax = this.t + 2.5 * this.t + 120; this.jamPending = true;
        }
      } else if (this.k !== null) {
        const k = this.k, bcs = this.bcs;
        if (!this.rth || this.rth.k !== k) this.rth = { k, c: new Chain([pe].concat(bcs.slice(0, k + 1).map((b) => b.pe).reverse())) };
        const c = this.rth.c, h = bcs[0].pe, dh = hyp(h[0] - pe[0], h[1] - pe[1]);
        const retD = (b) => -(this.zHome + (-b.D - this.zHome) * this.retAgl);   // retrace the outbound profile, scaled
        let carrot, spd;
        if (k > 0) {
          const s = c.project(pe), remain = c.total - s;
          carrot = c.point(s + Math.max(30, 2.5 * s0));
          spd = Math.min(this.vc, Math.sqrt(2 * 1.2 * Math.max(remain, 0)) + 0.5);
          if (this.cx) spd = Math.min(spd, c.turnSpeed(s, this.vc));
          this.Dt = retD(bcs[k]);
          const bk = bcs[k].pe, b1 = bcs[k - 1].pe, u = [b1[0] - bk[0], b1[1] - bk[1]], un = hyp(u[0], u[1]);
          const nearOk = !this.cx || hyp(pe[0] - bk[0], pe[1] - bk[1]) < 30;
          if (nearOk && un > 1e-3 && ((pe[0] - bk[0]) * u[0] + (pe[1] - bk[1]) * u[1]) / un >= 0) { this.k -= 1; this.adv.pass++; }
        } else {
          carrot = h; spd = Math.min(this.vc, Math.sqrt(2 * dh), 0.35 * dh + 0.3);   // gentle final approach (noisy fixes)
          this.Dt = retD(bcs[0]);
          if (this.phase === 'rth') { this.phase = 'home'; this.homeT0 = this.t; }
          this.settle = dh < 4 ? this.settle + DT : 0;   // rth_sim: 2 m / 5 s / 90 s; relaxed for a live demo
          if (this.settle > 2) this.finish('home_settled');
          else if (this.t - this.homeT0 > 40) this.finish('home_timeout');
        }
        const d = [carrot[0] - pe[0], carrot[1] - pe[1]], dn = hyp(d[0], d[1]);
        vDes = dn > 0.3 ? [spd * d[0] / dn, spd * d[1] / dn] : [0, 0];
        if (this.t > this.tMax) this.finish('timeout');
      }
      const ev = [vDes[0] - ve[0], vDes[1] - ve[1]];
      this.Iv = this.Iv.map((x, i) => Math.min(4, Math.max(-4, x + ev[i] * DT)));
      const ah = [0, 1].map((i) => 1.2 * ev[i] + 0.25 * this.Iv[i] + this.kd * s0 * ve[i]);
      const Dest = this.D - this.baroDrift;   // altitude hold on the baro estimate
      const vdDes = Math.min(2.5, Math.max(-2.5, 0.4 * (this.Dt - Dest)));
      const az = Math.min(3, Math.max(-3, 1.5 * (vdDes - this.vD)));
      this.aDes = [ah[0], ah[1], az];
      // nose into the air-relative velocity: in a crosswind the drone crabs, so camera yaw != ground track
      const va = this.realism ? [vDes[0] - this.wind[0], vDes[1] - this.wind[1]] : vDes;
      this.yawRate = hyp(vDes[0], vDes[1]) > 2 ? Math.max(-45 * DEG, Math.min(45 * DEG, wrap(Math.atan2(va[1], va[0]) - this.yaw) * 1.5)) : 0;
    }

    physics() {
      const R = this.r;
      const fd = [this.aDes[0], this.aDes[1], this.aDes[2] - G];
      const cap = Math.tan(TILT_MAX) * Math.abs(fd[2]), fn = hyp(fd[0], fd[1]);
      if (fn > cap) { fd[0] *= cap / fn; fd[1] *= cap / fn; }
      this.f = this.f.map((x, i) => x + (fd[i] - x) * (DT / 0.35));
      const a2 = Math.exp(-DT / this.gustTau), sg = this.gustSig * Math.sqrt(1 - a2 * a2);
      this.gust = this.gust.map((g) => g * a2 + sg * R.n());
      const va = [this.v[0] - this.wind[0], this.v[1] - this.wind[1]], vn = hyp(va[0], va[1]);
      const a = [0, 1].map((i) => this.f[i] - this.kd * vn * va[i] + this.gust[i]);
      this.v = this.v.map((x, i) => x + a[i] * DT);
      this.p = this.p.map((x, i) => x + this.v[i] * DT);
      this.vD += (this.f[2] + G) * DT; this.D += this.vD * DT;
      this.yaw = wrap(this.yaw + this.yawRate * DT);
      this.R = attitude(this.f, this.yaw);
      this.tiltMax = Math.max(this.tiltMax, this.tiltDeg());
      this.t += DT;
      // attitude-estimate error (OU, tau 30 s roll/pitch, 60 s yaw)
      const ou = (x, s, tau) => x * Math.exp(-DT / tau) + s * Math.sqrt(1 - Math.exp(-2 * DT / tau)) * R.n();
      this.attOU = [ou(this.attOU[0], 0.5 * DEG, 30), ou(this.attOU[1], 0.5 * DEG, 30), ou(this.attOU[2], 2 * DEG, 60)];
      this.jit = [ou(this.jit[0], this.jitSig, 0.7), ou(this.jit[1], this.jitSig, 0.7)];
      // IMU -> KF (drift only after the jam, as in the replay)
      const ae = this.tJam === null ? [0, 0] : this.opt.drift.at(this.t - this.tJam);
      this.kf.predict([a[0] + ae[0] + R.n(0.05), a[1] + ae[1] + R.n(0.05)], DT);
      this.baroDrift += R.n(0.0028);
      const step = Math.round(this.t / DT);
      if (step % 10 === 0) {   // GPS 5 Hz
        this.gm = this.gm.map((g) => g * Math.exp(-0.2 / 60) + R.n(0.8 * Math.sqrt(1 - Math.exp(-0.4 / 60))));
        if (this.phase === 'out') {
          this.kf.update([this.p[0] + this.gm[0] + R.n(0.4), this.p[1] + this.gm[1] + R.n(0.4)], [[0.9 ** 2, 0], [0, 0.9 ** 2]], [0, 1], 0);
          this.kf.update([this.v[0] + R.n(0.1), this.v[1] + R.n(0.1)], [[0.01, 0], [0, 0.01]], [2, 3], 0);
        }
      }
      if (step % 25 === 0) this.traj.push([this.t, this.p[0], this.p[1], this.kf.x[0], this.kf.x[1], this.phase === 'out' ? 1 : this.phase === 'rth' ? 2 : 3, this.k ?? -1,
        Math.sqrt(Math.max(this.kf.P[0][0] + this.kf.P[1][1], 0)), this.D, this.tiltDeg()]);
    }

    events() {
      if (this.phase === 'out' || this.jamPending) {
        const pe = this.kf.p;
        if (this.lastBc === null || hyp(pe[0] - this.lastBc[0], pe[1] - this.lastBc[1]) >= this.bcSpacing || this.jamPending) {
          const agl = this.aglTrue(), agl_s = agl * (1 + this.r.n(0.05)), bucket = bucketFor(agl_s);
          const bc = { i: this.bcs.length, t: this.t, pe: pe.slice(), P: this.kf.Pp, pt: this.p.slice(), D: this.D, alt: this.altEst(), agl, agl_s, bucket,
            fov_deg: 2 * Math.atan(bucket / 2 / Math.max(agl_s, 1)) / DEG };
          this.lastBc = pe.slice();
          const jam = this.jamPending;
          this.jamPending = false;
          if (jam) { this.k = this.bcs.length; this.nextCam = this.t + this.camDt; }
          this.bcs.push(bc);
          if (this.vision) {
            this.pending = { kind: 'bc', bc, req: this.camReq('bc', { i: bc.i }, bc.fov_deg) };
            return this.pending.req;
          }
        }
      }
      if ((this.phase === 'rth' || this.phase === 'home') && this.nextCam !== null && this.t >= this.nextCam - 1e-9) {
        this.nextCam += this.camDt;
        const b = this.bcs[this.k];
        const fr = { t: this.t, k: this.k, pt: this.p.slice(), pe: this.kf.p, status: 'none', agl: this.aglTrue(), crab: this.crabDeg() };
        this.frames.push(fr);
        if (this.vision) {
          const aglq = b.agl_s + (this.altEst() - b.alt);
          fr.aglq = aglq;
          fr.fov_deg = Math.min(2 * Math.atan(b.bucket / 2 / Math.max(aglq, 1)) / DEG, 170);
          // home phase: also match the next breadcrumbs out (same footprint bucket, so the same rectified frame) and fuse
          // the averaged fix -- one breadcrumb's bias no longer decides where the drone lands (opt.multiBc, default 3)
          const nb = this.k === 0 ? (this.opt.multiBc ?? 3) : 1;
          const extra = [];
          for (let j = 1; j < nb && j < this.bcs.length; j++) if (this.bcs[j].bucket === b.bucket) extra.push(j);
          fr.extra = extra;
          const req = this.camReq('cam', { k: this.k, extra }, fr.fov_deg);
          fr.tilt = req.tilt; fr.attErrDeg = req.attErrDeg;
          this.pending = { kind: 'cam', fr, b, req };
          return req;
        }
      }
      return null;
    }

    // bc: {desc, mask}; cam: {mu: [d_row, d_col], cov: [[..],[..]] (cells²), pArr, pNm, probs, ms}
    provide(res) {
      const P = this.pending;
      this.pending = null;
      if (!P) return;
      if (P.kind === 'bc') { Object.assign(P.bc, res); return; }
      const { fr, b } = P, m = b.bucket / 14, thr = this.opt.thr;
      const ne = [m * res.mu[0], -m * res.mu[1]];
      const c = res.cov, sa = Math.max(fr.aglq, 1) * Math.tan(this.sigAttRad) * Math.SQRT2;
      const covM = [[m * m * c[0][0], -m * m * c[0][1]], [-m * m * c[1][0], m * m * c[1][1]]];
      let cov = [[covM[0][0] + b.P[0][0] + sa * sa, covM[0][1] + b.P[0][1]], [covM[1][0] + b.P[1][0], covM[1][1] + b.P[1][1] + sa * sa]];
      let fix = [b.pe[0] - ne[0], b.pe[1] - ne[1]];
      // extra breadcrumbs (home phase): average the fixes that pass the no-match gate; covariance = mean covariance
      const ex = (res.extra || []).filter((x) => x.pNm < thr.nomatch_gate);
      if (ex.length && res.pNm < thr.nomatch_gate) {
        const fixes = [fix], covs = [cov];
        for (const x of ex) {
          const bj = this.bcs[x.j], mj = bj.bucket / 14, cj = x.cov;
          fixes.push([bj.pe[0] - mj * x.mu[0], bj.pe[1] + mj * x.mu[1]]);
          covs.push([[mj * mj * cj[0][0] + bj.P[0][0] + sa * sa, -mj * mj * cj[0][1] + bj.P[0][1]], [-mj * mj * cj[1][0] + bj.P[1][0], mj * mj * cj[1][1] + bj.P[1][1] + sa * sa]]);
        }
        const n = fixes.length;
        fix = [0, 1].map((i) => fixes.reduce((a, f) => a + f[i], 0) / n);
        cov = [0, 1].map((i) => [0, 1].map((j) => covs.reduce((a, cc) => a + cc[i][j], 0) / n));
        fr.nFused = n; fr.fix0 = [b.pe[0] - ne[0], b.pe[1] - ne[1]];
      }
      const trueNe = [b.pt[0] - fr.pt[0], b.pt[1] - fr.pt[1]];
      Object.assign(fr, { ne, trueNe, fix, cov, covModel: covM, mu: res.mu, covCells: c, probs: res.probs, bucket: b.bucket, pArr: res.pArr, pNm: res.pNm, ms: res.ms,
        trueCells: [trueNe[0] / m, -trueNe[1] / m], offErr: hyp(ne[0] - trueNe[0], ne[1] - trueNe[1]),
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
      const srt = (a) => a.slice().sort((x, y) => x - y);
      const fe = srt(this.frames.filter((f) => f.status === 'fused').map((f) => f.fixErr));
      const oe = srt(this.frames.filter((f) => f.offErr !== undefined).map((f) => f.offErr));
      const ms = srt(this.frames.filter((f) => f.ms).map((f) => f.ms));
      const tl = srt(this.frames.filter((f) => f.tilt !== undefined).map((f) => f.tilt));
      const ag = this.frames.filter((f) => f.agl !== undefined).map((f) => f.agl);
      const bcAgl = this.bcs.map((b) => b.agl);
      return {
        id: this.spec.id, vision: this.vision, realism: this.realism, end: this.end, final_dist_m: hyp(this.p[0] - this.home[0], this.p[1] - this.home[1]),
        final_est_err_m: hyp(this.p[0] - this.kf.x[0], this.p[1] - this.kf.x[1]), t_jam: this.tJam, t_end: this.t,
        breadcrumbs: this.bcs.length, frames: this.frames.length, fused: st('fused'), gated: st('gated'), no_match: st('no-match'),
        fix_rate: this.frames.length ? st('fused') / this.frames.length : null,
        fix_err_fused_median_m: q(fe, 0.5), fix_err_fused_p90_m: q(fe, 0.9),
        offset_err_all_median_m: q(oe, 0.5), offset_err_all_p90_m: q(oe, 0.9),
        home_fix_err_median_m: q(srt(this.frames.filter((f) => f.k === 0 && f.status === 'fused').map((f) => f.fixErr)), 0.5),
        est_err_return_median_m: q(e, 0.5), est_err_return_p95_m: q(e, 0.95), advance: this.adv,
        model_ms_median: q(ms, 0.5), model_ms_p90: q(ms, 0.9),
        tilt_deg_median: q(tl, 0.5), tilt_deg_p90: q(tl, 0.9), ret_agl_factor: this.retAgl, wind_mps: this.windSpeed, wind_from_deg: this.windFrom,
        crab_deg_abs_median: q(srt(this.frames.map((f) => Math.abs(f.crab || 0))), 0.5),
        agl_out_mean: bcAgl.length ? bcAgl.reduce((s, x) => s + x, 0) / bcAgl.length : null,
        agl_ret_mean: ag.length ? ag.reduce((s, x) => s + x, 0) / ag.length : null,
        route_out_m: this.chain.total * this.jamFrac,
      };
    }
  }

  const Nav = { Mission, Chain, KF, Drift, Ground, bucketFor, BUCKETS, rng, m3, CAM, TGT };
  if (typeof module !== 'undefined' && module.exports) module.exports = Nav; else root.Nav = Nav;
})(typeof window !== 'undefined' ? window : globalThis);
