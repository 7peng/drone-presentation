// Deterministic fixed-step simulation of two drones flying the same mission.
// Before jamming both navigate with GPS (estimate == truth). After jamming each
// drone steers using its own *estimated* position, so estimate error turns
// directly into flight error, which is the point of the demo.
import { mulberry32, gaussian, terrainHeight } from './terrain.js';

export const DT = 1 / 120;

const MODE_LABEL = {
  idle: 'ON PAD', takeoff: 'TAKEOFF', mission: 'MISSION', rtl: 'RETURN TO LAUNCH', land: 'LANDING', landed: 'LANDED',
};

function makeDrone(id, cfg, home, offset) {
  return {
    id, home, offset,
    pos: { x: home[0], y: terrainHeight(home[0], home[1]), z: home[1] },
    vel: { x: 0, y: 0, z: 0 },
    err: { x: 0, z: 0 },      // estimate − truth (horizontal)
    errVel: { x: 0, z: 0 },   // used by dead reckoning
    mode: 'idle', wp: 0,
    trail: [], landError: null,
    get est() { return { x: this.pos.x + this.err.x, y: this.pos.y, z: this.pos.z + this.err.z }; },
    get modeLabel() { return MODE_LABEL[this.mode]; },
  };
}

export class Sim {
  constructor(cfg) {
    this.cfg = cfg;
    this.rand = mulberry32(cfg.seed);
    this.t = 0;
    this.running = false;
    this.jammed = false;
    this.jamTime = null;
    this.events = [];
    const ox = cfg.baseline.home[0] - cfg.ours.home[0], oz = cfg.baseline.home[1] - cfg.ours.home[1];
    this.ours = makeDrone('ours', cfg, cfg.ours.home, [0, 0]);
    this.baseline = makeDrone('baseline', cfg, cfg.baseline.home, [ox, oz]);
    this.drones = [this.ours, this.baseline];
    const a = this.rand() * Math.PI * 2; // fixed dead-reckoning bias direction for this seed
    this.biasDir = { x: Math.cos(a), z: Math.sin(a) };
    this._trailClock = 0;
  }

  launch() {
    if (this.drones.some((d) => d.mode !== 'idle')) return;
    for (const d of this.drones) d.mode = 'takeoff';
    this.emit('launch', 'TAKEOFF');
  }

  jam() {
    if (this.jammed || this.ours.mode === 'idle') return;
    this.jammed = true;
    this.jamTime = this.t;
    this.emit('jam', 'JAMMER ACTIVE · GPS LOST');
    for (const d of this.drones) if (d.mode === 'mission' || d.mode === 'takeoff') d.mode = 'rtl';
    this.emit('rtl', 'FAILSAFE · RETURN TO LAUNCH');
  }

  emit(type, text, drone) { this.events.push({ t: this.t, type, text, drone }); }

  step() {
    const { cfg } = this;
    this.t += DT;
    const auto = cfg.jammer.autoAtWaypoint;
    if (!this.jammed && auto != null && this.ours.mode === 'mission' && this.ours.wp >= auto) this.jam();

    for (const d of this.drones) {
      if (this.jammed && d.mode !== 'landed') this.updateError(d);
      this.control(d);
    }

    this._trailClock += DT;
    if (this._trailClock >= 0.2) {
      this._trailClock = 0;
      for (const d of this.drones) if (d.mode !== 'idle' && d.mode !== 'landed') d.trail.push({ ...d.pos });
    }
  }

  updateError(d) {
    const sq = Math.sqrt(DT);
    if (d.id === 'ours') {
      // Ornstein–Uhlenbeck: noisy but pulled back toward zero, so error stays bounded.
      const { pull, noise } = this.cfg.nav.ours;
      d.err.x += -pull * d.err.x * DT + noise * sq * gaussian(this.rand);
      d.err.z += -pull * d.err.z * DT + noise * sq * gaussian(this.rand);
    } else {
      // Double-integrated accelerometer bias + noise: error grows roughly with t².
      const { bias, noise } = this.cfg.nav.baseline;
      d.errVel.x += (bias * this.biasDir.x) * DT + noise * sq * gaussian(this.rand);
      d.errVel.z += (bias * this.biasDir.z) * DT + noise * sq * gaussian(this.rand);
      // Cap drift speed so the drone can still "arrive" at its wrong home and land.
      const v = Math.hypot(d.errVel.x, d.errVel.z), vmax = 3.5;
      if (v > vmax) { d.errVel.x *= vmax / v; d.errVel.z *= vmax / v; }
      d.err.x += d.errVel.x * DT;
      d.err.z += d.errVel.z * DT;
    }
  }

  control(d) {
    const { cfg } = this;
    if (d.mode === 'idle' || d.mode === 'landed') return;
    const est = d.est;
    const homeY = terrainHeight(d.home[0], d.home[1]);
    let tx = est.x, tz = est.z, ty = homeY + cfg.cruiseAlt;

    if (d.mode === 'takeoff') {
      tx = d.home[0]; tz = d.home[1];
      if (d.pos.y > ty - 1.5) { d.mode = 'mission'; if (d.id === 'ours') this.emit('mission', 'MISSION START'); }
    } else if (d.mode === 'mission') {
      const w = cfg.waypoints[d.wp];
      tx = w[0] + d.offset[0]; tz = w[1] + d.offset[1];
      if (Math.hypot(tx - est.x, tz - est.z) < cfg.waypointRadius) {
        d.wp++;
        if (d.wp >= cfg.waypoints.length) d.mode = 'rtl';
      }
    } else if (d.mode === 'rtl') {
      tx = d.home[0]; tz = d.home[1];
      // Generous threshold: a drifting estimate never settles within a tight radius.
      if (Math.hypot(tx - est.x, tz - est.z) < 6) d.mode = 'land';
    } else if (d.mode === 'land') {
      tx = d.home[0]; tz = d.home[1];
      const ground = terrainHeight(d.pos.x, d.pos.z);
      ty = ground - 1;
      if (d.pos.y <= ground + 0.05) {
        d.pos.y = ground; d.vel = { x: 0, y: 0, z: 0 };
        d.mode = 'landed';
        d.landError = Math.hypot(d.pos.x - d.home[0], d.pos.z - d.home[1]);
        this.emit('landed', `${d.id === 'ours' ? 'OURS' : 'BASELINE'} LANDED · ${d.landError.toFixed(1)} m FROM HOME`, d.id);
        return;
      }
    }

    // Desired velocity toward target (based on the estimate), speed-limited, then accel-limited.
    let dx = tx - est.x, dz = tz - est.z;
    const dist = Math.hypot(dx, dz);
    const vmax = d.mode === 'takeoff' ? 0 : Math.min(cfg.cruiseSpeed, dist * 0.8);
    if (dist > 1e-6) { dx = (dx / dist) * vmax; dz = (dz / dist) * vmax; } else { dx = dz = 0; }
    const vyMax = d.mode === 'land' ? cfg.landRate : cfg.climbRate;
    const dy = Math.max(-vyMax, Math.min(vyMax, (ty - d.pos.y) * 1.2));

    const a = cfg.accel * DT;
    const lim = (cur, want) => cur + Math.max(-a, Math.min(a, want - cur));
    d.vel.x = lim(d.vel.x, dx); d.vel.z = lim(d.vel.z, dz); d.vel.y = lim(d.vel.y, dy);
    d.pos.x += d.vel.x * DT; d.pos.y += d.vel.y * DT; d.pos.z += d.vel.z * DT;
  }
}
