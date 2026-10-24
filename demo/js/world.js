// Static scenery + reusable visual objects (drones, jammer, trails, GPS links).
import * as THREE from 'three';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { terrainHeight, mulberry32 } from './terrain.js';

export const COLORS = { ours: 0x39d0ff, baseline: 0xff9a3c, jam: 0xff3d5e, ok: 0x2ee59d };

export function buildSky(scene) {
  const top = new THREE.Color(0x5d93c9), horizon = new THREE.Color(0xdbe6ef);
  const geo = new THREE.SphereGeometry(4000, 32, 16);
  const mat = new THREE.ShaderMaterial({
    side: THREE.BackSide, depthWrite: false, fog: false,
    uniforms: { top: { value: top }, horizon: { value: horizon } },
    vertexShader: 'varying vec3 vP; void main(){ vP = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
    fragmentShader: 'uniform vec3 top; uniform vec3 horizon; varying vec3 vP; void main(){ float h = normalize(vP).y; gl_FragColor = vec4(mix(horizon, top, smoothstep(0.0, 0.45, h)), 1.0); }',
  });
  scene.add(new THREE.Mesh(geo, mat));
  scene.fog = new THREE.Fog(0xd4e0ea, 350, 1500);
  scene.add(new THREE.HemisphereLight(0xdfefff, 0x4a5a3a, 1.1));
  const sun = new THREE.DirectionalLight(0xfff2dd, 2.2);
  sun.position.set(-300, 500, 200);
  scene.add(sun);
}

export function buildTerrain(scene, seed) {
  const size = 2400, seg = 240;
  const geo = new THREE.PlaneGeometry(size, size, seg, seg);
  geo.rotateX(-Math.PI / 2);
  const pos = geo.attributes.position;
  const colors = new Float32Array(pos.count * 3);
  const low = new THREE.Color(0x6f9a58), high = new THREE.Color(0x9aa38a), dry = new THREE.Color(0xa7b47a), c = new THREE.Color();
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), z = pos.getZ(i);
    const h = terrainHeight(x, z);
    pos.setY(i, h);
    c.copy(low).lerp(dry, Math.max(0, Math.min(1, (h + 10) / 25))).lerp(high, Math.max(0, Math.min(1, (h - 12) / 18)));
    colors.set([c.r, c.g, c.b], i * 3);
  }
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geo.computeVertexNormals();
  scene.add(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 1 })));

  // Trees (instanced) for a sense of scale.
  const rand = mulberry32(seed * 31 + 1);
  const N = 2200;
  const trunk = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.3, 0.45, 3, 5), new THREE.MeshStandardMaterial({ color: 0x5b4632, flatShading: true }), N);
  const crown = new THREE.InstancedMesh(new THREE.ConeGeometry(2.6, 8, 6), new THREE.MeshStandardMaterial({ color: 0x3f6b3a, flatShading: true }), N);
  const m = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(), p = new THREE.Vector3();
  let n = 0;
  while (n < N) {
    const x = (rand() - 0.5) * 1600, z = (rand() - 0.5) * 1600;
    if (Math.hypot(x, z) < 45) continue;
    const k = 0.7 + rand() * 0.9, y = terrainHeight(x, z);
    q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), rand() * 6.28);
    m.compose(p.set(x, y + 1.5 * k, z), q, s.set(k, k, k)); trunk.setMatrixAt(n, m);
    m.compose(p.set(x, y + 6.5 * k, z), q, s.set(k, k, k)); crown.setMatrixAt(n, m);
    n++;
  }
  scene.add(trunk, crown);
}

function padTexture(color) {
  const cv = document.createElement('canvas'); cv.width = cv.height = 256;
  const g = cv.getContext('2d');
  g.fillStyle = '#20262e'; g.beginPath(); g.arc(128, 128, 126, 0, 7); g.fill();
  g.strokeStyle = color; g.lineWidth = 10; g.beginPath(); g.arc(128, 128, 110, 0, 7); g.stroke();
  g.fillStyle = color; g.font = 'bold 140px system-ui, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText('H', 128, 136);
  const t = new THREE.CanvasTexture(cv); t.colorSpace = THREE.SRGBColorSpace; return t;
}

export function buildPad(scene, [x, z], color) {
  const y = terrainHeight(x, z);
  const mesh = new THREE.Mesh(new THREE.CircleGeometry(4, 40), new THREE.MeshBasicMaterial({ map: padTexture(color), transparent: true }));
  mesh.rotation.x = -Math.PI / 2; mesh.position.set(x, y + 0.06, z);
  scene.add(mesh);
}

export function buildRoute(scene, cfg) {
  const pts = [new THREE.Vector3(0, cfg.cruiseAlt, 0)];
  for (const [x, z] of cfg.waypoints) pts.push(new THREE.Vector3(x, cfg.cruiseAlt, z));
  pts.push(new THREE.Vector3(0, cfg.cruiseAlt, 0));
  const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineDashedMaterial({ color: 0xffffff, dashSize: 4, gapSize: 4, transparent: true, opacity: 0.7 }));
  line.computeLineDistances();
  scene.add(line);
  const wpGeo = new THREE.OctahedronGeometry(1.8), wpMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
  const markers = cfg.waypoints.map(([x, z]) => {
    const m = new THREE.Mesh(wpGeo, wpMat); m.position.set(x, cfg.cruiseAlt, z); scene.add(m);
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, cfg.cruiseAlt), new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.35 }));
    pole.position.set(x, cfg.cruiseAlt / 2 + terrainHeight(x, z) / 2, z); scene.add(pole);
    return m;
  });
  return { line, markers };
}

export function buildDrone(color) {
  const g = new THREE.Group();
  const dark = new THREE.MeshStandardMaterial({ color: 0x1d232b, roughness: 0.6, metalness: 0.3 });
  const accent = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.35, roughness: 0.4 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.22, 1.0), dark); g.add(body);
  const shell = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.14, 0.7), accent); shell.position.y = 0.16; g.add(shell);
  const rotors = [];
  for (const [sx, sz] of [[1, 1], [-1, 1], [1, -1], [-1, -1]]) {
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.08, 1.2), dark);
    arm.position.set(sx * 0.42, 0, sz * 0.42); arm.rotation.y = Math.atan2(sx, sz); g.add(arm);
    const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.18, 10), dark);
    motor.position.set(sx * 0.82, 0.08, sz * 0.82); g.add(motor);
    const rotor = new THREE.Group(); rotor.position.set(sx * 0.82, 0.2, sz * 0.82);
    const blade = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.02, 0.08), new THREE.MeshStandardMaterial({ color: 0x111111 }));
    rotor.add(blade, blade.clone().rotateY(Math.PI / 2));
    const disc = new THREE.Mesh(new THREE.CircleGeometry(0.46, 24), new THREE.MeshBasicMaterial({ color: 0x222222, transparent: true, opacity: 0.18, side: THREE.DoubleSide }));
    disc.rotation.x = -Math.PI / 2; rotor.add(disc);
    g.add(rotor); rotors.push(rotor);
  }
  const led = (c, z) => { const m = new THREE.Mesh(new THREE.SphereGeometry(0.06, 8, 8), new THREE.MeshBasicMaterial({ color: c })); m.position.set(0, 0, z); g.add(m); };
  led(0x00ff66, 0.52); led(0xff2222, -0.52);
  g.scale.setScalar(3); // exaggerated so it reads from a distance
  g.userData.rotors = rotors;
  return g;
}

export function buildGhost(color) {
  // "Where the drone thinks it is": wireframe sphere + link line to the real drone.
  const ghost = new THREE.Mesh(new THREE.SphereGeometry(2.2, 12, 8), new THREE.MeshBasicMaterial({ color, wireframe: true, transparent: true, opacity: 0.6 }));
  const link = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
    new THREE.LineDashedMaterial({ color, dashSize: 2, gapSize: 2 }));
  ghost.visible = link.visible = false;
  return { ghost, link };
}

export function buildShadow() {
  const m = new THREE.Mesh(new THREE.CircleGeometry(2.4, 24), new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.25, depthWrite: false }));
  m.rotation.x = -Math.PI / 2;
  return m;
}

// Thick screen-space trail (plain GL lines are 1px everywhere).
export class Trail {
  constructor(scene, color) {
    this.scene = scene;
    this.material = new LineMaterial({ color, linewidth: 5, transparent: true, opacity: 0.95 });
    this.line = new THREE.Group();
    scene.add(this.line);
    this.count = -1;
    this.mesh = null;
  }
  setResolution(w, h) { this.material.resolution.set(w, h); }
  sync(points) {
    if (points.length === this.count) return; // sim appends a point every 0.2 s
    this.count = points.length;
    if (this.mesh) { this.line.remove(this.mesh); this.mesh.geometry.dispose(); this.mesh = null; }
    if (points.length < 2) return;
    const geo = new LineGeometry();
    geo.setPositions(points.flatMap((p) => [p.x, p.y + 0.7, p.z]));
    this.mesh = new Line2(geo, this.material);
    this.mesh.computeLineDistances();
    this.line.add(this.mesh);
  }
}

export function buildJammer(scene, [x, z], radius) {
  const y = terrainHeight(x, z);
  const g = new THREE.Group(); g.position.set(x, y, z); scene.add(g);
  const steel = new THREE.MeshStandardMaterial({ color: 0x55585e, metalness: 0.6, roughness: 0.4 });
  const tower = new THREE.Mesh(new THREE.CylinderGeometry(0.6, 2.4, 30, 4, 1, true), new THREE.MeshStandardMaterial({ color: 0x777b82, wireframe: true }));
  tower.position.y = 15; g.add(tower);
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.25, 0.25, 8), steel); mast.position.y = 34; g.add(mast);
  const beacon = new THREE.Mesh(new THREE.SphereGeometry(0.9, 12, 12), new THREE.MeshBasicMaterial({ color: COLORS.jam })); beacon.position.y = 38.5; g.add(beacon);
  const shack = new THREE.Mesh(new THREE.BoxGeometry(6, 3.5, 4), new THREE.MeshStandardMaterial({ color: 0x6b6f5a })); shack.position.set(6, 1.75, 0); g.add(shack);

  const domeMat = new THREE.MeshBasicMaterial({ color: COLORS.jam, transparent: true, opacity: 0.07, side: THREE.DoubleSide, depthWrite: false, fog: false });
  const dome = new THREE.Mesh(new THREE.SphereGeometry(1, 48, 24, 0, Math.PI * 2, 0, Math.PI / 2), domeMat);
  const wire = new THREE.Mesh(dome.geometry, new THREE.MeshBasicMaterial({ color: COLORS.jam, wireframe: true, transparent: true, opacity: 0.12, depthWrite: false, fog: false }));
  dome.add(wire); dome.scale.setScalar(0.001); dome.visible = false; g.add(dome);

  const rings = [];
  for (let i = 0; i < 4; i++) {
    const r = new THREE.Mesh(new THREE.RingGeometry(0.92, 1, 64), new THREE.MeshBasicMaterial({ color: COLORS.jam, transparent: true, side: THREE.DoubleSide, depthWrite: false }));
    r.rotation.x = -Math.PI / 2; r.position.y = 38.5; r.visible = false; g.add(r); rings.push(r);
  }

  return {
    group: g, center: new THREE.Vector3(x, y, z),
    radiusNow: 0,
    update(t, jamAge) {
      beacon.material.color.setHex((jamAge != null && Math.sin(t * 10) > 0) ? COLORS.jam : 0x551522);
      if (jamAge == null) { dome.visible = false; rings.forEach((r) => (r.visible = false)); this.radiusNow = 0; return; }
      const k = Math.min(1, jamAge / 2.5);
      this.radiusNow = radius * (1 - Math.pow(1 - k, 3));
      dome.visible = true; dome.scale.setScalar(Math.max(0.001, this.radiusNow));
      rings.forEach((r, i) => {
        const ph = ((t * 0.5 + i / rings.length) % 1);
        r.visible = true; r.scale.setScalar(10 + ph * 220); r.material.opacity = 0.7 * (1 - ph);
      });
    },
  };
}

export function buildGpsLinks(scene) {
  // Four "satellites" high in the sky. Links go to the drone; once jammed they stop at the dome edge.
  const sats = [[-600, 900, -400], [500, 1000, -700], [900, 850, 300], [-300, 950, 700]].map((p) => new THREE.Vector3(...p));
  const lines = sats.map(() => {
    const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineDashedMaterial({ color: COLORS.ok, dashSize: 6, gapSize: 5, transparent: true, opacity: 0.8, fog: false }));
    l.frustumCulled = false; scene.add(l); return l;
  });
  const tmp = new THREE.Vector3(), dir = new THREE.Vector3();
  return {
    update(target, jammer, jammed) {
      sats.forEach((s, i) => {
        const l = lines[i];
        let end = target;
        if (jammed && jammer.radiusNow > 0) {
          // Ray from satellite to drone hits the dome sphere: clip the link there.
          dir.subVectors(target, s); const len = dir.length(); dir.divideScalar(len);
          const oc = tmp.subVectors(s, jammer.center);
          const b = oc.dot(dir), c = oc.lengthSq() - jammer.radiusNow ** 2, disc = b * b - c;
          if (disc > 0) { const tHit = -b - Math.sqrt(disc); if (tHit > 0 && tHit < len) end = s.clone().addScaledVector(dir, tHit); }
        }
        const p = l.geometry.attributes.position;
        p.setXYZ(0, s.x, s.y, s.z); p.setXYZ(1, end.x, end.y, end.z); p.needsUpdate = true;
        l.geometry.computeBoundingSphere(); l.computeLineDistances();
        l.material.color.setHex(jammed ? COLORS.jam : COLORS.ok);
        l.material.opacity = jammed ? 0.45 : 0.75;
      });
    },
  };
}
