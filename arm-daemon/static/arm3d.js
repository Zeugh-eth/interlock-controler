// 3D view of the SO-101 for the dashboard. Draws the URDF chain from GET /kinematics with the
// same joint origins, axes and degree->radian offsets the daemon's FK uses (scripts/kin.py), so
// the on-screen arm matches `tool` in /state. Geometry is the real SO-101 STL meshes from the
// URDF visuals (served at /meshes/, TheRobotStudio/SO-ARM100), placed with the URDF visual
// origins; a schematic (servo blocks + rods) is shown until they load or if they fail. Every
// joint carries a label with angle, load and temperature; the servo that drives a joint is
// coloured by its temperature, the halo at the joint by load. A thin ghost shows the goal pose.
import * as THREE from "three";
import { OrbitControls } from "./OrbitControls.js";
import { STLLoader } from "./STLLoader.js";

const CHAIN = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"];
const LOAD_MAX = 350;            // daemon's load guard (guards.LOAD_MAX)
const TEMP_COOL = 32, TEMP_HOT = 60;   // colour ramp; ST3215 firmware alarm is 70 C
const SERVO = { w: 0.024, h: 0.032, d: 0.046 };

function rpyEuler(rpy) {       // URDF rpy = Rz(y) * Ry(p) * Rx(r)
  return new THREE.Euler(rpy[0], rpy[1], rpy[2], "ZYX");
}

function tempColor(t) {
  if (t == null) return new THREE.Color(0xb7c0ca);
  const u = Math.min(1, Math.max(0, (t - TEMP_COOL) / (TEMP_HOT - TEMP_COOL)));
  // teal -> green -> amber -> red
  const stops = [[0, 0x7fc4ea], [0.45, 0x4aa3df], [0.7, 0xe3a93a], [1, 0xd9534f]];
  for (let i = 1; i < stops.length; i++) {
    if (u <= stops[i][0]) {
      const a = new THREE.Color(stops[i - 1][1]), b = new THREE.Color(stops[i][1]);
      return a.lerp(b, (u - stops[i - 1][0]) / (stops[i][0] - stops[i - 1][0]));
    }
  }
  return new THREE.Color(0xd9534f);
}

function rod(from, to, radius, material) {
  const a = new THREE.Vector3(...from), b = new THREE.Vector3(...to);
  const len = a.distanceTo(b);
  if (len < 1e-4) return new THREE.Group();
  const g = new THREE.CylinderGeometry(radius, radius, len, 14);
  const m = new THREE.Mesh(g, material);
  m.position.copy(a).lerp(b, 0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
  return m;
}

export class Arm3D {
  constructor(container, kin) {
    this.kin = kin;
    this.container = container;
    this.labels = {};
    this.joints = {};          // joint name -> {pivot, body, halo, link}
    this.frames = {};          // link name -> Object3D (post-rotation frame)
    this.schematic = [];       // placeholder geometry, hidden once the STL meshes are in

    const w = container.clientWidth || 600, h = container.clientHeight || 360;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    this.renderer.setSize(w, h);
    container.appendChild(this.renderer.domElement);
    this.leaders = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    this.leaders.setAttribute("class", "arm3d-leaders");
    container.appendChild(this.leaders);
    this.labelLayer = document.createElement("div");
    this.labelLayer.className = "arm3d-labels";
    container.appendChild(this.labelLayer);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(40, w / h, 0.01, 10);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(0.48, -0.42, 0.34);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.08, 0, 0.12);
    this.controls.enableDamping = true;
    this.controls.minDistance = 0.15; this.controls.maxDistance = 2.0;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xc9d1da, 1.0));
    const sun = new THREE.DirectionalLight(0xffffff, 1.2); sun.position.set(0.5, -0.8, 1.2); this.scene.add(sun);

    this.buildWorld();
    this.buildArm();
    this.buildGhost();
    this.loadMeshes().catch(err => console.warn("arm3d: meshes not loaded, keeping schematic", err));

    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(container);
    this.state = null;
    this.animate = this.animate.bind(this);
    requestAnimationFrame(this.animate);
  }

  buildWorld() {
    const tz = this.kin.table_z_m || 0;
    const grid = new THREE.GridHelper(1.0, 20, 0xb9c3cd, 0xdfe5eb);
    grid.rotation.x = Math.PI / 2; grid.position.z = tz;
    this.scene.add(grid);
    const table = new THREE.Mesh(new THREE.PlaneGeometry(1.0, 1.0),
      new THREE.MeshStandardMaterial({ color: 0xf7f9fb, roughness: 1, transparent: true, opacity: 0.95 }));
    table.position.z = tz - 0.0005; this.scene.add(table);
    const axes = new THREE.AxesHelper(0.08); axes.position.z = tz + 0.001; this.scene.add(axes);
    // base plate + base motor block
    const base = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.05, 0.03, 32),
      new THREE.MeshStandardMaterial({ color: 0x6b7785, roughness: 0.7 }));
    base.rotation.x = Math.PI / 2; base.position.set(0.01, 0, 0.015 + tz); this.scene.add(base); this.schematic.push(base);
  }

  buildArm() {
    const linkMat = new THREE.MeshStandardMaterial({ color: 0x8e99a6, roughness: 0.6, metalness: 0.05 });
    let parent = this.scene;
    this.frames.base_link = this.scene;
    const chain = this.kin.chain;
    for (let i = 0; i < chain.length; i++) {
      const c = chain[i];
      const origin = new THREE.Group();            // joint frame before rotation
      origin.position.set(...c.xyz); origin.setRotationFromEuler(rpyEuler(c.rpy));
      parent.add(origin);
      const pivot = new THREE.Group();             // rotates about c.axis
      origin.add(pivot);
      this.frames[c.link] = pivot;
      const next = chain[i + 1];
      if (c.type === "revolute") {
        const body = new THREE.Mesh(new THREE.BoxGeometry(SERVO.w, SERVO.h, SERVO.d),
          new THREE.MeshStandardMaterial({ color: 0x7fc4ea, roughness: 0.5, emissive: 0x000000 }));
        // the servo's horn lies along the joint axis: orient the long side perpendicular to it
        body.rotation.set(0, 0, 0);
        pivot.add(body); this.schematic.push(body);
        const halo = new THREE.Mesh(new THREE.TorusGeometry(0.03, 0.0035, 10, 40),
          new THREE.MeshBasicMaterial({ color: 0xe3a93a, transparent: true, opacity: 0.0 }));
        // torus normal along the joint axis (local z for every SO-101 joint)
        pivot.add(halo);
        this.joints[c.joint] = { pivot, body, halo, meshes: [], axis: new THREE.Vector3(...c.axis).normalize(), link: c.link };
        this.labels[c.joint] = this.makeLabel(c.joint);
      }
      if (next) { const r = rod([0, 0, 0], next.xyz, 0.011, linkMat); pivot.add(r); this.schematic.push(r); }
      parent = pivot;
    }
    // tool frame: between the jaw tips
    const tool = this.frames.gripper_frame_link;
    const tip = new THREE.Mesh(new THREE.SphereGeometry(0.006, 16, 12), new THREE.MeshBasicMaterial({ color: 0x2f8fcf }));
    tool.add(tip); tool.add(new THREE.AxesHelper(0.03));
    this.labels.tool = this.makeLabel("tool", "tool");
    // moving jaw
    const jaw = this.kin.jaw;
    const jo = new THREE.Group(); jo.position.set(...jaw.xyz); jo.setRotationFromEuler(rpyEuler(jaw.rpy));
    this.frames.gripper_link.add(jo);
    const jp = new THREE.Group(); jo.add(jp);
    const fixedTip = new THREE.Vector3(...this.kin.chain[this.kin.chain.length - 1].xyz);
    const jawLen = fixedTip.clone().sub(new THREE.Vector3(...jaw.xyz)).length();
    // the jaw sits in its own frame; its tip direction when closed points toward the fixed tip
    this.jaw = { pivot: jp, axis: new THREE.Vector3(...jaw.axis).normalize(), len: jawLen, fixedTip, frame: jo };
    this.jawMat = new THREE.MeshStandardMaterial({ color: 0x7c8794, roughness: 0.6 });
    this.jawMesh = null;
    this.labels.gripper = this.makeLabel("gripper");
    this.rebuildJaw(0);
    // vertical drop line from the lowest link to the table (lowest_link_z readout)
    this.dropLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineDashedMaterial({ color: 0x6b7785, dashSize: 0.01, gapSize: 0.006 }));
    this.scene.add(this.dropLine);
  }

  rebuildJaw(angle) {
    // Jaw direction is computed in the jaw's own frame so it coincides with the fixed jaw when closed.
    const j = this.jaw;
    if (!this.jawMesh) {
      this.frames.gripper_link.updateWorldMatrix(true, false);
      j.frame.updateWorldMatrix(true, false);
      const tipInJaw = j.frame.worldToLocal(this.frames.gripper_link.localToWorld(j.fixedTip.clone()));
      const m = rod([0, 0, 0], tipInJaw.toArray(), 0.008, this.jawMat);
      j.pivot.add(m); this.jawMesh = m; this.schematic.push(m);
    }
    j.pivot.setRotationFromAxisAngle(j.axis, angle);
  }

  // Real geometry: every <visual> of every URDF link, as STL, under that link's frame. The servo
  // mesh (sts3215) inside a link is the motor that drives the joint whose parent is that link,
  // so it is tinted by that joint's temperature.
  async loadMeshes() {
    const vis = this.kin.visuals; if (!vis) return;
    const loader = new STLLoader();
    const cache = {};
    const load = (f) => cache[f] || (cache[f] = new Promise((res, rej) => loader.load("/meshes/" + f, g => {
      g.computeBoundingBox(); const b = g.boundingBox, size = new THREE.Vector3(); b.getSize(size);
      if (Math.max(size.x, size.y, size.z) > 2) g.scale(0.001, 0.001, 0.001);   // mm export -> m
      g.computeVertexNormals(); res(g);
    }, undefined, rej)));
    const drivenBy = {}; for (const c of this.kin.chain) if (c.type === "revolute") drivenBy[c.parent] = c.joint;
    drivenBy[this.kin.jaw.parent] = "gripper";
    const plastic = new THREE.MeshStandardMaterial({ color: 0xd9dee5, roughness: 0.75, metalness: 0.0 });
    const frameOf = (link) => link === "base_link" ? this.scene : (link === this.kin.jaw.link ? this.jaw.pivot : this.frames[link]);
    const jobs = [];
    this.gripperMeshes = [];
    for (const [link, visuals] of Object.entries(vis)) {
      const frame = frameOf(link); if (!frame) continue;
      for (const v of visuals) {
        jobs.push(load(v.mesh).then(g => {
          const servo = v.mesh.startsWith("sts3215");
          const mat = servo ? new THREE.MeshStandardMaterial({ color: 0x7fc4ea, roughness: 0.55 }) : plastic;
          const m = new THREE.Mesh(g, mat);
          m.position.set(...v.xyz); m.setRotationFromEuler(rpyEuler(v.rpy));
          frame.add(m);
          if (servo) { const j = drivenBy[link]; if (j === "gripper") this.gripperMeshes.push(m); else if (this.joints[j]) this.joints[j].meshes.push(m); }
        }));
      }
    }
    await Promise.all(jobs);
    for (const o of this.schematic) o.visible = false;
    this.meshed = true;
    if (this.state) this.update(this.state);
  }

  buildGhost() {
    const g = new THREE.BufferGeometry().setFromPoints(new Array(this.kin.chain.length + 1).fill(0).map(() => new THREE.Vector3()));
    this.ghost = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0x2f8fcf, transparent: true, opacity: 0.6 }));
    this.scene.add(this.ghost);
  }

  makeLabel(name, cls) {
    const d = document.createElement("div");
    d.className = "arm3d-label" + (cls ? " " + cls : "");
    d.innerHTML = `<b>${name}</b><span></span>`;
    this.labelLayer.appendChild(d);
    return d;
  }

  // Forward kinematics on a plain joint dict (degrees), returning world positions of every chain
  // origin. Used for the ghost (goal) pose without touching the live scene graph.
  fkPoints(q) {
    const pts = [];
    let T = new THREE.Matrix4();
    for (const c of this.kin.chain) {
      const o = new THREE.Matrix4().compose(new THREE.Vector3(...c.xyz), new THREE.Quaternion().setFromEuler(rpyEuler(c.rpy)), new THREE.Vector3(1, 1, 1));
      T = T.clone().multiply(o);
      if (c.type === "revolute") {
        const rad = this.rad(c.joint, q[c.joint] ?? 0);
        const R = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(...c.axis).normalize(), rad);
        T.multiply(R);
      }
      pts.push(new THREE.Vector3().setFromMatrixPosition(T));
    }
    return pts;
  }

  rad(joint, deg) {
    const k = this.kin;
    return THREE.MathUtils.degToRad((k.sign[joint] ?? 1) * (deg + (k.offset_deg[joint] ?? 0)));
  }

  jawAngle(pct) {
    // Approximation: 0 % = jaws touching (URDF angle 0), max % = fully open at the URDF upper limit.
    const up = this.kin.jaw.upper_rad ?? 1.745, maxPct = (this.kin.limits_deg.gripper || [0, 95])[1] || 95;
    return Math.max(0, Math.min(up, (pct / maxPct) * up));
  }

  update(state) {
    this.state = state;
    const s = state.servos || {};
    for (const j of CHAIN) {
      const e = s[j], J = this.joints[j]; if (!e || !J) continue;
      J.pivot.setRotationFromAxisAngle(J.axis, this.rad(j, e.deg));
      J.body.material.color.copy(tempColor(e.temp_c));
      for (const m of J.meshes) { m.material.color.copy(tempColor(e.temp_c)); m.material.transparent = e.torque === false; m.material.opacity = e.torque === false ? 0.45 : 1; }
      const load = Math.min(1, Math.abs(e.load || 0) / LOAD_MAX);
      J.halo.material.opacity = load < 0.05 ? 0 : 0.25 + 0.7 * load;
      J.halo.material.color.set(load > 0.85 ? 0xd9534f : 0xe3a93a);
      J.halo.scale.setScalar(0.8 + 0.6 * load);
      J.body.material.transparent = e.torque === false; J.body.material.opacity = e.torque === false ? 0.45 : 1;
      this.setLabel(this.labels[j], `${e.deg.toFixed(1)}°`, e, j);
    }
    const g = s.gripper;
    if (g) {
      this.rebuildJaw(this.jawAngle(g.pct));
      for (const m of (this.gripperMeshes || [])) m.material.color.copy(tempColor(g.temp_c));
      this.setLabel(this.labels.gripper, `${g.pct.toFixed(0)}% open`, g, "gripper");
    }
    // ghost = goal pose (only when it differs from the present pose)
    const goal = {}; let moving = false;
    for (const j of CHAIN) { const e = s[j]; if (!e) continue; goal[j] = e.goal_deg ?? e.deg; if (Math.abs(goal[j] - e.deg) > 1.5) moving = true; }
    if (moving) {
      const pts = this.fkPoints(goal); const arr = this.ghost.geometry.attributes.position;
      arr.setXYZ(0, 0, 0, 0); pts.forEach((p, i) => arr.setXYZ(i + 1, p.x, p.y, p.z)); arr.needsUpdate = true;
    }
    this.ghost.visible = moving;
    // drop line from the lowest link
    const t = state.tool || {};
    if (t.lowest_link_z_m != null) {
      let low = null, lowZ = Infinity;
      for (const L of this.kin.links) { const f = this.frames[L]; if (!f) continue; const p = new THREE.Vector3(); f.updateWorldMatrix(true, false); f.getWorldPosition(p); if (p.z < lowZ) { lowZ = p.z; low = p; } }
      if (low) {
        const arr = this.dropLine.geometry.attributes.position;
        arr.setXYZ(0, low.x, low.y, low.z); arr.setXYZ(1, low.x, low.y, this.kin.table_z_m); arr.needsUpdate = true;
        this.dropLine.computeLineDistances();
        this.dropLine.material.color.set(t.lowest_link_z_m < 0.04 ? 0xd9534f : 0x6b7785);
      }
    }
    const xyz = (t.xyz_m || []).map(v => (v * 100).toFixed(1)).join("/");
    this.labels.tool.querySelector("span").textContent = `${xyz} cm · tilt ${t.tilt_from_down_deg ?? "?"}°`;
  }

  setLabel(el, pos, e, j) {
    const temp = e.temp_c == null ? "?" : e.temp_c;
    el.querySelector("span").textContent = `${pos} · load ${e.load} · ${temp}°C${e.torque === false ? " · limp" : ""}`;
    el.style.borderColor = "#" + tempColor(e.temp_c).getHexString();
    el.classList.toggle("hot", Math.abs(e.load || 0) > LOAD_MAX * 0.85 || (e.temp_c != null && e.temp_c >= 55));
  }

  placeLabels() {
    // Project every joint, then push overlapping labels apart vertically and draw a leader line
    // from each label to its joint so the HUD stays readable at any camera angle.
    const w = this.renderer.domElement.clientWidth, h = this.renderer.domElement.clientHeight;
    const items = [];
    const proj = (obj) => { const p = new THREE.Vector3(); obj.getWorldPosition(p); p.project(this.camera); return { x: (p.x + 1) / 2 * w, y: (1 - p.y) / 2 * h, z: p.z }; };
    for (const j of CHAIN) if (this.joints[j]) items.push({ el: this.labels[j], p: proj(this.joints[j].pivot) });
    items.push({ el: this.labels.gripper, p: proj(this.jaw.pivot) });
    items.push({ el: this.labels.tool, p: proj(this.frames.gripper_frame_link) });
    // HUD column down the right edge, ordered like the joints on screen (top to bottom)
    items.sort((a, b) => a.p.y - b.p.y);
    const gap = 34, top = 10, lx = w - 10;
    let y = Math.max(top, Math.min(h - gap * items.length - 6, items[0].p.y - 12));
    let svg = "";
    for (const it of items) {
      it.el.style.left = lx + "px"; it.el.style.top = y + "px";
      const vis = it.p.z < 1 && it.p.x > -50 && it.p.x < w + 50;
      it.el.style.display = vis ? "block" : "none";
      const lw = it.el.offsetWidth || 150;
      if (vis) svg += `<line x1="${it.p.x}" y1="${it.p.y}" x2="${lx - lw}" y2="${y + 14}"/><circle cx="${it.p.x}" cy="${it.p.y}" r="2.5"/>`;
      y += gap;
    }
    this.leaders.setAttribute("viewBox", `0 0 ${w} ${h}`); this.leaders.innerHTML = svg;
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight; if (!w || !h) return;
    this.renderer.setSize(w, h); this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }

  animate() {
    requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.placeLabels();
  }
}

export async function mountArm3D(container) {
  const kin = await (await fetch("/kinematics")).json();
  return new Arm3D(container, kin);
}
