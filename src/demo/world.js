import * as THREE from 'three';
import { v3 } from '../core/vec3.js';

export const CHAMBER_HALF = 26;
export const CHAMBER_HEIGHT = 26;
export const PLANET_RADIUS = 40;
export const PLAYER_HEIGHT = 1.5;

const COL = {
  sky: 0x070b18,
  fog: 0x0a1124,
  planet: 0x1d3a22,
  planetDark: 0x14261a,
  panel: 0x2a3550,
  panelAlt: 0x1c2338,
  accent: 0x5cb8ff,
  accentWarm: 0xff7a45,
  hatch: 0x8a93a8,
  white: 0xdfe6f2,
};

export function buildRenderer(canvas) {
  const renderer = new THREE.WebGLRenderer({
    canvas,
    antialias: true,
    powerPreference: 'high-performance',
    preserveDrawingBuffer: true,
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(COL.sky, 1);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  return renderer;
}

function starField(count = 2600, radius = 900) {
  const positions = new Float32Array(count * 3);
  const colors = new Float32Array(count * 3);
  const c = new THREE.Color();
  for (let i = 0; i < count; i += 1) {
    // Deterministic golden-spiral distribution, no RNG.
    const y = 1 - (2 * (i + 0.5)) / count;
    const r = Math.sqrt(Math.max(0, 1 - y * y));
    const phi = i * 2.399963229728653;
    positions[i * 3] = Math.cos(phi) * r * radius;
    positions[i * 3 + 1] = y * radius;
    positions[i * 3 + 2] = Math.sin(phi) * r * radius;
    c.setHSL(0.55 + (i % 13) / 60, 0.35, 0.55 + (i % 7) / 24);
    colors[i * 3] = c.r;
    colors[i * 3 + 1] = c.g;
    colors[i * 3 + 2] = c.b;
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  const mat = new THREE.PointsMaterial({
    size: 2.2, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: 0.9,
  });
  const points = new THREE.Points(geo, mat);
  points.frustumCulled = false;
  return points;
}

/**
 * Real planet geometry: a real sphere with a lat/long grid material, so
 * walking around it visibly rotates the world under a gravity-locked camera.
 */
function buildPlanet(scene, { radius = PLANET_RADIUS } = {}) {
  const group = new THREE.Group();
  group.name = 'planet';

  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(radius, 128, 96),
    new THREE.MeshStandardMaterial({ color: COL.planet, roughness: 0.92, metalness: 0.03 }),
  );
  sphere.castShadow = true;
  sphere.receiveShadow = true;
  group.add(sphere);

  const wire = new THREE.Mesh(
    new THREE.SphereGeometry(radius * 1.003, 48, 32),
    new THREE.MeshBasicMaterial({
      color: 0x3d6b45, wireframe: true, transparent: true, opacity: 0.22, depthWrite: false,
    }),
  );
  group.add(wire);

  // Meridian rings at every 30° of longitude, plus latitude rings: tangible
  // "world frame" evidence that the camera's up truly follows the surface.
  const ringMat = new THREE.LineBasicMaterial({ color: 0x7fd4a3, transparent: true, opacity: 0.38 });
  for (let lon = 0; lon < 180; lon += 30) {
    const a = (lon * Math.PI) / 180;
    const pts = [];
    for (let i = 0; i <= 96; i += 1) {
      const t = (i / 96) * Math.PI * 2;
      pts.push(new THREE.Vector3(Math.cos(t) * Math.cos(a), Math.sin(t), Math.cos(t) * Math.sin(a)).multiplyScalar(radius * 1.012));
    }
    group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), ringMat));
  }
  for (const lat of [-60, -30, 0, 30, 60]) {
    const y = Math.sin((lat * Math.PI) / 180);
    const r = Math.cos((lat * Math.PI) / 180);
    const pts = [];
    for (let i = 0; i <= 128; i += 1) {
      const t = (i / 128) * Math.PI * 2;
      pts.push(new THREE.Vector3(Math.cos(t) * r, y, Math.sin(t) * r).multiplyScalar(radius * 1.012));
    }
    group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), ringMat));
  }

  // Surface markers with real orientation: little pillars stand on the sphere
  // along their own normal, proving the gravity up is computed from geometry.
  const pillarGeo = new THREE.BoxGeometry(0.22, 1.6, 0.22);
  const pillarMat = new THREE.MeshStandardMaterial({ color: 0x9be0b6, emissive: 0x1d5a34, roughness: 0.6 });
  const capMat = new THREE.MeshBasicMaterial({ color: 0xffd479 });
  for (let i = 0; i < 26; i += 1) {
    const y = -0.9 + (1.8 * i) / 25;
    const rr = Math.sqrt(Math.max(0, 1 - y * y));
    const phi = i * 2.399963229728653;
    const normal = new THREE.Vector3(Math.cos(phi) * rr, y, Math.sin(phi) * rr);
    const pillar = new THREE.Mesh(pillarGeo, pillarMat);
    pillar.position.copy(normal).multiplyScalar(radius + 0.8).addScaledVector(normal, -0.55);
    pillar.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), normal);
    pillar.castShadow = true;
    group.add(pillar);
    const cap = new THREE.Mesh(new THREE.SphereGeometry(0.09, 8, 6), capMat);
    cap.position.copy(normal).multiplyScalar(radius + 0.05);
    group.add(cap);
  }

  scene.add(group);
  return group;
}

/**
 * A single "gravity chamber": six real surfaces with different designs
 * (floor, ceiling, four walls, plus a sloped ramp). Each surface has a distinct
 * visual pattern and a normal readout marker, so you can see which way is up
 * after a gravity switch. Walls have thickness and real collision boxes.
 */
function buildChamber(scene) {
  const group = new THREE.Group();
  group.name = 'chamber';
  const H = CHAMBER_HALF;
  const TH = 0.7;

  const wallMat = new THREE.MeshStandardMaterial({ color: COL.panel, roughness: 0.72, metalness: 0.22 });
  const floorMat = new THREE.MeshStandardMaterial({ color: 0x243a4a, roughness: 0.55, metalness: 0.35 });
  const accentMat = new THREE.MeshStandardMaterial({ color: COL.accent, emissive: 0x113c58, roughness: 0.35 });
  const hatchMat = new THREE.MeshStandardMaterial({ color: COL.hatch, roughness: 0.8 });

  const colliders = [];

  const addSlab = (size, position, material, name) => {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(size.x, size.y, size.z), material);
    mesh.position.copy(position);
    mesh.receiveShadow = true;
    mesh.castShadow = true;
    mesh.name = name;
    group.add(mesh);
    colliders.push(mesh);
    return mesh;
  };

  // Floor (gravity +Y) and ceiling (gravity -Y).
  addSlab(new THREE.Vector3(H * 2, TH, H * 2), new THREE.Vector3(0, -TH / 2, 0), floorMat, 'floor');
  addSlab(new THREE.Vector3(H * 2, TH, H * 2), new THREE.Vector3(0, CHAMBER_HEIGHT + TH / 2, 0), hatchMat, 'ceiling');

  // Four walls, each a real vertical surface.
  addSlab(new THREE.Vector3(TH, CHAMBER_HEIGHT, H * 2), new THREE.Vector3(-H - TH / 2, CHAMBER_HEIGHT / 2, 0), wallMat, 'wall-west');
  addSlab(new THREE.Vector3(TH, CHAMBER_HEIGHT, H * 2), new THREE.Vector3(H + TH / 2, CHAMBER_HEIGHT / 2, 0), wallMat, 'wall-east');
  addSlab(new THREE.Vector3(H * 2, CHAMBER_HEIGHT, TH), new THREE.Vector3(0, CHAMBER_HEIGHT / 2, -H - TH / 2), wallMat, 'wall-north');
  addSlab(new THREE.Vector3(H * 2, CHAMBER_HEIGHT, TH), new THREE.Vector3(0, CHAMBER_HEIGHT / 2, H + TH / 2), wallMat, 'wall-south');

  // Floor grid tiles for parallax and a clear horizon reference.
  const tileGeo = new THREE.BoxGeometry(4, 0.02, 4);
  for (let ix = -5; ix < 5; ix += 1) {
    for (let iz = -5; iz < 5; iz += 1) {
      const tile = new THREE.Mesh(tileGeo, (ix + iz) % 2 === 0 ? accentMat : hatchMat);
      tile.position.set(ix * 4 + 2, 0.01, iz * 4 + 2);
      tile.receiveShadow = true;
      group.add(tile);
    }
  }

  // Big emissive direction chevrons on the floor: "these point uphill".
  for (const i of [-1, 0, 1]) {
    const chevron = new THREE.Mesh(
      new THREE.TorusGeometry(6 - Math.abs(i) * 1.5, 0.1, 6, 48, Math.PI * 0.6),
      new THREE.MeshBasicMaterial({ color: COL.accent }),
    );
    chevron.rotation.x = Math.PI / 2;
    chevron.position.set(0, 0.06, i * 4);
    group.add(chevron);
  }

  // A sloped ramp on which gravity up = ramp normal (a non-axis-aligned surface).
  const rampLen = 18;
  const rampAngle = (35 * Math.PI) / 180;
  const ramp = new THREE.Mesh(
    new THREE.BoxGeometry(rampLen, 0.8, 10),
    new THREE.MeshStandardMaterial({ color: 0x3b3030, roughness: 0.9 }),
  );
  ramp.position.set(-14, 3.6, 12);
  ramp.rotation.z = rampAngle;
  ramp.rotation.y = 0;
  ramp.castShadow = true;
  ramp.receiveShadow = true;
  ramp.name = 'ramp';
  group.add(ramp);
  colliders.push(ramp);
  const rampNormal = new THREE.Vector3(-Math.sin(rampAngle), Math.cos(rampAngle), 0);
  const ladder = new THREE.Group();
  const rungMat = new THREE.MeshStandardMaterial({ color: 0xe0a060, roughness: 0.6 });
  for (let i = 0; i < 9; i += 1) {
    const rung = new THREE.Mesh(new THREE.BoxGeometry(9, 0.12, 0.18), rungMat);
    rung.position.copy(rampNormal).multiplyScalar(0.5).add(
      new THREE.Vector3(-Math.sin(rampAngle) * (i * 1.15), Math.cos(rampAngle) * (i * 1.15), 0),
    );
    ladder.add(rung);
  }
  ladder.position.set(-14, 3.6, 12);
  ladder.rotation.z = rampAngle;
  group.add(ladder);

  // Normal markers: short arrows on each surface pointing along its own normal.
  const arrowMat = new THREE.MeshBasicMaterial({ color: 0xffd479 });
  const placeNormalArrow = (pos, normal) => {
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 2.2, 8), arrowMat);
    shaft.position.copy(pos);
    shaft.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), normal);
    group.add(shaft);
    const head = new THREE.Mesh(new THREE.ConeGeometry(0.22, 0.6, 10), arrowMat);
    head.position.copy(pos).addScaledVector(normal, 1.3);
    head.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), normal);
    group.add(head);
  };
  placeNormalArrow(new THREE.Vector3(0, 0.05, 0), new THREE.Vector3(0, 1, 0));
  placeNormalArrow(new THREE.Vector3(0, CHAMBER_HEIGHT - 0.05, 0), new THREE.Vector3(0, -1, 0));
  placeNormalArrow(new THREE.Vector3(-H + 0.05, 3, -8), new THREE.Vector3(1, 0, 0));
  placeNormalArrow(new THREE.Vector3(H - 0.05, 3, 8), new THREE.Vector3(-1, 0, 0));
  placeNormalArrow(new THREE.Vector3(8, 3, -H + 0.05), new THREE.Vector3(0, 0, 1));
  placeNormalArrow(new THREE.Vector3(-8, 3, H - 0.05), new THREE.Vector3(0, 0, -1));
  placeNormalArrow(new THREE.Vector3(-14, 4.4, 12), rampNormal);

  // Wall paneling: recessed frames and emissive seams on every vertical
  // surface, so the room reads as a real 3D box from any gravity orientation.
  const seamMat = new THREE.MeshBasicMaterial({ color: 0x6fc6ff, transparent: true, opacity: 0.65 });
  const frameMat = new THREE.MeshStandardMaterial({ color: 0x394967, roughness: 0.6, metalness: 0.45 });
  const addWallDetail = (axis, sign) => {
    // axis: 'x' means a wall perpendicular to X (east/west), 'z' north/south.
    for (let i = -2; i <= 2; i += 1) {
      for (let j = 0; j < 3; j += 1) {
        const u = i * 5;
        const y = 4.5 + j * 8.5;
        const depth = 0.09;
        let frame;
        if (axis === 'x') {
          frame = new THREE.Mesh(new THREE.BoxGeometry(depth, 6.2, 3.6), frameMat);
          frame.position.set(sign * (H - 0.06), y, u);
        } else {
          frame = new THREE.Mesh(new THREE.BoxGeometry(3.6, 6.2, depth), frameMat);
          frame.position.set(u, y, sign * (H - 0.06));
        }
        group.add(frame);
        let glow;
        if (axis === 'x') {
          glow = new THREE.Mesh(new THREE.BoxGeometry(depth * 1.4, 0.1, 3.1), seamMat);
          glow.position.set(sign * (H - 0.12), y + 3.0, u);
        } else {
          glow = new THREE.Mesh(new THREE.BoxGeometry(3.1, 0.1, depth * 1.4), seamMat);
          glow.position.set(u, y + 3.0, sign * (H - 0.12));
        }
        group.add(glow);
      }
    }
  };
  addWallDetail('x', -1);
  addWallDetail('x', 1);
  addWallDetail('z', -1);
  addWallDetail('z', 1);

  // Ceiling detail: same panel rhythm so the ceiling is as readable as the
  // floor, which matters because it becomes your floor under -Y gravity.
  const ceilFrameMat = new THREE.MeshStandardMaterial({ color: 0x3a4a60, roughness: 0.75, metalness: 0.3 });
  for (let ix = -2; ix <= 2; ix += 1) {
    for (let iz = -2; iz <= 2; iz += 1) {
      const panel = new THREE.Mesh(new THREE.BoxGeometry(7.4, 0.08, 7.4), ceilFrameMat);
      panel.position.set(ix * 8.4, CHAMBER_HEIGHT - 0.06, iz * 8.4);
      group.add(panel);
      if ((ix + iz) % 2 === 0) {
        const strip = new THREE.Mesh(new THREE.BoxGeometry(7.0, 0.04, 0.26), seamMat);
        strip.position.set(ix * 8.4, CHAMBER_HEIGHT - 0.12, iz * 8.4);
        group.add(strip);
      }
    }
  }

  scene.add(group);
  return { group, colliders };
}

function buildPlayer(scene) {
  const group = new THREE.Group();
  group.name = 'player';

  const skin = new THREE.MeshStandardMaterial({ color: 0xf0b48c, roughness: 0.55 });
  const suit = new THREE.MeshStandardMaterial({ color: 0xff6b3d, roughness: 0.42, metalness: 0.25 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x232a38, roughness: 0.7 });
  const visor = new THREE.MeshStandardMaterial({
    color: 0x9fe9ff, emissive: 0x1c6f8c, emissiveIntensity: 0.6, roughness: 0.15, metalness: 0.6,
  });

  const body = new THREE.Mesh(new THREE.CapsuleGeometry(0.34, 0.72, 6, 12), suit);
  body.position.y = 0.95;
  body.castShadow = true;
  group.add(body);

  const head = new THREE.Mesh(new THREE.SphereGeometry(0.27, 20, 16), skin);
  head.position.y = 1.66;
  head.castShadow = true;
  group.add(head);

  const helmet = new THREE.Mesh(new THREE.SphereGeometry(0.31, 20, 16, 0, Math.PI * 2, 0, Math.PI * 0.55), visor);
  helmet.position.y = 1.68;
  helmet.rotation.x = -0.2;
  group.add(helmet);

  // A clear "front" wedge so the character facing is visually distinct from
  // camera roll, and its animation stays attached to the model only.
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.09, 0.3, 8), new THREE.MeshStandardMaterial({ color: 0xffd166 }));
  nose.position.set(0, 1.62, -0.34);
  nose.rotation.x = -Math.PI / 2;
  group.add(nose);

  const pack = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.55, 0.22), dark);
  pack.position.set(0, 1.05, 0.33);
  pack.castShadow = true;
  group.add(pack);

  const bootGeo = new THREE.BoxGeometry(0.22, 0.24, 0.4);
  const bootL = new THREE.Mesh(bootGeo, dark);
  bootL.position.set(-0.15, 0.2, -0.03);
  bootL.castShadow = true;
  group.add(bootL);
  const bootR = new THREE.Mesh(bootGeo, dark);
  bootR.position.set(0.15, 0.2, -0.03);
  bootR.castShadow = true;
  group.add(bootR);

  scene.add(group);
  return {
    group,
    parts: { body, head, bootL, bootR, nose, pack },
  };
}

export function buildWorld(scene) {
  scene.background = new THREE.Color(COL.sky);
  scene.fog = new THREE.FogExp2(COL.fog, 0.0065);

  const stars = starField();
  scene.add(stars);

  const sun = new THREE.DirectionalLight(0xfff1d6, 2.35);
  sun.position.set(46, 74, 34);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = -58;
  sun.shadow.camera.right = 58;
  sun.shadow.camera.top = 58;
  sun.shadow.camera.bottom = -58;
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 260;
  sun.shadow.bias = -0.0007;
  scene.add(sun);
  scene.add(sun.target);

  const hemi = new THREE.HemisphereLight(0x9fc2ff, 0x2a2c1e, 1.35);
  scene.add(hemi);

  // Soft keyboard/document light attached to the view. Guarantees the surface
  // the camera is looking at is readable in every gravity orientation.
  const headlight = new THREE.DirectionalLight(0xbfd8ff, 0.85);
  headlight.position.set(0, 0, 0);
  scene.add(headlight);
  scene.add(headlight.target);

  const fill = new THREE.PointLight(0x66bbff, 34, 110, 1.4);
  fill.position.set(-26, 18, -26);
  scene.add(fill);
  const fill2 = new THREE.PointLight(0xff8a5c, 10, 70, 1.5);
  fill2.position.set(30, 6, 26);
  scene.add(fill2);

  const planet = buildPlanet(scene, { radius: PLANET_RADIUS });
  const chamber = buildChamber(scene);
  const player = buildPlayer(scene);

  // Gravity-aware ambient: intensity follows the dot product between the
  // active local up and the world-space sun, so surfaces brighten and darken
  // as the gravity frame turns. Handled in the main loop via setAmbientForUp.
  const ambient = new THREE.AmbientLight(0x8899cc, 0.55);
  scene.add(ambient);

  return { stars, sun, hemi, headlight, ambient, planet, chamber, player };
}

export function lightLabels() {
  return COL;
}
export { v3 };
