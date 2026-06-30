// lifeOs Visualizer scene.
//
// Renders a labeled cube whose orientation tracks the device quaternion pushed
// from Python over QWebChannel. createModel() is the single seam to swap the
// box for a glTF mannequin head / hand later.

let renderer, scene, camera, controls, model;

// Latest orientation from the device, already remapped into three.js axes.
let lastDevice = new THREE.Quaternion();
// Inverse of the pose captured when "Zero / Level" was pressed (identity = none).
let zeroRef = new THREE.Quaternion();

// BoxGeometry material order is +X, -X, +Y, -Y, +Z, -Z.
const FACES = [
  { label: 'RIGHT',  color: '#c0392b' },  // +X
  { label: 'LEFT',   color: '#e67e22' },  // -X
  { label: 'TOP',    color: '#27ae60' },  // +Y
  { label: 'BOTTOM', color: '#16a085' },  // -Y
  { label: 'FRONT',  color: '#2980b9' },  // +Z
  { label: 'BACK',   color: '#8e44ad' },  // -Z
];

function makeFaceTexture(label, color) {
  const s = 256;
  const cv = document.createElement('canvas');
  cv.width = cv.height = s;
  const ctx = cv.getContext('2d');
  ctx.fillStyle = color;
  ctx.fillRect(0, 0, s, s);
  ctx.strokeStyle = 'rgba(255,255,255,0.5)';
  ctx.lineWidth = 8;
  ctx.strokeRect(8, 8, s - 16, s - 16);
  ctx.fillStyle = '#fff';
  ctx.font = 'bold 44px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, s / 2, s / 2);
  const tex = new THREE.CanvasTexture(cv);
  tex.anisotropy = 4;
  return tex;
}

function createModel() {
  // SWAP POINT: replace with a glTF loader later; keep returning { mesh }.
  const geom = new THREE.BoxGeometry(2, 2, 2);
  const mats = FACES.map(f => new THREE.MeshStandardMaterial({
    map: makeFaceTexture(f.label, f.color),
    metalness: 0.1,
    roughness: 0.8,
  }));
  return { mesh: new THREE.Mesh(geom, mats) };
}

function init() {
  const canvas = document.getElementById('c');
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a1a);

  camera = new THREE.PerspectiveCamera(50, 1, 0.1, 100);
  camera.position.set(4, 3, 5);

  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;

  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const dir = new THREE.DirectionalLight(0xffffff, 0.9);
  dir.position.set(5, 10, 7);
  scene.add(dir);

  scene.add(new THREE.AxesHelper(3));

  model = createModel();
  scene.add(model.mesh);

  window.addEventListener('resize', onResize);
  onResize();
  connectBridge();
  animate();
}

function onResize() {
  const w = window.innerWidth, h = window.innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

// The MPU6050 DMP quaternion (w, x, y, z) is roughly Z-up; three.js is Y-up, so
// swap Y/Z. Signs/order may need tuning against your physical sensor mounting;
// the "Zero / Level" button cancels any constant resting offset on top of this.
function remap(w, x, y, z) {
  return new THREE.Quaternion(x, z, -y, w);
}

function onOrientation(w, x, y, z) {
  lastDevice = remap(w, x, y, z);
}

function connectBridge() {
  if (typeof qt === 'undefined' || !qt.webChannelTransport) {
    return;  // opened outside QWebEngine (e.g. plain browser) -> no live data
  }
  new QWebChannel(qt.webChannelTransport, function (channel) {
    const bridge = channel.objects.bridge;
    bridge.orientation.connect(onOrientation);
    bridge.zeroRequested.connect(function () {
      zeroRef.copy(lastDevice).invert();
    });
  });
}

function animate() {
  requestAnimationFrame(animate);
  // displayed orientation = zeroRef * device
  model.mesh.quaternion.copy(zeroRef).multiply(lastDevice);
  controls.update();
  renderer.render(scene, camera);
}

window.addEventListener('DOMContentLoaded', init);
