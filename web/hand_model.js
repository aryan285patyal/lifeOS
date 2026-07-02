// Procedural stylized right hand, built from THREE.BoxGeometry primitives.
// Loaded as a classic script (THREE is a global). Exposes createHandModel(),
// which returns { mesh: <THREE.Group> } so main.js can drop it in place of the
// cube and drive group.quaternion each frame. Pivot sits at the palm center
// (like a sensor mounted on the back of the hand).
window.createHandModel = function () {
  const group = new THREE.Group();
  const skin = new THREE.MeshStandardMaterial({ color: 0xd9a066, metalness: 0.1, roughness: 0.8 });

  function addBox(w, h, d, x, y, z, rotY) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), skin);
    m.position.set(x, y, z);
    if (rotY) m.rotation.y = rotY;
    group.add(m);
    return m;
  }

  // Palm slab, centered at origin; spans z from -0.85 to +0.85.
  addBox(1.6, 0.35, 1.7, 0, 0, 0);

  // Four fingers, each 3 stacked segments pointing +Z from the palm front edge.
  const fingers = [
    { x: -0.6, total: 1.1 },   // index
    { x: -0.2, total: 1.3 },   // middle
    { x:  0.2, total: 1.15 },  // ring
    { x:  0.6, total: 0.9 },   // pinky
  ];
  fingers.forEach(function (finger) {
    const segLen = finger.total / 3;
    let zCursor = 0.85;  // reset at the palm edge for every finger
    for (let i = 0; i < 3; i++) {
      addBox(0.32, 0.3, segLen, finger.x, 0, zCursor + segLen / 2);
      zCursor += segLen + 0.04;
    }
  });

  // Thumb: 2 segments rooted on the +X side near the wrist, angled outward
  // toward +X and forward toward +Z.
  const thumbAngle = 0.7;
  const dirX = Math.sin(thumbAngle);
  const dirZ = Math.cos(thumbAngle);
  const rootX = 0.8, rootZ = -0.2, segLen = 0.5;
  for (let i = 0; i < 2; i++) {
    const along = segLen * (i + 0.5);
    addBox(0.34, 0.32, segLen, rootX + dirX * along, 0, rootZ + dirZ * along, thumbAngle);
  }

  return { mesh: group };
};
