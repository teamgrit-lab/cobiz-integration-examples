let camera, scene, renderer, controls;
let positionMesh; // 현재 위치를 나타내는 Mesh
let pointMesh; // 포인트 클라우드를 렌더링하는 Mesh
let isDragging = false; // 드래그 상태 플래그
let previousMousePosition = { x: 0, y: 0 }; // 이전 마우스 위치
const SPHERICAL = {
  radius: 10,
  theta: 0,
  phi: Math.PI / 2,
};
// 높이별 point 색상
const COLORS = [
  { limit: -0.2, color: new THREE.Color(0x000000) },
  { limit: 0.0, color: new THREE.Color(0x1133ff) },
  { limit: 0.2, color: new THREE.Color(0x5cddff) },
  { limit: 0.4, color: new THREE.Color(0x5cffd5) },
  { limit: 0.6, color: new THREE.Color(0x00ff00) },
  { limit: 0.8, color: new THREE.Color(0xffff00) },
  { limit: 1.0, color: new THREE.Color(0xff8000) },
  { limit: 5.0, color: new THREE.Color(0xff0000) },
];

// LiDAR View 초기화 함수
const initLidar = (canvasContainer, width, height) => {
  // Three.js 씬 생성
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xeeeeee);

  // 조명 추가
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
  scene.add(ambientLight);

  // 카메라 생성 및 초기 위치 설정
  camera = new THREE.PerspectiveCamera();
  camera.position.set(0, 0, 10);

  // WebGL 렌더러 생성
  renderer = new THREE.WebGLRenderer();
  renderer.setSize(width, height);
  setTimeout(() => {
    canvasContainer.appendChild(renderer.domElement); // 렌더러 캔버스를 화면에 추가
  }, 100);

  // 현재 위치를 나타내는 Mesh 생성
  const geometry = new THREE.BufferGeometry();
  const vertices = new Float32Array([0, 0.2, 0, 0, -0.2, 0, 0.4, 0, 0]);
  geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
  const indices = [0, 1, 2];
  geometry.setIndex(indices);

  const material = new THREE.MeshBasicMaterial({
    color: 0xff0000,
    side: THREE.DoubleSide,
  });
  positionMesh = new THREE.Mesh(geometry, material);
  scene.add(positionMesh);

  // 캔버스 크기 변경에 대응
  onCanvasResize(width, height);
  camera.rotateOnAxis(new THREE.Vector3(0, 0, 1), -Math.PI / 2);

  animate();
};

// 캔버스 크기 조정 함수
const onCanvasResize = (width, height) => {
  renderer.setSize(width, height);
  renderer.setPixelRatio(window.devicePixelRatio);

  camera.aspect = width / height;
  camera.updateProjectionMatrix();
};

// 마우스 드래그 시작 처리
const onPointerDown = (event) => {
  isDragging = true;
  previousMousePosition.x = event.clientX;
  previousMousePosition.y = event.clientY;
};

// 마우스 이동 처리
const onPointerMove = (event) => {
  if (!isDragging) return;

  // 마우스 이동량 계산
  const deltaX = event.clientX - previousMousePosition.x;
  const deltaY = event.clientY - previousMousePosition.y;

  // 카메라 각도 갱신
  SPHERICAL.theta -= deltaX * 0.005;
  SPHERICAL.phi -= deltaY * 0.005;
  SPHERICAL.phi = Math.max(0.1, Math.min(Math.PI - 0.1, SPHERICAL.phi));

  // 카메라 위치 갱신
  updateCameraPosition();

  previousMousePosition.x = event.clientX;
  previousMousePosition.y = event.clientY;
};

// 마우스 드래그 종료 처리
const onPointerUp = () => {
  isDragging = false;
};

// 마우스 휠 처리 (줌 인/아웃)
const onWheel = (event) => {
  SPHERICAL.radius += event.deltaY * 0.01; // 카메라 반지름 변경
  SPHERICAL.radius = Math.max(2, Math.min(20, SPHERICAL.radius)); // 반지름 제한
  updateCameraPosition(); // 카메라 위치 갱신
};

// 카메라 위치 업데이트
const updateCameraPosition = () => {
  const x =
    SPHERICAL.radius * Math.sin(SPHERICAL.phi) * Math.sin(SPHERICAL.theta);
  const y = SPHERICAL.radius * Math.cos(SPHERICAL.phi);
  const z =
    SPHERICAL.radius * Math.sin(SPHERICAL.phi) * Math.cos(SPHERICAL.theta);

  camera.position.set(x, y, z);
  camera.lookAt(0, 0, 0);
  camera.rotation.z -= Math.PI / 2;
};

// 포인트 클라우드 렌더링 함수
const drawPoints = (points) => {
  const vertices = new Float32Array(points);

  // 기존 포인트 클라우드 제거
  if (pointMesh && scene) {
    scene.remove(pointMesh);
    if (pointMesh.geometry) {
      pointMesh.geometry.dispose();
    }
    if (Array.isArray(points.material)) {
      pointMesh.material.forEach((material) => material.dispose());
    } else if (points.material) {
      pointMesh.material.dispose();
    }
  }

  // 새로운 포인트 클라우드 생성
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));

  const colors = new Float32Array(vertices.length);
  for (let i = 0; i < vertices.length; i += 3) {
    const z = vertices[i + 2];
    const color = getColorByZ(z);
    colors[i] = color.r;
    colors[i + 1] = color.g;
    colors[i + 2] = color.b;
  }
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));

  const material = new THREE.PointsMaterial({
    vertexColors: true,
    size: 0.1,
  });
  pointMesh = new THREE.Points(geometry, material);

  if (pointMesh && scene) scene.add(pointMesh); // 씬에 포인트 클라우드 추가

  animate();
};

// 높이(Z) 값에 따라 색상을 가져오는 함수
const getColorByZ = (z) => {
  for (let i = 0; i < COLORS.length; i++) {
    if (z <= COLORS[i].limit) {
      return COLORS[i].color;
    }
  }
  return new THREE.Color(0xffffff); // 기본 색상
};

// scene을 렌더링하는 함수 (화면 업데이트)
const animate = () => {
  renderer.render(scene, camera);
};
