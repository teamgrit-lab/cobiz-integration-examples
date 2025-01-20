const canvasContainer = document.getElementById("canvasContainer");

const textEncoder = new TextEncoder();
let mime, mimeObject;
let videoStreamTrack, videoWriter, stream;
let decoderModule, dracoDecoder, buffer;

const HOST = ""; //Moth Host 정보를 입력하세요
const PORT = ""; //Moth Port 정보를 입력하세요
const CHANNEL_ID = ""; //Moth Channel 정보를 입력하세요
const TRACK = ""; //Moth Track 정보를 입력하세요

// 페이지 로드 시 실행되는 메인 함수
window.onload = async () => {
  // lidar view 움직임을 위한 이벤트 추가
  canvasContainer.addEventListener("pointerdown", onPointerDown);
  canvasContainer.addEventListener("pointermove", onPointerMove);
  canvasContainer.addEventListener("pointerup", onPointerUp);
  canvasContainer.addEventListener("wheel", onWheel);

  // Lidar view 초기화
  const { width, height } = canvasContainer.getBoundingClientRect();
  initLidar(canvasContainer, width, height);

  // draco decoder module 생성
  createDracoDecoderModule();

  connectWebsocket();
};

// websocket으로 moth연결
const connectWebsocket = () => {
  const socket = new WebSocket(
    `wss://${HOST}:${PORT}/pang/ws/sub?channel=${CHANNEL_ID}&track=${TRACK}&mode=bundle`
  );
  socket.binaryType = "arraybuffer";

  socket.onopen = () => {
    console.log("Connected to server");
    // 연결 유지를 위해 10초마다 ping 전송
    setInterval(() => {
      socket.send(textEncoder.encode("ping"));
    }, 10000);
  };

  socket.onmessage = async (event) => {
    if (typeof event.data === "string") {
      // mime 수신
      if (mime === event.data) return; // 기존 MIME과 동일하면 처리 생략

      mime = event.data;
      mimeObject = mimeStringToMimeObject(mime);
    } else if (typeof event.data === "object") {
      // mime 들어오기 전 data 무시 & ping 수신 시 무시
      if (!mimeObject || new TextDecoder().decode(event.data) === "ping")
        return;

      // lidar data 수신 시 draco를 통해 lidar 압축 해제
      const data = processData(event, mimeObject);
      decodeMesh(data);
    }
  };

  socket.onclose = () => {
    console.log("Disconnected from server");
  };

  socket.onerror = (error) => {
    alert(`Error: ${error.message}`);
  };
};

// draco decoder module 생성
const createDracoDecoderModule = async () => {
  const dracoDecoderType = {};

  // Draco decoder module 로드 완료 시 호출
  dracoDecoderType["onModuleLoaded"] = (module) => {
    decoderModule = module;
    dracoDecoder = new decoderModule.Decoder();
    buffer = new decoderModule.DecoderBuffer();
  };

  try {
    // draco decoder module 로드
    DracoDecoderModule(dracoDecoderType);
  } catch (error) {
    console.error("Error loading DracoDecoderModule:", error);
  }
};

// draco를 통해 lidar data 압축 해제
const decodeMesh = (compressedData) => {
  if (!decoderModule) {
    console.error("Decoder module not initialized");
    return;
  }
  if (!dracoDecoder) {
    console.error("Decoder not initialized");
    return;
  }
  if (!buffer) {
    console.error("Buffer not initialized");
    return;
  }

  // 압축 데이터를 Draco 버퍼에 초기화
  buffer.Init(new Int8Array(compressedData), compressedData.byteLength);

  // 포인트 클라우드 디코딩
  const pointCloud = new decoderModule.PointCloud();
  const decodingStatus = dracoDecoder.DecodeBufferToPointCloud(
    buffer,
    pointCloud
  );

  if (decodingStatus.ok() && pointCloud.ptr !== 0) {
    // 디코딩 성공 시 포인트 클라우드 데이터 가져오기
    const positions = getAttributeData(pointCloud, dracoDecoder, 0);
    drawPoints(positions); // 포인트 데이터 시각화
  } else {
    console.error("Decoding failed: ", decodingStatus.error_msg());
  }

  // 사용한 포인트 클라우드 메모리 해제
  decoderModule.destroy(pointCloud);
};

// 포인트 클라우드 데이터 가져오기 (속성데이터)
const getAttributeData = (pointCloud, dracoDecoder, attributeId) => {
  const attribute = dracoDecoder.GetAttribute(pointCloud, attributeId);
  if (!attribute) {
    console.error("Failed to get attribute for ID: ", attributeId);
    return null;
  }

  // 속성 데이터 가져오기
  const numPoints = pointCloud.num_points();
  const numComponents = attribute.num_components();
  const vertices = [];
  const attributeData = new decoderModule.DracoFloat32Array();
  dracoDecoder.GetAttributeFloatForAllPoints(
    pointCloud,
    attribute,
    attributeData
  );

  // 포인트 데이터 배열로 변환
  for (let i = 0; i < numPoints; i++) {
    const index = i * numComponents;
    const x = attributeData.GetValue(index);
    const y = attributeData.GetValue(index + 1);
    const z = attributeData.GetValue(index + 2);
    vertices.push(x, y, z);
  }

  // 속성 데이터 메모리 해제
  decoderModule.destroy(attributeData);
  return vertices;
};

// mime string을 사용하고자하는 mime object로 변환
const mimeStringToMimeObject = (mimeString) => {
  const [mimeType, ...mimeOption] = mimeString.split(";");

  const mimeOptionObj = mimeOption.reduce((acc, option) => {
    const [key, value] = option.trim().split("=");
    acc[key] = value;
    return acc;
  }, {});

  const mimeObject = {
    mimeType: mimeType,
    data_mode: "",
  };

  // 데이터 모드 설정 (데이터 앞에 붙은 정보가 있는지)
  if (mimeType.includes("+")) {
    const mode = mimeType.split("/")[1].split("+")[0];
    mimeObject.data_mode = mode;
  }

  return mimeObject;
};

// 데이터 앞에 붙은 정보가 있는지에 따라 데이터 정리
const processData = (event, mime) => {
  let sliceNumber = 0;

  if (mime.data_mode === "seq") {
    sliceNumber = 1;
  } else if (mime.data_mode === "ts") {
    sliceNumber = 8;
  }
  const data = event.data.slice(sliceNumber);
  return data;
};
