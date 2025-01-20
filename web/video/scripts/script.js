const videoElement = document.getElementById("videoElement");

const textEncoder = new TextEncoder();
let mime, mimeObject;
let videoStreamTrack, videoWriter;

const HOST = ""; //Moth Host 정보를 입력하세요
const PORT = ""; //Moth Port 정보를 입력하세요
const CHANNEL_ID = ""; //Moth Channel 정보를 입력하세요
const TRACK = ""; //Moth Track 정보를 입력하세요

// 페이지 로드 시 실행되는 메인 함수
window.onload = async () => {
  videoStreamTrack = new MediaStreamTrackGenerator({
    kind: "video",
  });
  videoWriter = videoStreamTrack.writable.getWriter();
  await videoWriter.ready;

  const mediaStream = new MediaStream([videoStreamTrack]);
  videoElement.srcObject = mediaStream;

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
      // mime 수신 시 video decoder 설정
      if (mime === event.data) return; // 기존 MIME과 동일하면 처리 생략

      mime = event.data;
      mimeObject = mimeStringToMimeObject(mime);

      // decode된 video frame을 video stream에 write
      const handleVideoFrame = async (frame) => {
        console.log("handleVideoFrame");
        if (frame && videoStreamTrack) {
          videoWriter.write(frame);
          frame.close();
        }
      };

      await setDecoder(mimeObject, handleVideoFrame);
    } else if (typeof event.data === "object") {
      // mime 들어오기 전 data 무시 & ping 수신 시 무시
      if (!mimeObject || new TextDecoder().decode(event.data) === "ping") {
        return;
      }
      // video data 수신 시 video decode
      await startDecode(processVideoData(event, mimeObject));
    }
  };

  socket.onclose = () => {
    console.log("Disconnected from server");
  };

  socket.onerror = (error) => {
    alert(`Error: ${error.message}`);
  };
};

// mime string을 사용하고자하는 mime object로 변환
const mimeStringToMimeObject = (mimeString) => {
  const [mimeType, ...mimeOption] = mimeString.split(";");

  const mimeOptionObj = mimeOption.reduce((acc, option) => {
    const [key, value] = option.trim().split("=");
    acc[key] = value;
    return acc;
  }, {});

  if (!mimeOptionObj.codec) mimeOptionObj.codec = mimeOptionObj.codecs;

  // 데이터 모드 설정 (데이터 앞에 붙은 정보가 있는지)
  if (mimeType.includes("+")) {
    const mode = mimeType.split("/")[1].split("+")[0];
    mimeOptionObj.data_mode = mode;
  }
  const type = mimeType.split("/")[0];
  mimeOptionObj.data_type = type;

  return mimeOptionObj;
};

// video data를 decode하기 위해 데이터 정리
const processVideoData = (event, mime) => {
  let sliceNumber = 0;

  if (mime.data_mode === "seq") {
    sliceNumber = 1;
  } else if (mime.data_mode === "ts") {
    sliceNumber = 8;
  }

  const data = new Uint8Array(event.data.slice(sliceNumber));

  // 이미지(JPEG) 데이터의 경우 객체로 반환
  if (mime.data_type === "image") {
    return {
      data,
      timestamp: event.timeStamp,
    };
  }

  // 비디오 데이터의 경우 EncodedVideoChunk 객체로 변환
  const encodedChunk = new EncodedVideoChunk({
    type: data.length > 100000 ? "key" : "delta",
    data: data,
    timestamp: event.timeStamp,
    duration: 0,
  });

  return encodedChunk;
};
