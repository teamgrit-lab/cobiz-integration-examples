# Moth 연동 예제

## 로봇 연동 예제

ROS 토픽을 구독하여 영상 프레임을 수신하고, GStreamer로 h.264 코덱으로 인코딩하여 Moth로 전달하는 예제입니다.

### 초기화

#### 생성자

이번 예제에서 사용될 `Video`객체의 생성자입니다. 연동을 위해 다음 변수 값 설정이 필요합니다.

- `host_` : Moth 서버의 주소
- `port_` : Moth 서버의 포트 번호
- `ROS TOPIC(/your/image_raw/topic)` : 로봇에서 영상 프레임을 전송하는 ROS 토픽


```cpp
Video::Video() : Node("video")
{
  host_ = "your_host";
  port_ = "your_port";
  text = "video/h264;width=640;height=480;framerate=30;codecs=avc1.42002A";

  while (!initWebSocket()){
    std::cout << "reconnecting..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  initVideo();

  RCLCPP_INFO(this->get_logger(), "Node has been started.");
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/your/image_raw/topic", 10, std::bind(&Video::video_callback, this, std::placeholders::_1));
}
```

#### Websocket 초기화

Moth 서버로 연결하기 위한 웹소켓 초기화 코드입니다. 다음 값을 설정해 주어야 합니다.

- `"your_path"` : Moth에 웹소켓으로 연결하기 위한 도메인 이하 Path값을 설정합니다. 자세한 내용은 [Moth 서버의 API 문서](https://docs-cobiz.cobot.center/docs/protocol/api-service)를 참고하세요.

```cpp
bool Video::initWebSocket(){
  try{
    tcp::resolver resolver{ioc}; // HOST, PORT의 값을 ip로 변환하기 위한 resolver 생성
    ws_ptr = std::make_shared<websocket::stream<tcp::socket>>(ioc); // 웹소켓 포인터 객체 생성

    auto const results = resolver.resolve(host_, port_); // ip 변환
    auto ep = net::connect(ws_ptr->next_layer(), results); // ip에 접근

    std::string host = host_ + ':' + std::to_string(ep.port());

    ws_ptr->set_option(websocket::stream_base::decorator( // HTTP 헤더 설정
      [](websocket::request_type& req)
      {
        req.set(http::field::user_agent,
        std::string(BOOST_BEAST_VERSION_STRING) +
        " websocket-client-coro");
      }));
    ws_ptr->handshake(host, "your_path"); // 웹소켓 PATH 설정

    ws_ptr->write(net::buffer(text)); // MIME 전송 (현재 String)
    ws_ptr->binary(true); // 이후 데이터 바이너리로 전송

    std::cout << "connect" << std::endl;
    return true;
  } catch (...){
    std::cout << "ERR initWebSocket()" << std::endl;
    return false;
  }
}
```

#### GStreamer 초기화

영상 인코딩을 위해 GStreamer를 초기화하는 코드입니다. `appsrc`파이프라인으로 프레임을 전달하면 GStreamer에서 인코딩합니다. 인코딩이 완료된 프레임은 `appsink`로 전달되고, `appsink_callback()`함수에서 프레임을 서버로 전송합니다.

```cpp
void Video::initVideo(){
  // appsrc: 영상 데이터를 직접 넣어주는 source, videoconvert: 비디오 데이터를 원하는 형식으로 변환 (지금은 기본으로), openh264enc: H.264 코덱으로 이미지를 인코딩 하는 하드웨어 인코더, h264parse: NAL Unit 및 SPS/PPS 정보를 담기 위한 파서, appsink: 데이터를 직접 받을 수 있는 sink
  const char* pipeline_cmd = "appsrc name=src do-timestamp=true is-live=true emit-signals=true format=time caps=video/x-raw,format=RGB,width=640,height=480 ! videoconvert ! openh264enc bitrate=1000000 ! video/x-h264, profile=high, alignment=au, stream-format=byte-stream ! h264parse config-interval=1 ! queue leaky=2 ! appsink name=sink sync=false drop=true emit-signals=true max-buffers=3";
  pipeline = gst_parse_launch(pipeline_cmd, NULL); // 파이프라인을 pipeline_cmd기반으로 생성
  appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src"); // src라는 이름의 오브젝트를 가져와 appsrc에 저장
  appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink"); // sink라는 이름의 오브젝트를 가져와 appsink에 저장
  g_signal_connect(appsink, "new-sample", G_CALLBACK(appsink_callback), this); // appsink의 출력을 on_new_sample_static이라는 callback함수에 전달
  gst_element_set_state(pipeline, GST_STATE_PLAYING); // 파이프라인의 상태를 PLAYING으로 변환, 즉 파이프라인 시작
}
```

### ROS 프레임 수신 - Video::video_callback()

ROS 토픽을 구독하여 프레임을 수신하는 콜백함수입니다. 수신된 프레임을 GStreamer appsrc 파이프라인에 전달합니다.

```cpp
void Video::video_callback(const sensor_msgs::msg::Image::SharedPtr msg){
  GstBuffer* buffer = gst_buffer_new_allocate(NULL, msg->data.size(), NULL); // ROS를 통해 들어온 msg 데이터의 size 만큼 GstBuffer 객체 할당
  GstMapInfo map; // buffer의 데이터를 매핑시킬 GstMapInfo 객체 생성
  gst_buffer_map(buffer, &map, GST_MAP_WRITE); // buffer의 주소값을 map의 주소값에 매핑
  std::memcpy(map.data, msg->data.data(), msg->data.size()); / map에 ROS 메세지의 데이터를 size만큼 복사
  GstFlowReturn ret;
  g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret); // buffer의 위치에 저장된 map 데이터를 GStreamer appsrc 파이프라인에 전달
  gst_buffer_unmap(buffer, &map);
  gst_buffer_unref(buffer);
  if (ret != GST_FLOW_OK) {
    g_printerr("Failed to push buffer to appsrc.\n");
  }
}
```

### GStreamer 프레임 인코딩 - Video::appsink_callback()
GStreamer에서 인코딩된 프레임을 Moth 서버로 전송하는 콜백함수입니다.

```cpp
GstFlowReturn Video::appsink_callback(GstAppSink *appsink, gpointer user_data){
  Video *self = static_cast<Video *>(user_data);
  return self->send_frame(appsink);
}

GstFlowReturn Video::send_frame(GstAppSink *appsink){
  GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink)); // appsink에서 나오는 데이터를 sample 객체에 저장
  if (!sample) {
    std::cerr << "Failed to pull sample" << std::endl;
    return GST_FLOW_ERROR;
  }
  GstBuffer *buffer_ = gst_sample_get_buffer(sample); // sample에서 GstBuffer 객체를 뽑아 buffer_에 저장
  GstMapInfo map_; // buffer에서 꺼낼 GstMapInfo라는 매핑 객체 생성
  if (gst_buffer_map(buffer_, &map_, GST_MAP_READ)){ // buffer_의 데이터를 map_에 매핑
    try{
      if (ws_ptr && ws_ptr->is_open() && (ws_ptr->next_layer().available()==0)){
        ws_ptr->write(net::buffer(map_.data, map_.size)); // map 데이터를 웹소켓으로 MOTH 서버에 전송
      }
    } catch (const beast::system_error& se) {
      std::cerr << "Broken pipe error. reconnecting..." << std::endl;
      exit(0);
    } catch (...) {
      std::cout << "unknown Error" << std::endl;
    }
    gst_buffer_unmap(buffer_, &map_);
  }
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}
```

## 웹 연동 예제

웹소켓을 통해 h.264로 인코딩된 비디오프레임을 수신하여 디코딩하고 화면에 표시하는 예제입니다.

### script.js

웹소켓을 통해 Moth와 연결하고, 수신된 데이터를 디코딩하여 화면에 표시하는 스크립트입니다.

```javascript
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
```

### decoder.js

웹코덱을 이용해 코덱을 디코딩하는 보조 함수입니다.

```javascript
let videoDecoder = null;
let decoderConfig = null;
let useWebCodec = true;
let handleVideoFrame = null;

// decoder 초기 설정
const setDecoder = async (config, handle) => {
  handleVideoFrame = handle;
  decoderConfig = config;

  // image codec을 사용하는지 확인
  if (decoderConfig.data_type === "image") {
    useWebCodec = false;
  } else {
    useWebCodec = true;

    // video codec을 사용하는 경우 decoder 설정이 유효한지 확인
    if (await isConfigSupported(decoderConfig).supported) {
      return;
    }

    videoDecoder = new VideoDecoder({
      // decode 후 나온 frame 처리
      output: async (frame) => {
        handleFrame(frame);
      },
      // decoder error 처리
      error: (error) => {
        console.error("Decoder error:", error);
        decoder.close();
      },
    });

    // decoder 설정
    videoDecoder.configure(decoderConfig);
    console.log("Decoder", videoDecoder);
  }
};

const isConfigSupported = async (config) => {
  return await VideoDecoder.isConfigSupported(config);
};

const handleFrame = async (frame) => {
  handleVideoFrame(frame);
  frame.close();
};

const startDecode = async (encodedVideoChunk) => {
  if (useWebCodec && typeof encodedVideoChunk === "object") {
    // video codec을 사용하는 경우 decoder로 decode
    try {
      if (!videoDecoder) return;

      if (videoDecoder.state === "closed") {
        await setDecoder(decoderConfig, handleVideoFrame);
      } else {
        videoDecoder.decode(encodedVideoChunk);
      }
    } catch (error) {
      // decoder error 발생 시
      console.error("Decoder error:", error);
      await setDecoder(decoderConfig, handleVideoFrame);
    }
  } else {
    // image codec을 사용하는 경우 blob을 imageBitmap으로 변환 후 frame 처리
    const videoChunk = encodedVideoChunk;
    const blob = new Blob([videoChunk.data], { type: "image/jpeg" });

    createImageBitmap(blob)
      .then((imageBitmap) => {
        const decodedChunk = new VideoFrame(imageBitmap, {
          timestamp: videoChunk.timestamp,
        });
        handleVideoFrame(decodedChunk);
      })
      .catch((error) => {
        console.log("ImageBitmap creation error:", error);
      });
  }
};
```

### index.html
```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Video | CoBiz</title>
  <style>
    body {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }
    video {
      width: 100%;
      height: 100%;
      background-color: black;
    }
  </style>
</head>
<body>
  <video id="videoElement" autoplay muted controls ></video>
  <script src="scripts/decoder.js"></script>
  <script src="scripts/script.js"></script>
</body>
</html>
```
