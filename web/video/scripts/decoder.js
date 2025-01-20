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
