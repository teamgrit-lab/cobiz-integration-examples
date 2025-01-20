#include "video.hpp"

int main(int argc, char * argv[])
{
	gst_init(&argc, &argv);			// gstreamer 사용을 위한 초기화
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Video>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}


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


// Moth와 연결하는 웹소켓 객체를 초기화하는 메소드
bool Video::initWebSocket(){
	try{
		tcp::resolver resolver{ioc};		// HOST, PORT의 값을 ip로 변환하기 위한 resolver 생성
		ws_ptr = std::make_shared<websocket::stream<tcp::socket>>(ioc);	// 웹소켓 포인터 객체 생성

		auto const results = resolver.resolve(host_, port_);		// ip 변환
		auto ep = net::connect(ws_ptr->next_layer(), results);		// ip에 접근

		std::string host = host_ + ':' + std::to_string(ep.port());

		ws_ptr->set_option(websocket::stream_base::decorator(		// HTTP 헤더 설정
			[](websocket::request_type& req)
			{
				req.set(http::field::user_agent,
				std::string(BOOST_BEAST_VERSION_STRING) +
				" websocket-client-coro");
			}));
		ws_ptr->handshake(host, "your_path");					// 웹소켓 PATH 설정

		ws_ptr->write(net::buffer(text));				// MIME 전송 (현재 String)
		ws_ptr->binary(true);							// 이후 데이터 바이너리로 전송

		std::cout << "connect" << std::endl;
		return true;
	} catch (...){
		std::cout << "ERR initWebSocket()" << std::endl;
		return false;
	}
}

/** 
* GStreamer 파이프라인을 생성하고 초기화하는 메소드
* appsrc : gstreamer encoding input
* appsink : gstreamer encoding output
*/ 
void Video::initVideo(){
	// appsrc: 영상 데이터를 직접 넣어주는 source, videoconvert: 비디오 데이터를 원하는 형식으로 변환 (지금은 기본으로), openh264enc: H.264 코덱으로 이미지를 인코딩 하는 하드웨어 인코더, h264parse: NAL Unit 및 SPS/PPS 정보를 담기 위한 파서, appsink: 데이터를 직접 받을 수 있는 sink
	const char* pipeline_cmd = "appsrc name=src do-timestamp=true is-live=true emit-signals=true format=time caps=video/x-raw,format=RGB,width=640,height=480 ! videoconvert ! openh264enc bitrate=1000000 ! video/x-h264, profile=high, alignment=au, stream-format=byte-stream ! h264parse config-interval=1 ! queue leaky=2 ! appsink name=sink sync=false drop=true emit-signals=true max-buffers=3";
	pipeline = gst_parse_launch(pipeline_cmd, NULL);			// 파이프라인을 pipeline_cmd기반으로 생성
	appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");		// src라는 이름의 오브젝트를 가져와 appsrc에 저장
	appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");	// sink라는 이름의 오브젝트를 가져와 appsink에 저장
	g_signal_connect(appsink, "new-sample", G_CALLBACK(appsink_callback), this);	// appsink의 출력을 on_new_sample_static이라는 callback함수에 전달
	gst_element_set_state(pipeline, GST_STATE_PLAYING);			// 파이프라인의 상태를 PLAYING으로 변환, 즉 파이프라인 시작
}

// ROS topic을 통해 메시지를 전달받으면 실행되는 콜백
void Video::video_callback(const sensor_msgs::msg::Image::SharedPtr msg){
	GstBuffer* buffer = gst_buffer_new_allocate(NULL, msg->data.size(), NULL);		// ROS를 통해 들어온 msg 데이터의 size 만큼 GstBuffer 객체 할당
	GstMapInfo map;																	// buffer의 데이터를 매핑시킬 GstMapInfo 객체 생성
	gst_buffer_map(buffer, &map, GST_MAP_WRITE);									// buffer의 주소값을 map의 주소값에 매핑
	std::memcpy(map.data, msg->data.data(), msg->data.size());						// map에 ROS 메세지의 데이터를 size만큼 복사
	GstFlowReturn ret;
	g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);						// buffer의 위치에 저장된 map 데이터를 GStreamer appsrc 파이프라인에 전달
	gst_buffer_unmap(buffer, &map);													// 메모리 할당 해제
	gst_buffer_unref(buffer);														// 메모리 할당 해제
	if (ret != GST_FLOW_OK) {
		g_printerr("Failed to push buffer to appsrc.\n");
	}
}

// appsink로부터 데이터를 전달받을 때, 즉 인코딩 완료된 프레임을 수신할 때 실행되는 코드
GstFlowReturn Video::appsink_callback(GstAppSink *appsink, gpointer user_data){
	Video *self = static_cast<Video *>(user_data);
	return self->send_frame(appsink);
}

GstFlowReturn Video::send_frame(GstAppSink *appsink){
	GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));		// appsink에서 나오는 데이터를 sample 객체에 저장
	if (!sample) {
		std::cerr << "Failed to pull sample" << std::endl;
		return GST_FLOW_ERROR;
	}
	GstBuffer *buffer_ = gst_sample_get_buffer(sample);			// sample에서 GstBuffer 객체를 뽑아 buffer_에 저장
	GstMapInfo map_;											// buffer에서 꺼낼 GstMapInfo라는 매핑 객체 생성
	if (gst_buffer_map(buffer_, &map_, GST_MAP_READ)){			// buffer_의 데이터를 map_에 매핑
		try{
			if (ws_ptr && ws_ptr->is_open() && (ws_ptr->next_layer().available()==0)){
				ws_ptr->write(net::buffer(map_.data, map_.size));			// map 데이터를 웹소켓으로 MOTH 서버에 전송
			}
		} catch (const beast::system_error& se) {
			std::cerr << "Broken pipe error. reconnecting..." << std::endl;
			exit(0);
		} catch (...) {
			std::cout << "unknown Error" << std::endl;
		}
		gst_buffer_unmap(buffer_, &map_);			// 메모리 할당 해제
	}
	gst_sample_unref(sample);						// 메모리 할당 해제
	return GST_FLOW_OK;
}
