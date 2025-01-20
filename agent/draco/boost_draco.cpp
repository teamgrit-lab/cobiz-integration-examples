#include "boost_draco.hpp"


PointCloudSender::PointCloudSender() : Node("pointcloud_sender")
{
	host_ = "your_host";	// TeamGRIT에서 제공하는 HOST 작성
	port_ = "your_port";	// TeamGRIT에서 제공하는 PORT 작성
	text = "lidar/draco";	// MIME 데이터

	while (!initWebSocket()){
		std::cout << "connecting..." << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	RCLCPP_INFO(this->get_logger(), "Node has been started.");
	subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/your/pointcloud2/topic", 10, std::bind(&PointCloudSender::topic_callback, this, std::placeholders::_1));
}

void PointCloudSender::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
	draco::PointCloudBuilder pc_builder;									// draco 압축을 위한 builder 객체 생성
	const int32_t num_points = msg->width * msg->height;
	pc_builder.Start(num_points);											// builder에 원본 데이터 size 초기화
	int pos_att_id = pc_builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);		// 속성 유형 설정 (위치), (x, y, z), (데이터 타입) 등

	std::vector<float> point_data(num_points * 3);							// 원본 데이터를 담을 float vector 생성
	for (int i = 0; i < num_points; ++i) {									// 원본 데이터 복사
		float x, y, z;
		memcpy(&x, &msg->data[i * msg->point_step + msg->fields[0].offset], sizeof(float));
		memcpy(&y, &msg->data[i * msg->point_step + msg->fields[1].offset], sizeof(float));
		memcpy(&z, &msg->data[i * msg->point_step + msg->fields[2].offset], sizeof(float));

		point_data[i * 3 + 0] = x;
		point_data[i * 3 + 1] = y;
		point_data[i * 3 + 2] = z;
	}

	pc_builder.SetAttributeValuesForAllPoints(pos_att_id, point_data.data(), sizeof(float)*3);				// builder에 데이터 전달
	std::unique_ptr<draco::PointCloud> pc = pc_builder.Finalize(false);										// pointcloud 데이터 생성

	draco::Encoder encoder;
	encoder.SetSpeedOptions(10, 10);																		// 인/디코딩 속도 설정 0~10, 10이 가장 빠름
	encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 10);								// 데이터 정밀도를 10으로 줄여 압축률을 높임 (1~30)
	draco::EncoderBuffer buffer;
	encoder.EncodePointCloudToBuffer(*pc, &buffer);															// 인코딩

	try{
		if (ws_ptr && ws_ptr->is_open() && (ws_ptr->next_layer().available()==0)){
			ws_ptr->write(net::buffer(buffer.data(), buffer.size()));										// buffer에 저장된 pointcloud 데이터를 웹소켓을 통해 MOTh로 전송
		}
	} catch (const beast::system_error& se) {
		std::cerr << "Broken pipe error. reconnecting..." << std::endl;
		exit(0);
	} catch (std::exception const& e) {
		std::cerr << "Failed to send message: " << e.what() << std::endl;
	}



}

bool PointCloudSender::initWebSocket(){
	try{
		tcp::resolver resolver{ioc};										// HOST, PORT의 값을 ip로 변환하기 위한 resolver 생성
		ws_ptr = std::make_shared<websocket::stream<tcp::socket>>(ioc);		// 웹소켓 포인터 객체 생성

		auto const results = resolver.resolve(host_, port_);				// ip 변환
		auto ep = net::connect(ws_ptr->next_layer(), results);				// ip에 접근

		std::string host = host_ + ':' + std::to_string(ep.port());

		ws_ptr->set_option(websocket::stream_base::decorator(				// HTTP 헤더 설정
			[](websocket::request_type& req)
			{
				req.set(http::field::user_agent,
				std::string(BOOST_BEAST_VERSION_STRING) +
				" websocket-client-coro");
			}));
		ws_ptr->handshake(host, "your path");								// TeamGRIT이 제공하는 path 사용

		ws_ptr->write(net::buffer(text));									// MIME 데이터 전송 (String)
		ws_ptr->binary(true);												// 이후 전송되는 데이터 Binary로 설정

		std::cout << "connect" << std::endl;
		return true;

	}catch (...){
		std::cout << "err initWebSocket()" << std::endl;
		return false;
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<PointCloudSender>();

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
