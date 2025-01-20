#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <chrono>
#include <sstream>
#include <thread>
#include <fstream>
#include <vector>

#include <draco/compression/encode.h>
#include <draco/compression/decode.h>
#include <draco/mesh/mesh.h>
#include <draco/compression/draco_compression_options.h>
#include <draco/compression/point_cloud/point_cloud_encoder.h>
#include <draco/core/encoder_buffer.h>
#include <draco/core/decoder_buffer.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <string>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

class PointCloudSender : public rclcpp::Node
{
public:
	PointCloudSender();

	void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

	bool initWebSocket();
	void websocket_cancel(){
		boost::system::error_code ec;
		ws_ptr->next_layer().cancel(ec);
		ws_ptr->next_layer().shutdown(tcp::socket::shutdown_both, ec);
		rclcpp::shutdown();
	}

	bool is_timed_out(){return timed_out;}
	void reset_timed_out(){timed_out = false;}

private:
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;

	net::io_context ioc;
	std::shared_ptr<websocket::stream<tcp::socket>> ws_ptr = nullptr;
	std::string host_;
	std::string port_;
	std::string text;
	bool timed_out = false;

};
