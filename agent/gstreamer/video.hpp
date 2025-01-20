#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/app.h>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <string>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

class Video : public rclcpp::Node
{
public:
	Video();

	void video_callback(const sensor_msgs::msg::Image::SharedPtr msg);

	void initVideo();

	bool initWebSocket();
	
	void websocket_cancel(){
		boost::system::error_code ec;
		ws_ptr->next_layer().cancel(ec);
		ws_ptr->next_layer().shutdown(tcp::socket::shutdown_both, ec);
		rclcpp::shutdown();
	}
	bool is_timed_out(){return timed_out;}
	void reset_timed_out(){timed_out = false;}

	static GstFlowReturn appsink_callback(GstAppSink *appsink, gpointer user_data);
	GstFlowReturn send_frame(GstAppSink *appsink);
private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

	net::io_context ioc;
	std::shared_ptr<websocket::stream<tcp::socket>> ws_ptr = nullptr;
	std::string host_;
	std::string port_;
	std::string text;
	bool timed_out = false;
	int index = 0;

	GstElement* pipeline;
	GstElement* appsrc;
	GstElement* appsink;
};
