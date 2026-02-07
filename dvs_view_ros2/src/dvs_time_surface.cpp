#include "dvs_view_ros2/dvs_time_surface.hpp"

#include "dvs_msgs/msg/event.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <vector>
#include <memory>

DVSTimeSurface::DVSTimeSurface() : Node("dvs_time_surface")
{
    subscription_ = this->create_subscription<dvs_msgs::msg::EventArray>(
        "dvs/events", 10, std::bind(&DVSTimeSurface::event_callback, this, std::placeholders::_1));

    publisher_time_surface_ = this->create_publisher<sensor_msgs::msg::Image>("dvs/time_surface", 10);

    this->declare_parameter<double>("decay_time", 0.05);
    decay_time_constant_ = this->get_parameter("decay_time").as_double();
}

void DVSTimeSurface::event_callback(const dvs_msgs::msg::EventArray::SharedPtr msg)
{
    if (!initialized_ && !msg->events.empty()) {
        time_surface_ = cv::Mat::zeros(msg->height, msg->width, CV_64FC1); // Use double for time values
        initialized_ = true;
    }

    if (!initialized_) return;

    rclcpp::Time latest_time(msg->events.back().ts);

    for (const auto& event : msg->events) {
        rclcpp::Time event_time(event.ts);

        // Store the timestamp of the last event at each pixel
        if (event.x < msg->width && event.y < msg->height) {
            time_surface_.at<double>(event.y, event.x) = event_time.seconds();
        }
    }

    publish_time_surface(latest_time.seconds());
}

void DVSTimeSurface::publish_time_surface(double t_now)
{

    // Compute exp(-(t_now - t_last(x,y)) / tau) for every pixel
    cv::Mat time_diff = t_now - time_surface_;
    cv::Mat decay;
    cv::exp(-time_diff / decay_time_constant_, decay);

    // Convert [0.0, 1.0] range to [0, 255]
    cv::Mat normalized_time_surface;
    decay.convertTo(normalized_time_surface, CV_8UC1, 255.0);

    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = rclcpp::Time(static_cast<uint64_t>(t_now * 1e9));
    cv_image.header.frame_id = "dvs_frame";
    cv_image.encoding = "mono8";
    cv_image.image = normalized_time_surface;
    publisher_time_surface_->publish(*cv_image.toImageMsg());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("DVSTimeSurface"), "Starting DVSTimeSurface node...");
    auto node = std::make_shared<DVSTimeSurface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}