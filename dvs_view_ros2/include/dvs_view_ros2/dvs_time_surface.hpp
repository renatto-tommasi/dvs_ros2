#pragma once

#include "rclcpp/rclcpp.hpp"
#include "dvs_msgs/msg/event_array.hpp"
#include "dvs_msgs/msg/event.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <vector>
#include <memory>

class DVSTimeSurface : public rclcpp::Node
{
public:
    DVSTimeSurface();
private:

    void event_callback(const dvs_msgs::msg::EventArray::SharedPtr msg);
    
    void publish_time_surface(double t_now);
    // Subscribers and publishers
    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_time_surface_;

    // Time surface data
    cv::Mat time_surface_;
    double decay_time_constant_; // in seconds
    bool initialized_ = false;
};