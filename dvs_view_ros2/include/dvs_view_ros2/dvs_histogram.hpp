#pragma once

#include "rclcpp/rclcpp.hpp"
#include "dvs_msgs/msg/event_array.hpp"
#include "dvs_msgs/msg/event.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <vector>
#include <memory>

class DVSHistogram : public rclcpp::Node
{
public:
    DVSHistogram();

private:
    void event_callback(const dvs_msgs::msg::EventArray::SharedPtr msg);
    void publish_frames();

    // Subscribers and publishers
    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_no_polarity_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_with_polarity_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_binary_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_color_polarity_;

    // Frame data
    cv::Mat frame_no_polarity_;
    cv::Mat frame_with_polarity_;
    cv::Mat frame_binary_;
    cv::Mat frame_positive_events_;
    cv::Mat frame_negative_events_;

    rclcpp::Time current_frame_start_time_;
    rclcpp::Duration accumulation_time_{0, 0};
    bool initialized_ = false;

};