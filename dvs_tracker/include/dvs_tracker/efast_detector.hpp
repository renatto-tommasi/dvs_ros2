#pragma once

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "sensor_msgs/msg/image.hpp"

class EFastDetector : public rclcpp::Node
{
public:
    EFastDetector();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr feature_image_pub_;

    int fast_threshold_;
    bool non_max_suppression_;

    cv::Ptr<cv::FastFeatureDetector> fast_detector_;
};
