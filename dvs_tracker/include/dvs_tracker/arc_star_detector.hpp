#pragma once

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "dvs_msgs/msg/event.hpp"
#include "dvs_msgs/msg/event_array.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <vector>

class DVSCornerDetector : public rclcpp::Node
{
public:
    DVSCornerDetector();

private:
    void event_callback(const dvs_msgs::msg::EventArray::SharedPtr msg);
    cv::Mat compute_decay_image(double t_now);
    void publish_corner_image(double t_now);
    bool is_corner(const dvs_msgs::msg::Event& event, double t);
    bool is_suppressed(int ex, int ey, double t);

    // Arc test on a single circle: greedy bidirectional expansion
    // Returns the newest segment size
    int arc_test(int ex, int ey, int pol,
                 const int circle[][2], int circle_size);

    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr corner_image_pub_;
    rclcpp::Publisher<dvs_msgs::msg::EventArray>::SharedPtr corner_event_pub_;

    double tau_;
    double filter_threshold_;
    double nms_threshold_;

    int sensor_width_;
    int sensor_height_;

    // Hardcoded Bresenham circles
    static constexpr int kSmallCircleSize = 16;
    static constexpr int kLargeCircleSize = 20;
    static constexpr int kSmallMinThresh = 3;
    static constexpr int kSmallMaxThresh = 6;
    static constexpr int kLargeMinThresh = 4;
    static constexpr int kLargeMaxThresh = 8;
    static constexpr int kBorderLimit = 4;
    static constexpr int kNmsRadius = 4;

    static const int kSmallCircle[16][2];
    static const int kLargeCircle[20][2];

    // Polarity-separated SAE: sae_[0] = negative, sae_[1] = positive
    cv::Mat sae_[2];
    // Tracks latest timestamp per pixel per polarity (for refractory filter)
    cv::Mat sae_latest_[2];
    // Tracks last corner detection time per pixel (for NMS)
    cv::Mat corner_last_ts_;

    std::vector<dvs_msgs::msg::Event> corner_events_;
};
