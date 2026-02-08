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

    void publish_corner_image(double t_now);

    bool is_corner(const dvs_msgs::msg::Event& event);

    std::vector<std::pair<int,int>> get_circle_indices(int x, int y);

    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr event_subscription_;
    // Publisher for visualization
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr corner_image_pub_;

    int R;
    double k;

    int l_min;
    int l_max;
    double tau_;

    int sensor_width_;
    int sensor_height_;

    // Polarity-separated SAE: sae_[0] = negative, sae_[1] = positive
    // Stores raw timestamps in seconds (CV_64F)
    cv::Mat sae_[2];

    // Storage for detected corners
    std::vector<dvs_msgs::msg::Event> corner_events_;

 






};