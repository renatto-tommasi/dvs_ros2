#pragma once

#include "rclcpp/rclcpp.hpp"
#include "dvs_msgs/msg/event_array.hpp"
#include "dvs_msgs/msg/event.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <vector>
#include <memory>

class DVSVoxelGrid : public rclcpp::Node
{
public:
    DVSVoxelGrid();
private:
    void event_callback(const dvs_msgs::msg::EventArray::SharedPtr msg);
    
    void publish_voxel_grid();
    // Subscribers and publishers
    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_voxel_grid_;

    // Voxel grid data
    cv::Mat voxel_grid_; // 3D grid: [height][width][time_bin]
    double voxel_time_bin_size_; // in seconds
    int num_time_bins_;
    bool initialized_ = false;
    double window_start_time_ = 0.0;
    bool accumulating_ = false;
};