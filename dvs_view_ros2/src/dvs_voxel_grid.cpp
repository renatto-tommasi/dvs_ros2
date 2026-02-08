#include "dvs_view_ros2/dvs_voxel_grid.hpp"



DVSVoxelGrid::DVSVoxelGrid() : Node("dvs_voxel_grid")
{
    // Parameters
    this->declare_parameter<double>("voxel_time_bin_size", 0.01);
    this->declare_parameter<int>("num_time_bins", 10);
    voxel_time_bin_size_ = this->get_parameter("voxel_time_bin_size").as_double();
    num_time_bins_ = this->get_parameter("num_time_bins").as_int();
    accumulating_ = false;

    // Subscriber for DVS events
    subscription_ = this->create_subscription<dvs_msgs::msg::EventArray>(
        "/dvs/events", 10, std::bind(&DVSVoxelGrid::event_callback, this, std::placeholders::_1));

    // Publisher for voxel grid visualization
    publisher_voxel_grid_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/dvs/voxel_grid", 10);

    RCLCPP_INFO(this->get_logger(), "DVSVoxelGrid node initialized.");
}

void DVSVoxelGrid::event_callback(const dvs_msgs::msg::EventArray::SharedPtr msg)
{
    if (msg->events.empty()) return;

    if (!initialized_) {
        int H = msg->height;
        int W = msg->width;
        int B = num_time_bins_;

        int sizes[] = {H, W, B};
        voxel_grid_ = cv::Mat(3, sizes, CV_32FC1, cv::Scalar(0));

        initialized_ = true;
    }

    double window_duration = num_time_bins_ * voxel_time_bin_size_;
    int B = num_time_bins_;

    for (const auto& event : msg->events) {
        double t_event = event.ts.sec + event.ts.nanosec * 1e-9;

        // Start a new accumulation window from the first event
        if (!accumulating_) {
            window_start_time_ = t_event;
            voxel_grid_.setTo(0);
            accumulating_ = true;
        }

        double elapsed = t_event - window_start_time_;

        // If this event falls beyond the window, publish and start a new window
        if (elapsed >= window_duration) {
            publish_voxel_grid();

            // Start new window from this event
            window_start_time_ = t_event;
            voxel_grid_.setTo(0);
            elapsed = 0.0;
        }

        // Place event into the correct time bin
        int time_bin = static_cast<int>(elapsed / voxel_time_bin_size_);
        if (time_bin >= B) time_bin = B - 1;

        if (event.y < voxel_grid_.size[0] &&
            event.x < voxel_grid_.size[1]) {
            voxel_grid_.at<float>(event.y, event.x, time_bin) = 1.0f;
        }
    }
}

void DVSVoxelGrid::publish_voxel_grid()
{
    // 1. Create the MarkerArray message
    visualization_msgs::msg::MarkerArray marker_array_msg;

    // 2. Create on marker of type CUBE_LIST
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = "dvs_frame"; // Set your frame ID
    marker.ns = "voxel_grid";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 3. Set the size of each cube (voxel)
    marker.scale.x = 0.01; // Size of the cube in x (width)
    marker.scale.y = 0.01; // Size of the cube in y (height)
    marker.scale.z = 0.01; // Size of the cube in z (time)

    // 4. Set a pose (identity pose since we will use the position of each cube to represent the voxel
    marker.pose.orientation.w = 1.0;

    // 5. Get dimensions of the voxel grid
    int H = voxel_grid_.size[0];
    int W = voxel_grid_.size[1];
    int B = voxel_grid_.size[2];

    // 6. Iterate over every cell in the 3D grid
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            for (int b = 0; b < B; ++b) {

                // 7. Only add a cube where an event exists
                if (voxel_grid_.at<float>(y, x, b) > 0.0f) {

                    // 8. Create a 3D point for this voxel
                    geometry_msgs::msg::Point p;
                    p.x = x * marker.scale.x;  // pixel X -> 3D X
                    p.y = b * marker.scale.y;  // time bin -> 3D Y (depth)
                    p.z = (H - y) * marker.scale.z;  // pixel Y -> 3D Z (up, flipped so origin is bottom-left)
                    marker.points.push_back(p);

                    // 9. Color per-voxel (optional)
                    //    e.g., color by time bin: older=blue, newer=red
                    std_msgs::msg::ColorRGBA color;
                    color.r = static_cast<float>(b) / B;  // red increases with time
                    color.g = 0.2f;
                    color.b = 1.0f - static_cast<float>(b) / B;  // blue decreases
                    color.a = 1.0f;  // fully opaque
                    marker.colors.push_back(color);
                }
            }
        }
    }
    // 10. Add the marker to the array and publish
    marker_array_msg.markers.push_back(marker);
    publisher_voxel_grid_->publish(marker_array_msg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting DVSVoxelGrid node...");
    rclcpp::spin(std::make_shared<DVSVoxelGrid>());
    rclcpp::shutdown();
    return 0;
}