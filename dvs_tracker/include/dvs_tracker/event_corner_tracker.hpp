#pragma once


#include "rclcpp/rclcpp.hpp"
#include "dvs_msgs/msg/event_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <vector>
#include <memory>

struct GraphNode{
    // Data from the event
    double t;       // Timestamp of the event
    double newest_t; // Newest timestamp in this tree (only meaningful on root)
    float x , y;    // Event Coordinates

    // Graph Meta Data
    int depth;
    bool active;

    // Edges
    GraphNode* parent = nullptr;

    // Children
    std::vector<std::shared_ptr<GraphNode>> children;

    // Constructor
    GraphNode(double timestamp, float px, float py);
};

class EventTracker : public rclcpp::Node
{

public:
    EventTracker();

private:
    void corner_callback(const dvs_msgs::msg::EventArray::SharedPtr msg);

    std::vector<GraphNode*> GetActiveNeighbourhoodVertices(std::shared_ptr<GraphNode> v_new, float d_conn);

    void DiscardOldVertices(std::vector<GraphNode*>& v_neigh, double t_new, float dt_max);

    std::pair<std::vector<GraphNode*>, std::vector<GraphNode*>> SegmentLeafVertices(const std::vector<GraphNode*>& v_neigh);

    GraphNode* GetClosestAndNewestVertex(const std::vector<GraphNode*> v_neigh, std::shared_ptr<GraphNode> v_new);

    void GrowExistingTree(GraphNode* v_parent, std::shared_ptr<GraphNode> v_new);

    void UpdateActiveVerticesInTree(std::shared_ptr<GraphNode> t_parent, int rho_thresh);

    void InitializeNewTreeFromVertex(std::shared_ptr<GraphNode> v);

    void RemoveTreeFromGrid(GraphNode* root);
    std::vector<GraphNode*> find_recent_path(GraphNode* root);
    void publish_track_image();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void print_metrics();

    rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr corner_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr track_image_pub_;
    rclcpp::TimerBase::SharedPtr metrics_timer_;
    cv::Mat background_;

    std::pair<int,int> toCell(float x, float y) {
    return { static_cast<int>(y / d_conn_), static_cast<int>(x / d_conn_) };
    }

    
    
    const float d_conn_ = 5.0f;
    const float dt_max_ = 0.1f;
    const int rho_thresh_ = 5;
    const double min_track_duration_ = 0.5;
    double track_window_;
    double last_event_t_ = 0.0;
    int grid_cols_ = 0;
    int grid_rows_ = 0;
    int sensor_width_ = 0;   
    int sensor_height_ = 0;

    size_t trees_pruned_ = 0;

    std::vector<std::shared_ptr<GraphNode>> trees_;
    std::vector<std::vector<std::vector<GraphNode*>>> active_grid_;

};


