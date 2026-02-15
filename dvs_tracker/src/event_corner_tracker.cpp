#include "dvs_tracker/event_corner_tracker.hpp"

#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <algorithm>

GraphNode::GraphNode(double timestamp, float px, float py)
    : t(timestamp), newest_t(timestamp), x(px), y(py), depth(0), active(true){}

EventTracker::EventTracker() : Node("corner_detector")
{
    this->declare_parameter<double>("track_window", 3.0);
    this->declare_parameter<double>("d_conn", 5.0);
    this->declare_parameter<int>("rho_thresh", 10);
    this->declare_parameter<int>("max_track_length", 10);
    dt_max_ = this->get_parameter("track_window").as_double();
    d_conn_ = this->get_parameter("d_conn").as_double();
    rho_thresh_ = this->get_parameter("rho_thresh").as_int();
    max_track_length_ = this->get_parameter("max_track_length").as_int();

    corner_subscription_ = this->create_subscription<dvs_msgs::msg::EventArray>(
        "/dvs/corners", 10, std::bind(&EventTracker::corner_callback, this, std::placeholders::_1));
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/dvs/corner_image", 10, std::bind(&EventTracker::image_callback, this, std::placeholders::_1));
    track_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/dvs/tracks", 10);
    metrics_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&EventTracker::print_metrics, this));
}

void EventTracker::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    background_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    publish_track_image();
}

void EventTracker::corner_callback(const dvs_msgs::msg::EventArray::SharedPtr msg){
    if (sensor_width_ == 0 || sensor_height_ == 0) {
        sensor_width_ = msg->width;
        sensor_height_ = msg->height;
        grid_cols_ = static_cast<int>(std::ceil(sensor_width_ / d_conn_));
        grid_rows_ = static_cast<int>(std::ceil(sensor_height_ / d_conn_));
        active_grid_.resize(grid_rows_, std::vector<std::vector<GraphNode*>>(grid_cols_));
    }
    for (const auto& corner : msg->events){
        double timestamp = corner.ts.sec + corner.ts.nanosec * 1e-9;
        auto v_new = std::make_shared<GraphNode>(timestamp, corner.x, corner.y);
        auto v_neigh = GetActiveNeighbourhoodVertices(v_new, d_conn_);
        DiscardOldVertices(v_neigh, v_new->t, dt_max_);

        if (v_neigh.size() != 0){
            auto [leafs, non_leafs] = SegmentLeafVertices(v_neigh);
            GraphNode* v_parent = nullptr;
            if (leafs.size() > 0){
                v_parent = GetClosestAndNewestVertex(leafs, v_new);
            } else{
                v_parent = GetClosestAndNewestVertex(non_leafs, v_new);
            }
            GrowExistingTree(v_parent, v_new);
            UpdateActiveVerticesInTree(v_new, rho_thresh_);
        } else{
            InitializeNewTreeFromVertex(v_new);
        }
    }
    last_event_t_ = msg->events.back().ts.sec + msg->events.back().ts.nanosec * 1e-9;

    size_t w = 0;
    for (size_t i = 0; i < trees_.size(); i++) {
        if ((last_event_t_ - trees_[i]->newest_t) <= dt_max_) {
            trees_[w++] = trees_[i];
        } else {
            RemoveTreeFromGrid(trees_[i].get());
            trees_pruned_++;
        }
    }
    trees_.resize(w);
}

std::vector<GraphNode*> EventTracker::GetActiveNeighbourhoodVertices(std::shared_ptr<GraphNode> v_new, float d_conn){
    std::vector<GraphNode*> neighbors = {};
    
    auto [r, c] = toCell(v_new->x, v_new->y);
    for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
            int nr = r + dr, nc = c + dc;
            if (nr < 0 || nr >= grid_rows_ || nc < 0 || nc >= grid_cols_) continue;
            for (GraphNode* node : active_grid_[nr][nc]) {
                // check actual distance: hypot(node->x - v_new->x, node->y - v_new->y) <= d_conn
                if (std::hypot(node->x - v_new->x, node->y - v_new->y) <= d_conn) {
                    neighbors.push_back(node);
                }
            }
        }
    }
    return neighbors;
}

void EventTracker::DiscardOldVertices(std::vector<GraphNode*>& v_neigh, double t_new, float dt_max){
    v_neigh.erase(
        std::remove_if(v_neigh.begin(), v_neigh.end(),
            [t_new, dt_max](GraphNode* node) {
                return (t_new - node->t) > dt_max;
            }),
        v_neigh.end());
}

void EventTracker::InitializeNewTreeFromVertex(std::shared_ptr<GraphNode> v){
    v->id = next_track_id_++;
    trees_.push_back(v);
    auto [r, c] = toCell(v->x, v->y);
    active_grid_[r][c].push_back(v.get());

}

std::pair<std::vector<GraphNode*>, std::vector<GraphNode*>> EventTracker::SegmentLeafVertices(const std::vector<GraphNode*>& v_neigh){
    std::vector<GraphNode*> leaf = {};
    std::vector<GraphNode*> non_leaf = {};
    for (GraphNode* node : v_neigh){
        if (node->children.size() == 0){
            leaf.push_back(node);
        } else{
            non_leaf.push_back(node);
        }
    }

    return {leaf, non_leaf};
}

GraphNode* EventTracker::GetClosestAndNewestVertex(const std::vector<GraphNode*> v_neigh, std::shared_ptr<GraphNode> v_new){
    GraphNode* closest_vertex = nullptr;
    float min_dist = INFINITY;
    for (GraphNode* node : v_neigh){
        float dist = std::hypot(node->x-v_new->x,node->y-v_new->y);
        if (dist < min_dist){
            closest_vertex = node;
            min_dist = dist;
        } else if (dist == min_dist){
            closest_vertex = (node->t < closest_vertex->t) ? closest_vertex : node;
        } 
    }
    return closest_vertex;
}

void EventTracker::GrowExistingTree(GraphNode* v_parent, std::shared_ptr<GraphNode> v_new){
    v_parent->children.push_back(v_new);
    v_new->parent = v_parent;
    v_new->depth = v_parent->depth + 1;

    GraphNode* root = v_parent;
    while (root->parent != nullptr) root = root->parent;
    root->newest_t = v_new->t;
}

// Claude did this function
void EventTracker::UpdateActiveVerticesInTree(std::shared_ptr<GraphNode> leaf, int rho_thresh){
    GraphNode* cursor = leaf.get();
    int count = 0;
    while (cursor != nullptr) {
        if (count < rho_thresh) {
            // Ensure node is active and in the grid
            if (count < rho_thresh) {
                auto [r, c] = toCell(cursor->x, cursor->y);
                auto& cell = active_grid_[r][c];
                if (!cursor->active) {
                    cursor->active = true;
                }
                if (std::find(cell.begin(), cell.end(), cursor) == cell.end()) {
                    cell.push_back(cursor);
                }
            }
        } else {
            // Deactivate and remove from grid
            if (cursor->active) {
                cursor->active = false;
                auto [r, c] = toCell(cursor->x, cursor->y);
                auto& cell = active_grid_[r][c];
                auto it = std::find(cell.begin(), cell.end(), cursor);
                if (it != cell.end()) {
                    cell.erase(it);
                }
            } else {
                break; // If we hit an already inactive node, we can stop going up the tree
            }
        }
        count++;
        cursor = cursor->parent;
    }

}






void EventTracker::RemoveTreeFromGrid(GraphNode* root){
    std::vector<GraphNode*> stack = {root};
    while (!stack.empty()) {
        GraphNode* node = stack.back();
        stack.pop_back();
        if (node->active) {
            node->active = false;
            auto [r, c] = toCell(node->x, node->y);
            auto& cell = active_grid_[r][c];
            auto it = std::find(cell.begin(), cell.end(), node);
            if (it != cell.end()) cell.erase(it);
        }
        for (auto& child : node->children) {
            stack.push_back(child.get());
        }
    }
}

std::vector<GraphNode*> EventTracker::find_recent_path(GraphNode* root){
    // DFS to find deepest leaf
    GraphNode* deepest = root;
    std::vector<GraphNode*> stack = {root};
    while (!stack.empty()) {
        GraphNode* node = stack.back();
        stack.pop_back();
        if (node->t > deepest->t) {
            deepest = node;
        }
        for (auto& child : node->children) {
            stack.push_back(child.get());
        }
    }
    // Walk back within the time window from the leaf
    double t_leaf = deepest->t;
    std::vector<GraphNode*> path;
    GraphNode* cursor = deepest;
    while (cursor != nullptr) {
        path.push_back(cursor);
        cursor = cursor->parent;
    }
    return path;
}

void EventTracker::publish_track_image(){
    if (sensor_width_ == 0 || sensor_height_ == 0) return;

    cv::Mat vis;
    if (!background_.empty()) {
        vis = background_.clone();
    } else {
        vis = cv::Mat::zeros(sensor_height_, sensor_width_, CV_8UC3);
    }

    size_t write = 0;
    for (size_t t = 0; t < trees_.size(); t++) {
        auto path = find_recent_path(trees_[t].get());

        if (path.empty() || (last_event_t_ - path.front()->t) > dt_max_) {
            RemoveTreeFromGrid(trees_[t].get());
            trees_pruned_++;
            continue; // stale tree — skip and don't keep
        }

        trees_[write++] = trees_[t]; // compact surviving trees

        if (path.size() < 2) continue;
        double duration = path.front()->t - path.back()->t;
        if (duration < min_track_duration_) continue;

        int hue = (trees_[t]->id * 47) % 180;
        cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
        cv::Mat bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
        cv::Scalar color(bgr.at<cv::Vec3b>(0, 0)[0],
                         bgr.at<cv::Vec3b>(0, 0)[1],
                         bgr.at<cv::Vec3b>(0, 0)[2]);
        size_t limit = std::min(path.size(), static_cast<size_t>(max_track_length_));

        for (size_t i = 0; i < limit - 1; i++) {
            cv::line(vis,
                cv::Point(static_cast<int>(path[i]->x), static_cast<int>(path[i]->y)),
                cv::Point(static_cast<int>(path[i+1]->x), static_cast<int>(path[i+1]->y)),
                color, 1);
        }
    }
    trees_.resize(write);

    cv_bridge::CvImage cv_image;
    cv_image.header.frame_id = "dvs_frame";
    cv_image.encoding = "bgr8";
    cv_image.image = vis;
    track_image_pub_->publish(*cv_image.toImageMsg());
}
void EventTracker::print_metrics(){
    if (trees_.empty()) {
        RCLCPP_INFO(this->get_logger(), "No active trees");
        return;
    }

    double max_duration = 0.0;
    int max_nodes = 0;
    int drawable_tracks = 0;
    
    // Accumulators for valid paths (size >= 2)
    double total_duration = 0.0;
    long long total_nodes = 0;
    int valid_paths_count = 0;

    for (size_t i = 0; i < trees_.size(); i++) {
        auto path = find_recent_path(trees_[i].get());
        
        if (path.size() < 2) continue;

        double duration = path.front()->t - path.back()->t;
        int nodes = static_cast<int>(path.size());

        // Update global aggregates
        total_duration += duration;
        total_nodes += nodes;
        valid_paths_count++;

        // Check thresholds
        if (duration >= min_track_duration_) {
            drawable_tracks++;
        }

        // Track maximums independently
        if (duration > max_duration) {
            max_duration = duration;
        }
        
        // Since time is capped, max_nodes is the most meaningful peak metric
        if (nodes > max_nodes) {
            max_nodes = nodes;
        }
    }

    double avg_nodes = valid_paths_count > 0 ? (double)total_nodes / valid_paths_count : 0.0;

    // Output:
    // - MaxDur: Verifies we are hitting the 1.0s cap
    // - MaxNodes: The size of the "best" track (most meaningful metric)
    // - AvgNodes: The average size of all tracked features
    RCLCPP_INFO(this->get_logger(),
        "Trees: %zu | Drawable: %d | Pruned: %zu | MaxDur: %.2fs | MaxNodes: %d | AvgNodes: %.1f",
        trees_.size(), drawable_tracks, trees_pruned_,
        max_duration, max_nodes, avg_nodes);

    trees_pruned_ = 0;
}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Corner Tracker node...");
    rclcpp::spin(std::make_shared<EventTracker>());
    rclcpp::shutdown();
    return 0;
}



