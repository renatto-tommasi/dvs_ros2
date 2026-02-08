#include "dvs_tracker/corner_detector.hpp"

#include "rclcpp/rclcpp.hpp"

DVSCornerDetector::DVSCornerDetector() : Node("corner_detector")
{
  // Parameters
  this->declare_parameter<int>("radius", 3);
  this->declare_parameter<double>("k_threshold", 0.01);
  this->declare_parameter<int>("l_min", 3);
  this->declare_parameter<int>("l_max", 5);
  this->declare_parameter<double>("decay_time", 0.1);
  R = this->get_parameter("radius").as_int();
  k = this->get_parameter("k_threshold").as_double();
  l_min = this->get_parameter("l_min").as_int();
  l_max = this->get_parameter("l_max").as_int();
  tau_ = this->get_parameter("decay_time").as_double();

  // SAE will be initialized from the first EventArray message
  sensor_width_ = 0;
  sensor_height_ = 0;

  // Subscriber for DVS events
  event_subscription_ = this->create_subscription<dvs_msgs::msg::EventArray>(
    "/dvs/events", 10, std::bind(&DVSCornerDetector::event_callback, this, std::placeholders::_1));

  corner_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/dvs/corner_image", 10);


}
void DVSCornerDetector::event_callback(const dvs_msgs::msg::EventArray::SharedPtr msg)
{
  // Initialize SAE from the first message's sensor dimensions
  if (sensor_width_ == 0 || sensor_height_ == 0) {
    sensor_width_ = msg->width;
    sensor_height_ = msg->height;
    sae_[0] = cv::Mat::zeros(sensor_height_, sensor_width_, CV_64F);
    sae_[1] = cv::Mat::zeros(sensor_height_, sensor_width_, CV_64F);
    RCLCPP_INFO(this->get_logger(), "SAE initialized: %dx%d", sensor_width_, sensor_height_);
  }

  for (const auto& event : msg->events) {
    // Update SAE for this event's polarity
    int pol = event.polarity ? 1 : 0;
    double t = event.ts.sec + event.ts.nanosec * 1e-9;

    if (event.x < sensor_width_ && event.y < sensor_height_) {
      sae_[pol].at<double>(event.y, event.x) = t;
    }

    if (is_corner(event)) {
      corner_events_.push_back(event);
    }
  }

  if (!msg->events.empty()) {
    rclcpp::Time latest_time(msg->events.back().ts);
    publish_corner_image(latest_time.seconds());
  }
}

bool DVSCornerDetector::is_corner(const dvs_msgs::msg::Event& event)
{
  // Border check: reject events where the circle would go out of bounds
  if (event.x < R || event.x >= sensor_width_ - R ||
      event.y < R || event.y >= sensor_height_ - R) {
    return false;
  }

  int pol = event.polarity ? 1 : 0;
  double event_t = sae_[pol].at<double>(event.y, event.x);

  // 1. Get Circle Indices
  std::vector<std::pair<int,int>> circle = get_circle_indices(event.x, event.y);
  int n = static_cast<int>(circle.size());
  if (n == 0) {
    return false;
  }

  // 2-3. Read timestamps from the same-polarity SAE and classify active pixels.
  //       A pixel is "active" if it fired recently: delta_t = event_t - pixel_t < k.
  std::vector<bool> active(n, false);
  for (int i = 0; i < n; i++) {
    int cx = circle[i].first;
    int cy = circle[i].second;
    double pixel_t = sae_[pol].at<double>(cy, cx);
    double delta_t = event_t - pixel_t;
    active[i] = (delta_t >= 0.0 && delta_t < k);
  }

  // 4-5. Group consecutive active pixels and find the longest arc.
  //       The circle wraps around, so iterate over 2*n with modular indexing.
  int max_arc = 0;
  int current_arc = 0;

  for (int i = 0; i < 2 * n; i++) {
    if (active[i % n]) {
      current_arc++;
      if (current_arc > n) {
        break;
      }
      max_arc = std::max(max_arc, current_arc);
    } else {
      current_arc = 0;
    }
  }

  return (max_arc >= l_min && max_arc <= l_max);
}

std::vector<std::pair<int,int>> DVSCornerDetector::get_circle_indices(int x, int y)
{
  std::vector<std::pair<int,int>> indices;
  int num_points = 4 * R;

  for (int i = 0; i < num_points; i++) {
    double angle = 2.0 * M_PI * i / num_points;
    int cx = x + static_cast<int>(std::round(R * std::cos(angle)));
    int cy = y + static_cast<int>(std::round(R * std::sin(angle)));
    if (indices.empty() || indices.back() != std::make_pair(cx, cy)) {
      indices.emplace_back(cx, cy);
    }
  }

  if (indices.size() > 1 && indices.front() == indices.back()) {
    indices.pop_back();
  }

  return indices;
}


void DVSCornerDetector::publish_corner_image(double t_now)
{
  if (corner_events_.empty()) {
    return;
  }

  // Compute exp(-(t_now - t_last(x,y)) / tau) for every pixel

  cv::Mat combined;
  cv::max(sae_[0], sae_[1], combined);

  cv::Mat time_diff = t_now - combined;
  cv::Mat decay;
  cv::exp(-time_diff / tau_, decay);

  // Convert [0.0, 1.0] range to [0, 255]
  cv::Mat gray;
  decay.convertTo(gray, CV_8U, 255.0);

  cv::Mat vis;
  cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);

  for (const auto& e : corner_events_) {
    cv::circle(vis, cv::Point(e.x, e.y), 2, cv::Scalar(0, 0, 255), -1);
  }

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = rclcpp::Time(static_cast<uint64_t>(t_now * 1e9));
  cv_image.header.frame_id = "dvs_frame";
  cv_image.encoding = "bgr8";
  cv_image.image = vis;
  corner_image_pub_->publish(*cv_image.toImageMsg());

  corner_events_.clear();
}




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting DVS CornerDetector node...");
  rclcpp::spin(std::make_shared<DVSCornerDetector>());
  rclcpp::shutdown();
  return 0;
}
