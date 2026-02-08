#include "dvs_tracker/arc_star_detector.hpp"

#include "rclcpp/rclcpp.hpp"

// Hardcoded Bresenham circles (offsets from center)
const int DVSCornerDetector::kSmallCircle[16][2] = {
    {0, 3}, {1, 3}, {2, 2}, {3, 1},
    {3, 0}, {3, -1}, {2, -2}, {1, -3},
    {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
    {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}
};

const int DVSCornerDetector::kLargeCircle[20][2] = {
    {0, 4}, {1, 4}, {2, 3}, {3, 2},
    {4, 1}, {4, 0}, {4, -1}, {3, -2},
    {2, -3}, {1, -4}, {0, -4}, {-1, -4},
    {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
    {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}
};

DVSCornerDetector::DVSCornerDetector() : Node("arc_star_detector")
{
  this->declare_parameter<double>("decay_time", 0.1);
  this->declare_parameter<double>("filter_threshold", 0.05);

  tau_ = this->get_parameter("decay_time").as_double();
  filter_threshold_ = this->get_parameter("filter_threshold").as_double();

  sensor_width_ = 0;
  sensor_height_ = 0;

  event_subscription_ = this->create_subscription<dvs_msgs::msg::EventArray>(
    "/dvs/events", 10, std::bind(&DVSCornerDetector::event_callback, this, std::placeholders::_1));

  corner_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/dvs/corner_image", 10);
}

void DVSCornerDetector::event_callback(const dvs_msgs::msg::EventArray::SharedPtr msg)
{
  // Initialize surfaces from the first message
  if (sensor_width_ == 0 || sensor_height_ == 0) {
    sensor_width_ = msg->width;
    sensor_height_ = msg->height;
    sae_[0] = cv::Mat::zeros(sensor_height_, sensor_width_, CV_64F);
    sae_[1] = cv::Mat::zeros(sensor_height_, sensor_width_, CV_64F);
    sae_latest_[0] = cv::Mat::zeros(sensor_height_, sensor_width_, CV_64F);
    sae_latest_[1] = cv::Mat::zeros(sensor_height_, sensor_width_, CV_64F);
    RCLCPP_INFO(this->get_logger(), "SAE initialized: %dx%d", sensor_width_, sensor_height_);
  }

  for (const auto& event : msg->events) {
    int ex = event.x;
    int ey = event.y;
    if (ex >= sensor_width_ || ey >= sensor_height_) {
      continue;
    }

    int pol = event.polarity ? 1 : 0;
    int pol_inv = event.polarity ? 0 : 1;
    double et = event.ts.sec + event.ts.nanosec * 1e-9;

    // Refractory filter: suppress redundant same-polarity events at the same pixel
    double& t_last = sae_latest_[pol].at<double>(ey, ex);
    double& t_last_inv = sae_latest_[pol_inv].at<double>(ey, ex);

    if ((et > t_last + filter_threshold_) || (t_last_inv > t_last)) {
      // Event passes filter: update both surfaces
      t_last = et;
      sae_[pol].at<double>(ey, ex) = et;
    } else {
      // Event is redundant: only update latest tracker, skip corner test
      t_last = et;
      continue;
    }

    if (is_corner(event, et)) {
      corner_events_.push_back(event);
    }
  }

  if (!msg->events.empty()) {
    rclcpp::Time latest_time(msg->events.back().ts);
    publish_corner_image(latest_time.seconds());
  }
}

bool DVSCornerDetector::is_corner(const dvs_msgs::msg::Event& event, double t)
{
  int ex = event.x;
  int ey = event.y;

  // Border check: largest circle has radius 4
  if (ex < kBorderLimit || ex >= sensor_width_ - kBorderLimit ||
      ey < kBorderLimit || ey >= sensor_height_ - kBorderLimit) {
    return false;
  }

  int pol = event.polarity ? 1 : 0;

  // Small circle test first (R=3, 16 points)
  int small_seg = arc_test(ex, ey, pol, kSmallCircle, kSmallCircleSize);
  bool small_valid =
      (small_seg >= kSmallMinThresh && small_seg <= kSmallMaxThresh) ||
      (small_seg >= (kSmallCircleSize - kSmallMaxThresh) &&
       small_seg <= (kSmallCircleSize - kSmallMinThresh));

  if (!small_valid) {
    return false;
  }

  // Large circle test (R=4, 20 points)
  int large_seg = arc_test(ex, ey, pol, kLargeCircle, kLargeCircleSize);
  bool large_valid =
      (large_seg >= kLargeMinThresh && large_seg <= kLargeMaxThresh) ||
      (large_seg >= (kLargeCircleSize - kLargeMaxThresh) &&
       large_seg <= (kLargeCircleSize - kLargeMinThresh));

  return large_valid;
}

int DVSCornerDetector::arc_test(int ex, int ey, int pol,
                                 const int circle[][2], int circle_size)
{
  // Find the pixel on the circle with the maximum (newest) timestamp
  int newest_idx = 0;
  double newest_t = sae_[pol].at<double>(ey + circle[0][1], ex + circle[0][0]);

  for (int i = 1; i < circle_size; i++) {
    double t = sae_[pol].at<double>(ey + circle[i][1], ex + circle[i][0]);
    if (t > newest_t) {
      newest_t = t;
      newest_idx = i;
    }
  }

  // Initialize bidirectional expansion from the newest pixel
  int arc_left_idx = (newest_idx - 1 + circle_size) % circle_size;
  int arc_right_idx = (newest_idx + 1) % circle_size;

  double arc_left_value = sae_[pol].at<double>(
      ey + circle[arc_left_idx][1], ex + circle[arc_left_idx][0]);
  double arc_right_value = sae_[pol].at<double>(
      ey + circle[arc_right_idx][1], ex + circle[arc_right_idx][0]);

  double arc_left_min_t = arc_left_value;
  double arc_right_min_t = arc_right_value;

  double segment_new_min_t = newest_t;
  int newest_segment_size = 1;

  int min_thresh = (circle_size == kSmallCircleSize) ? kSmallMinThresh : kLargeMinThresh;

  // Phase 1: forced expansion for min_thresh - 1 iterations
  for (int iteration = 1; iteration < min_thresh; iteration++) {
    if (arc_right_value > arc_left_value) {
      // Extend right
      if (arc_right_min_t < segment_new_min_t) {
        segment_new_min_t = arc_right_min_t;
      }
      arc_right_idx = (arc_right_idx + 1) % circle_size;
      arc_right_value = sae_[pol].at<double>(
          ey + circle[arc_right_idx][1], ex + circle[arc_right_idx][0]);
      if (arc_right_value < arc_right_min_t) {
        arc_right_min_t = arc_right_value;
      }
    } else {
      // Extend left
      if (arc_left_min_t < segment_new_min_t) {
        segment_new_min_t = arc_left_min_t;
      }
      arc_left_idx = (arc_left_idx - 1 + circle_size) % circle_size;
      arc_left_value = sae_[pol].at<double>(
          ey + circle[arc_left_idx][1], ex + circle[arc_left_idx][0]);
      if (arc_left_value < arc_left_min_t) {
        arc_left_min_t = arc_left_value;
      }
    }
    newest_segment_size = iteration + 1;
  }

  // Phase 2: conditional expansion for remaining iterations
  for (int iteration = min_thresh; iteration < circle_size; iteration++) {
    if (arc_right_value > arc_left_value) {
      // Extend right
      if (arc_right_value >= segment_new_min_t) {
        newest_segment_size = iteration + 1;
        if (arc_right_min_t < segment_new_min_t) {
          segment_new_min_t = arc_right_min_t;
        }
      }
      arc_right_idx = (arc_right_idx + 1) % circle_size;
      arc_right_value = sae_[pol].at<double>(
          ey + circle[arc_right_idx][1], ex + circle[arc_right_idx][0]);
      if (arc_right_value < arc_right_min_t) {
        arc_right_min_t = arc_right_value;
      }
    } else {
      // Extend left
      if (arc_left_value >= segment_new_min_t) {
        newest_segment_size = iteration + 1;
        if (arc_left_min_t < segment_new_min_t) {
          segment_new_min_t = arc_left_min_t;
        }
      }
      arc_left_idx = (arc_left_idx - 1 + circle_size) % circle_size;
      arc_left_value = sae_[pol].at<double>(
          ey + circle[arc_left_idx][1], ex + circle[arc_left_idx][0]);
      if (arc_left_value < arc_left_min_t) {
        arc_left_min_t = arc_left_value;
      }
    }
  }

  return newest_segment_size;
}

cv::Mat DVSCornerDetector::compute_decay_image(double t_now)
{
  cv::Mat combined;
  cv::max(sae_[0], sae_[1], combined);

  cv::Mat time_diff = t_now - combined;
  cv::Mat decay;
  cv::exp(-time_diff / tau_, decay);

  cv::Mat gray;
  decay.convertTo(gray, CV_8U, 255.0);

  cv::Mat vis;
  cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
  return vis;
}

void DVSCornerDetector::publish_corner_image(double t_now)
{
  if (corner_events_.empty()) {
    return;
  }

  cv::Mat vis = compute_decay_image(t_now);

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
