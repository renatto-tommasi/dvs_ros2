#include "dvs_view_ros2/dvs_histogram.hpp"

#include "dvs_msgs/msg/event.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <vector>
#include <memory>


DVSHistogram::DVSHistogram() : Node("dvs_histogram")
{
    subscription_ = this->create_subscription<dvs_msgs::msg::EventArray>(
        "dvs/events", 10, std::bind(&DVSHistogram::event_callback, this, std::placeholders::_1));

    // Create publishers for different accumulation modes
    publisher_no_polarity_ = this->create_publisher<sensor_msgs::msg::Image>("dvs/histogram_no_polarity", 10);
    publisher_with_polarity_ = this->create_publisher<sensor_msgs::msg::Image>("dvs/histogram_with_polarity", 10);
    publisher_binary_ = this->create_publisher<sensor_msgs::msg::Image>("dvs/histogram_binary", 10);
    publisher_color_polarity_ = this->create_publisher<sensor_msgs::msg::Image>("dvs/histogram_color_polarity", 10);

    // Define Delta T (e.g., 33ms for ~30 FPS)
    accumulation_time_ = rclcpp::Duration::from_seconds(0.033);
    initialized_ = false;
}   

void DVSHistogram::event_callback(const dvs_msgs::msg::EventArray::SharedPtr msg)
{

if (!initialized_ && !msg->events.empty()) {
    frame_no_polarity_ = cv::Mat::zeros(msg->height, msg->width, CV_32SC1);  // Use signed int for counting
    frame_with_polarity_ = cv::Mat::zeros(msg->height, msg->width, CV_32SC1);  // Use signed int for polarity
    frame_binary_ = cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
    frame_positive_events_ = cv::Mat::zeros(msg->height, msg->width, CV_32SC1);  // Count positive events
    frame_negative_events_ = cv::Mat::zeros(msg->height, msg->width, CV_32SC1);  // Count negative events
    current_frame_start_time_ = rclcpp::Time(msg->events[0].ts);
    initialized_ = true;
}

if (!initialized_) return;


for (const auto& event : msg->events) {
    rclcpp::Time event_time(event.ts);

    if (event_time >= current_frame_start_time_ + accumulation_time_) {
        publish_frames();

        // Reset for the next window
        frame_no_polarity_.setTo(0);
        frame_with_polarity_.setTo(0);
        frame_binary_.setTo(0);
        frame_positive_events_.setTo(0);
        frame_negative_events_.setTo(0);

        // Move the time window forward by fixed steps until it catches up to current event
        while (current_frame_start_time_ + accumulation_time_ <= event_time) {
            current_frame_start_time_ += accumulation_time_;
        }
}

    if (event.x < msg->width && event.y < msg->height) {
        // Mode 1: Count all events (no polarity)
        frame_no_polarity_.at<int32_t>(event.y, event.x) += 1;

        // Mode 2: Count with polarity (positive adds, negative subtracts)
        if (event.polarity) {
            frame_with_polarity_.at<int32_t>(event.y, event.x) += 1;
            frame_positive_events_.at<int32_t>(event.y, event.x) += 1;
        } else {
            frame_with_polarity_.at<int32_t>(event.y, event.x) -= 1;
            frame_negative_events_.at<int32_t>(event.y, event.x) += 1;
        }

        // Mode 3: Binary (255 if event occurred, 0 otherwise)
        frame_binary_.at<uint8_t>(event.y, event.x) = 255;
    }
}


}

void DVSHistogram::publish_frames()
{
    auto stamp = current_frame_start_time_;

    // Normalize and publish frame without polarity
    cv::Mat normalized_no_pol;
    double min_val, max_val;
    cv::minMaxLoc(frame_no_polarity_, &min_val, &max_val);
    if (max_val > 0) {
        frame_no_polarity_.convertTo(normalized_no_pol, CV_8UC1, 255.0 / max_val);
    } else {
        normalized_no_pol = cv::Mat::zeros(frame_no_polarity_.size(), CV_8UC1);
    }

    cv_bridge::CvImage cv_image_no_pol;
    cv_image_no_pol.header.stamp = stamp;
    cv_image_no_pol.header.frame_id = "dvs_frame";
    cv_image_no_pol.encoding = "mono8";
    cv_image_no_pol.image = normalized_no_pol;
    publisher_no_polarity_->publish(*cv_image_no_pol.toImageMsg());

    // Normalize and publish frame with polarity (handle negative values)
    cv::Mat normalized_with_pol;
    cv::minMaxLoc(frame_with_polarity_, &min_val, &max_val);
    double abs_max = std::max(std::abs(min_val), std::abs(max_val));
    if (abs_max > 0) {
        // Map [-abs_max, abs_max] to [0, 255], with 127 as zero
        frame_with_polarity_.convertTo(normalized_with_pol, CV_8UC1, 127.0 / abs_max, 127.0);
    } else {
        normalized_with_pol = cv::Mat(frame_with_polarity_.size(), CV_8UC1, cv::Scalar(127));
    }

    cv_bridge::CvImage cv_image_with_pol;
    cv_image_with_pol.header.stamp = stamp;
    cv_image_with_pol.header.frame_id = "dvs_frame";
    cv_image_with_pol.encoding = "mono8";
    cv_image_with_pol.image = normalized_with_pol;
    publisher_with_polarity_->publish(*cv_image_with_pol.toImageMsg());

    // Create and publish color polarity image (Red = positive, Blue = negative)
    cv::Mat color_frame = cv::Mat::zeros(frame_positive_events_.size(), CV_8UC3);

    // Normalize positive and negative events separately
    cv::Mat normalized_pos, normalized_neg;
    double pos_min, pos_max, neg_min, neg_max;
    cv::minMaxLoc(frame_positive_events_, &pos_min, &pos_max);
    cv::minMaxLoc(frame_negative_events_, &neg_min, &neg_max);

    if (pos_max > 0) {
        frame_positive_events_.convertTo(normalized_pos, CV_8UC1, 255.0 / pos_max);
    } else {
        normalized_pos = cv::Mat::zeros(frame_positive_events_.size(), CV_8UC1);
    }

    if (neg_max > 0) {
        frame_negative_events_.convertTo(normalized_neg, CV_8UC1, 255.0 / neg_max);
    } else {
        normalized_neg = cv::Mat::zeros(frame_negative_events_.size(), CV_8UC1);
    }

    // Assign to color channels: Blue = negative, Red = positive, Green = 0
    std::vector<cv::Mat> channels(3);
    channels[0] = normalized_neg;  // Blue channel
    channels[1] = cv::Mat::zeros(normalized_pos.size(), CV_8UC1);  // Green channel
    channels[2] = normalized_pos;  // Red channel
    cv::merge(channels, color_frame);

    cv_bridge::CvImage cv_image_color;
    cv_image_color.header.stamp = stamp;
    cv_image_color.header.frame_id = "dvs_frame";
    cv_image_color.encoding = "bgr8";
    cv_image_color.image = color_frame;
    publisher_color_polarity_->publish(*cv_image_color.toImageMsg());

    // Publish binary frame (no normalization needed)
    cv_bridge::CvImage cv_image_binary;
    cv_image_binary.header.stamp = stamp;
    cv_image_binary.header.frame_id = "dvs_frame";
    cv_image_binary.encoding = "mono8";
    cv_image_binary.image = frame_binary_;
    publisher_binary_->publish(*cv_image_binary.toImageMsg());
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting DVSHistogram node :)...");
    rclcpp::spin(std::make_shared<DVSHistogram>());
    rclcpp::shutdown();
    return 0;
}