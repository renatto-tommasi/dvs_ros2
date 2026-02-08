#include "dvs_tracker/efast_detector.hpp"

EFastDetector::EFastDetector() : Node("efast_detector")
{
    this->declare_parameter<int>("fast_threshold", 20);
    this->declare_parameter<bool>("non_max_suppression", true);

    fast_threshold_ = this->get_parameter("fast_threshold").as_int();
    non_max_suppression_ = this->get_parameter("non_max_suppression").as_bool();

    fast_detector_ = cv::FastFeatureDetector::create(
        fast_threshold_,
        non_max_suppression_,
        cv::FastFeatureDetector::TYPE_9_16);

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "dvs/histogram_with_polarity", 10,
        std::bind(&EFastDetector::image_callback, this, std::placeholders::_1));

    feature_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "dvs/efast_features", 10);

    RCLCPP_INFO(this->get_logger(),
        "eFAST detector started (threshold=%d, nms=%s)",
        fast_threshold_, non_max_suppression_ ? "true" : "false");
}

void EFastDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints;
    fast_detector_->detect(gray, keypoints);

    cv::Mat output = cv_ptr->image.clone();
    cv::drawKeypoints(output, keypoints, output,
                      cv::Scalar(0, 255, 0),
                      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

    cv_bridge::CvImage out_image;
    out_image.header = msg->header;
    out_image.encoding = "bgr8";
    out_image.image = output;
    feature_image_pub_->publish(*out_image.toImageMsg());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting eFAST Detector node...");
    rclcpp::spin(std::make_shared<EFastDetector>());
    rclcpp::shutdown();
    return 0;
}
