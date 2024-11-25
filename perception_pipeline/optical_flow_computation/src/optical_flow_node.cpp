#include "optical_flow_computation/optical_flow_node.hpp"
#include <cv_bridge/cv_bridge.hpp>

using namespace cv;

OpticalFlowNode::OpticalFlowNode()
    : Node("optical_flow_node"), prev_image_() {

    // Declare and initialize parameters
    pyr_scale_ = this->declare_parameter<double>("pyr_scale", 0.5);
    levels_ = this->declare_parameter<int>("levels", 3);
    winsize_ = this->declare_parameter<int>("winsize", 15);
    iterations_ = this->declare_parameter<int>("iterations", 3);
    poly_n_ = this->declare_parameter<int>("poly_n", 5);
    poly_sigma_ = this->declare_parameter<double>("poly_sigma", 1.2);
    flags_ = this->declare_parameter<int>("flags", 0);

    // Subscriber to camera image topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/left/image_raw", 10,
        std::bind(&OpticalFlowNode::imageCallback, this, std::placeholders::_1));

    // Publisher for optical flow
    optical_flow_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/optical_flow", 10);

    RCLCPP_INFO(this->get_logger(), "OpticalFlowNode initialized with parameters:");
    RCLCPP_INFO(this->get_logger(), "  pyr_scale: %.2f", pyr_scale_);
    RCLCPP_INFO(this->get_logger(), "  levels: %d", levels_);
    RCLCPP_INFO(this->get_logger(), "  winsize: %d", winsize_);
    RCLCPP_INFO(this->get_logger(), "  iterations: %d", iterations_);
    RCLCPP_INFO(this->get_logger(), "  poly_n: %d", poly_n_);
    RCLCPP_INFO(this->get_logger(), "  poly_sigma: %.2f", poly_sigma_);
    RCLCPP_INFO(this->get_logger(), "  flags: %d", flags_);
}

void OpticalFlowNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    
    // Convert ROS Image message to OpenCV Mat
    Mat current_image;
    try {
        current_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        RCLCPP_INFO(this->get_logger(), "image callback!");
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Convert to grayscale
    Mat gray_image;
    cvtColor(current_image, gray_image, COLOR_BGR2GRAY);

    if (prev_image_.empty()) {
        
        // Initialize previous image for the first time
        prev_image_ = gray_image.clone();
        return;
    }

    // Calculate optical flow using Farneback method
    Mat flow;
    calcOpticalFlowFarneback(prev_image_, gray_image, flow, pyr_scale_, levels_,
                             winsize_, iterations_, poly_n_, poly_sigma_, flags_);

    // Convert optical flow to HSV for visualization
    Mat flow_hsv(flow.size(), CV_8UC3);
    for (int y = 0; y < flow.rows; y++) {
        for (int x = 0; x < flow.cols; x++) {
            Point2f flow_at_point = flow.at<Point2f>(y, x);

            // Compute magnitude and angle
            float magnitude = sqrt(flow_at_point.x * flow_at_point.x + flow_at_point.y * flow_at_point.y);
            float angle = atan2(flow_at_point.y, flow_at_point.x); // Angle in radians

            // Normalize angle to [0, 180] for HSV (OpenCV hue range)
            float hue = angle * 180 / CV_PI;
            if (hue < 0) hue += 360;

            // Set HSV values
            flow_hsv.at<Vec3b>(y, x) = Vec3b(
                static_cast<uchar>(hue / 2),             // Hue: [0, 180] (OpenCV range)
                static_cast<uchar>(255),                // Saturation: full
                static_cast<uchar>(std::min(255.0f, magnitude)) // Value: scaled magnitude
            );
        }
    }

    // Convert HSV to BGR for publishing
    Mat flow_bgr;
    cvtColor(flow_hsv, flow_bgr, COLOR_HSV2BGR);

    // Publish the optical flow as an image
    auto flow_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", flow_bgr).toImageMsg();
    optical_flow_pub_->publish(*flow_msg);

    // Update previous image
    prev_image_ = gray_image.clone();
}