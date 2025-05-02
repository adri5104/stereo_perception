#include "optical_flow_computation/optical_flow_node.hpp"
#ifdef ROS_VERSION_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace perception_pipeline
{
namespace optical_flow_computation
{

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

    image_topic_ = this->declare_parameter<std::string>("image_topic", "/left/image_raw");
    optical_flow_topic_ = this->declare_parameter<std::string>("optical_flow_topic", "/optical_flow");

    // Subscriber to camera image topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 10,
        std::bind(&OpticalFlowNode::imageCallback, this, std::placeholders::_1));

    // Publisher for optical flow
    optical_flow_debug_colors_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/debug/colored_image", 10);

    optical_flow_debug_arrows_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/debug/arrows", 10);
      
    optical_flow_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        optical_flow_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "OpticalFlowNode initialized with parameters:");
    RCLCPP_INFO(this->get_logger(), "  pyr_scale: %.2f", pyr_scale_);
    RCLCPP_INFO(this->get_logger(), "  levels: %d", levels_);
    RCLCPP_INFO(this->get_logger(), "  winsize: %d", winsize_);
    RCLCPP_INFO(this->get_logger(), "  iterations: %d", iterations_);
    RCLCPP_INFO(this->get_logger(), "  poly_n: %d", poly_n_);
    RCLCPP_INFO(this->get_logger(), "  poly_sigma: %.2f", poly_sigma_);
    RCLCPP_INFO(this->get_logger(), "  flags: %d", flags_);
    RCLCPP_INFO(this->get_logger(), "  image_topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  optical_flow_topic: %s", optical_flow_topic_.c_str());
    //RCLCPP_INFO(this->get_logger(), "  %s", cv::getBuildInformation() .c_str());

    
}

void OpticalFlowNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    
    // Convert ROS Image message to OpenCV Mat
    Mat current_image;
    try {
        current_image = cv_bridge::toCvShare(msg, "bgr8")->image;
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

    // Upload the images to CUDA memory
    cv::cuda::GpuMat d_prev_image, d_next_image;
    d_prev_image.upload(prev_image_);
    d_next_image.upload(gray_image);

    

    // Create a GpuMat to store the flow result
    cv::cuda::GpuMat d_flow;
    

    Ptr<cuda::FarnebackOpticalFlow> optical_flow_compute = cv::cuda::FarnebackOpticalFlow::create	(
      levels_,
      pyr_scale_, 
      false,
      winsize_,
      iterations_,
      poly_n_,
      poly_sigma_,
      flags_);

    optical_flow_compute->calc(d_prev_image, d_next_image, d_flow);

    // Download the flow result to the host for further processing
    Mat flow;
    d_flow.download(flow);

    // We publish the optical flow message
    auto flow_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC2", flow).toImageMsg();
    flow_msg -> header = msg -> header;
    optical_flow_pub_->publish(*flow_msg);

    // We publish the debug messages
    publishDebugMessages(current_image, flow);
    
    // Update previous image
    prev_image_ = gray_image.clone();
}

void OpticalFlowNode::publishDebugMessages(const Mat current_image, const Mat flow)
{
    // Convert optical flow to HSV for visualization
    Mat flow_hsv(flow.size(), CV_8UC3);
    for (int y = 0; y < flow.rows; y++) {
        for (int x = 0; x < flow.cols; x++) {
            Point2f flow_at_point = flow.at<Point2f>(y, x);

            // Compute magnitude and angle
            float magnitude = sqrt(flow_at_point.x * flow_at_point.x + flow_at_point.y * flow_at_point.y);
            if (magnitude > 2) magnitude = 8 * magnitude ; 
            if (magnitude > 255) magnitude = 255;


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
    optical_flow_debug_colors_pub_->publish(*flow_msg);


    // Create an image to visualize optical flow with arrows
    Mat flow_vis = current_image.clone();

    // Parameters for arrow visualization
    const int step = 10; // Grid step size
    const Scalar arrow_color(0, 255, 0); // Green arrows
    const int arrow_thickness = 1;

    for (int y = 0; y < flow.rows; y += step) {
        for (int x = 0; x < flow.cols; x += step) {
            // Get the flow vector at the current point
            const Point2f flow_at_point = flow.at<Point2f>(y, x);
            Point start(x, y);
            Point end(cvRound(x + flow_at_point.x), cvRound(y + flow_at_point.y));

            // Draw the arrow on the visualization image
            arrowedLine(flow_vis, start, end, arrow_color, arrow_thickness, LINE_AA, 0, 0.2);
        }
    }

    // Publish the visualization image with arrows
    sensor_msgs::msg::Image::SharedPtr flow_arrow_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", flow_vis).toImageMsg();
    optical_flow_debug_arrows_pub_->publish(*flow_arrow_msg);
}


} // namespace optical_flow_computation
} // namespace perception_pipeline