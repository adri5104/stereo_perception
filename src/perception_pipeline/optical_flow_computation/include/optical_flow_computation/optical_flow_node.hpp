#ifndef OPTICAL_FLOW_NODE_HPP
#define OPTICAL_FLOW_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "std_msgs/msg/header.hpp"
#include <chrono>
#ifdef ROS_VERSION_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaoptflow.hpp>


#include <iostream>

namespace perception_pipeline
{
namespace optical_flow_computation
{

// OpticalFlowNode class definition
class OpticalFlowNode : public rclcpp::Node {
public:
    OpticalFlowNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishDebugMessages(const cv::Mat current_image, const cv::Mat flow); 

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr optical_flow_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr optical_flow_debug_colors_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr optical_flow_debug_arrows_pub_;
    cv::Mat prev_image_;

    // Parameters for topic names
    std::string image_topic_;
    std::string optical_flow_topic_;

    // Parameters for Farneback Optical Flow
    double pyr_scale_;
    int levels_;
    int winsize_;
    int iterations_;
    int poly_n_;
    double poly_sigma_;
    int flags_;
};

}
} // namespace perception_pipeline

#endif // OPTICAL_FLOW_NODE_HPP