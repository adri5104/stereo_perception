#ifndef OPTICAL_FLOW_NODE_HPP
#define OPTICAL_FLOW_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.hpp> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
//#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <opencv2/tracking.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>  
#include <opencv2/cudaoptflow.hpp>


#include <iostream>

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

    // Parameters for Farneback Optical Flow
    double pyr_scale_;
    int levels_;
    int winsize_;
    int iterations_;
    int poly_n_;
    double poly_sigma_;
    int flags_;
};

#endif // OPTICAL_FLOW_NODE_HPP