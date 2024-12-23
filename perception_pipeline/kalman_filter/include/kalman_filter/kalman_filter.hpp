#ifndef _KALMAN_FILTER_
#define _KALMAN_FILTER_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <iostream>


class KalmanFilterCompute : public rclcpp::Node
{
  public:
    KalmanFilterCompute();
  private:

    // Subscribers and publishers
    void opticalFlowCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr optical_flow_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_;

    cv::Mat imageMessageToMat(const sensor_msgs::msg::Image::SharedPtr msg);

    
};  



#endif