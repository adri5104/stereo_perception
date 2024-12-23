#include "kalman_filter/kalman_filter.hpp"
#include <cv_bridge/cv_bridge.hpp>

using namespace cv;


KalmanFilterCompute::KalmanFilterCompute() :
  Node("Kalmanfilternode")
{
  // Subscriber to optical flow image topic
  optical_flow_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/optical_flow", 10,
      std::bind(&KalmanFilterCompute::opticalFlowCallback, this, std::placeholders::_1));

}


void KalmanFilterCompute::opticalFlowCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  Mat image = imageMessageToMat(msg);
  RCLCPP_INFO(this-> get_logger(),"Image received");
}

Mat KalmanFilterCompute::imageMessageToMat(const sensor_msgs::msg::Image::SharedPtr msg) 
{
    try 
    {
      RCLCPP_INFO(this-> get_logger(),"Image processed");
      return cv_bridge::toCvCopy(msg, msg->encoding)->image;
    } 
    catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      Mat xd;
      return xd;
    }
}

