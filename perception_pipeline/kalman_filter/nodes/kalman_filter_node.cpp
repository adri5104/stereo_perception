#include "kalman_filter_node.hpp"

#include <cv_bridge/cv_bridge.hpp>

namespace perception_pipeline
{
namespace kalman_filter
{

KalmanFilterNode::KalmanFilterNode()
: Node("kalman_filter_node")
{
  // Declare parameters with default values
  this->declare_parameter<std::string>("optical_flow_topic", "/optical_flow");
  this->declare_parameter<std::string>("disparity_topic", "/disparity");
  this->declare_parameter<std::string>("camera_info_topic", "/camera_info");

  // Read parameters
  optical_flow_topic_ = this->get_parameter("optical_flow_topic").as_string();
  disparity_topic_    = this->get_parameter("disparity_topic").as_string();
  camera_info_topic_  = this->get_parameter("camera_info_topic").as_string();

  RCLCPP_INFO(this->get_logger(),
              "optical_flow_topic: '%s'", optical_flow_topic_.c_str());
  RCLCPP_INFO(this->get_logger(),
              "disparity_topic: '%s'", disparity_topic_.c_str());
  RCLCPP_INFO(this->get_logger(),
              "camera_info_topic: '%s'", camera_info_topic_.c_str());

  // Subscriptions
  optical_flow_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    optical_flow_topic_, 
    rclcpp::QoS(10), 
    std::bind(&KalmanFilterNode::opticalFlowCallback, this, std::placeholders::_1));

  disparity_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    disparity_topic_, 
    rclcpp::QoS(10), 
    std::bind(&KalmanFilterNode::disparityCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 
    rclcpp::QoS(10),
    std::bind(&KalmanFilterNode::cameraInfoCallback, this, std::placeholders::_1));
 

  RCLCPP_INFO(this->get_logger(), "KalmanFilterNode started.");
}

void KalmanFilterNode::opticalFlowCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat flow_image = imageMsgToMat(msg);
  RCLCPP_INFO(this->get_logger(),
              "Optical Flow image received (%d x %d).",
              flow_image.cols, flow_image.rows);

  // Forward the optical flow to the KalmanCore update
  kalman_core_.updateOpticalFlow(flow_image);
}

void KalmanFilterNode::disparityCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat disp_image = imageMsgToMat(msg);
  RCLCPP_INFO(this->get_logger(),
              "Disparity image received (%d x %d).",
              disp_image.cols, disp_image.rows);

  // Forward the disparity to the KalmanCore update
  kalman_core_.updateDepth(disp_image);
}

void KalmanFilterNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),
              "CameraInfo received: width=%d, height=%d.",
              msg->width, msg->height);

  // Typically, we can extract fx, fy, cx, cy from the CameraInfo's K matrix
  double fx = msg->k[0];
  double fy = msg->k[4];
  double cx = msg->k[2];
  double cy = msg->k[5];

  // Pass them to the KalmanCore
  kalman_core_.setCameraParameters(fx, fy, cx, cy);
}

cv::Mat KalmanFilterNode::imageMsgToMat(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    // Convert the ROS Image message to an OpenCV Mat using cv_bridge
    return cv_bridge::toCvCopy(msg, msg->encoding)->image;
  }
  catch(const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }
}

} // namespace kalman_filter
} // namespace perception_pipeline

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perception_pipeline::kalman_filter::KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
} 