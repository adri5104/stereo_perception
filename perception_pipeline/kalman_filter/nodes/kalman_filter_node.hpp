/**
 * @file kalman_core.cpp
 * @author adrian.rieker@tum.de
 * @brief 
 * @version 
 * @date 
 * 
 * 
 * 
 */

#ifndef KALMAN_FILTER__KALMAN_FILTER_NODE_HPP_
#define KALMAN_FILTER__KALMAN_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>

#include "kalman_filter/kalman_core.hpp"

namespace perception_pipeline
{
namespace kalman_filter
{

/**
 * @class KalmanFilterNode
 * @brief ROS2 Node that subscribes to optical flow, depth, and camera info,
 *        and uses an internal KalmanCore object for the actual filter logic.
 */
class KalmanFilterNode : public rclcpp::Node
{
public:

  /**
   * @brief Constructor. Initializes ROS subscriptions and the internal KalmanCore.
   */
  KalmanFilterNode();

private:

  // ROS subscription callbacks
  void opticalFlowCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void colorImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Helper function to convert ROS Image to cv::Mat
  cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::SharedPtr msg);

  // Internal Kalman filter logic object
  KalmanCore kalman_core_;

  // Input values 
  cv::Mat optical_flow_;
  cv::Mat depth_image_;
  cv::Mat color_image_;

  // Output values
  cv::Mat output_6d_;
  cv::Mat output_debug_image_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr optical_flow_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_6d_pub_;

  // Parameters (topic names)
  std::string optical_flow_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string color_image_topic_;
  std::string debug_image_topic_;
  std::string output_6d_topic_;
};

} // namespace kalman_filter
} // namespace perception_pipeline

#endif  // KALMAN_FILTER__KALMAN_FILTER_NODE_HPP_
