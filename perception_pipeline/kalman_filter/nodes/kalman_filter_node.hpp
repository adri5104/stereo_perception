#ifndef KALMAN_FILTER__KALMAN_FILTER_NODE_HPP_
#define KALMAN_FILTER__KALMAN_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "kalman_filter/kalman_core.hpp"

namespace perception_pipeline
{
namespace kalman_filter
{

/**
 * @class KalmanFilterNode
 * @brief ROS2 Node that subscribes to optical flow, disparity, and camera info,
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
  void disparityCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // Helper function to convert ROS Image to cv::Mat
  cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::SharedPtr msg);

  // Internal Kalman filter logic object
  KalmanCore kalman_core_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr optical_flow_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr disparity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Parameters (topic names)
  std::string optical_flow_topic_;
  std::string disparity_topic_;
  std::string camera_info_topic_;
};

} // namespace kalman_filter
} // namespace perception_pipeline

#endif  // KALMAN_FILTER__KALMAN_FILTER_NODE_HPP_
