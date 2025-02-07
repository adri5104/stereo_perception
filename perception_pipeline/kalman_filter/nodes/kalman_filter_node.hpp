/**
 * @file kalman_filter_node.hpp
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
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // Para tf2::Quaternion
#include <tf2/LinearMath/Matrix3x3.h>   // Opcional, para manejar rotaciones en tf2
#include <tf2_ros/static_transform_broadcaster.h>



#include <opencv2/opencv.hpp>
#ifdef ROS_VERSION_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "kalman_filter/kalman_core.hpp"


namespace perception_pipeline
{
namespace kalman_filter
{


using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    geometry_msgs::msg::TransformStamped
>;

/**
 * @class KalmanFilterNode
 * @brief ROS2 Node that subscribes to optical flow, depth, color, and camera info,
 *        and uses an internal KalmanCore object for the actual filter logic.
 *
 *        Implements message_filters to synchronize the three images.
 */
class KalmanFilterNode : public rclcpp::Node
{
public:
  
  /**
   * @brief Construct a new Kalman Filter Node object
   * 
   */
  KalmanFilterNode();

private:

  /**
   * @brief Callback for the message syncronizer
   * 
   * @param flow_msg optical flow message
   * @param depth_msg depth image in mm
   * @param color_msg left color image
   */
  void updateSync(
    const sensor_msgs::msg::Image::ConstSharedPtr flow_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
    const geometry_msgs::msg::TransformStamped::ConstSharedPtr frame_tf_msg);

  
  /**
   * @brief Camera info callback
   * 
   * @param msg 
   */
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  /**
   * @brief  method that outputs a MarkerArray given a 6d output image
   * 
   * @param image Input 6D image (cv::Mat with 32F6C codification)
   * @return visualization_msgs::msg::MarkerArray 
   */
  visualization_msgs::msg::MarkerArray createMarkers(const cv::Mat &image_6d, const cv::Mat &image_6d_val, double delta_time);

  /**
   * @brief Helper function to convert from a ROS Image message to openCV Mat
   * 
   * @param msg 
   * @return cv::Mat 
   */
  cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  

  // KalmanCore
  std::unique_ptr<KalmanCore> kalman_core_;
  
  // message_filters subscribers
  message_filters::Subscriber<sensor_msgs::msg::Image> optical_flow_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<geometry_msgs::msg::TransformStamped> frame_tf_sub_;

  // Synchronizer pointer
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Separate camera_info subscriber
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_6d_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_6d_val_pub_;


  // Parameters
  std::string optical_flow_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_tf_topic_;
  std::string color_image_topic_;
  std::string debug_image_topic_;
  std::string debug_markers_topic_;
  std::string output_6d_topic_;
  std::string output_6d_val_topic_;


  bool camera_parameters_set_ = false;


};

} // namespace kalman_filter
} // namespace perception_pipeline

#endif  // KALMAN_FILTER__KALMAN_FILTER_NODE_HPP_
