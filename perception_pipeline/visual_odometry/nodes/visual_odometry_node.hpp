/**
 * @file visual_odometry_node.hpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief ROS node that performs visual odometry using a stereo camera setup.
 *
 * 
 * 
 * 
 */

#ifndef VIS_ODOM_NODE_
#define VIS_ODOM_NODE_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "visual_odometry/visual_odometry.hpp"

namespace perception_pipeline
{
namespace visual_odometry
{ 
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>;

  /**
   * @class VisualOdometryNode
   * @brief ROS2 Node that subscribes to color and depth images, and uses an internal VisualOdometry object for the actual odometry logic.
   * 
   */
  class VisualOdometryNode : public rclcpp::Node
  {
    public:
      /**
       * @brief Construct a new Visual Odometry Node object
       * 
       */
      VisualOdometryNode();

    private:
      
      /**
       * @brief Callback for the color and depth image subscriber
       * 
       * @param color_msg The color image message
       * @param depth_msg The depth image message
       */
      void updateSync(
          const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
          const sensor_msgs::msg::Image::ConstSharedPtr depth_msg);
      
      /**
       * @brief CameraInfo callback
       * 
       * @param msg The camera info message
       */
      void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

      cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

      // Subscribers
      message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_; 
      message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_; 
      rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_; 

      // Publishers 
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

      /// Synchronizer
      std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
      
      /// Visual Odometry object
      std::unique_ptr<VisualOdometry> visual_odometry_; 

      // Parameters
      std::string color_image_topic_;
      std::string depth_image_topic_;
      std::string camera_info_topic_;
      std::string odometry_topic_;
      std::string odometry_debug_topic_;
      double max_depth_odom_;
      double min_depth_odom_;
      bool odometry_debug_image_;

      // Other attributes
      bool camera_info_arrived_;
  };  

} // Namespace perception_pipeline
} // Namespace visual_odometry


#endif // VIS_ODOM_NODE_