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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  
#include <tf2/LinearMath/Matrix3x3.h>   
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#ifdef ROS_VERSION_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
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

      /**
       * @brief Convert sensor_msgs::Image -> cv::Mat
       * 
       * @param msg The image message
       * @return cv::Mat The image as cv::Mat
       */
      cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

      /// Accumulated transform from world to camera optical frame
      cv::Mat camera_tf_accumulated_;

      // Subscribers
      message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_; 
      message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_; 
      rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_; 

      // Publishers 
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
      rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr camera_frame_tf_pub_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

      /// Synchronizer
      std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

      /// Transform broadcaster
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
      std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
      
      /// Visual Odometry object
      std::unique_ptr<VisualOdometry> visual_odometry_; 

      // Parameters
      std::string color_image_topic_; ///< Color image topic
      std::string depth_image_topic_; ///< Depth image topic
      std::string camera_info_topic_; ///< Camera info topic
      std::string odometry_debug_topic_; ///< Debug image topic
      std::string camera_frame_tf_topic_; ///< Camera frame transform between two consewuence frames
      bool publish_path_; ///< Publish path
      std::string path_topic_; ///< Path topic
      nav_msgs::msg::Path path_msg_; ///< Path message
      bool publish_odom_; ///< Publish odometry
      std::string odom_topic_; ///< Odometry topic
      double max_depth_odom_;
      double min_depth_odom_;
      bool odometry_debug_image_;

      // Other attributes
      bool camera_info_arrived_;
  };  

} // Namespace perception_pipeline
} // Namespace visual_odometry


#endif // VIS_ODOM_NODE_