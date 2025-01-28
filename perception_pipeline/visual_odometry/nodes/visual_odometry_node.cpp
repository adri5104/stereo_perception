#include "visual_odometry_node.hpp"

namespace perception_pipeline
{
namespace visual_odometry
{

  VisualOdometryNode::VisualOdometryNode() 
  : Node("visual_odometry_node"), camera_info_arrived_(false) {
    // Get parameters
    this->declare_parameter("color_image_topic", "/camera/color/image_raw");
    this->declare_parameter("depth_image_topic", "/camera/depth/image_raw");
    this->declare_parameter("camera_info_topic", "/camera/depth/camera_info");
    this->declare_parameter("odometry_topic", "/odometry");
    this->declare_parameter("odometry_debug_topic", "/twist");
    this->declare_parameter("max_depth_odom", 10.0);
    this->declare_parameter("min_depth_odom", 0.1);
    this->declare_parameter("odometry_debug_image", true);
    this->declare_parameter("apply_statistical_filtering", true);
    this->declare_parameter("apply_expotential_smoothing", true);
    this->declare_parameter("exponential_alpha", 0.1);

    this->get_parameter("color_image_topic", color_image_topic_);
    this->get_parameter("depth_image_topic", depth_image_topic_);
    this->get_parameter("camera_info_topic", camera_info_topic_);
    this->get_parameter("odometry_topic", odometry_topic_);
    this->get_parameter("odometry_debug_topic", odometry_debug_topic_);
    this->get_parameter("max_depth_odom", max_depth_odom_);
    this->get_parameter("min_depth_odom", min_depth_odom_);
    this->get_parameter("odometry_debug_image", odometry_debug_image_);

    RCLCPP_INFO(this->get_logger(), "color_image_topic: '%s'", color_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_image_topic: '%s'", depth_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_info_topic: '%s'", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "odometry_topic: '%s'", odometry_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "odometry_debug_topic: '%s'", odometry_debug_topic_.c_str());
  
    // Create VisualOdometry object
    visual_odometry_ = std::make_unique<VisualOdometry>(
      min_depth_odom_, 
      max_depth_odom_, 
      odometry_debug_image_,
      get_parameter("apply_statistical_filtering").as_bool(),
      get_parameter("exponential_alpha").as_double(),
      get_parameter("apply_expotential_smoothing").as_bool());
      


    // Create message_filters
    color_sub_.subscribe(this, color_image_topic_);
    depth_sub_.subscribe(this, depth_image_topic_);

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), color_sub_, depth_sub_);

    // Create tf broadcaster  
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sync_->registerCallback(
      std::bind(&VisualOdometryNode::updateSync, 
      this, 
      std::placeholders::_1, 
      std::placeholders::_2));  

    // Create publishers
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);

    // Create camera_info subscriber
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10, 
      std::bind(&VisualOdometryNode::cameraInfoCallback, this, std::placeholders::_1));

    if (odometry_debug_image_)
      debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(odometry_debug_topic_, 10);
  }

  void VisualOdometryNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    // Set camera parameters
    visual_odometry_->setCameraParameters(msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
    camera_info_arrived_ = true;
  }

  void VisualOdometryNode::updateSync(
    const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
  {
    if (!camera_info_arrived_)
    {
      RCLCPP_ERROR(this->get_logger(), "Camera parameters have not been set.");
      return;
    }

    // Convert sensor_msgs::Image -> cv::Mat
    cv::Mat color_image = imageMsgToMat(color_msg);
    cv::Mat depth_image = imageMsgToMat(depth_msg);

    // Update VisualOdometry object
    visual_odometry_->updateSync(color_image, depth_image);

    // Retrieve output
    
    cv::Mat translation, rotation, covariance, debug_image;
    visual_odometry_->getOutput(translation, rotation, covariance, debug_image);

    double roll = atan2(rotation.at<double>(2, 1), rotation.at<double>(2, 2));
    double pitch = asin(-rotation.at<double>(2, 0));
    double yaw = atan2(rotation.at<double>(1, 0), rotation.at<double>(0, 0));
  
    // Publish odometry
    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header = color_msg->header;
    odometry_msg.twist.twist.linear.x = translation.at<double>(0);
    odometry_msg.twist.twist.linear.y = translation.at<double>(1);
    odometry_msg.twist.twist.linear.z = translation.at<double>(2);
    odometry_msg.twist.twist.angular.x = roll; // Roll
    odometry_msg.twist.twist.angular.y = pitch; // Pitch
    odometry_msg.twist.twist.angular.z = yaw; // Yaw   
    odometry_msg.twist.covariance = {
      covariance.at<double>(0, 0), covariance.at<double>(0, 1), covariance.at<double>(0, 2), 0, 0, 0,
      covariance.at<double>(1, 0), covariance.at<double>(1, 1), covariance.at<double>(1, 2), 0, 0, 0,
      covariance.at<double>(2, 0), covariance.at<double>(2, 1), covariance.at<double>(2, 2), 0, 0, 0,
      0, 0, 0, covariance.at<double>(3, 3), covariance.at<double>(3, 4), covariance.at<double>(3, 5),
      0, 0, 0, covariance.at<double>(4, 3), covariance.at<double>(4, 4), covariance.at<double>(4, 5),
      0, 0, 0, covariance.at<double>(5, 3), covariance.at<double>(5, 4), covariance.at<double>(5, 5)
    };
    odometry_pub_->publish(odometry_msg);


      // Construct the relative transform matrix (T_relative)
      cv::Mat T_relative = (cv::Mat_<double>(4, 4) << 
          rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), translation.at<double>(0),
          rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), 0.0,
          rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), translation.at<double>(2),
          0, 0, 0, 1);

      // Initialize or accumulate the transformation
      if (T_world_to_camera_optical_.empty())
      {
          T_world_to_camera_optical_ = T_relative.clone(); // Initialize the accumulated transform
      }
      else
      {
          T_world_to_camera_optical_ = T_world_to_camera_optical_ * T_relative; // Accumulate the relative transform
      }

      // Extract the accumulated translation and rotation
      cv::Mat R_accumulated = T_world_to_camera_optical_(cv::Rect(0, 0, 3, 3)); // Top-left 3x3 block
      cv::Mat t_accumulated = T_world_to_camera_optical_(cv::Rect(3, 0, 1, 3)); // Top-right 3x1 column

    // Convert to quaternion for TF broadcasting
    tf2::Matrix3x3 tf2_rotation(
        R_accumulated.at<double>(0, 0), R_accumulated.at<double>(0, 1), R_accumulated.at<double>(0, 2),
        R_accumulated.at<double>(1, 0), R_accumulated.at<double>(1, 1), R_accumulated.at<double>(1, 2),
        R_accumulated.at<double>(2, 0), R_accumulated.at<double>(2, 1), R_accumulated.at<double>(2, 2));
    tf2::Quaternion tf2_quat;
    tf2_rotation.getRotation(tf2_quat);


    // Publish world -> camera_optical_frame
    geometry_msgs::msg::TransformStamped world_to_camera_optical;
    world_to_camera_optical.header.stamp = this->get_clock()->now();
    world_to_camera_optical.header.frame_id = "world";
    world_to_camera_optical.child_frame_id = "camera_optical_frame";

    world_to_camera_optical.transform.translation.x = - t_accumulated.at<double>(2);
    world_to_camera_optical.transform.translation.y = t_accumulated.at<double>(1);
    world_to_camera_optical.transform.translation.z = 2;

    world_to_camera_optical.transform.rotation.x = 0.0;
    world_to_camera_optical.transform.rotation.y = 0.0;
    world_to_camera_optical.transform.rotation.z = 0.0;
    world_to_camera_optical.transform.rotation.w = 1;

    tf_broadcaster_->sendTransform(world_to_camera_optical);

    // Publish debug image
    if (odometry_debug_image_)
    {
      sensor_msgs::msg::Image::SharedPtr debug_image_msg = cv_bridge::CvImage(
        color_msg->header, "bgr8", debug_image).toImageMsg();
      debug_pub_->publish(*debug_image_msg);
    }
  }

  cv::Mat VisualOdometryNode::imageMsgToMat(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try {
      return cv_bridge::toCvCopy(msg, msg->encoding)->image;
    }
    catch(const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return cv::Mat(); 
    }
  }


} // Namespace perception_pipeline
} // Namespace visual_odometry

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perception_pipeline::visual_odometry::VisualOdometryNode>());
    rclcpp::shutdown();
    return 0;
}