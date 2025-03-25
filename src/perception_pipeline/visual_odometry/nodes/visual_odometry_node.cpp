#include "visual_odometry_node.hpp"

namespace perception_pipeline
{
namespace visual_odometry
{

  VisualOdometryNode::VisualOdometryNode() 
  : Node("visual_odometry_node"), camera_info_arrived_(false), camera_tf_accumulated_(cv::Mat::eye(4, 4, CV_64F))  // 4x4 identity matrix
 {
    // Get parameters
    this->declare_parameter("color_image_topic", "/camera/color/image_raw");
    this->declare_parameter("depth_image_topic", "/camera/depth/image_raw");
    this->declare_parameter("camera_info_topic", "/camera/depth/camera_info");
    this->declare_parameter("odometry_debug_topic", "/twist");
    this->declare_parameter("camera_frame_tf_topic", "/camera_frame_tf");
    this->declare_parameter("max_depth_odom", 10.0);
    this->declare_parameter("min_depth_odom", 0.1);
    this->declare_parameter("odometry_debug_image", true);
    this->declare_parameter("apply_statistical_filtering", true);
    this->declare_parameter("apply_expotential_smoothing", true);
    this->declare_parameter("exponential_alpha", 0.1);
    this->declare_parameter("publish_path", true);
    this->declare_parameter("path_topic", "/path");
    this->declare_parameter("publish_odom", true);
    this->declare_parameter("odom_topic", "/odom");

    this->get_parameter("color_image_topic", color_image_topic_);
    this->get_parameter("depth_image_topic", depth_image_topic_);
    this->get_parameter("camera_info_topic", camera_info_topic_);
    this->get_parameter("odometry_debug_topic", odometry_debug_topic_);
    this->get_parameter("camera_frame_tf_topic", camera_frame_tf_topic_);
    this->get_parameter("max_depth_odom", max_depth_odom_);
    this->get_parameter("min_depth_odom", min_depth_odom_);
    this->get_parameter("odometry_debug_image", odometry_debug_image_);
    this->get_parameter("publish_path", publish_path_);
    this->get_parameter("path_topic", path_topic_);
    this->get_parameter("publish_odom", publish_odom_);
    this->get_parameter("odom_topic", odom_topic_);

    RCLCPP_INFO(this->get_logger(), "================== Visual Odometry Node Parameters ==================");
    RCLCPP_INFO(this->get_logger(), "color_image_topic: '%s'", color_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_image_topic: '%s'", depth_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_info_topic: '%s'", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_frame_tf_topic: '%s'", camera_frame_tf_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "odometry_debug_topic: '%s'", odometry_debug_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "max_depth_odom: '%f'", max_depth_odom_);
    RCLCPP_INFO(this->get_logger(), "min_depth_odom: '%f'", min_depth_odom_);
    RCLCPP_INFO(this->get_logger(), "odometry_debug_image: '%d'", odometry_debug_image_);
    RCLCPP_INFO(this->get_logger(), "publish_path: '%d'", publish_path_);
    RCLCPP_INFO(this->get_logger(), "path_topic: '%s'", path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_odom: '%d'", publish_odom_);
    RCLCPP_INFO(this->get_logger(), "odom_topic: '%s'", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "======================================================================");
  
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
    camera_frame_tf_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(camera_frame_tf_topic_, 10);
    if (odometry_debug_image_)
      debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(odometry_debug_topic_, 10);
    if (publish_path_)
    {
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
      path_msg_.header.frame_id = "camera_optical_frame_initial";
    }
    if (publish_odom_)
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    // Create camera_info subscriber
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10, 
      std::bind(&VisualOdometryNode::cameraInfoCallback, this, std::placeholders::_1));

    
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

    // Convert relative translation and rotation to 4x4 transformation matrix
    cv::Mat relative_transform = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(relative_transform(cv::Rect(0, 0, 3, 3))); // Copy rotation
    translation.copyTo(relative_transform(cv::Rect(3, 0, 1, 3))); // Copy translation

    // Invert the relative transform
    cv::Mat inverse_relative_transform = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat inverse_rotation = rotation.t(); // Transpose of rotation matrix (R^-1)
    cv::Mat inverse_translation = -inverse_rotation * translation; // Inverted translation (-R^T * T)

    inverse_rotation.copyTo(inverse_relative_transform(cv::Rect(0, 0, 3, 3)));
    inverse_translation.copyTo(inverse_relative_transform(cv::Rect(3, 0, 1, 3)));

    // Accumulate the global transform
    camera_tf_accumulated_ = camera_tf_accumulated_ * inverse_relative_transform;

    // **Publish relative transform (previous -> current camera frame)**
    geometry_msgs::msg::TransformStamped transform_msg;
    
    transform_msg.header.stamp = color_msg->header.stamp;
    transform_msg.header.frame_id = "previous_camera_frame";
    transform_msg.child_frame_id = "current_camera_frame";
    transform_msg.transform.translation.x = translation.at<double>(0);
    transform_msg.transform.translation.y = translation.at<double>(1);
    transform_msg.transform.translation.z = translation.at<double>(2);
    tf2::Matrix3x3 tf_rotation(
      rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
      rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
      rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2)
    );
    tf2::Quaternion tf_quat;
    tf_rotation.getRotation(tf_quat);
    transform_msg.transform.rotation.x = tf_quat.x();
    transform_msg.transform.rotation.y = tf_quat.y();
    transform_msg.transform.rotation.z = tf_quat.z();
    transform_msg.transform.rotation.w = tf_quat.w();
    camera_frame_tf_pub_->publish(transform_msg);

    
    // Broadcast absolute transform (initial -> current camera frame)
    tf2::Vector3 absolute_translation(
        camera_tf_accumulated_.at<double>(0, 3),
        camera_tf_accumulated_.at<double>(1, 3),
        camera_tf_accumulated_.at<double>(2, 3)
    );

    tf2::Matrix3x3 absolute_rotation(
        camera_tf_accumulated_.at<double>(0, 0), camera_tf_accumulated_.at<double>(0, 1), camera_tf_accumulated_.at<double>(0, 2),
        camera_tf_accumulated_.at<double>(1, 0), camera_tf_accumulated_.at<double>(1, 1), camera_tf_accumulated_.at<double>(1, 2),
        camera_tf_accumulated_.at<double>(2, 0), camera_tf_accumulated_.at<double>(2, 1), camera_tf_accumulated_.at<double>(2, 2)
    );

    tf2::Quaternion absolute_quat;
    absolute_rotation.getRotation(absolute_quat);

    geometry_msgs::msg::TransformStamped absolute_tf_msg;
    absolute_tf_msg.header.stamp = color_msg->header.stamp;
    absolute_tf_msg.header.frame_id = "camera_optical_frame_initial";   
    absolute_tf_msg.child_frame_id = "camera_optical_frame";

    absolute_tf_msg.transform.translation.x = absolute_translation.x();
    absolute_tf_msg.transform.translation.y = 0.0;
    absolute_tf_msg.transform.translation.z = absolute_translation.z();
    absolute_tf_msg.transform.rotation.x = absolute_quat.x();
    absolute_tf_msg.transform.rotation.y = absolute_quat.y();
    absolute_tf_msg.transform.rotation.z = absolute_quat.z();
    absolute_tf_msg.transform.rotation.w = absolute_quat.w();

    tf_broadcaster_->sendTransform(absolute_tf_msg);

    // Publish debug image
    if (odometry_debug_image_)
    {
      sensor_msgs::msg::Image::SharedPtr debug_image_msg = cv_bridge::CvImage(
        color_msg->header, "bgr8", debug_image).toImageMsg();
      debug_pub_->publish(*debug_image_msg);
    }

    if (publish_odom_)
    {
      // Publish odometry
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = color_msg->header.stamp;
      odom_msg.header.frame_id = "camera_optical_frame_initial";
      odom_msg.child_frame_id = "camera_optical_frame";
      odom_msg.pose.pose.position.x = absolute_translation.x();
      odom_msg.pose.pose.position.y = 0.0;
      odom_msg.pose.pose.position.z = absolute_translation.z();
      odom_msg.pose.pose.orientation.x = absolute_quat.x();
      odom_msg.pose.pose.orientation.y = absolute_quat.y();
      odom_msg.pose.pose.orientation.z = absolute_quat.z();
      odom_msg.pose.pose.orientation.w = absolute_quat.w();
      odom_pub_->publish(odom_msg);
    }

    if (publish_path_)
    {
      // Publish path
      path_msg_.header.stamp = color_msg->header.stamp;
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = color_msg->header.stamp;
      pose_msg.header.frame_id = "camera_optical_frame_initial";
      pose_msg.pose.position.x = absolute_translation.x();
      pose_msg.pose.position.y = 0.0;
      pose_msg.pose.position.z = absolute_translation.z();
      pose_msg.pose.orientation.x = absolute_quat.x();
      pose_msg.pose.orientation.y = absolute_quat.y();
      pose_msg.pose.orientation.z = absolute_quat.z();
      pose_msg.pose.orientation.w = absolute_quat.w();
      path_msg_.poses.push_back(pose_msg);
      path_pub_->publish(path_msg_);
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


    auto node = std::make_shared<perception_pipeline::visual_odometry::VisualOdometryNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}