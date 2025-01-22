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
      odometry_debug_image_);

    // Create message_filters
    color_sub_.subscribe(this, color_image_topic_);
    depth_sub_.subscribe(this, depth_image_topic_);

    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), color_sub_, depth_sub_);

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

    // Publish odometry
    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header = color_msg->header;
    odometry_msg.twist.twist.linear.x = translation.at<double>(0);
    odometry_msg.twist.twist.linear.y = translation.at<double>(1);
    odometry_msg.twist.twist.linear.z = translation.at<double>(2);
    odometry_msg.twist.twist.angular.x = rotation.at<double>(0); // Roll
    odometry_msg.twist.twist.angular.y = rotation.at<double>(1); // Pitch
    odometry_msg.twist.twist.angular.z = rotation.at<double>(2); // Yaw   
    odometry_msg.twist.covariance = {
      covariance.at<double>(0, 0), covariance.at<double>(0, 1), covariance.at<double>(0, 2), 0, 0, 0,
      covariance.at<double>(1, 0), covariance.at<double>(1, 1), covariance.at<double>(1, 2), 0, 0, 0,
      covariance.at<double>(2, 0), covariance.at<double>(2, 1), covariance.at<double>(2, 2), 0, 0, 0,
      0, 0, 0, covariance.at<double>(3, 3), covariance.at<double>(3, 4), covariance.at<double>(3, 5),
      0, 0, 0, covariance.at<double>(4, 3), covariance.at<double>(4, 4), covariance.at<double>(4, 5),
      0, 0, 0, covariance.at<double>(5, 3), covariance.at<double>(5, 4), covariance.at<double>(5, 5)
    };
    odometry_pub_->publish(odometry_msg);

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