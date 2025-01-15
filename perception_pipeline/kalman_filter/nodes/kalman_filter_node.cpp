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
  this->declare_parameter<std::string>("depth_topic", "/device_0/sensor_0/Depth_0/image/data");
  this->declare_parameter<std::string>("camera_info_topic", "/perception_pipeline/camera_info_sync");
  this->declare_parameter<std::string>("color_image_topic", "/device_0/sensor_1/Color_0/image/data");
  this->declare_parameter<std::string>("output_6d_topic", "/output_6d");
  this->declare_parameter<std::string>("debug_image_topic", "/debug_image");

  // Read parameters
  optical_flow_topic_ = this->get_parameter("optical_flow_topic").as_string();
  depth_topic_        = this->get_parameter("depth_topic").as_string();
  camera_info_topic_  = this->get_parameter("camera_info_topic").as_string();
  color_image_topic_  = this->get_parameter("color_image_topic").as_string();
  output_6d_topic_    = this->get_parameter("output_6d_topic").as_string();
  debug_image_topic_  = this->get_parameter("debug_image_topic").as_string();

  RCLCPP_INFO(this->get_logger(), "optical_flow_topic: '%s'", optical_flow_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "depth_topic: '%s'", depth_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "camera_info_topic: '%s'", camera_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "color_image_topic: '%s'", color_image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_6d_topic: '%s'", output_6d_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "debug_image_topic: '%s'", debug_image_topic_.c_str());

  // Create message_filters subscribers
  optical_flow_sub_.subscribe(this, optical_flow_topic_, rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
  color_sub_.subscribe(this, color_image_topic_, rmw_qos_profile_sensor_data);

  // Create the synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100),   // queue size
            optical_flow_sub_,
            depth_sub_,
            color_sub_);

  // Register the synchronized callback
  sync_->registerCallback(&KalmanFilterNode::updateSync, this);

  // Camera info subscription (standard rclcpp subscription)
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 10,
    std::bind(&KalmanFilterNode::cameraInfoCallback, this, std::placeholders::_1));

  // Publishers
  debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 10);
  output_6d_pub_   = this->create_publisher<sensor_msgs::msg::Image>(output_6d_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "KalmanFilterNode with message_filters started.");
}

// Synchronized callback
void KalmanFilterNode::updateSync(
  const sensor_msgs::msg::Image::ConstSharedPtr flow_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr color_msg)
{
  RCLCPP_INFO(this->get_logger(), "updateSync() called with synchronized messages");
  cout << "Callback" << endl;
  // Convert each to cv::Mat
  cv::Mat flow_image  = imageMsgToMat(flow_msg);
  cv::Mat depth_image = imageMsgToMat(depth_msg);
  cv::Mat color_image = imageMsgToMat(color_msg);

  // Perform your KalmanCore logic
  KalmanCoreErrorCode result = kalman_core_.updateSyncedData(flow_image, depth_image, color_image);

  // Retrieve outputs
  cv::Mat output_6d, output_debug_image;
  kalman_core_.getOutput(output_6d, output_debug_image);

  // Convert to sensor_msgs
  sensor_msgs::msg::Image::SharedPtr output_6d_msg =
    cv_bridge::CvImage(flow_msg->header, "bgr8", output_6d).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr debug_image_msg =
    cv_bridge::CvImage(flow_msg->header, "bgr8", output_debug_image).toImageMsg();

  // Publish
  if (result == KalmanCoreErrorCode::OK)
    RCLCPP_INFO(this->get_logger(), "KalmanCore output: OK");
  else
    RCLCPP_ERROR(this->get_logger(), "KalmanCore error: %s", getErrorMessage(result).c_str());

  debug_image_pub_->publish(*debug_image_msg);
  output_6d_pub_->publish(*output_6d_msg);
}

// Camera info callback
void KalmanFilterNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{

  
  double fx = msg->k[0];
  double fy = msg->k[4];
  double cx = msg->k[2];
  double cy = msg->k[5];

  kalman_core_.setCameraParameters(fx, fy, cx, cy);
}

// Convert sensor_msgs::Image -> cv::Mat
cv::Mat KalmanFilterNode::imageMsgToMat(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    return cv_bridge::toCvCopy(msg, msg->encoding)->image;
  }
  catch(const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }
}

} // namespace kalman_filter
} // namespace perception_pipeline

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_pipeline::kalman_filter::KalmanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
