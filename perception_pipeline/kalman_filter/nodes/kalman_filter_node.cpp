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
  this->declare_parameter<std::string>("depth_topic", "/depth");
  this->declare_parameter<std::string>("camera_info_topic", "/camera_info");
  this->declare_parameter<std::string>("color_image_topic", "/color_image");
  this->declare_parameter<std::string>("output_6d_topic", "/output_6d");
  this->declare_parameter<std::string>("debug_image_topic", "/debug_image");

  // Read parameters
  optical_flow_topic_ = this->get_parameter("optical_flow_topic").as_string();
  depth_topic_    = this->get_parameter("depth_topic").as_string();
  camera_info_topic_  = this->get_parameter("camera_info_topic").as_string();
  color_image_topic_  = this->get_parameter("color_image_topic").as_string();
  output_6d_topic_  = this->get_parameter("output_6d_topic").as_string();
  debug_image_topic_  = this->get_parameter("debug_image_topic").as_string();

  RCLCPP_INFO(this->get_logger(),
              "optical_flow_topic: '%s'", optical_flow_topic_.c_str());
  RCLCPP_INFO(this->get_logger(),
              "depth_topic: '%s'", depth_topic_.c_str());
  RCLCPP_INFO(this->get_logger(),
              "camera_info_topic: '%s'", camera_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(),
              "color_image_topic: '%s'", color_image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(),
              "output_6d_topic: '%s'", output_6d_topic_.c_str());
  RCLCPP_INFO(this->get_logger(),
              "debug_image_topic: '%s'", debug_image_topic_.c_str());


  // Subscriptions
  optical_flow_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    optical_flow_topic_, 
    rclcpp::QoS(10), 
    std::bind(&KalmanFilterNode::opticalFlowCallback, this, std::placeholders::_1));

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, 
    rclcpp::QoS(10), 
    std::bind(&KalmanFilterNode::depthCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 
    rclcpp::QoS(10),
    std::bind(&KalmanFilterNode::cameraInfoCallback, this, std::placeholders::_1));

  color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    color_image_topic_, 
    rclcpp::QoS(10),
    std::bind(&KalmanFilterNode::colorImageCallback, this, std::placeholders::_1));
  
  // Publishers
  debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    debug_image_topic_, 
    rclcpp::QoS(10));
  
  output_6d_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_6d_topic_, 
    rclcpp::QoS(10));
 

  RCLCPP_INFO(this->get_logger(), "KalmanFilterNode started.");
}

void KalmanFilterNode::opticalFlowCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat flow_image = imageMsgToMat(msg);
  RCLCPP_INFO(this->get_logger(),
              "Optical Flow image received (%d x %d).",
              flow_image.cols, flow_image.rows);

  // Forward the optical flow to the KalmanCore update
  kalman_core_.updateSyncedData(flow_image, depth_image_, color_image_);

  // Get the output of the KalmanCore
  cv::Mat output_6d, output_debug_image;
  sensor_msgs::msg::Image::SharedPtr output_6d_msg, output_debug_image_msg;



  kalman_core_.getOutput(output_6d, output_debug_image);

  output_6d_msg = cv_bridge::CvImage(msg->header, "bgr8", output_6d).toImageMsg();
  output_debug_image_msg = cv_bridge::CvImage(msg->header, "bgr8", output_debug_image).toImageMsg();

  // Publish the output
  debug_image_pub_->publish(*output_debug_image_msg);
  output_6d_pub_->publish(*output_6d_msg);
}

void KalmanFilterNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat disp_image = imageMsgToMat(msg);
  RCLCPP_INFO(this->get_logger(),
              "depth image received (%d x %d).",
              disp_image.cols, disp_image.rows);

  depth_image_ = disp_image;

}

void KalmanFilterNode::colorImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat color_image = imageMsgToMat(msg);
  RCLCPP_INFO(this->get_logger(),
              "Color image received (%d x %d).",
              color_image.cols, color_image.rows);

  color_image_ = color_image;
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