#include "object_detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace perception_pipeline
{
namespace object_detector
{

using namespace std;

ObjectDetectorNode::ObjectDetectorNode() : 
  Node("object_detector_node")
{
  // Get parameters
  this->declare_parameter("input_6d_topic", "/6d_image");
  this->declare_parameter("output_markers_topic", "/object_markers");
  this->get_parameter("input_6d_topic", input_6d_topic_);
  this->get_parameter("output_markers_topic", output_markers_topic_);

  // Subscribers
  input_6d_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_6d_topic_, 
    10, 
    std::bind(&ObjectDetectorNode::input6dCallback, 
    this, 
    std::placeholders::_1));

  // Publishers
  output_markers_pub_ = 
    this->create_publisher<visualization_msgs::msg::MarkerArray>(output_markers_topic_, 10);

  // Object detector
  object_detector_ = std::make_unique<ObjectDetector>();

}
 
void object_detector::ObjectDetectorNode::input6dCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto result = object_detector_->update6DImage(rosImageToCvMat(msg));
  if (result != objectDetectorErrorCode::OK)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", getErrorMessageObjectDetector(result).c_str());
    return;
  }
}

cv::Mat ObjectDetectorNode::rosImageToCvMat(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }

  return cv_ptr->image;
}


} // namespace object_detector
} // namespace perception_pipeline

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_pipeline::object_detector::ObjectDetectorNode>());
  rclcpp::shutdown();
  return 0;
}   