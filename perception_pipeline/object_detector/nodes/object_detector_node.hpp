#ifndef OBJECT_DETECTOR_NODE_HPP
#define OBJECT_DETECTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "object_detector/object_detector.hpp"
#include "object_detector/world_entity.hpp"

namespace perception_pipeline
{
namespace object_detector
{ 
  /**
   * @class ObjectDetectorNode
   * @brief ROS2 Node that subscribes to a 6D image topic and uses an internal object detector for the actual detection logic.
   */
  class ObjectDetectorNode : public rclcpp::Node
  {
  public:
    /**
     * @brief Construct a new Object Detector Node object
     */
    ObjectDetectorNode();

  private:
    
    /**
     * @brief Callback for the 6D image topic
     * 
     * @param msg Shared pointer to the received image message
     */
    void input6dCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Helper function to convert a ROS2 image message to a cv::Mat
     * 
     * @param msg Shared pointer to the received image message
     * @return cv::Mat The converted cv::Mat
     */
    cv::Mat rosImageToCvMat(const sensor_msgs::msg::Image::SharedPtr msg);

    /// Object detector instance
    std::unique_ptr<ObjectDetector> object_detector_;

    // Subscribers 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_6d_sub_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr output_markers_pub_;

    // Attributes
    cv::Mat input_6d_image_filtered_;

    // Parameters
    std::string input_6d_topic_;
    std::string output_markers_topic_;
  };
} // namespace object_detector
} // namespace perception_pipeline
#endif // OBJECT_DETECTOR_NODE_HPP
