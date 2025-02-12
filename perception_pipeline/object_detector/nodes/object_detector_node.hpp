#ifndef OBJECT_DETECTOR_NODE_HPP
#define OBJECT_DETECTOR_NODE_HPP

#include <cmath>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#ifdef ROS_VERSION_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/core/cuda.hpp>

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
     * @brief Callback for the 6d image subscriber
     * 
     * @param msg 
     */
    void callback6dImage(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief Helper function to convert a ROS2 image message to a cv::Mat
     * 
     * @param msg Shared pointer to the received image message
     * @return cv::Mat The converted cv::Mat
     */
    cv::Mat rosImageToCvMat(const sensor_msgs::msg::Image::SharedPtr msg);

    geometry_msgs::msg::Point pclPointToGeometryMsgPoint(const pcl::PointXYZ& pcl_point);

    void publishClusters(std::vector<WorldEntity>& clusters);

    /// Object detector instance
    std::unique_ptr<ObjectDetector> object_detector_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr output_markers_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_6d_sub_;

    // Parameters
    std::string input_6d_topic_;
    std::string output_markers_topic_;
  };
} // namespace object_detector
} // namespace perception_pipeline
#endif // OBJECT_DETECTOR_NODE_HPP
