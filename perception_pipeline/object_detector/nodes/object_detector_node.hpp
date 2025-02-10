#ifndef OBJECT_DETECTOR_NODE_HPP
#define OBJECT_DETECTOR_NODE_HPP

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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>
#include <unordered_map>

#include "object_detector/object_detector.hpp"
#include "object_detector/world_entity.hpp"

namespace perception_pipeline
{
namespace object_detector
{ 

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;

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
     * @brief Input callback for the synchronized 6D image topics
     * 
     * @param in6d input 6D image
     * @param in6d_val input 6D validation image
     */
    void inputSyncCallback(const sensor_msgs::msg::Image::SharedPtr in6d, const sensor_msgs::msg::Image::SharedPtr in6d_val);
    
    /**
     * @brief Helper function to convert a ROS2 image message to a cv::Mat
     * 
     * @param msg Shared pointer to the received image message
     * @return cv::Mat The converted cv::Mat
     */
    cv::Mat rosImageToCvMat(const sensor_msgs::msg::Image::SharedPtr msg);

    void publishClusters(const std::vector<WorldEntity>& clusters);


    /// Object detector instance
    std::unique_ptr<ObjectDetector> object_detector_;

    // Message filters subscribers 
    message_filters::Subscriber<sensor_msgs::msg::Image> input_6d_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> input_6d_val_sub_;

    // Syncronizer pointer
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr output_markers_pub_;



    // Parameters
    std::string input_6d_topic_;
    std::string input_6d_val_topic_;
    std::string output_markers_topic_;
  };
} // namespace object_detector
} // namespace perception_pipeline
#endif // OBJECT_DETECTOR_NODE_HPP
