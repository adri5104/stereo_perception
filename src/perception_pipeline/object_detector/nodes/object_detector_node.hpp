/**
 * @file object_detector_node.hpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief ROS2 Node that subscribes to a 6D image topic and uses an internal object detector for the cluster detection logic.
 * 
 */

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
#include "stereo_perception_msgs/msg/clustered_object.hpp"
#include "stereo_perception_msgs/msg/clustered_object_array.hpp"

namespace stereo_perception
{
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

    /**
     * @brief Helper function to convert a pcl::PointXYZ to a geometry_msgs::msg::Point
     * 
     * @param pcl_point 
     * @return geometry_msgs::msg::Point 
     */
    geometry_msgs::msg::Point pclPointToGeometryMsgPoint(const pcl::PointXYZ& pcl_point);

    /**
     * @brief gets the bounding box marker for a cluster
     * 
     * @param cluster 
     * @return visualization_msgs::msg::Marker  
     */
    visualization_msgs::msg::Marker clusterToBoundingBoxMarker(const WorldEntity& cluster);

    /**
     * @brief gets the bounding box marker for a cluster
     * 
     * @param cluster 
     * @return visualization_msgs::msg::Marker  
     */
    visualization_msgs::msg::Marker clusterToArrowMarker(const WorldEntity& cluster);

    /**
     * @brief Publishes the clusters as markers
     * 
     * @param clusters 
     */
    void publishClustersMarkers(const std::vector<WorldEntity>& clusters);

    /**
     * @brief Publishes the clusters as a stereo_perception_msgs::msg::ClusteredObjectArray
     * 
     * @param clusters 
     */
    void publishClusters(const std::vector<WorldEntity>& clusters);

    /// Object detector instance
    std::unique_ptr<ObjectDetector> object_detector_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr output_markers_pub_;
    rclcpp::Publisher<stereo_perception_msgs::msg::ClusteredObjectArray>::SharedPtr output_clusters_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_6d_sub_;

    // Parameters
    std::string input_6d_topic_;
    std::string output_markers_topic_;
    std::string output_clusters_topic_;
    std::string frame_id_;
  };

} // namespace object_detector
} // namespace perception_pipeline
} // namespace stereo_perception

#endif // OBJECT_DETECTOR_NODE_HPP
