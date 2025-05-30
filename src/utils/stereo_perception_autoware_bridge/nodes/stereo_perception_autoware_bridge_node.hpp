/**
 * @file stereo_perception_autoware_bridge_node.hpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief  Header file for the StereoPerceptionAutowareBridgeNode class 
 */
#ifndef _STEREO_PERCEPTION_AUTOWARE_BRIDGE_NODE_HPP_
#define _STEREO_PERCEPTION_AUTOWARE_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_object_kinematics.hpp"
#include "stereo_perception_msgs/msg/clustered_object_array.hpp"
#include "stereo_perception_msgs/msg/clustered_object.hpp"
#include "stereo_perception_msgs/msg/bounding_box.hpp"

namespace stereo_perception
{
namespace utils
{
namespace stereo_perception_autoware_bridge
{

/**
 * @brief Node that bridges Autoware perception and stereo perception
 * This node subscribes to the Autoware tracked objects topic and converts it to a stereo perception clustered object array message.
 * The published topics are in the camera_optical_frame, which is the same as the stereo camera frame.
 * Zero is the center of the camera, Z-axis is pointing forward, Y-axis is pointing to the left, and X-axis is pointing down.
 */
class StereoPerceptionAutowareBridgeNode : public rclcpp::Node
{
  public:

      /**
       * @brief Construct a new Stereo Perception Autoware Bridge Node object
       */
      StereoPerceptionAutowareBridgeNode();

  private:
    
    /**
     * @brief Callback for the Autoware tracked objects topic. Takes the received
     * tracked objects message and converts it to a stereo perception clustered object array message.
     * 
     * @param msg The received Autoware tracked objects message
     */
    void autowareTrackedObjectsCallback(const autoware_perception_msgs::msg::TrackedObjects::SharedPtr msg);

    /**
     * @brief Converts a single Autoware tracked object to a stereo perception clustered object.
     * 
     * @param autoware_object The Autoware tracked object to convert
     * @return stereo_perception_msgs::msg::ClusteredObject The converted clustered object
     */
    stereo_perception_msgs::msg::ClusteredObject convertTrackedObjectToClusteredObject(
        const autoware_perception_msgs::msg::TrackedObject & autoware_object);

    rclcpp::Publisher<stereo_perception_msgs::msg::ClusteredObjectArray>::SharedPtr clustered_object_array_pub_; ///< Publisher for clustered object array messages.
    rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr tracked_objects_sub_; ///< Subscriber for Autoware tracked objects messages.
    std::string clustered_object_array_topic_ ;
    std::string tracked_objects_topic_;
    
};

} // namespace stereo_perception_autoware_bridge
} // namespace utils
} // namespace stereo_perception



#endif // _STEREO_PERCEPTION_AUTOWARE_BRIDGE_NODE_HPP_