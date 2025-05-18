/**
 * @file stereo_perception_edgar_bridge_node.hpp
 * @author Adrian Rieker
 * @brief Header file for the StereoPerceptionEdgarBridgeNode class
 * 
 */
#ifndef _STEREO_PERCEPTION_EDGAR_BRIDGE_NODE_HPP_
#define _STEREO_PERCEPTION_EDGAR_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include "tum_edgar_can_msgs/msg/tum_edgar_motion.hpp"
#include "tod_automation_msgs/msg/trajectory.hpp"
#include "tod_automation_msgs/msg/trajectory_point.hpp"

namespace stereo_perception
{
namespace utils
{
namespace stereo_perception_edgar_bridge
{

/**
 * @brief Node that bridges the stereo perception and EDGAR
 * 
 * This node subscribes to the EDGAR motion topic and trayectory topic, and republishes
 * the information in ROS2 standard twist and path messages.
 * 
 * The published topics are in the camera_optical_frame, which is the same as the stereo camera frame.
 * Zero is the center of the camera, Z-axis is pointing forward, Y-axis is pointing to the left, and X-axis is pointing down.
 * 
 */
class StereoPerceptionEdgarBridgeNode : public rclcpp::Node
{
  public:

    /**
     * @brief Construct a new Stereo Perception Edgar Bridge Node object
     * 
     */
    StereoPerceptionEdgarBridgeNode();
  
  private:
  
    /**
     * @brief Callback for the EDGAR motion topic. Takes the received
     * motion message and converts it to a ROS2 twist message in the camera_optical_frame.
     * 
     * 
     * @param msg The received EDGAR motion message
     */
    void edgarMotionCallback(const tum_edgar_can_msgs::msg::TUMEdgarMotion::SharedPtr msg);

    /**
     * @brief Callback for the EDGAR trajectory topic. Takes the received trayectory
     * message and converts it to a ROS2 path message in the camera_optical_frame.
     * 
     * @param msg The received EDGAR trajectory message
     */
    void edgarTrajectoryCallback(const tod_automation_msgs::msg::Trajectory::SharedPtr msg);


    /// \name Subscribers
    /// These subscriptions handle incoming motion and trajectory data.
    /// \{

    /// Subscriber to Edgar's motion messages (e.g., velocity, yaw rate).
    rclcpp::Subscription<tum_edgar_can_msgs::msg::TUMEdgarMotion>::SharedPtr sub_edgar_motion_;

    /// Subscriber to the planned trajectory for Edgar.
    rclcpp::Subscription<tod_automation_msgs::msg::Trajectory>::SharedPtr sub_edgar_trajectory_;

    /// \}

    /// \name Publishers
    /// These publications send out the converted data in standard ROS2 formats.
    /// \{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    /// \}

    /// \name ROS2 Parameters
    /// These parameters can be set in the launch file or via command line.
    /// \{
    std::string edgar_motion_topic_; ///< Topic for receiving EDGAR motion messages.
    std::string edgar_trajectory_topic_; ///< Topic for receiving EDGAR trajectory messages.
    std::string twist_topic_; ///< Topic for publishing the converted twist messages.
    std::string path_topic_; ///< Topic for publishing the converted path messages.
    /// \}

};


} // namespace stereo_perception_edgar_bridge
} // namespace utils
} // namespace stereo_perception

#endif