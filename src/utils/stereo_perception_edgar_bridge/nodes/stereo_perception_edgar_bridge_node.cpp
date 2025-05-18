#include "stereo_perception_edgar_bridge_node.hpp"

namespace stereo_perception
{
namespace utils
{
namespace stereo_perception_edgar_bridge
{

  StereoPerceptionEdgarBridgeNode::StereoPerceptionEdgarBridgeNode()
  : Node("stereo_perception_edgar_bridge_node")
  {
    // Declare parameters
    declare_parameter("edgar_motion_topic", "/edgar/motion");
    declare_parameter("edgar_trajectory_topic", "/edgar/trajectory");
    declare_parameter("twist_topic", "/stereo_perception/ego_twist");
    declare_parameter("path_topic", "/stereo_perception/ego_path");

    // Get parameters
    get_parameter("edgar_motion_topic", edgar_motion_topic_);
    get_parameter("edgar_trajectory_topic", edgar_trajectory_topic_);
    get_parameter("twist_topic", twist_topic_); 
    get_parameter("path_topic", path_topic_);

    // Log the parameters
    RCLCPP_INFO(get_logger(), "EDGAR motion topic: %s", edgar_motion_topic_.c_str());
    RCLCPP_INFO(get_logger(), "EDGAR trajectory topic: %s", edgar_trajectory_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Twist topic: %s", twist_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Path topic: %s", path_topic_.c_str());

    // Create subscribers
    sub_edgar_motion_ = create_subscription<tum_edgar_can_msgs::msg::TUMEdgarMotion>(
      edgar_motion_topic_, 
      10,
      std::bind(
        &StereoPerceptionEdgarBridgeNode::edgarMotionCallback, 
        this, 
        std::placeholders::_1));
    
    sub_edgar_trajectory_ = create_subscription<tod_automation_msgs::msg::Trajectory>(
      edgar_trajectory_topic_, 
      10,
      std::bind(
        &StereoPerceptionEdgarBridgeNode::edgarTrajectoryCallback, 
        this, 
        std::placeholders::_1));

    // Create publishers
    pub_twist_ = create_publisher<geometry_msgs::msg::Twist>(twist_topic_, 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
  }

 
  void StereoPerceptionEdgarBridgeNode::edgarMotionCallback(const tum_edgar_can_msgs::msg::TUMEdgarMotion::SharedPtr msg)
  {
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
    
    // Camera optical frame
    twist_msg->linear.z = msg->vehicle_velocity; // Forward velocity
    twist_msg->angular.y = -msg->yaw_rate;
    pub_twist_->publish(*twist_msg);
  }

  void StereoPerceptionEdgarBridgeNode::edgarTrajectoryCallback(const tod_automation_msgs::msg::Trajectory::SharedPtr msg)
  {
    // Create a new Path message in camera_optical_frame
    auto path_msg = std::make_shared<nav_msgs::msg::Path>();
    path_msg->header = msg->header;
    path_msg->header.frame_id = "camera_optical_frame";
    path_msg->header.stamp = this->now();
    
    // Fill the path message with the trajectory points
    // Convert each trajectory point from the vehicle frame to the camera frame
    for (const auto& point : msg->points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "camera_optical_frame";
      pose.pose.position.z = point.pose.pose.position.x;
      pose.pose.position.x = point.pose.pose.position.y;
      pose.pose.position.y = point.pose.pose.position.z;
      pose.pose.orientation = point.pose.pose.orientation;
      pose.header.stamp = this->now();

      // Add the pose to the path message
      path_msg->poses.push_back(pose);
    }
    
    // Publish the path message
    pub_path_->publish(*path_msg);

  }

} // namespace stereo_perception_edgar_bridge
} // namespace utils
} // namespace stereo_perception

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stereo_perception::utils::stereo_perception_edgar_bridge::StereoPerceptionEdgarBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}