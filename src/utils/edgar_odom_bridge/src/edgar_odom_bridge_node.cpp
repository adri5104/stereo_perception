// edgar_odom_bridge_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class EdgarOdomBridgeNode : public rclcpp::Node {
public:
  EdgarOdomBridgeNode()
  : Node("edgar_odom_bridge_node"), received_first_msg_(false) {
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", 10,
      std::bind(&EdgarOdomBridgeNode::odomCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
      "/multisense/perception_pipeline/edgar_tf", 10);

    RCLCPP_INFO(this->get_logger(), "edgar_odom_bridge_node started");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_;
  nav_msgs::msg::Odometry prev_odom_;
  bool received_first_msg_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!received_first_msg_) {
      prev_odom_ = *msg;
      received_first_msg_ = true;
      return;
    }

    // 1. Compute translation in world frame
    tf2::Vector3 delta_world(
      msg->pose.pose.position.x - prev_odom_.pose.pose.position.x,
      msg->pose.pose.position.y - prev_odom_.pose.pose.position.y,
      msg->pose.pose.position.z - prev_odom_.pose.pose.position.z);

    // 2. Get current orientation
    tf2::Quaternion q_curr;
    tf2::fromMsg(msg->pose.pose.orientation, q_curr);

    // 3. Rotate delta into camera frame
    tf2::Matrix3x3 R(q_curr.inverse());
    tf2::Vector3 delta_cam = R * delta_world;

    // 4. Convert to camera optical frame (Z forward, X right, Y down)

tf2::Vector3 delta_optical;
delta_optical.setX(  delta_cam.y() );   // optical X ← base Y
delta_optical.setY( delta_cam.z() );   // optical Y ← -base Z
delta_optical.setZ( delta_cam.x() );   // optical Z ← -base X → avanzar = z negativo



    // 5. Prepare transform message
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = "multisense/left_camera_optical_frame";
    tf_msg.child_frame_id = "multisense/left_camera_optical_frame_previous";
    tf_msg.transform.translation.x = delta_optical.x();
    tf_msg.transform.translation.y = delta_optical.y();
    tf_msg.transform.translation.z = delta_optical.z();
    tf_msg.transform.rotation = msg->pose.pose.orientation;  // Optional

    pub_->publish(tf_msg);
    prev_odom_ = *msg;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EdgarOdomBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
