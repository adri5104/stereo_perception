/**
 * @file stereo_perception_autoware_bridge_node.cpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief  Implementation file for the StereoPerceptionAutowareBridgeNode class
 */
#include "stereo_perception_autoware_auto_bridge_node.hpp"

namespace stereo_perception
{
namespace utils
{
namespace stereo_perception_autoware_auto_bridge
{

StereoPerceptionAutowareAutoBridgeNode::StereoPerceptionAutowareAutoBridgeNode()
: Node("stereo_perception_autoware__auto_bridge_node")
{
    // Declare parameters
    declare_parameter<std::string>("clustered_object_array_topic", "/autoware/clustered_objects");
    declare_parameter<std::string>("predicted_objects_topic", "/autoware/predicted_objects");

    // Get parameters
    get_parameter("clustered_object_array_topic", clustered_object_array_topic_);
    get_parameter("predicted_objects_topic", predicted_objects_topic_);
    RCLCPP_INFO(get_logger(), "Clustered Object Array Topic: %s", clustered_object_array_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Predicted Objects Topic: %s", predicted_objects_topic_.c_str());

    // Initialize subscribers and publishers
    clustered_object_array_pub_ = create_publisher<stereo_perception_msgs::msg::ClusteredObjectArray>
      (
        clustered_object_array_topic_, rclcpp::QoS(10)
      );
    
    predicted_objects_sub_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>
    (
      predicted_objects_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(
        &StereoPerceptionAutowareAutoBridgeNode::autowareAutoPredictedObjectsCallback, 
        this, 
        std::placeholders::_1
      ) 
    );
}

void StereoPerceptionAutowareAutoBridgeNode::autowareAutoPredictedObjectsCallback(
    const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{   
    RCLCPP_INFO(get_logger(), "Received PredictedObjects message with %zu objects", msg->objects.size());
    // Convert Autoware Auto PredictedObjects to Stereo Perception ClusteredObjectArray
    stereo_perception_msgs::msg::ClusteredObjectArray clustered_object_array;
    clustered_object_array.header = msg->header;
    clustered_object_array.num_clusters = msg->objects.size();

    for (const auto & autoware_object : msg->objects) {
        clustered_object_array.objects.push_back(convertPredictedObjectToClusteredObject(autoware_object));
    }

    clustered_object_array_pub_->publish(clustered_object_array);
}

stereo_perception_msgs::msg::ClusteredObject StereoPerceptionAutowareAutoBridgeNode::convertPredictedObjectToClusteredObject(
    const autoware_auto_perception_msgs::msg::PredictedObject & autoware_object)
{
    stereo_perception_msgs::msg::ClusteredObject clustered_object;
    clustered_object.position = autoware_object.kinematics.initial_pose.pose.position;
    clustered_object.velocity = autoware_object.kinematics.initial_twist.twist.linear; 
    return clustered_object;
}





} // namespace stereo_perception_autoware_auto_bridge
} // namespace utils
} // namespace stereo_perception

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stereo_perception::utils::stereo_perception_autoware_auto_bridge::StereoPerceptionAutowareAutoBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}