/**
 * @file stereo_perception_autoware_bridge_node.cpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief  Implementation file for the StereoPerceptionAutowareBridgeNode class
 */
#include "stereo_perception_autoware_bridge_node.hpp"

namespace stereo_perception
{
namespace utils
{
namespace stereo_perception_autoware_bridge
{

StereoPerceptionAutowareBridgeNode::StereoPerceptionAutowareBridgeNode()
: Node("stereo_perception_autoware_bridge_node")
{
    RCLCPP_INFO(this->get_logger(), "Stereo Perception Autoware Bridge Node initialized");
    
}

} // namespace stereo_perception_autoware_bridge
} // namespace utils
} // namespace stereo_perception

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stereo_perception::utils::stereo_perception_autoware_bridge::StereoPerceptionAutowareBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}