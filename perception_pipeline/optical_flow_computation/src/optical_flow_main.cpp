
#include "optical_flow_computation/optical_flow_node.hpp"
#include "rclcpp/rclcpp.hpp"



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpticalFlowNode>());
    rclcpp::shutdown();
    return 0;
}