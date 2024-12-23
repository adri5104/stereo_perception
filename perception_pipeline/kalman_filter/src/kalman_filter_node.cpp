#include "kalman_filter/kalman_filter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterCompute>());
    rclcpp::shutdown();
    return 0;
}