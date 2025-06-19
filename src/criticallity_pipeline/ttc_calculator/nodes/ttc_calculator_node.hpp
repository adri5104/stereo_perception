/**
 * @file ttc_calculator_node.hpp
 * @author adrian.rieker@tum.de
 * @brief Contains the TTC calculator node class definition
 * @date 2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef _TTC_CALCULATOR_NODE_HPP_
#define _TTC_CALCULATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include <std_msgs/msg/float64.hpp>

#include <Eigen/Dense>
#include <fstream>
#include <filesystem>



#include "stereo_perception_msgs/msg/clustered_object_array.hpp"
#include "stereo_perception_msgs/msg/clustered_object.hpp"
#include "stereo_perception_msgs/msg/bounding_box.hpp"

namespace stereo_perception 
{
namespace criticallity_pipeline
{
namespace ttc_calculator
{
  /**
   * @class TTCCalculatorNode
   * @brief ROS2 Node that subscribes to a clustered object array and calculates the time to collision for each object
   */
  class TTCCalculatorNode : public rclcpp::Node
  {
  public:
    /**
     * @brief Construct a new TTC Calculator Node object
     */
    TTCCalculatorNode();

    /**
     * @brief Destroy the TTC Calculator Node object
     */
    ~TTCCalculatorNode();

  private:
    
    // Subscribers
    rclcpp::Subscription<stereo_perception_msgs::msg::ClusteredObjectArray>::SharedPtr input_clusters_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_twist_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr input_path_sub_;

    // Publishers
    rclcpp::Publisher<stereo_perception_msgs::msg::ClusteredObjectArray>::SharedPtr output_ttc_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_min_ttc_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr output_markers_pub_;

    // Callbacks
    void callbackClusters(const stereo_perception_msgs::msg::ClusteredObjectArray::SharedPtr msg);
    void callbackTwist(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     *    
     * @brief Calculates the time to collision (TTC) between the ego vehicle's planned path
     *        and a given object's predicted linear trajectory.
     * 
     * This method iterates over the poses in the current ego path and checks when the predicted
     * future position of the object (based on its velocity) becomes close enough to the ego
     * vehicle's future position along the path. The TTC is approximated by the accumulated time
     * it would take for the object to reach that point under constant velocity.
     * 
     * @param obj_pos The current 3D position of the object in the ego frame
     * @param obj_vel The current 3D velocity of the object in the ego frame
     * @return double The estimated time to collision in seconds. Returns infinity if no collision is predicted.
     */
    double computeTTCFromPath(const Eigen::Vector3d& obj_pos, const Eigen::Vector3d& obj_vel);



    void publishMarkers();
    void addEgoMarker();

    visualization_msgs::msg::MarkerArray current_markers_;
    nav_msgs::msg::Path current_path_;
    stereo_perception_msgs::msg::ClusteredObjectArray output_ttc_;
    double current_min_ttc_;
    double vel_of_min_ttc_;
    std::ofstream csv_file_;
    
    geometry_msgs::msg::Twist last_twist_;

    // Topic names
    std::string input_clusters_topic_;
    std::string input_twist_topic_;
    std::string input_path_topic_;
    std::string output_ttc_topic_;
    std::string output_min_ttc_topic_;
    std::string output_marker_topic_;
    

    // Parameters
    double ego_width_;
    double ego_length_;
    double ego_height_;
    double distance_threshold_;
    bool publish_ego_marker_;
    bool use_path_;
    bool ignore_receding_objects_;
    std::string frame_id_;
    std::string output_csv_path_;

  };
}
} // namespace criticallity_pipeline
} // namespace ttc_calculator


#endif