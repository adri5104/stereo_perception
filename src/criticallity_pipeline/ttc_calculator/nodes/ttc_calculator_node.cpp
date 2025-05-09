#include "ttc_calculator_node.hpp"

namespace criticallity_pipeline
{
namespace ttc_calculator
{

  TTCCalculatorNode::TTCCalculatorNode() :
    Node("ttc_calculator_node"),
    last_twist_()
  {
    // declare parameters
    declare_parameter("input_clusters_topic", "/object_clusters");
    declare_parameter("input_twist_topic", "/ego_twist");
    declare_parameter("output_ttc_topic", "/object_ttc");
    declare_parameter("output_min_ttc_topic", "/min_ttc");
    declare_parameter("output_marker_topic", "/ego_marker");
    declare_parameter("ego_width", 1.0);
    declare_parameter("ego_length", 1.0);
    declare_parameter("ego_height", 1.0);
    declare_parameter("publish_ego_marker", true);

    // Read parameters
    get_parameter("input_clusters_topic", input_clusters_topic_);
    get_parameter("input_twist_topic", input_twist_topic_);
    get_parameter("output_ttc_topic", output_ttc_topic_);
    get_parameter("output_min_ttc_topic", output_min_ttc_topic_);
    get_parameter("output_marker_topic", output_marker_topic_);
    get_parameter("ego_width", ego_width_);
    get_parameter("ego_length", ego_length_);
    get_parameter("ego_height", ego_height_);
    get_parameter("publish_ego_marker", publish_ego_marker_);

    // Print parameters
    RCLCPP_INFO(this->get_logger(), "========== TTC Calculator Parameters =========");
    RCLCPP_INFO(this->get_logger(), "input_clusters_topic: %s", input_clusters_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_twist_topic: %s", input_twist_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_ttc__topic: %s", output_ttc_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_marker_topic: %s", output_marker_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "ego_width: %f", ego_width_);
    RCLCPP_INFO(this->get_logger(), "ego_length: %f", ego_length_);
    RCLCPP_INFO(this->get_logger(), "ego_height: %f", ego_height_);
    RCLCPP_INFO(this->get_logger(), "publish_ego_marker: %d", publish_ego_marker_);
    RCLCPP_INFO(this->get_logger(), "=============================================");

    // Subscribe to the input topics
    input_clusters_sub_ = create_subscription<stereo_perception_msgs::msg::ClusteredObjectArray>(
      input_clusters_topic_, 
      10, 
      std::bind(&TTCCalculatorNode::callbackClusters, 
      this, 
      std::placeholders::_1)
    );

    input_twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_twist_topic_, 
      10, 
      std::bind(&TTCCalculatorNode::callbackTwist, 
      this, 
      std::placeholders::_1)
    );

    // Create publishers
    output_ttc_pub_ = create_publisher<stereo_perception_msgs::msg::ClusteredObjectArray>(
      output_ttc_topic_, 
      10
    );

    output_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      output_marker_topic_, 
      10
    );

    output_min_ttc_pub_ = create_publisher<std_msgs::msg::Float64>(
      output_min_ttc_topic_, 
      10
    );
  }

  void TTCCalculatorNode::callbackClusters(const stereo_perception_msgs::msg::ClusteredObjectArray::SharedPtr msg)
  {
    output_ttc_.objects.clear();
    output_ttc_.header = msg->header;
    current_min_ttc_ = std::numeric_limits<double>::infinity();
    vel_of_min_ttc_ = 0.0;

    // Extract the ego vehicle's velocity
    Eigen::Vector3d ego_velocity(
      last_twist_.linear.x,
      last_twist_.linear.y,
      last_twist_.linear.z
    );

    for(const auto& obj : msg -> objects)
    {
      stereo_perception_msgs::msg::ClusteredObject obj_with_ttc = obj;

      // Position of the object
      Eigen::Vector3d obj_position(
        obj.position.x,
        obj.position.y,
        obj.position.z
      );

      // Velocity of the object
      Eigen::Vector3d obj_velocity(
        obj.velocity.x,
        obj.velocity.y,
        obj.velocity.z
      );

      // Normalized direction vector from the ego vehicle to the object
      Eigen::Vector3d direction = obj_position.normalized();

      // Compute the relative velocity
      Eigen::Vector3d rel_velocity = obj_velocity - ego_velocity;

      // Relative velocity of the object projected onto the direction vector
      double closing_velocity = rel_velocity.dot(direction);

      // Distance to the object
      double distance = obj_position.norm();

      // Time to collision
      double ttc = std::numeric_limits<double>::infinity();
      if (closing_velocity < 0.0 && distance > 0.01)  // evita divisiones por cero
      {
        ttc = distance / std::abs(closing_velocity);
      }

      // Update the minimum time to collision
      if (ttc < current_min_ttc_)
      {
        current_min_ttc_ = ttc;
        vel_of_min_ttc_ = closing_velocity;
      }

      obj_with_ttc.ttc = ttc;
      output_ttc_.objects.push_back(obj_with_ttc);
    }
    
    std_msgs::msg::Float64 min_ttc_msg;
    min_ttc_msg.data = current_min_ttc_;

    double vel_km_h = vel_of_min_ttc_ * 3.6;
    RCLCPP_INFO(this->get_logger(), "Minimum TTC: %.2f s with velocity: %.2f m/s | %.2f km/h", current_min_ttc_, vel_of_min_ttc_, vel_km_h);
    output_min_ttc_pub_->publish(min_ttc_msg);
    output_ttc_pub_->publish(output_ttc_);
  }

  void TTCCalculatorNode::callbackTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_twist_ = *msg;

    publishMarkers();
  
  }

  void TTCCalculatorNode::publishMarkers()
  {
    current_markers_.markers.clear();

    if (publish_ego_marker_)
    {
      addEgoMarker();
    }

    output_markers_pub_->publish(current_markers_);
  }

  void TTCCalculatorNode::addEgoMarker()
  {
    // Erase the previous ego marker
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "camera_optical_frame";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "ego_vehicle";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETE;
    current_markers_.markers.push_back(clear_marker);

    // Erase the previous ego velocity arrow
    visualization_msgs::msg::Marker clear_arrow;
    clear_arrow.header.frame_id = "camera_optical_frame";
    clear_arrow.header.stamp = this->now();
    clear_arrow.ns = "ego_velocity";
    clear_arrow.id = 0;


    // Create marker in the camera optical frame node to visualize the ego vehicle    
    visualization_msgs::msg::Marker ego_marker;
    ego_marker.header.frame_id = "camera_optical_frame";
    ego_marker.header.stamp = this->now();
    ego_marker.ns = "ego_vehicle";
    ego_marker.id = 0;
    ego_marker.type = visualization_msgs::msg::Marker::CUBE;
    ego_marker.action = visualization_msgs::msg::Marker::ADD;
    ego_marker.pose.position.x = 0.0;
    ego_marker.pose.position.y = 0.0;
    ego_marker.pose.position.z = 0.0;
    ego_marker.pose.orientation.x = 0.0;
    ego_marker.pose.orientation.y = 0.0;
    ego_marker.pose.orientation.z = 0.0;
    ego_marker.pose.orientation.w = 1.0;
    ego_marker.scale.x = ego_width_;
    ego_marker.scale.y = ego_height_;
    ego_marker.scale.z = ego_length_;
    ego_marker.color.a = 0.5;
    ego_marker.color.r = 0.0;
    ego_marker.color.g = 1.0;
    ego_marker.color.b = 0.0; // Green

    // Create an arrow to visualize the ego vehicle's velocity in the camera optical frame
    visualization_msgs::msg::Marker ego_arrow;
    ego_arrow.header.frame_id = "camera_optical_frame";
    ego_arrow.header.stamp = this->now();
    ego_arrow.ns = "ego_velocity";
    ego_arrow.id = 0;
    ego_arrow.type = visualization_msgs::msg::Marker::ARROW;
    ego_arrow.action = visualization_msgs::msg::Marker::ADD;
    
    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;  

    geometry_msgs::msg::Point end;
    end.x = last_twist_.linear.x;
    end.y = last_twist_.linear.y;
    end.z = last_twist_.linear.z;

    ego_arrow.points.reserve(2);
    ego_arrow.points.push_back(start);
    ego_arrow.points.push_back(end);

    ego_arrow.scale.x = 0.1;
    ego_arrow.scale.y = 0.1;
    ego_arrow.scale.z = 0.1;
    ego_arrow.color.a = 1.0;
    ego_arrow.color.r = 1.0;
    ego_arrow.color.g = 0.0;
    ego_arrow.color.b = 0.0; // Red

    current_markers_.markers.push_back(ego_marker);
    current_markers_.markers.push_back(ego_arrow);
  }

} // namespace criticallity_pipeline
} // namespace ttc_calculator


// Main with multi-threaded executor
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<criticallity_pipeline::ttc_calculator::TTCCalculatorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}