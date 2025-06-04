#include "ttc_calculator_node.hpp"

namespace stereo_perception 
{
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
    declare_parameter("use_path", false);
    declare_parameter("frame_id", "camera_optical_frame");
    declare_parameter("output_csv_path", "./");


    


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
    get_parameter("use_path", use_path_);
    get_parameter("frame_id", frame_id_);
    get_parameter("output_csv_path", output_csv_path_);


    // Print parameters
    RCLCPP_INFO(this->get_logger(), "========== TTC Calculator Parameters =========");
    RCLCPP_INFO(this->get_logger(), "input_clusters_topic: %s", input_clusters_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_twist_topic: %s", input_twist_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "input_path_topic: %s", input_path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_ttc__topic: %s", output_ttc_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_marker_topic: %s", output_marker_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "ego_width: %f", ego_width_);
    RCLCPP_INFO(this->get_logger(), "ego_length: %f", ego_length_);
    RCLCPP_INFO(this->get_logger(), "ego_height: %f", ego_height_);
    RCLCPP_INFO(this->get_logger(), "publish_ego_marker: %d", publish_ego_marker_);
    RCLCPP_INFO(this->get_logger(), "use_path: %d", use_path_);
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_csv_path: %s", output_csv_path_.c_str());
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

    if (use_path_)
    {
      declare_parameter("input_path_topic", "/ego_path");
      get_parameter("input_path_topic", input_path_topic_);
      
      input_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        input_path_topic_,
        10,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
          current_path_ = *msg;
        });
    }


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

    // Open csv file for logging
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::stringstream filename_ss;
    filename_ss << std::put_time(std::localtime(&now_time), "ttc_log_%Y%m%d_%H%M%S.csv");
    std::string filename = filename_ss.str();

    std::filesystem::path full_path = std::filesystem::path(output_csv_path_) / filename;
    csv_file_.open(full_path);

    if (!csv_file_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file at: %s", full_path.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Writing TTC log to: %s", full_path.c_str());
      csv_file_ << "timestamp,num_objects,min_ttc\n";
    }
  }

  TTCCalculatorNode::~TTCCalculatorNode()
  {
    if (csv_file_.is_open())
      csv_file_.close();
  }


  void TTCCalculatorNode::callbackClusters(const stereo_perception_msgs::msg::ClusteredObjectArray::SharedPtr msg)
  {
    output_ttc_.objects.clear();
    output_ttc_.header = msg->header;
    current_min_ttc_ = std::numeric_limits<double>::infinity();
    vel_of_min_ttc_ = 0.0;
  
    // Extract ego velocity
    Eigen::Vector3d ego_velocity(
      last_twist_.linear.x,
      last_twist_.linear.y,
      last_twist_.linear.z
    );
  
    double ego_speed = ego_velocity.norm();
  
    for (const auto& obj : msg->objects)
    {
      stereo_perception_msgs::msg::ClusteredObject obj_with_ttc = obj;
  
      // Object state
      Eigen::Vector3d obj_position(obj.position.x, obj.position.y, obj.position.z);
      Eigen::Vector3d obj_velocity(obj.velocity.x, obj.velocity.y, obj.velocity.z);
  
      double ttc = std::numeric_limits<double>::infinity();
  
      // ---- PATH-BASED TTC ----
      if (use_path_ && current_path_.poses.size() > 1 && ego_speed > 1e-3)
      {
        ttc = computeTTCFromPath(obj_position, obj_velocity);
        RCLCPP_DEBUG(this->get_logger(), "Computed TTC using path: %.2f", ttc);
      }
      else
      {
        // ---- FALLBACK: TWIST-BASED TTC ----
        Eigen::Vector3d direction = obj_position.normalized();
        Eigen::Vector3d rel_velocity = obj_velocity - ego_velocity;
        double closing_velocity = rel_velocity.dot(direction);
        double distance = obj_position.norm();
  
        if (closing_velocity < 0.0 && distance > 0.01)
        {
          ttc = distance / std::abs(closing_velocity);
        }
  
        RCLCPP_DEBUG(this->get_logger(), "Computed TTC using twist: %.2f", ttc);
      }
  
      // Update min TTC
      if (ttc < current_min_ttc_)
      {
        current_min_ttc_ = ttc;
        vel_of_min_ttc_ = (obj_velocity - ego_velocity).norm();
      }
  
      obj_with_ttc.ttc = ttc;
      output_ttc_.objects.push_back(obj_with_ttc);
    }
  
    // Publish results
    std_msgs::msg::Float64 min_ttc_msg;
    min_ttc_msg.data = current_min_ttc_;
  
    double vel_km_h = vel_of_min_ttc_ * 3.6;
    RCLCPP_INFO(this->get_logger(), "Minimum TTC: %.2f s with rel. velocity: %.2f m/s | %.2f km/h", 
                current_min_ttc_, vel_of_min_ttc_, vel_km_h);

    // Log to CSV
    rclcpp::Time timestamp = this->now();

    int num_objects = static_cast<int>(msg->objects.size());

    csv_file_ << std::fixed << std::setprecision(6) << timestamp.seconds() << ","
              << num_objects << "," 
              << current_min_ttc_ << "\n";
    csv_file_.flush();
  
    output_min_ttc_pub_->publish(min_ttc_msg);
    output_ttc_pub_->publish(output_ttc_);
  }
  

  void TTCCalculatorNode::callbackTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_twist_ = *msg;
    publishMarkers();
  }


  double TTCCalculatorNode::computeTTCFromPath(const Eigen::Vector3d& obj_pos, const Eigen::Vector3d& obj_vel)
  {
    double ego_speed = std::sqrt(
      last_twist_.linear.x * last_twist_.linear.x +
      last_twist_.linear.y * last_twist_.linear.y +
      last_twist_.linear.z * last_twist_.linear.z
    );

    if (ego_speed < 1e-3)  // Avoid division by zero
      return std::numeric_limits<double>::infinity();

    double accumulated_time = 0.0;

    for (size_t i = 1; i < current_path_.poses.size(); ++i)
    {
      const auto& prev_pose = current_path_.poses[i - 1].pose;
      const auto& curr_pose = current_path_.poses[i].pose;

      Eigen::Vector3d p_prev(prev_pose.position.x, prev_pose.position.y, prev_pose.position.z);
      Eigen::Vector3d p_curr(curr_pose.position.x, curr_pose.position.y, curr_pose.position.z);

      double d = (p_curr - p_prev).norm();
      accumulated_time += d / ego_speed;

      Eigen::Vector3d future_obj_pos = obj_pos + accumulated_time * obj_vel;
      double distance = (p_curr - future_obj_pos).norm();

      if (distance < 1.0)  // threshold for collision
      {
        return accumulated_time;
      }
    }

    return std::numeric_limits<double>::infinity();
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
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "ego_vehicle";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETE;
    current_markers_.markers.push_back(clear_marker);

    // Erase the previous ego velocity arrow
    visualization_msgs::msg::Marker clear_arrow;
    clear_arrow.header.frame_id = frame_id_;
    clear_arrow.header.stamp = this->now();
    clear_arrow.ns = "ego_velocity";
    clear_arrow.id = 0;


    // Create marker in the camera optical frame node to visualize the ego vehicle    
    visualization_msgs::msg::Marker ego_marker;
    ego_marker.header.frame_id = frame_id_;
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
    ego_arrow.header.frame_id = frame_id_;
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
} // namespace stereo_perception


// Main with multi-threaded executor
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stereo_perception::criticallity_pipeline::ttc_calculator::TTCCalculatorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}