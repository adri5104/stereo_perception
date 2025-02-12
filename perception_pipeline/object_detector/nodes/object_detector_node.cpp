#include "object_detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace perception_pipeline
{
namespace object_detector
{

using namespace std;

ObjectDetectorNode::ObjectDetectorNode() : 
  Node("object_detector_node")
{
  // Get parameters
  this->declare_parameter("input_6d_topic", "/6d_image");
  this->declare_parameter("output_markers_topic", "/object_markers");
  this->declare_parameter("eps", 0.1);
  this->declare_parameter("minPts", 10);
  this->declare_parameter("pos_weight", 1.0);
  this->declare_parameter("vel_weight", 1.0);
  this->declare_parameter("vel_threshold", 0.1);
  this->get_parameter("input_6d_topic", input_6d_topic_);
  this->get_parameter("output_markers_topic", output_markers_topic_);

  // Subscribe to the 6d image topic
  input_6d_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_6d_topic_, 10,
    std::bind(&ObjectDetectorNode::callback6dImage, this, std::placeholders::_1)
  );

  // Create publishers
  output_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_markers_topic_, 10);

  // Object detector
  object_detector_ = std::make_unique<ObjectDetector>(
    this->get_parameter("eps").as_double(),
    this->get_parameter("minPts").as_int(),
    this->get_parameter("pos_weight").as_double(),
    this->get_parameter("vel_weight").as_double(),
    this->get_parameter("vel_threshold").as_double()
  );
}

void ObjectDetectorNode::callback6dImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Convert ROS image to cv::Mat
  cv::Mat image = rosImageToCvMat(msg);

  // Detect obj ects
  object_detector_->update(image);
  auto clusters = object_detector_->getClusters();

  // Publish the clusters as markers
  publishClusters(clusters);
 
}

cv::Mat ObjectDetectorNode::rosImageToCvMat(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }

    return cv_ptr->image; 
}

void ObjectDetectorNode::publishClusters(std::vector<WorldEntity>& clusters)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    int marker_id_bb = 8000;
    int marker_id_vel = 9000;

    // Clear old cluster point markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "camera_optical_frame";
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "cluster_points";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // Clear old bounding box markers
    visualization_msgs::msg::Marker clear_marker_bb;
    clear_marker_bb.header.frame_id = "camera_optical_frame";
    clear_marker_bb.header.stamp = this->now();
    clear_marker_bb.ns = "bounding_boxes";
    clear_marker_bb.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker_bb);

    // Clear old velocity arrow markers
    visualization_msgs::msg::Marker clear_marker_vel;
    clear_marker_vel.header.frame_id = "camera_optical_frame";
    clear_marker_vel.header.stamp = this->now();
    clear_marker_vel.ns = "velocity_arrow";
    clear_marker_vel.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker_vel);

    // Now publish new markers
    int cluster_index = 0;

    // Process each cluster
    for (auto& cluster : clusters)
    {
        // Retrieve all points from the cluster
        const auto& points = cluster.getPoints();

        // Generate ONE random color per cluster
        float cluster_r = static_cast<float>(rand()) / RAND_MAX;
        float cluster_g = static_cast<float>(rand()) / RAND_MAX;
        float cluster_b = static_cast<float>(rand()) / RAND_MAX;
        
        // Create a marker for each point in the cluster
        for (const auto& point : points)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "camera_optical_frame";
            marker.header.stamp = this->now();
            marker.ns = "cluster_points";

            // Each marker must have a unique ID
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set position
            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = point.z;

            // Small sphere for each point
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // Use the one random color for this entire cluster
            marker.color.r = cluster_r;
            marker.color.g = cluster_g;
            marker.color.b = cluster_b;
            marker.color.a = 1.0; 

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(marker);
        }

        // Create a bounding box marker for the cluster
        auto corners = cluster.getBoundingBox();
        if(!corners.empty())
        {
          visualization_msgs::msg::Marker bb_marker;
          bb_marker.header.frame_id = "camera_optical_frame";
          bb_marker.header.stamp = this->now();
          bb_marker.ns = "bounding_boxes";
          bb_marker.id = marker_id_bb++;
          bb_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
          bb_marker.action = visualization_msgs::msg::Marker::ADD;

          // Set the line width
          bb_marker.scale.x = 0.1;

          // Set the color
          bb_marker.color.r = 1.0;
          bb_marker.color.g = 0.0;
          bb_marker.color.b = 1.0;
          bb_marker.color.a = 0.8;

          // Define the edges of the bounding box.
          // This assumes that the ordering of corners in getBoundingBox() is:
          // 0: (min_x, min_y, min_z)
          // 1: (min_x, min_y, max_z)
          // 2: (min_x, max_y, min_z)
          // 3: (min_x, max_y, max_z)
          // 4: (max_x, min_y, min_z)
          // 5: (max_x, min_y, max_z)
          // 6: (max_x, max_y, min_z)
          // 7: (max_x, max_y, max_z)
          std::vector<std::pair<int, int>> edges = {
            {0,1}, {0,2}, {0,4},
            {1,3}, {1,5},
            {2,3}, {2,6},
            {3,7},
            {4,5}, {4,6},
            {5,7},
            {6,7}
          };


          // Add the line segments for each edge.
          for (const auto& edge : edges)
          {
            bb_marker.points.push_back(
              pclPointToGeometryMsgPoint(corners[edge.first])
              );
            bb_marker.points.push_back(
              pclPointToGeometryMsgPoint(corners[edge.second])
              );
          }

          bb_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

          marker_array.markers.push_back(bb_marker);
        }

        // Create a velocity arrow marker for the cluster
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "camera_optical_frame";
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = "velocity_arrow";
        arrow_marker.id = marker_id_vel++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        // The arrow will start at the cluster centroid
        Eigen::Vector3f centroid = cluster.getCentroid();
        geometry_msgs::msg::Point start_point;
        start_point.x = centroid.x();
        start_point.y = centroid.y();
        start_point.z = centroid.z();

        // And end at centroid + scaled velocity vector
        Eigen::Vector3f velocity = cluster.getVelocity();

        // Scale the velocity vector to make the arrow more visible, if needed
        float scale_factor = 1.0;
        Eigen::Vector3f end_point_vec = centroid + velocity * scale_factor;
        geometry_msgs::msg::Point end_point;
        end_point.x = end_point_vec.x();
        end_point.y = end_point_vec.y();
        end_point.z = end_point_vec.z();

        // Set the two points of the arrow marker
        arrow_marker.points.push_back(start_point);
        arrow_marker.points.push_back(end_point);

        // Set the arrow's scale: 
        // scale.x is the shaft diameter,
        // scale.y is the head diameter,
        // scale.z is the head length.
        arrow_marker.scale.x = 0.1;  
        arrow_marker.scale.y = 0.2;
        arrow_marker.scale.z = 0.2;
        
        // Set the color
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 0.0;
        arrow_marker.color.b = 0.0;
        arrow_marker.color.a = 1.0;
        
        arrow_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(arrow_marker);

        cluster_index++;
    }

    // Publish the marker array
    output_markers_pub_->publish(marker_array);
}

geometry_msgs::msg::Point ObjectDetectorNode::pclPointToGeometryMsgPoint(const pcl::PointXYZ& pcl_point)
{
    geometry_msgs::msg::Point point;
    point.x = pcl_point.x;
    point.y = pcl_point.y;
    point.z = pcl_point.z;
    return point;
}



} // namespace object_detector
} // namespace perception_pipeline

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_pipeline::object_detector::ObjectDetectorNode>());
  rclcpp::shutdown();
  return 0;
}   