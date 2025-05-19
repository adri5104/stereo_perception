#include "object_detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace stereo_perception
{
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
  this->declare_parameter("output_clusters_topic", "/object_clusters");
  this->declare_parameter("frame_id", "camera_optical_frame");
  this->declare_parameter("eps", 0.1);
  this->declare_parameter("minPts", 10);
  this->declare_parameter("pos_weight", 1.0);
  this->declare_parameter("vel_weight", 1.0);
  this->declare_parameter("vel_threshold", 0.1);
  this->get_parameter("input_6d_topic", input_6d_topic_);
  this->get_parameter("output_markers_topic", output_markers_topic_);
  this->get_parameter("output_clusters_topic", output_clusters_topic_);
  this->get_parameter("frame_id", frame_id_);
  
  // Subscribe to the 6d image topic
  input_6d_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_6d_topic_, 10,
    std::bind(&ObjectDetectorNode::callback6dImage, this, std::placeholders::_1)
  );

  // Create publishers
  output_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    output_markers_topic_, 
    10);
  output_clusters_pub_ = this->create_publisher<stereo_perception_msgs::msg::ClusteredObjectArray>(
    output_clusters_topic_, 
    10);

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
  publishClustersMarkers(clusters);

  // Publish the clusters as a message
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

void ObjectDetectorNode::publishClustersMarkers(const std::vector<WorldEntity>& clusters)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 1000;

    // Clear old cluster point markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "cluster_points";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    // Clear old bounding box markers
    visualization_msgs::msg::Marker clear_marker_bb;
    clear_marker_bb.header.frame_id = frame_id_;
    clear_marker_bb.header.stamp = this->now();
    clear_marker_bb.ns = "bounding_boxes";
    clear_marker_bb.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker_bb);

    // Clear old velocity arrow markers
    visualization_msgs::msg::Marker clear_marker_vel;
    clear_marker_vel.header.frame_id = frame_id_;
    clear_marker_vel.header.stamp = this->now();
    clear_marker_vel.ns = "velocity_arrow";
    clear_marker_vel.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker_vel);

    // Now publish new markers
    int cluster_index = 0;

    // Process each cluster
    for (auto& cluster : clusters)
    {
        const auto& points = cluster.getPoints();
        float cluster_r = static_cast<float>(rand()) / RAND_MAX;
        float cluster_g = static_cast<float>(rand()) / RAND_MAX;
        float cluster_b = static_cast<float>(rand()) / RAND_MAX;
        for (const auto& point : points)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = this->now();
            marker.ns = "cluster_points";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = point.z;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = cluster_r;
            marker.color.g = cluster_g;
            marker.color.b = cluster_b;
            marker.color.a = 1.0; 
            marker_array.markers.push_back(marker);
        }

        // Create a bounding box marker for the cluster
        marker_array.markers.push_back(clusterToBoundingBoxMarker(cluster));
       
        // Create a velocity arrow marker for the cluster
        marker_array.markers.push_back(clusterToArrowMarker(cluster));
        cluster_index++;
    }

    // Publish the marker array
    output_markers_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker 
  ObjectDetectorNode::clusterToBoundingBoxMarker(const WorldEntity& cluster)
{ 
  visualization_msgs::msg::Marker bb_marker;
  auto corners = cluster.getBoundingBox();
  if(!corners.empty())
  {
    bb_marker.header.frame_id = frame_id_;
    bb_marker.header.stamp = this->now();
    bb_marker.ns = "bounding_boxes";
    bb_marker.id = cluster.getId();
    bb_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    bb_marker.action = visualization_msgs::msg::Marker::ADD;
    bb_marker.scale.x = 0.1;
    bb_marker.color.r = 1.0;
    bb_marker.color.g = 0.0;
    bb_marker.color.b = 1.0;
    bb_marker.color.a = 0.8;
    // Define the edges of the bounding box.
    std::vector<std::pair<int, int>> edges = {
      {0,1}, {0,2}, {0,4}, {1,3}, {1,5}, {2,3}, {2,6}, {3,7},
      {4,5}, {4,6}, {5,7}, {6,7}
    };
    for (const auto& edge : edges) {
      bb_marker.points.push_back(
        pclPointToGeometryMsgPoint(corners[edge.first])
        );
      bb_marker.points.push_back(
        pclPointToGeometryMsgPoint(corners[edge.second])
        );
    }
  return bb_marker;
  }
  std::cerr << "Error: clusterToBoundingBoxMarker: empty corners" << std::endl;
  return bb_marker;
}

visualization_msgs::msg::Marker 
  ObjectDetectorNode::clusterToArrowMarker(const WorldEntity& cluster)
{
  // Create a velocity arrow marker for the cluster
  visualization_msgs::msg::Marker arrow_marker;
  arrow_marker.header.frame_id = frame_id_;
  arrow_marker.header.stamp = this->now();
  arrow_marker.ns = "velocity_arrow";
  arrow_marker.id = cluster.getId() + 1000;
  arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
  arrow_marker.action = visualization_msgs::msg::Marker::ADD;

  // Start at the centroid
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

  return arrow_marker;
}

void ObjectDetectorNode::publishClusters(const std::vector<WorldEntity>& clusters)
{
    stereo_perception_msgs::msg::ClusteredObjectArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.num_clusters = clusters.size();

    for (auto& cluster : clusters)
    {
        stereo_perception_msgs::msg::ClusteredObject obj;
        obj.id = cluster.getId();
        obj.position.x = cluster.getCentroid().x();
        obj.position.y = cluster.getCentroid().y();
        obj.position.z = cluster.getCentroid().z();
        obj.velocity.x = cluster.getVelocity().x();
        obj.velocity.y = cluster.getVelocity().y();
        obj.velocity.z = cluster.getVelocity().z();
        obj.bb_marker = clusterToBoundingBoxMarker(cluster);

        auto corners = cluster.getBoundingBox();
        stereo_perception_msgs::msg::BoundingBox bb;
        for (const auto& pt : corners)
        {
            geometry_msgs::msg::Point corner;
            corner.x = pt.x;
            corner.y = pt.y;
            corner.z = pt.z;
            bb.corners.push_back(corner);
        }
        obj.bounding_box = bb;
        msg.objects.push_back(obj);
    }

    // Publish the message
    output_clusters_pub_->publish(msg);
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
} // namespace stereo_perception

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<stereo_perception::perception_pipeline::object_detector::ObjectDetectorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}   