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
  this->declare_parameter("input_6d_val_topic", "/6d_image_val");
  this->declare_parameter("output_markers_topic", "/object_markers");
  this->declare_parameter("eps", 0.1);
  this->declare_parameter("minPts", 10);
  this->declare_parameter("pos_weight", 1.0);
  this->declare_parameter("vel_weight", 1.0);
  this->get_parameter("input_6d_topic", input_6d_topic_);
  this->get_parameter("input_6d_val_topic", input_6d_val_topic_);
  this->get_parameter("output_markers_topic", output_markers_topic_);

  // Create message filters subscribers
  input_6d_sub_.subscribe(this, input_6d_topic_, rmw_qos_profile_sensor_data);
  input_6d_val_sub_.subscribe(this, input_6d_val_topic_,rmw_qos_profile_sensor_data);

  // Create the synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10),   // queue size
    input_6d_sub_,
    input_6d_val_sub_
  );

  // Register the synchronized callback
  sync_->registerCallback(&ObjectDetectorNode::inputSyncCallback, this);

  // Create publishers
  output_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_markers_topic_, 10);


    // Object detector
  object_detector_ = std::make_unique<ObjectDetector>(
    this->get_parameter("eps").as_double(),
    this->get_parameter("minPts").as_int(),
    this->get_parameter("pos_weight").as_double(),
    this->get_parameter("vel_weight").as_double()
  );

}

void ObjectDetectorNode::inputSyncCallback(const sensor_msgs::msg::Image::SharedPtr in6d, const sensor_msgs::msg::Image::SharedPtr in6d_val)
{
  auto image_6d = rosImageToCvMat(in6d);
  auto image_6d_val = rosImageToCvMat(in6d_val);

  // Update the object detector
  object_detector_->updateSync(image_6d, image_6d_val);
  
  // Publish the detected clusters as markers
  publishClusters(object_detector_->getClusters());
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

void ObjectDetectorNode::publishClusters(const std::vector<WorldEntity>& clusters)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int cluster_id = 0;

    // First, clear old markers by publishing a DELETE action
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "camera_optical_frame";  // Adjust the frame as needed
    clear_marker.header.stamp = this->now();
    clear_marker.ns = "cluster_points";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;  // Deletes all previously published markers
    marker_array.markers.push_back(clear_marker);

    // Now publish new markers
    for (const auto& cluster : clusters)
    {
        // Retrieve all points from the cluster
        const auto& points = cluster.getPoints();

        for (const auto& point : points)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "camera_optical_frame";  // Adjust the frame as needed
            marker.header.stamp = this->now();
            marker.ns = "cluster_points";
            marker.id = cluster_id++;
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

            // Assign a random color per cluster
            marker.color.r = static_cast<float>(rand()) / RAND_MAX;
            marker.color.g = static_cast<float>(rand()) / RAND_MAX;
            marker.color.b = static_cast<float>(rand()) / RAND_MAX;
            marker.color.a = 1.0; // Fully opaque

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(marker);
        }
    }

    // Publish the marker array
    output_markers_pub_->publish(marker_array);
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