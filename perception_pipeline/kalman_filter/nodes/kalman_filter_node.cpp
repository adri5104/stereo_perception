/**
 * @file kalman_filter_node.hpp
 * @author adrian.rieker@tum.de
 * @brief 
 * @version 
 * @date 
 * 
 * 
 * 
 */

#include "kalman_filter_node.hpp"
#include <cv_bridge/cv_bridge.hpp>

namespace perception_pipeline
{
namespace kalman_filter
{

KalmanFilterNode::KalmanFilterNode()
: Node("kalman_filter_node")
{
  // Declare parameters with default values

  // Topic names
  this->declare_parameter<std::string>("optical_flow_topic", "/optical_flow");
  this->declare_parameter<std::string>("depth_topic", "/device_0/sensor_0/Depth_0/image/data");
  this->declare_parameter<std::string>("camera_info_topic", "/perception_pipeline/camera_info_sync");
  this->declare_parameter<std::string>("color_image_topic", "/device_0/sensor_1/Color_0/image/data");
  this->declare_parameter<std::string>("output_6d_topic", "/output_6d");
  this->declare_parameter<std::string>("debug_image_topic", "/debug/image_6d");
  this->declare_parameter<std::string>("debug_markers_topic", "/debug/image_6d_markers");

  // Kalman filter parameters
  this->declare_parameter<int>("grid_size", 10);
  this->declare_parameter<bool>("use_ego_motion", false);

  // Covariance of system model
  this->declare_parameter<double>("sigma2_x_system", 10);
  this->declare_parameter<double>("sigma2_y_system", 10);
  this->declare_parameter<double>("sigma2_z_system", 10);

  // Covariance of measurement model
  this->declare_parameter<double>("sigma2_flow_y_measurement", 10);
  this->declare_parameter<double>("sigma2_flow_x_measurement", 10);
  this->declare_parameter<double>("sigma2_depth_system", 10);

  // Covariance of ego motion estimation
  this->declare_parameter<double>("sigma2_tx_measurement", 10);
  this->declare_parameter<double>("sigma2_ty_measurement", 10);
  this->declare_parameter<double>("sigma2_tz_measurement", 10);
  this->declare_parameter<double>("sigma2_theta_measurement", 10);
  this->declare_parameter<double>("sigma2_phi_measurement", 10);
  this->declare_parameter<double>("sigma2_psi_system", 10);

  // Camera parameters
  this->declare_parameter<double>("fx", 421.37701);
  this->declare_parameter<double>("fy", 421.37701);
  this->declare_parameter<double>("cx", 424.7990);
  this->declare_parameter<double>("cy", 231.86268);

  // Read parameters
  optical_flow_topic_ = this->get_parameter("optical_flow_topic").as_string();
  depth_topic_        = this->get_parameter("depth_topic").as_string();
  camera_info_topic_  = this->get_parameter("camera_info_topic").as_string();
  color_image_topic_  = this->get_parameter("color_image_topic").as_string();
  output_6d_topic_    = this->get_parameter("output_6d_topic").as_string();
  debug_image_topic_  = this->get_parameter("debug_image_topic").as_string();
  debug_markers_topic_ = this->get_parameter("debug_markers_topic").as_string();
  RCLCPP_INFO(this->get_logger(), "optical_flow_topic: '%s'", optical_flow_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "depth_topic: '%s'", depth_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "camera_info_topic: '%s'", camera_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "color_image_topic: '%s'", color_image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_6d_topic: '%s'", output_6d_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "debug_image_topic: '%s'", debug_image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "debug_markers_topic: '%s'", debug_markers_topic_.c_str());
  
  // Kalman filter parameters
  bool use_ego_motion = this->get_parameter("use_ego_motion").as_bool();
  int grid_size = this->get_parameter("grid_size").as_int();
  double sigma2_x_system = this->get_parameter("sigma2_x_system").as_double();
  double sigma2_y_system = this->get_parameter("sigma2_y_system").as_double();
  double sigma2_z_system = this->get_parameter("sigma2_z_system").as_double();
  double sigma2_flow_y_measurement = this->get_parameter("sigma2_flow_y_measurement").as_double();
  double sigma2_flow_x_measurement = this->get_parameter("sigma2_flow_x_measurement").as_double();
  double sigma2_depth_system = this->get_parameter("sigma2_depth_system").as_double();
  double sigma2_tx_measurement = this->get_parameter("sigma2_tx_measurement").as_double();
  double sigma2_ty_measurement = this->get_parameter("sigma2_ty_measurement").as_double();
  double sigma2_tz_measurement = this->get_parameter("sigma2_tz_measurement").as_double();
  double sigma2_theta_measurement = this->get_parameter("sigma2_theta_measurement").as_double();
  double sigma2_phi_measurement = this->get_parameter("sigma2_phi_measurement").as_double();
  double sigma2_psi_system = this->get_parameter("sigma2_psi_system").as_double();
  double fx = this->get_parameter("fx").as_double();
  double fy = this->get_parameter("fy").as_double();
  double cx = this->get_parameter("cx").as_double();
  double cy = this->get_parameter("cy").as_double();
  cv::Mat C = Mat::zeros(6, 6, CV_64FC1);
  cv::Mat T = Mat::zeros(3, 3, CV_64FC1);
  cv::Mat sigma_system_ = Mat::zeros(3, 3, CV_64FC1);

  // Set covariance matrix of system model
  sigma_system_.at<double>(0,0) = sigma2_x_system;         
  sigma_system_.at<double>(1,1) = sigma2_y_system;         
  sigma_system_.at<double>(2,2) = sigma2_z_system;        

  // Set covariance matrix of measurement model
  T.at<double>(0,0) = sigma2_flow_y_measurement;
  T.at<double>(1,1) = sigma2_flow_x_measurement;
  T.at<double>(2,2) = sigma2_depth_system;                    

  // Set covariance matrix of egomotion
  C.at<double>(0,0) = sigma2_tx_measurement;
  C.at<double>(1,1) = sigma2_ty_measurement;
  C.at<double>(2,2) = sigma2_tz_measurement;
  C.at<double>(3,3) = sigma2_theta_measurement;
  C.at<double>(4,4) = sigma2_phi_measurement;
  C.at<double>(5,5) = sigma2_psi_system;

  kalman_core_ = std::make_unique<KalmanCore> 
  ( 
    sigma_system_,
    C,
    T,
    fx, fy, cx, cy,
    use_ego_motion,
    grid_size
  );

  // Create message_filters subscribers
  optical_flow_sub_.subscribe(this, optical_flow_topic_, rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
  color_sub_.subscribe(this, color_image_topic_, rmw_qos_profile_sensor_data);

  // Create the synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100),   // queue size
            optical_flow_sub_,
            depth_sub_,
            color_sub_);

  // Register the synchronized callback
  sync_->registerCallback(&KalmanFilterNode::updateSync, this);

  // Camera info subscription (standard rclcpp subscription)
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 10,
    std::bind(&KalmanFilterNode::cameraInfoCallback, this, std::placeholders::_1));

  // Publishers
  debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 10);
  output_6d_pub_   = this->create_publisher<sensor_msgs::msg::Image>(output_6d_topic_, 10);
  debug_markers_pub_   = this->create_publisher<visualization_msgs::msg::MarkerArray>(debug_markers_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "KalmanFilterNode with message_filters started.");
}

// Synchronized callback
void KalmanFilterNode::updateSync(
  const sensor_msgs::msg::Image::ConstSharedPtr flow_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr color_msg)
{
  RCLCPP_INFO(this->get_logger(), "updateSync() called with synchronized messages");

  // Convert each to cv::Mat
  cv::Mat flow_image  = imageMsgToMat(flow_msg);
  cv::Mat depth_image = imageMsgToMat(depth_msg);
  cv::Mat color_image = imageMsgToMat(color_msg);

  //try {
  //  depth_image = cv_bridge::toCvCopy(depth_msg, "8UC1")->image;
  //}
  //catch(const cv_bridge::Exception& e) {
  //  RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  //}

  // Perform your KalmanCore logic
  KalmanCoreErrorCode result = kalman_core_->updateSyncedData(flow_image, depth_image, color_image);

  // Retrieve outputs
  cv::Mat output_6d, output_6d_val, output_debug_image;
  double delta_time;

  kalman_core_->getOutput(output_6d, output_6d_val ,output_debug_image);
  delta_time = kalman_core_->getDeltaTime();

  // Convert to sensor_msgs
  sensor_msgs::msg::Image::SharedPtr output_6d_msg =
    cv_bridge::CvImage(flow_msg->header, "bgr8", output_6d).toImageMsg();
  sensor_msgs::msg::Image::SharedPtr debug_image_msg =
    cv_bridge::CvImage(flow_msg->header, "bgr8", output_debug_image).toImageMsg();

  // Publish
  if (result == KalmanCoreErrorCode::OK)
    RCLCPP_INFO(this->get_logger(), "KalmanCore output: OK");
  else
    RCLCPP_ERROR(this->get_logger(), "KalmanCore error: %s", getErrorMessage(result).c_str());

  visualization_msgs::msg::MarkerArray markers =  
    createMarkers(output_6d, output_6d_val, delta_time);


  debug_image_pub_->publish(*debug_image_msg);
  debug_markers_pub_->publish(markers);
  output_6d_pub_->publish(*output_6d_msg);


}

// Camera info callback
void KalmanFilterNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  double fx = msg->k[0];
  double fy = msg->k[4];
  double cx = msg->k[2];
  double cy = msg->k[5];

  kalman_core_->setCameraParameters(fx, fy, cx, cy);
}


visualization_msgs::msg::MarkerArray KalmanFilterNode::createMarkers(
    const cv::Mat &image_6d, 
    const cv::Mat &image_6d_val, 
    double delta_time) 
{
    visualization_msgs::msg::MarkerArray marker_array;

    try {
        // Validate that both matrices are not empty and have compatible dimensions
        if (image_6d.empty() || image_6d_val.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Input images are empty.");
            return marker_array;
        }

        if (image_6d.rows != image_6d_val.rows || image_6d.cols != image_6d_val.cols) {
            RCLCPP_ERROR(this->get_logger(), "Dimension mismatch between image_6d and image_6d_val.");
            return marker_array;
        }

        // Ensure the input matrix types are as expected
        if (image_6d.type() != CV_32FC(6)) {
            RCLCPP_ERROR(this->get_logger(), "image_6d must have type CV_32FC(6).");
            return marker_array;
        }

        if (image_6d_val.type() != CV_8UC1) {
            RCLCPP_ERROR(this->get_logger(), "image_6d_val must have type CV_8UC1.");
            return marker_array;
        }

        cv::Vec6f x; // 6D state vector
        int id = 0;  // Unique ID for each marker
    

        int total = 0;
        int valid_ = 0;

        // Iterate over all valid points
        for (int i = 0; i < image_6d.rows; i++) {
            for (int j = 0; j < image_6d.cols; j++) {
                try {
                    // Check if there is a valid point
                    uchar valid = image_6d_val.at<uchar>(i, j);
                    total++;
                    if (valid != 1) {
                        continue;
                    }

                    // Extract the state vector safely
                    x = image_6d.at<cv::Vec6f>(i, j);

                    //// Validate the extracted state values (optional range checks)
                    //if (std::isnan(x[0]) || std::isnan(x[1]) || std::isnan(x[2]) ||
                    //    std::isnan(x[3]) || std::isnan(x[4]) || std::isnan(x[5])) {
                    //    RCLCPP_WARN(this->get_logger(), "NaN value detected in state vector at (%d, %d).", i, j);
                    //    continue;
                    //}

                    // Create a new arrow marker
                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "camera_frame";
                    marker.header.stamp = this->now();
                    marker.ns = "kalman_arrows";
                    marker.id = id++;
                    marker.type = visualization_msgs::msg::Marker::ARROW;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    marker.lifetime = rclcpp::Duration::from_seconds(delta_time);

                    // Start point of the arrow
                    geometry_msgs::msg::Point start;
                    start.x = x[0];
                    start.y = x[1];
                    start.z = x[2];

                    // End point of the arrow
                    geometry_msgs::msg::Point end;
                    end.x = x[0] + delta_time * x[3];
                    end.y = x[1] + delta_time * x[4];
                    end.z = x[2] + delta_time * x[5];

                    marker.points.push_back(start);
                    marker.points.push_back(end);

                    // Set arrow color and size
                    marker.scale.x = 0.05; // Thickness of the arrow shaft
                    marker.scale.y = 0.1; // Thickness of the arrow head
                    marker.scale.z = 0.0  ;

                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 1.0;

                    // Add the marker to the MarkerArray
                    marker_array.markers.push_back(marker);
                    valid_++;
                    
                } catch (const cv::Exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "OpenCV exception at pixel (%d, %d): %s", i, j, e.what());
                    continue;
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Standard exception at pixel (%d, %d): %s", i, j, e.what());
                    continue;
                }
            }
        }
    
    } catch (const cv::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception in createMarkers: %s", e.what());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Standard exception in createMarkers: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown error in createMarkers.");
    }

    
    return marker_array;
}


// Convert sensor_msgs::Image -> cv::Mat
cv::Mat KalmanFilterNode::imageMsgToMat(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    return cv_bridge::toCvCopy(msg, msg->encoding)->image;
  }
  catch(const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat(); 
  }
}

} // namespace kalman_filter
} // namespace perception_pipeline

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_pipeline::kalman_filter::KalmanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
