
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
  #ifdef ROS_VERSION_JAZZY
  #include <cv_bridge/cv_bridge.hpp>
  #else
  #include <cv_bridge/cv_bridge.h>
  #endif


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
    this->declare_parameter<std::string>("camera_frame_tf_topic", "/odometry");
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
    this->declare_parameter<double>("min_depth", 0.1);
    this->declare_parameter<double>("max_depth", 15.0);
    this->declare_parameter<double>("min_height", 0.0);
    this->declare_parameter<double>("max_height", 4.0);

    // Read parameters
    optical_flow_topic_ = this->get_parameter("optical_flow_topic").as_string();
    depth_topic_        = this->get_parameter("depth_topic").as_string();
    camera_info_topic_  = this->get_parameter("camera_info_topic").as_string();
    camera_frame_tf_topic_     = this->get_parameter("camera_frame_tf_topic").as_string();
    color_image_topic_  = this->get_parameter("color_image_topic").as_string();
    output_6d_topic_    = this->get_parameter("output_6d_topic").as_string();
    debug_image_topic_  = this->get_parameter("debug_image_topic").as_string();
    debug_markers_topic_ = this->get_parameter("debug_markers_topic").as_string();
    RCLCPP_INFO(this->get_logger(), "optical_flow_topic: '%s'", optical_flow_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_topic: '%s'", depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_info_topic: '%s'", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_frame_tf_topic: '%s'", camera_frame_tf_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "color_image_topic: '%s'", color_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_6d_topic: '%s'", output_6d_topic_.c_str());
      RCLCPP_INFO(this->get_logger(), "debug_image_topic: '%s'", debug_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "debug_markers_topic: '%s'", debug_markers_topic_.c_str());
    
    // Kalman filter parameters
    use_ego_motion_ = this->get_parameter("use_ego_motion").as_bool();
    grid_size_ = this->get_parameter("grid_size").as_int();
    sigma2_x_system_ = this->get_parameter("sigma2_x_system").as_double();
    sigma2_y_system_ = this->get_parameter("sigma2_y_system").as_double();
    sigma2_z_system_ = this->get_parameter("sigma2_z_system").as_double();
    sigma2_flow_y_measurement_ = this->get_parameter("sigma2_flow_y_measurement").as_double();
    sigma2_flow_x_measurement_ = this->get_parameter("sigma2_flow_x_measurement").as_double();
    sigma2_depth_system_ = this->get_parameter("sigma2_depth_system").as_double();
    sigma2_tx_measurement_ = this->get_parameter("sigma2_tx_measurement").as_double();
    sigma2_ty_measurement_ = this->get_parameter("sigma2_ty_measurement").as_double();
    sigma2_tz_measurement_ = this->get_parameter("sigma2_tz_measurement").as_double();
    sigma2_theta_measurement_ = this->get_parameter("sigma2_theta_measurement").as_double();
    sigma2_phi_measurement_ = this->get_parameter("sigma2_phi_measurement").as_double();
    sigma2_psi_system_ = this->get_parameter("sigma2_psi_system").as_double();

    min_depth_ = this->get_parameter("min_depth").as_double();
    max_depth_ = this->get_parameter("max_depth").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    C_ = cv::Mat::zeros(6, 6, CV_64FC1);
    T_ = cv::Mat::zeros(3, 3, CV_64FC1);
    sigma_system_ = cv::Mat::zeros(3, 3, CV_64FC1);   

    // Set covariance matrix of system model
    sigma_system_.at<double>(0,0) = sigma2_x_system_;         
    sigma_system_.at<double>(1,1) = sigma2_y_system_;         
    sigma_system_.at<double>(2,2) = sigma2_z_system_;        

    // Set covariance matrix of measurement model
    T_.at<double>(0,0) = sigma2_flow_y_measurement_;
    T_.at<double>(1,1) = sigma2_flow_x_measurement_;
    T_.at<double>(2,2) = sigma2_depth_system_;                    

    // Set covariance matrix of egomotion
    C_.at<double>(0,0) = sigma2_tx_measurement_;
    C_.at<double>(1,1) = sigma2_ty_measurement_;
    C_.at<double>(2,2) = sigma2_tz_measurement_;
    C_.at<double>(3,3) = sigma2_theta_measurement_;
    C_.at<double>(4,4) = sigma2_phi_measurement_;
    C_.at<double>(5,5) = sigma2_psi_system_;

    kalman_core_ = std::make_unique<KalmanCore> 
    ( 
      sigma_system_,
      C_,
      T_,
      min_depth_, max_depth_,
      min_height_, max_height_, 
      use_ego_motion_,
      grid_size_
    );   
    
    // Create message_filters subscribers
    optical_flow_sub_.subscribe(this, optical_flow_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    color_sub_.subscribe(this, color_image_topic_, rmw_qos_profile_sensor_data);
    frame_tf_sub_.subscribe(this, camera_frame_tf_topic_, rmw_qos_profile_sensor_data);

    // Create the synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
              SyncPolicy(100),   // queue size
              optical_flow_sub_,
              depth_sub_,
              color_sub_,
              frame_tf_sub_);

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

    // Parameter reconfigure handler
    param_reconfigure_handler_ = this->add_on_set_parameters_callback(
      std::bind(&KalmanFilterNode::paramCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "KalmanFilterNode with message_filters started.");
  }

  // Synchronized callback
  void KalmanFilterNode::updateSync(
    const sensor_msgs::msg::Image::ConstSharedPtr flow_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr color_msg,
    const geometry_msgs::msg::TransformStamped::ConstSharedPtr frame_tf_msg)
  {
    if(camera_parameters_set_ == false)
    {
      RCLCPP_ERROR(this->get_logger(), "Camera parameters not set yet.");
      return;
    }

    // Convert each to cv::Mat
    cv::Mat flow_image  = imageMsgToMat(flow_msg);
    cv::Mat depth_image = imageMsgToMat(depth_msg);
    cv::Mat color_image = imageMsgToMat(color_msg);

    // Get the odometry matrix from camera frame tf message
    cv::Mat odometry_matrix = cv::Mat::zeros(4, 4, CV_64FC1); 

    // Convert from geometry_msgs::msg::TransformStamped to cv::Mat
    // Convert quaternion to rotation matrix
    tf2::Quaternion q(
      frame_tf_msg->transform.rotation.x,
      frame_tf_msg->transform.rotation.y,
      frame_tf_msg->transform.rotation.z,
      frame_tf_msg->transform.rotation.w);
    tf2::Matrix3x3 m(q);

    // Fill the matrix
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        odometry_matrix.at<double>(i, j) = m[i][j];
      }
    }
    odometry_matrix.at<double>(0, 3) = frame_tf_msg->transform.translation.x;
    odometry_matrix.at<double>(1, 3) = frame_tf_msg->transform.translation.y;
    odometry_matrix.at<double>(2, 3) = frame_tf_msg->transform.translation.z;
    odometry_matrix.at<double>(3, 3) = 1.0;


    // Update the Kalman filter
    KalmanCoreErrorCode result = kalman_core_->updateSyncedData(flow_image, depth_image, color_image, odometry_matrix);

    // Retrieve outputs
    cv::Mat output_6d, output_debug_image;
    double delta_time;

    kalman_core_->getOutput(output_6d ,output_debug_image);
    delta_time = kalman_core_->getDeltaTime();


    // Create header object
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_optical_frame";


    // Convert to sensor_msgs
    sensor_msgs::msg::Image::SharedPtr output_6d_msg =
      cv_bridge::CvImage(header, "32FC7", output_6d).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr debug_image_msg =
      cv_bridge::CvImage(header, "bgr8", output_debug_image).toImageMsg();

    // Publish
    if (!(result == KalmanCoreErrorCode::OK))
      RCLCPP_ERROR(this->get_logger(), "KalmanCore error: %s", getErrorMessage(result).c_str());

    visualization_msgs::msg::MarkerArray markers =  
      createMarkers(output_6d, delta_time);  
    
    debug_image_pub_->publish(*debug_image_msg);
    debug_markers_pub_->publish(markers);
    output_6d_pub_->publish(*output_6d_msg);
  }

  // Camera info callback
  void KalmanFilterNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    if (camera_parameters_set_)
    { 
      // Camera parameters already set
      return;
    } 

    double fx = msg->k[0];
    double fy = msg->k[4];
    double cx = msg->k[2];
    double cy = msg->k[5];

    kalman_core_->setCameraParameters(fx, fy, cx, cy);

    camera_parameters_set_ = true;
  }


  visualization_msgs::msg::MarkerArray KalmanFilterNode::createMarkers(
      const cv::Mat &image_6d, 
      double delta_time) 
  {
      visualization_msgs::msg::MarkerArray marker_array;

      try {
          // Validate that both matrices are not empty and have compatible dimensions
          if (image_6d.empty()) {
              RCLCPP_ERROR(this->get_logger(), "Input image is empty.");
              return marker_array;
          }

          // Ensure the input matrix types are as expected
          if (image_6d.type() != OUT6D_TYPE) {
              RCLCPP_ERROR(this->get_logger(), "image_6d invalid type. Type = %d", image_6d.type());
              return marker_array;
          }

          OutVec x; // 6D state vector
          int id = 0;  // Unique ID for each marker
          int total = 0;
          

          // First, clear old markers by publishing a DELETE action
          visualization_msgs::msg::Marker clear_marker;
          clear_marker.header.frame_id = "camera_optical_frame";  // Adjust the frame as needed
          clear_marker.header.stamp = this->now();
          clear_marker.ns = "cluster_points";
          clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;  // Deletes all previously published markers
          marker_array.markers.push_back(clear_marker);

          // Iterate over all valid points in parallel
          #pragma omp parallel for collapse(2)
          for (int i = 0; i < image_6d.rows; i++) {
              for (int j = 0; j < image_6d.cols; j++) {
                  try {
                      // Create thread-local variables
                      OutVec x;
                      visualization_msgs::msg::Marker marker;

                      // Check if there is a valid point
                      float valid = image_6d.at<OutVec>(i, j)[OUT6D_VAL_IDX];
                      #pragma omp atomic
                      total++;
                      if (valid != 1.0) {
                          continue;
                      }

                      // Extract the state vector safely
                      x = image_6d.at<OutVec>(i, j);

                      // Create a new arrow marker
                      marker.header.frame_id = "camera_optical_frame";
                      marker.header.stamp = this->now();
                      marker.ns = "kalman_arrows";
                      
                      int local_id;
                      #pragma omp atomic capture
                      local_id = id++;

                      marker.id = local_id;
                      marker.type = visualization_msgs::msg::Marker::ARROW;
                      marker.action = visualization_msgs::msg::Marker::ADD;
            
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
                      marker.scale.x = 0.02; // Thickness of the arrow shaft
                      marker.scale.y = 0.04; // Thickness of the arrow head
                      marker.scale.z = 0.0;

                      marker.color.r = 1.0;
                      marker.color.g = 0.0;
                      marker.color.b = 0.0;
                      marker.color.a = 1.0;

                      // Add the marker to the MarkerArray
                      #pragma omp critical
                      {
                          marker_array.markers.push_back(marker);
                      }
          } catch (const cv::Exception &e) {
              #pragma omp critical
              RCLCPP_ERROR(this->get_logger(), "OpenCV exception at pixel (%d, %d): %s", i, j, e.what());
              continue;
          } catch (const std::exception &e) {
              #pragma omp critical
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

  rcl_interfaces::msg::SetParametersResult 
    KalmanFilterNode::paramCallback(const std::vector<rclcpp::Parameter> &params)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto &param : params)
    {
      const auto &name = param.get_name();
      
      if (name == "sigma2_x_system" || 
          name == "sigma2_y_system" || 
          name == "sigma2_z_system")
      {
        if (name == "sigma2_x_system") sigma2_x_system_ = param.as_double();
        if (name == "sigma2_y_system") sigma2_y_system_ = param.as_double();
        if (name == "sigma2_z_system") sigma2_z_system_ = param.as_double();

        sigma_system_ = cv::Mat::zeros(3, 3, CV_64FC1);
        sigma_system_.at<double>(0, 0) = sigma2_x_system_;
        sigma_system_.at<double>(1, 1) = sigma2_y_system_;
        sigma_system_.at<double>(2, 2) = sigma2_z_system_;
        kalman_core_->setSigmaSystem(sigma_system_);
      }

      if (name == "sigma2_flow_y_measurement" || 
          name == "sigma2_flow_x_measurement" || 
          name == "sigma2_depth_system")
      {
        if (name == "sigma2_flow_y_measurement") sigma2_flow_y_measurement_ = param.as_double();
        if (name == "sigma2_flow_x_measurement") sigma2_flow_x_measurement_ = param.as_double();
        if (name == "sigma2_depth_system") sigma2_depth_system_ = param.as_double();

        T_ = cv::Mat::zeros(3, 3, CV_64FC1);
        T_.at<double>(0, 0) = sigma2_flow_y_measurement_;
        T_.at<double>(1, 1) = sigma2_flow_x_measurement_;
        T_.at<double>(2, 2) = sigma2_depth_system_;
        kalman_core_->setT(T_);
      }

      if (name == "sigma2_tx_measurement" ||
          name == "sigma2_ty_measurement" || 
          name == "sigma2_tz_measurement" || 
          name == "sigma2_theta_measurement" || 
          name == "sigma2_phi_measurement" || 
          name == "sigma2_psi_system")
      {
        if (name == "sigma2_tx_measurement") sigma2_tx_measurement_ = param.as_double();
        if (name == "sigma2_ty_measurement") sigma2_ty_measurement_ = param.as_double();
        if (name == "sigma2_tz_measurement") sigma2_tz_measurement_ = param.as_double();
        if (name == "sigma2_theta_measurement") sigma2_theta_measurement_ = param.as_double();
        if (name == "sigma2_phi_measurement") sigma2_phi_measurement_ = param.as_double();
        if (name == "sigma2_psi_system") sigma2_psi_system_ = param.as_double();

        C_ = cv::Mat::zeros(6, 6, CV_64FC1);
        C_.at<double>(0, 0) = sigma2_tx_measurement_;
        C_.at<double>(1, 1) = sigma2_ty_measurement_;
        C_.at<double>(2, 2) = sigma2_tz_measurement_;
        C_.at<double>(3, 3) = sigma2_theta_measurement_;
        C_.at<double>(4, 4) = sigma2_phi_measurement_;
        C_.at<double>(5, 5) = sigma2_psi_system_;
        kalman_core_->setC(C_);
      }

      if (name == "min_depth") 
      {
        min_depth_ = param.as_double();
        kalman_core_->setMinDepth(min_depth_);
      }

      if (name == "max_depth") 
      {
        max_depth_ = param.as_double();
        kalman_core_->setMaxDepth(max_depth_);
      }

      if (name == "min_height") 
      {
        min_height_ = param.as_double();
        kalman_core_->setMinHeight(min_height_);
      }

      if (name == "max_height") 
      {
        max_height_ = param.as_double();
        kalman_core_->setMaxHeight(max_height_);
      }

      if (name == "use_ego_motion") 
      {
        use_ego_motion_ = param.as_bool();
        kalman_core_->setIncludeEgoMotion(use_ego_motion_);
      }
    }
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
    auto node = std::make_shared<perception_pipeline::kalman_filter::KalmanFilterNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
