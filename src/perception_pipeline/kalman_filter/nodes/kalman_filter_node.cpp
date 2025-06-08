
  /**
   * @file kalman_filter_node.hpp
   * @author adrian.rieker@tum.de
   * @brief ROS2 Node that subscribes to optical flow, depth, color, and camera info,
   *        and uses an internal KalmanCore object for the actual filter logic.
   * 
   */

  #include "kalman_filter_node.hpp"
  #ifdef ROS_VERSION_JAZZY
  #include <cv_bridge/cv_bridge.hpp>
  #else
  #include <cv_bridge/cv_bridge.h>
  #endif

  namespace stereo_perception
  {
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
    this->declare_parameter<std::string>("frame_id", "camera_optical_frame");

    // Kalman filter parameters
    this->declare_parameter<int>("grid_size", 10);
    this->declare_parameter<bool>("debug_image_grid", false);
    this->declare_parameter<bool>("use_ego_motion", false);
    this->declare_parameter<bool>("use_ego_var", false);

    // Covariance of system model 
    this->declare_parameter<double>("sigma2_x_system", 10);
    this->declare_parameter<double>("sigma2_y_system", 10);
    this->declare_parameter<double>("sigma2_z_system", 10);

    // Covariance of measurement model
    this->declare_parameter<double>("sigma2_flow_y_measurement", 10);
    this->declare_parameter<double>("sigma2_flow_x_measurement", 10);
    this->declare_parameter<double>("sigma2_depth_measurement", 10);

    // Covariance of ego motion estimation
    this->declare_parameter<double>("sigma2_tx_measurement", 10);
    this->declare_parameter<double>("sigma2_ty_measurement", 10);
    this->declare_parameter<double>("sigma2_tz_measurement", 10);
    this->declare_parameter<double>("sigma2_rx_measurement", 10);
    this->declare_parameter<double>("sigma2_ry_measurement", 10);
    this->declare_parameter<double>("sigma2_rz_measurement", 10);

    // Camera parameters ego_compensation_factor_
    this->declare_parameter<double>("min_depth", 0.1);
    this->declare_parameter<double>("max_depth", 15.0);
    this->declare_parameter<double>("min_height", 0.0);
    this->declare_parameter<double>("camera_ground_distance", 4.0);
    this->declare_parameter<double>("ego_compensation_factor", 1.0);


    // Read parameters
    optical_flow_topic_ = this->get_parameter("optical_flow_topic").as_string();
    depth_topic_        = this->get_parameter("depth_topic").as_string();
    camera_info_topic_  = this->get_parameter("camera_info_topic").as_string();
    camera_frame_tf_topic_     = this->get_parameter("camera_frame_tf_topic").as_string();
    color_image_topic_  = this->get_parameter("color_image_topic").as_string();
    output_6d_topic_    = this->get_parameter("output_6d_topic").as_string();
    debug_image_topic_  = this->get_parameter("debug_image_topic").as_string();
    debug_markers_topic_ = this->get_parameter("debug_markers_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    RCLCPP_INFO(this->get_logger(), "optical_flow_topic: '%s'", optical_flow_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_topic: '%s'", depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_info_topic: '%s'", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_frame_tf_topic: '%s'", camera_frame_tf_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "color_image_topic: '%s'", color_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_6d_topic: '%s'", output_6d_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "debug_image_topic: '%s'", debug_image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "debug_markers_topic: '%s'", debug_markers_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "frame_id: '%s'", frame_id_.c_str());
    
    // Time variables
    first_time_ = true;
    time_diff_ = 0.0;
    last_time_ = this->now();

    // Kalman filter parameters
    use_ego_motion_ = this->get_parameter("use_ego_motion").as_bool();
    use_ego_var_ = this->get_parameter("use_ego_var").as_bool();
    grid_size_ = this->get_parameter("grid_size").as_int();
    debug_image_grid_ = this->get_parameter("debug_image_grid").as_bool();

    sigma2_x_system_ = this->get_parameter("sigma2_x_system").as_double();
    sigma2_y_system_ = this->get_parameter("sigma2_y_system").as_double();
    sigma2_z_system_ = this->get_parameter("sigma2_z_system").as_double();

    sigma2_flow_y_measurement_ = this->get_parameter("sigma2_flow_y_measurement").as_double();
    sigma2_flow_x_measurement_ = this->get_parameter("sigma2_flow_x_measurement").as_double();
    sigma2_depth_measurement_ = this->get_parameter("sigma2_depth_measurement").as_double();

    sigma2_tx_measurement_ = this->get_parameter("sigma2_tx_measurement").as_double();
    sigma2_ty_measurement_ = this->get_parameter("sigma2_ty_measurement").as_double();
    sigma2_tz_measurement_ = this->get_parameter("sigma2_tz_measurement").as_double();
    sigma2_rx_measurement_ = this->get_parameter("sigma2_rx_measurement").as_double();
    sigma2_ry_measurement_ = this->get_parameter("sigma2_ry_measurement").as_double();
    sigma2_rz_measurement_ = this->get_parameter("sigma2_rz_measurement").as_double();

    min_depth_ = this->get_parameter("min_depth").as_double();
    max_depth_ = this->get_parameter("max_depth").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    camera_ground_distance_ = this->get_parameter("camera_ground_distance").as_double();
    ego_compensation_factor_ = this->get_parameter("ego_compensation_factor").as_double();
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
    T_.at<double>(2,2) = sigma2_depth_measurement_;                    

    // Set covariance matrix of egomotion
    C_.at<double>(0,0) = sigma2_tx_measurement_;
    C_.at<double>(1,1) = sigma2_ty_measurement_;
    C_.at<double>(2,2) = sigma2_tz_measurement_;
    C_.at<double>(3,3) = sigma2_rx_measurement_;
    C_.at<double>(4,4) = sigma2_ry_measurement_;
    C_.at<double>(5,5) = sigma2_rz_measurement_;


    kalman_core_ = std::make_unique<KalmanCore> 
    ( 
      sigma_system_,
      C_,
      T_,
      min_depth_, max_depth_,
      min_height_, camera_ground_distance_, 
      use_ego_motion_,
      use_ego_var_,
      ego_compensation_factor_,
      grid_size_,
      debug_image_grid_
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
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));

    // Camera info subscription (standard rclcpp subscription)
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 
      rclcpp::SensorDataQoS(),
      std::bind(&KalmanFilterNode::cameraInfoCallback, this, std::placeholders::_1));

    // Publishers
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 5);
    output_6d_pub_   = this->create_publisher<sensor_msgs::msg::Image>(output_6d_topic_, 5);
    debug_markers_pub_   = this->create_publisher<visualization_msgs::msg::MarkerArray>(debug_markers_topic_, 2);

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
    cv::Mat color_image_raw = imageMsgToMat(color_msg);

    // Detect grayscale and convert to BGR
    cv::Mat color_image;
    if (color_image_raw.channels() == 1) {
      cv::cvtColor(color_image_raw, color_image, cv::COLOR_GRAY2BGR);
    } else {
      color_image = color_image_raw;
    }

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

    
    // We invert the rotation matrix to get the transformation from camera to world
    //m = m.inverse();  

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

    // Compute delta time
    if (first_time_)
    {
      time_diff_ = 0.05;
      last_time_ = this->now();
      first_time_ = false;  
    }
    else
    {
      time_diff_ = calculateDeltaTime(last_time_);
      last_time_ = this->now();
    }

    // Update the Kalman filter
    KalmanCoreErrorCode result = kalman_core_->updateSyncedData(
      flow_image, 
      depth_image, 
      color_image, 
      odometry_matrix,
      time_diff_);

    // Retrieve outputs
    cv::Mat output_6d, output_debug_image;
    double delta_time;

    kalman_core_->getOutput(output_6d ,output_debug_image);
    delta_time = kalman_core_->getDeltaTime();
    

    // Create header object
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = frame_id_;

    // Convert output_6d and output_debug_image to sensor_msgs::msg::Image
    sensor_msgs::msg::Image::SharedPtr output_6d_msg =
      cv_bridge::CvImage(header, "32FC7", output_6d).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr debug_image_msg =
      cv_bridge::CvImage(header, "bgr8", output_debug_image).toImageMsg();

    // Publish
    if (!(result == KalmanCoreErrorCode::OK))
      RCLCPP_ERROR(this->get_logger(), "KalmanCore error: %s", getErrorMessage(result).c_str());

    visualization_msgs::msg::MarkerArray markers =  
      createMarkers(output_6d, delta_time);  
      
    
    output_6d_pub_->publish(*output_6d_msg);
    debug_image_pub_->publish(*debug_image_msg);
    debug_markers_pub_->publish(markers);
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


  visualization_msgs::msg::MarkerArray KalmanFilterNode::createMarkers(const cv::Mat &image_6d, double delta_time) {

  visualization_msgs::msg::MarkerArray marker_array;

  if (image_6d.empty() || image_6d.type() != OUT6D_TYPE) {
    RCLCPP_WARN(this->get_logger(), "createMarkers: Empty or incorrect image type (type=%d).", image_6d.type());
    return marker_array;
  }

  // Clear previous markers
  visualization_msgs::msg::Marker clear_markers;
  clear_markers.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_markers);

  const int rows = image_6d.rows;
  const int cols = image_6d.cols;

  // Pre-calculate approximate total markers to avoid dynamic resizing overhead
  marker_array.markers.reserve(rows * cols / 4); // approximation

  #pragma omp parallel
  {
    visualization_msgs::msg::MarkerArray local_markers;
    #pragma omp for nowait collapse(2)
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        const OutVec &vec = image_6d.at<OutVec>(i, j);
        if (vec[OUT6D_VAL_IDX] != 1.0f) continue;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = this->now();
        marker.id = i * cols + j; // unique ID, no contention
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        geometry_msgs::msg::Point start;
        start.x = vec[0];
        start.y = vec[1];
        start.z = vec[2];

        geometry_msgs::msg::Point end;
        end.x = vec[0] +   vec[3];
        end.y = vec[1] +   vec[4];
        end.z = vec[2] +   vec[5];
  

        marker.points.reserve(2);
        marker.points.push_back(start);
        marker.points.push_back(end);

        marker.scale.x = 0.035;
        marker.scale.y = 0.04;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        #pragma omp critical
        marker_array.markers.push_back(std::move(marker));
      }
    }

  }
  return marker_array;
}

  double KalmanFilterNode::calculateDeltaTime(rclcpp::Time& last_time)
  {
    rclcpp::Time current_time = this->now();
    rclcpp::Duration delta_time = current_time - last_time;
    return delta_time.seconds();
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
          name == "sigma2_depth_measurement")
      {
        if (name == "sigma2_flow_y_measurement") sigma2_flow_y_measurement_ = param.as_double();
        if (name == "sigma2_flow_x_measurement") sigma2_flow_x_measurement_ = param.as_double();
        if (name == "sigma2_depth_measurement") sigma2_depth_measurement_ = param.as_double();

        T_ = cv::Mat::zeros(3, 3, CV_64FC1);
        T_.at<double>(0, 0) = sigma2_flow_y_measurement_;
        T_.at<double>(1, 1) = sigma2_flow_x_measurement_;
        T_.at<double>(2, 2) = sigma2_depth_measurement_;
        kalman_core_->setT(T_);
      }

      if (name == "sigma2_tx_measurement" ||
          name == "sigma2_ty_measurement" || 
          name == "sigma2_tz_measurement" || 
          name == "sigma2_rx_measurement_" || 
          name == "sigma2_ry_measurement_" || 
          name == "sigma2_rz_measurement_")
      {
        if (name == "sigma2_tx_measurement") sigma2_tx_measurement_ = param.as_double();
        if (name == "sigma2_ty_measurement") sigma2_ty_measurement_ = param.as_double();
        if (name == "sigma2_tz_measurement") sigma2_tz_measurement_ = param.as_double();
        if (name == "sigma2_rx_measurement_") sigma2_rx_measurement_ = param.as_double();
        if (name == "sigma2_ry_measurement_") sigma2_ry_measurement_ = param.as_double();
        if (name == "sigma2_rz_measurement_") sigma2_rz_measurement_ = param.as_double();

        C_ = cv::Mat::zeros(6, 6, CV_64FC1);
        C_.at<double>(0, 0) = sigma2_tx_measurement_;
        C_.at<double>(1, 1) = sigma2_ty_measurement_;
        C_.at<double>(2, 2) = sigma2_tz_measurement_;
        C_.at<double>(3, 3) = sigma2_rx_measurement_;
        C_.at<double>(4, 4) = sigma2_ry_measurement_;
        C_.at<double>(5, 5) = sigma2_rz_measurement_;
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

      if (name == "camera_ground_distance") 
      {
        camera_ground_distance_ = param.as_double();
        kalman_core_->setMaxHeight(camera_ground_distance_);
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
  } // namespace stereo_perception

  int main(int argc, char **argv)
  {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stereo_perception::perception_pipeline::kalman_filter::KalmanFilterNode>();

    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }
