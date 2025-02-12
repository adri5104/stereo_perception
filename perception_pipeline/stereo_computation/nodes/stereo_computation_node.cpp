#include "stereo_computation_node.hpp"

namespace perception_pipeline
{
namespace stereo_computation 
{
  StereoComputationNode::StereoComputationNode() :
    Node("stereo_computation_node"),
    focal_length_(0.0),
    baseline_(0.0),
    camera_parameters_set_(false)
  {
    // Topic names parameters
    this->declare_parameter<std::string>("in_left_image_topic", "/left_image");
    this->declare_parameter<std::string>("in_right_image_topic", "/right_image");
    this->declare_parameter<std::string>("in_camera_info_topic", "/camera_info");
    this->declare_parameter<std::string>("out_depth_image_topic", "/depth_image");
    this->declare_parameter<std::string>("out_disparity_image_topic", "/disparity_image");

    // Read parameters
    in_left_image_topic_ = this->get_parameter("in_left_image_topic").as_string();
    in_right_image_topic_ = this->get_parameter("in_right_image_topic").as_string();
    in_camera_info_topic_ = this->get_parameter("in_camera_info_topic").as_string();
    out_depth_image_topic_ = this->get_parameter("out_depth_image_topic").as_string();
    out_disparity_image_topic_ = this->get_parameter("out_disparity_image_topic").as_string();

    // Message filter subscribers
    left_image_sub_.subscribe(this, in_left_image_topic_, rmw_qos_profile_sensor_data);
    right_image_sub_.subscribe(this, in_right_image_topic_, rmw_qos_profile_sensor_data);

    // Synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), 
      left_image_sub_, 
      right_image_sub_);

    // Register the synchronized callback
    sync_->registerCallback(&StereoComputationNode::updateSync, this);

    // Camera info subscription (standard rclcpp subscription)
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      in_camera_info_topic_, 10,
      std::bind(&StereoComputationNode::cameraInfoCallback, this, std::placeholders::_1));

    int minDisparity = 1;
    int numDisparities = 16; // Ensure numDisparities is a positive multiple of 16 and within the supported range
    int blockSize = 15;
    stereoBM_ = cv::cuda::createStereoBM();

    // Publisher
    depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_depth_image_topic_, 10);
    disparity_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_disparity_image_topic_, 10);
  }

void StereoComputationNode::updateSync(
  const sensor_msgs::msg::Image::ConstSharedPtr left_image_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr right_image_msg)
{
  // Print the focal length and baseline
  RCLCPP_INFO(this->get_logger(), "Focal length: %f", focal_length_);
  RCLCPP_INFO(this->get_logger(), "Baseline: %f", baseline_);

  // Convert ROS images to OpenCV format.
  cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;
  try 
  {
    cv_ptr_left = cv_bridge::toCvCopy(left_image_msg, left_image_msg->encoding);
    cv_ptr_right = cv_bridge::toCvCopy(right_image_msg, right_image_msg->encoding);
  } 
  catch (cv_bridge::Exception &e) 
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  // Ensure the images are grayscale.
  if (cv_ptr_left->image.channels() > 1)
    cv::cvtColor(cv_ptr_left->image, cv_ptr_left->image, cv::COLOR_BGR2GRAY);
  if (cv_ptr_right->image.channels() > 1)
    cv::cvtColor(cv_ptr_right->image, cv_ptr_right->image, cv::COLOR_BGR2GRAY);

  // Upload images to GPU.
  cv::cuda::GpuMat d_left, d_right;
  d_left.upload(cv_ptr_left->image);
  d_right.upload(cv_ptr_right->image);

  // Compute disparity on the GPU.
  cv::cuda::GpuMat d_disparity;
  stereoBM_->compute(d_left, d_right, d_disparity);

  // Download disparity map to CPU.
  cv::Mat disparity;
  d_disparity.download(disparity);

  

  // Compute the depth image in CV_32FC1 (depth in mm)
  cv::Mat depth(disparity.size(), CV_32F);
  for (int i = 0; i < disparity.rows; ++i) 
  {
    for (int j = 0; j < disparity.cols; ++j) 
    {
      float disp = static_cast<float>(disparity.at<short>(i, j)) / 16.0f;
      if (disp <= 0.0f)
        depth.at<float>(i, j) = 0.0f;
      else
        depth.at<float>(i, j) = static_cast<float>(focal_length_ * baseline_ / disp);
    }
  }


  // Publish the disparity image (optional).
  cv_bridge::CvImage disparity_msg;
  disparity_msg.header = left_image_msg->header;
  disparity_msg.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
  disparity_msg.image = disparity;
  disparity_image_pub_->publish(*disparity_msg.toImageMsg());

  // Publish the depth image as MONO8.
  cv_bridge::CvImage depth_msg;
  depth_msg.header = left_image_msg->header;
  depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg.image = depth;
  depth_image_pub_->publish(*depth_msg.toImageMsg());
} 


  void StereoComputationNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    if (camera_parameters_set_)
    {
      return;
    }

    // Print camera info
    RCLCPP_INFO(this->get_logger(), "Received camera info: ");
    RCLCPP_INFO(this->get_logger(), "  width: %d", msg->width);
    RCLCPP_INFO(this->get_logger(), "  height: %d", msg->height);
    RCLCPP_INFO(this->get_logger(), "  distortion_model: %s", msg->distortion_model.c_str());
    RCLCPP_INFO(this->get_logger(), "  D: %f %f %f %f %f", msg->d[0], msg->d[1], msg->d[2], msg->d[3], msg->d[4]);
    RCLCPP_INFO(this->get_logger(), "  K: %f %f %f %f %f %f %f %f %f", msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5], msg->k[6], msg->k[7], msg->k[8]);
    RCLCPP_INFO(this->get_logger(), "  R: %f %f %f %f %f %f %f %f %f", msg->r[0], msg->r[1], msg->r[2], msg->r[3], msg->r[4], msg->r[5], msg->r[6], msg->r[7], msg->r[8]);
    RCLCPP_INFO(this->get_logger(), "  P: %f %f %f %f %f %f %f %f %f %f %f %f", msg->p[0], msg->p[1], msg->p[2], msg->p[3], msg->p[4], msg->p[5], msg->p[6], msg->p[7], msg->p[8], msg->p[9], msg->p[10], msg->p[11]);
    RCLCPP_INFO(this->get_logger(), "  binning_x: %d", msg->binning_x);
    RCLCPP_INFO(this->get_logger(), "  binning_y: %d", msg->binning_y);
    

    focal_length_ = msg->k[0];
    baseline_ = -msg->p[3] / focal_length_;

    RCLCPP_INFO(this->get_logger(), "Baseline: %f", baseline_);
    RCLCPP_INFO(this->get_logger(), "Focal length: %f", focal_length_);

    camera_parameters_set_ = true;
  }

} // namespace stereo_computation
} // namespace perception_pipeline

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_pipeline::stereo_computation::StereoComputationNode>());
  rclcpp::shutdown();
  return 0;
}   