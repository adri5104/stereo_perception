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

    // 	the linear size of the blocks compared by the algorithm. 
    //The size should be odd (as the block is centered at the current pixel). 
    //Larger block size implies smoother, though less accurate disparity map. 
    //Smaller block size gives more detailed disparity map, but there is higher chance 
    //for algorithm to find a wrong correspondence.
    this->declare_parameter<int>("block_size", 21);
    
    // 	Maximum disparity minus minimum disparity. The value is always greater than zero. 
    //In the current implementation, this parameter must be divisible by 16.
    this->declare_parameter<int>("num_disparities", 64);
    
    this->declare_parameter<int>("pre_filter_cap", 0); 
    this->declare_parameter<int>("pre_filter_size", 0); 
    this->declare_parameter<int>("pre_filter_type", 0); 

    this->declare_parameter<int>("texture_threshold", 0);

    
  
    //Maximum size of smooth disparity regions to consider their noise speckles and invalidate. 
    //Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
    //this->declare_parameter<int>("speckle_window_size", 0);

    // Maximum disparity variation within each connected component. 
    //If you do speckle filtering, set the parameter to a positive value,
    // it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.
    //this->declare_parameter<int>("speckle_range", 0);



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

    // Parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&StereoComputationNode::paramCallback, this, std::placeholders::_1));

    // Camera info subscription (standard rclcpp subscription)
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      in_camera_info_topic_, 10,
      std::bind(&StereoComputationNode::cameraInfoCallback, this, std::placeholders::_1));

    
    stereoBM_ = cv::cuda::createStereoBM(
      get_parameter("num_disparities").as_int(),
      get_parameter("block_size").as_int()
    );

    //stereoBM_->setPreFilterCap(get_parameter("pre_filter_cap").as_int());
    //stereoBM_->setPreFilterSize(get_parameter("pre_filter_size").as_int());
    //stereoBM_->setPreFilterType(get_parameter("pre_filter_type").as_int());
    stereoBM_->setTextureThreshold(get_parameter("texture_threshold").as_int());

    //stereoBM_->setSpeckleWindowSize(get_parameter("speckle_window_size").as_int());
    //stereoBM_->setSpeckleRange(get_parameter("speckle_range").as_int());


    // Publisher
    depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_depth_image_topic_, 10);
    disparity_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_disparity_image_topic_, 10);
  }

  void StereoComputationNode::updateSync(
    const sensor_msgs::msg::Image::ConstSharedPtr left_image_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr right_image_msg)
  {
    // Convert ROS Image messages to OpenCV Mat images and upload to GPU.
    cv::cuda::GpuMat image_left, image_right;
    image_left.upload(imageMsgToMat(left_image_msg));
    image_right.upload(imageMsgToMat(right_image_msg));


    // Compute disparity on the GPU.
    cv::cuda::GpuMat disparity;
    stereoBM_->compute(image_left, image_right, disparity);

    // Create colored disparity image for visualization.
    cv::cuda::GpuMat disparity_color;
    cv::cuda::drawColorDisp(disparity, disparity_color, 128);

    // Compute the depth image in CV_32FC1 (depth in mm)
    cv::cuda::GpuMat depth = disparityToDepth(disparity);

    // Download the depth image and disparity image to CPU for publishing.
    cv::Mat depth_cpu, disparity_cpu;
    depth.download(depth_cpu);
    disparity_color.download(disparity_cpu);  


    // Publish the disparity image as bgra
    cv_bridge::CvImage disparity_msg;
    disparity_msg.header = left_image_msg->header;
    disparity_msg.encoding = sensor_msgs::image_encodings::BGRA8;
    disparity_msg.image = disparity_cpu;
    disparity_image_pub_->publish(*disparity_msg.toImageMsg());

    // Publish the depth image as MONO8.
    cv_bridge::CvImage depth_msg;
    depth_msg.header = left_image_msg->header;
    depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_msg.image = depth_cpu;
    depth_image_pub_->publish(*depth_msg.toImageMsg());
  } 


  void StereoComputationNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    if (camera_parameters_set_)
    {
      return;
    }
    
    focal_length_ = msg->k[0];
    baseline_ = -msg->p[3] / focal_length_;

    RCLCPP_INFO(this->get_logger(), "Baseline: %f", baseline_);
    RCLCPP_INFO(this->get_logger(), "Focal length: %f", focal_length_);

    camera_parameters_set_ = true;
  }

  rcl_interfaces::msg::SetParametersResult 
    StereoComputationNode::paramCallback(const std::vector<rclcpp::Parameter> &params)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (auto &p : params)
    {
      const auto &name = p.get_name();



      if (name == "texture_threshold")
      {
        stereoBM_->setTextureThreshold(p.as_int());
      }


      if (name == "pre_filter_cap")
      {
        stereoBM_->setPreFilterCap(p.as_int());
      }

      if (name == "pre_filter_size")
      {
        stereoBM_->setPreFilterSize(p.as_int());
      }

      if (name == "pre_filter_type")
      {
        stereoBM_->setPreFilterType(p.as_int());
      }

      //if (name == "speckle_window_size")
      //{
      //  stereoBM_->setSpeckleWindowSize(p.as_int());
      //}
//
      //if (name == "speckle_range")
      //{
      //  stereoBM_->setSpeckleRange(p.as_int());
      //}

    }
    return result;
  }

  cv::cuda::GpuMat StereoComputationNode::disparityToDepth(const cv::cuda::GpuMat & disparity)
  {
    // The raw disparity is typically CV_16S with scale=16, meaning stored as "disp * 16".
    // Convert it to float: disp_float = disp / 16
    cv::cuda::GpuMat d_disp_float;
    disparity.convertTo(d_disp_float, CV_32F, 1.0f / 16.0f);

    // We'll compute: depth = (focal_length * baseline) / disp_float
    // First, create a mask for invalid disparity (disp <= 0)
    cv::cuda::GpuMat d_invalidMask;
    cv::cuda::compare(d_disp_float, 0.0f, d_invalidMask, cv::CMP_LE); 
    // d_invalidMask = 255 where disp_float <= 0, else 0

    // Compute the inverse of the disparity: 1 / disp_float
    cv::cuda::GpuMat d_inv_disp;
    cv::cuda::divide(1.0f, d_disp_float, d_inv_disp);

    // Multiply by (focal_length * baseline)
    cv::cuda::GpuMat d_depth;
    float scale = focal_length_ * baseline_;
    cv::cuda::multiply(d_inv_disp, scale, d_depth);

    // Zero out depth where disparity was invalid
    d_depth.setTo(0.0f, d_invalidMask);

    return d_depth;
  }

  cv::Mat StereoComputationNode::imageMsgToMat(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try
    {
      // 1) Convert to the original encoding
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      cv::Mat image = cv_ptr->image;

      // 2) If it's a color image (more than one channel), convert to grayscale
      if (image.channels() > 1)
      {
        // Typically BGR or RGB, but could be different. If you know for sure it’s BGR, use COLOR_BGR2GRAY.
        // If it could be RGB, use COLOR_RGB2GRAY. 
        // For simplicity, assume BGR here.
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
      }

      // 3) Ensure the final image is indeed 8-bit, single channel (mono8)
      //    If it's already 8UC1, this does nothing; otherwise it will convert bit depth.
      if (image.type() != CV_8UC1)
      {
        // If the current type is something else (like 16U, 32F, etc.), scale appropriately:
        double minVal, maxVal;
        cv::minMaxLoc(image, &minVal, &maxVal);
        // If the image is already 8-bit, this won't do anything.
        // Otherwise, e.g. if it's 16-bit, we scale it to [0,255].
        // Adjust the scaling if your data range is known or you prefer a different approach.
        image.convertTo(image, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
      }

      // Return guaranteed mono8
      return image;
    }
    catch (cv_bridge::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return cv::Mat();
    }
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