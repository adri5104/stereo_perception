#ifndef _STEREO_COMPUTATION_NODE_HPP_
#define _STEREO_COMPUTATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#ifdef ROS_VERSION_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/core.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudaarithm.hpp>

namespace perception_pipeline
{
namespace stereo_computation 
{

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image
>;

/**
 * @class StereoComputationNode
 * @brief ROS2 Node that subscribes to left and right images and camera info,
 *       and computes the disparity map.
 * 
 */
class StereoComputationNode : public rclcpp::Node
{
  public:
    StereoComputationNode();

  private:

  // Callback for the message syncronizer
  void updateSync(
    const sensor_msgs::msg::Image::ConstSharedPtr left_image_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr right_image_msg);
  // CameraInfo callback
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  /**
   * @brief Callback invoked whenever one or more parameters are changed.
   *
   * @param params The list of parameters that were updated.
   * @return A result indicating whether the parameter update is accepted.
   */
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> &params);

  /**
   * @brief Helper function to convert from a ROS Image message to openCV Mat
   * 
   * @param msg 
   * @return cv::Mat 
   */
  cv::Mat imageMsgToMat(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /**
   * @brief Converts a 16-bit fixed-point (4 fractional bits) disparity map to a depth map on the GPU.
   *
   * StereoSGM output is 16-bit signed, scaled by 16 (4 fractional bits):
   *   raw_disparity_value = disparity_in_pixels * 16
   *
   * Thus, to get the real disparity in pixels, we divide by 16.0f. Then, we compute
   *   depth = (focal_length_ * baseline_) / disparity_in_pixels
   * 
   * @param disparity  Input 16-bit signed disparity map (CV_16S), from StereoSGM->compute().
   * @return           32-bit floating-point depth map (CV_32F)
   */
  cv::cuda::GpuMat disparityToDepth(const cv::cuda::GpuMat & disparity);

  // Message filter subscribers
  message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;

  // Camera info ros subscriber without message filter
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Synchronizer pointer
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Parameter handling
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;


  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_image_pub_;

  // CUDA-based stereo matcher.
  cv::Ptr<cv::cuda::StereoBM> stereoBM_; 

  // Topic names
  std::string in_left_image_topic_;
  std::string in_right_image_topic_;
  std::string in_camera_info_topic_;
  std::string out_depth_image_topic_;
  std::string out_disparity_image_topic_;

  // Calibration parameters.
  double focal_length_;  // typically from camera_info.k[0]
  double baseline_;      
  bool camera_parameters_set_ = false;



};

} // namespace stereo_computation
} // namespace perception_pipeline


#endif // _STEREO_COMPUTATION_NODE_HPP_
