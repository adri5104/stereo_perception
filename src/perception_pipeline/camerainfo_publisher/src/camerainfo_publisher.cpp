/**
 * @file camerainfo_publisher.cpp
 * @author Adrian Rieker 
 * @brief ROS2 node to syncronize and re-publish camera info and color image with the 
 * depth image.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>

#ifdef ROS_VERSION_JAZZY
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h>
#endif


namespace stereo_perception
{
namespace perception_pipeline
{
namespace camerainfo_publisher
{

using namespace std::chrono_literals;
using namespace cv;


/**
 * @brief ROS2 node to synchronize and re-publish camera info and color image with the
 * depth image.
 * 
 */
class CameraInfoPublisher : public rclcpp::Node 
{
  public:
    CameraInfoPublisher() 
    : Node("camerainfo_publisher"), color_image_received_(false), camera_info_received_(false)
    { 
      // Declare parameters with default values
      this->declare_parameter<std::string>("color_image_pub_topic", "/perception_pipeline/color_sync");
      this->declare_parameter<std::string>("depth_image_pub_topic", "/perception_pipeline/depth_sync");
      this->declare_parameter<std::string>("camera_info_pub_topic", "/perception_pipeline/camera_info_sync");
      this->declare_parameter<std::string>("color_image_sub_topic", "/device_0/sensor_1/Color_0/image/data");
      this->declare_parameter<std::string>("depth_image_sub_topic", "/device_0/sensor_0/Depth_0/image/data");
      this->declare_parameter<std::string>("camera_info_sub_topic", "/device_0/sensor_0/Depth_0/info/camera_info");
      this->declare_parameter<bool>("publish_color_image", false);
      this->declare_parameter<bool>("publish_depth_image", false);
      this->declare_parameter<bool>("publish_camera_info", true);
      this->declare_parameter<std::string>("frame_id", "camera_optical_frame");

      // Read parameters
      color_image_pub_topic_ = this->get_parameter("color_image_pub_topic").as_string();
      depth_image_pub_topic_ = this->get_parameter("depth_image_pub_topic").as_string();
      camera_info_pub_topic_ = this->get_parameter("camera_info_pub_topic").as_string();
      color_image_sub_topic_ = this->get_parameter("color_image_sub_topic").as_string();
      depth_image_sub_topic_ = this->get_parameter("depth_image_sub_topic").as_string();
      camera_info_sub_topic_ = this->get_parameter("camera_info_sub_topic").as_string();
      publish_image_ = this->get_parameter("publish_color_image").as_bool();
      publish_depth_ = this->get_parameter("publish_depth_image").as_bool();
      publish_camera_info_ = this->get_parameter("publish_camera_info").as_bool();
      frame_id_ = this->get_parameter("frame_id").as_string();

      // Print parameters to console
      RCLCPP_INFO(this->get_logger(), "Publishing color image on topic: %s", color_image_pub_topic_.c_str());
      RCLCPP_INFO(this->get_logger(), "Publishing depth image on topic: %s", depth_image_pub_topic_.c_str());
      RCLCPP_INFO(this->get_logger(), "Publishing camera info on topic: %s", camera_info_pub_topic_.c_str());
      RCLCPP_INFO(this->get_logger(), "Subscribing to color image on topic: %s", color_image_sub_topic_.c_str());
      RCLCPP_INFO(this->get_logger(), "Subscribing to depth image on topic: %s", depth_image_sub_topic_.c_str());
      RCLCPP_INFO(this->get_logger(), "Subscribing to camera info on topic: %s", camera_info_sub_topic_.c_str());
      RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
      RCLCPP_INFO(this->get_logger(), "Publish color image: %s", publish_image_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "Publish depth image: %s", publish_depth_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "Publish camera info: %s", publish_camera_info_ ? "true" : "false");
      
      depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          depth_image_sub_topic_, 10,
          std::bind(&CameraInfoPublisher::depthCallback, this, std::placeholders::_1));
      if (publish_depth_)
      {
        
          
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_image_pub_topic_, 10);
        }
      
      if (publish_image_)
      {
        color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          color_image_sub_topic_, 10,
          std::bind(&CameraInfoPublisher::colorCallback, this, std::placeholders::_1));

        color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(color_image_pub_topic_, 10);
      }
        
      if (publish_camera_info_)
      {
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          camera_info_sub_topic_, 10,
          std::bind(&CameraInfoPublisher::infoCallback, this, std::placeholders::_1));
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_pub_topic_, 10);
      }


    }

  private:

    void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(color_image_mutex_);
      color_image_ = *msg;
      color_image_received_ = true;
    }

    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_info_ = *msg;
      camera_info_received_ = true;
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      sensor_msgs::msg::CameraInfo camera_info_msg;
      if(camera_info_received_) 
      {
        camera_info_msg = camera_info_;
        camera_info_msg.header = msg->header;
        camera_info_msg.header.frame_id = this->frame_id_;
        camera_info_msg.width = 848;
        camera_info_msg.height = 480;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Camera info not received yet.");
        return;
      }

    if (publish_depth_)
    { 
      msg->header.frame_id = this->frame_id_;

      // Change encoding from mono16 to 32FC1
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      cv_ptr->image.convertTo(cv_ptr->image, CV_32FC1, 0.001);
      msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depth_image_pub_->publish(*msg);
    }
    

    if (publish_image_)
    {
      if (color_image_received_)
      {
        sensor_msgs::msg::Image rgb_image_;
        rgb_image_ = color_image_;
        rgb_image_.header = msg->header;
        rgb_image_.header.frame_id = this->frame_id_;
        color_image_pub_->publish(rgb_image_);
      }
    }

    camera_info_pub_->publish(camera_info_msg);
    
    }

    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Topic name paramters
    std::string color_image_pub_topic_;
    std::string depth_image_pub_topic_;
    std::string camera_info_pub_topic_;
    std::string color_image_sub_topic_;
    std::string depth_image_sub_topic_;
    std::string camera_info_sub_topic_;
    std::string frame_id_;
    std::mutex camera_info_mutex_;
    std::mutex color_image_mutex_;

    sensor_msgs::msg::CameraInfo camera_info_;
    sensor_msgs::msg::Image color_image_;
    size_t count_;
    bool publish_image_;
    bool publish_depth_;
    bool publish_camera_info_;
    bool camera_info_received_;
    bool color_image_received_;
    bool depth_image_received_;
};


} // namespace camerainfo_publisher
} // namespace perception_pipeline
} // namespace stereo_perception

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<stereo_perception::perception_pipeline::camerainfo_publisher::CameraInfoPublisher>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}