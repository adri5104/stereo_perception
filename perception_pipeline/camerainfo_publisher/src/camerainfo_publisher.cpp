#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;



class CameraInfoPublisher : public rclcpp::Node
{
  public:
    CameraInfoPublisher()
    : Node("camerainfo_publisher"), camera_info_()
    {
      publisher_image_ = this->create_publisher<sensor_msgs::msg::Image>("depth_registered/image_rect", 10);
      publisher_camerainfo_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 10);
      publisher_rightcamerainfo_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);
      // Subscriber to camera image topic
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/device_0/sensor_0/Depth_0/image/data", 10,
          std::bind(&CameraInfoPublisher::imageCallback, this, std::placeholders::_1));

      image_sub_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/device_0/sensor_1/Color_0/image/data", 10,
          std::bind(&CameraInfoPublisher::imageRgbCallback, this, std::placeholders::_1));

      info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          "/device_0/sensor_0/Depth_0/info/camera_info", 10,
          std::bind(&CameraInfoPublisher::infoCallback, this, std::placeholders::_1));

      publisher_image_rgb_ = this->create_publisher<sensor_msgs::msg::Image>("rgb/image_rect_color", 10);
      
    }

  private:
    void imageRgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      rgb_image_ = *msg;
    }
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    { 

      // Convert ROS Image message to OpenCV format
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
      // Ensure the image is interpreted as a 16-bit unsigned integer image
      cv::Mat converted_image;
      cv_ptr->image.convertTo(converted_image, CV_16UC1);

      // Prepare the new ROS Image message
      cv_bridge::CvImage out_msg;
      out_msg.header = msg->header; // Preserve the original message header
      out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      out_msg.image = converted_image;


      publisher_image_->publish(*out_msg.toImageMsg());


      publisher_image_rgb_->publish(rgb_image_);
      sensor_msgs::msg::CameraInfo camera_info_msg;
      
      camera_info_msg = camera_info_; 
      camera_info_msg.header = msg->header;
   
      // Intrinsic camera matrix (K)
      camera_info_msg.k = {
          421.37701416015625, 0.0, 424.79901123046875,
          0.0, 421.37701416015625, 231.86268615722656,
          0.0, 0.0, 1.0
      };

      // Distortion coefficients (D) using the Plumb Bob model
      double brown_d[5] = {-0.051035, 0.056578, -0.000681636, -0.000924746,-0.016071};
        k_1_ = brown_d[0];
        k_2_ = brown_d[1];
        p_1_ = brown_d[2];
        p_2_ = brown_d[3];
        k_3_ = brown_d[4];

      // Plumb Bob uses k1, k2, k3, p1, p2
      plumb_bob_d_ = {k_1_, k_2_, k_3_, p_1_, p_2_};
      camera_info_msg.distortion_model = "plumb_bob";
      camera_info_msg.d = plumb_bob_d_;

      // Rectification matrix (R)
      camera_info_msg.r = {
          0.0, 0.0, 0.0,
          0.0, 0.0, 0.0,
          0.0, 0.0, 0.0
      };

      // Projection matrix (P)
      camera_info_msg.p = {
          421.37701416015625, 0.0, 424.79901123046875, 0.0,
          0.0, 421.37701416015625, 231.86268615722656, 0.0,
          0.0, 0.0, 1.0, 0.0
      };

      publisher_camerainfo_->publish(camera_info_msg);
      sensor_msgs::msg::CameraInfo right_camera_info_msg;

      right_camera_info_msg = camera_info_msg;
      
      // Baseline in meters (converted from millimeters)
      double baseline_m = 95.13829040527344 / 1000.0;  // Convert mm to meters

      // Projection matrix (P)
      // Adding the baseline to the projection matrix for the right camera
      right_camera_info_msg.p = {
          421.37701416015625, 0.0, 424.79901123046875, -421.37701416015625 * baseline_m,  // fx * baseline
          0.0, 421.37701416015625, 231.86268615722656, 0.0,
          0.0, 0.0, 1.0, 0.0
      };

      publisher_rightcamerainfo_->publish(right_camera_info_msg);
      
    }

    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
      camera_info_ = *msg;
    }


    
    double k_1_, k_2_, k_3_, p_1_, p_2_;
    std::vector<double> plumb_bob_d_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_rgb_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_camerainfo_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_rightcamerainfo_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    sensor_msgs::msg::CameraInfo camera_info_;
    sensor_msgs::msg::Image rgb_image_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}