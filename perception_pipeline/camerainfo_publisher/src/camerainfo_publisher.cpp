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
using namespace cv;

/**
 * @brief Class for publishing camera info, depth image, and color image 
 *       with synchronized timestamps for visualization purposes using
 *      the depth_image_proc package.
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

      this->declare_parameter<bool>("publish_color_image", true);
      this->declare_parameter<bool>("publish_depth_image", true);


      // Read parameters
      color_image_pub_topic_ = this->get_parameter("color_image_pub_topic").as_string();
      depth_image_pub_topic_ = this->get_parameter("depth_image_pub_topic").as_string();
      camera_info_pub_topic_ = this->get_parameter("camera_info_pub_topic").as_string();
      color_image_sub_topic_ = this->get_parameter("color_image_sub_topic").as_string();
      depth_image_sub_topic_ = this->get_parameter("depth_image_sub_topic").as_string();
      camera_info_sub_topic_ = this->get_parameter("camera_info_sub_topic").as_string();
      publish_image_ = this->get_parameter("publish_color_image").as_bool();
      publish_depth_ = this->get_parameter("publish_depth_image").as_bool();


      // Initialize subscribers
      color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          color_image_sub_topic_, 10,
          std::bind(&CameraInfoPublisher::colorCallback, this, std::placeholders::_1));

      depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          depth_image_sub_topic_, 10,
          std::bind(&CameraInfoPublisher::depthCallback, this, std::placeholders::_1));
      
      camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          camera_info_sub_topic_, 10,
          std::bind(&CameraInfoPublisher::infoCallback, this, std::placeholders::_1));
      
      // Initialize publishers
      if (publish_depth_)
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_image_pub_topic_, 10);
      
      if (publish_image_)
        color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(color_image_pub_topic_, 10);

      
      camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_pub_topic_, 10);


    }

  private:

    void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      color_image_ = *msg;
      color_image_received_ = true;
    }

    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
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
      }
      // Default camera info values
      else
      {
        camera_info_msg.k = {
          421.37701416015625, 0.0, 424.79901123046875,
          0.0, 421.37701416015625, 231.86268615722656,
          0.0, 0.0, 1.0};
        
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
            0.0, 0.0, 0.0};
        

            // Projection matrix (P)
        camera_info_msg.p = {
          421.37701416015625, 0.0, 424.79901123046875, 0.0,
          0.0, 421.37701416015625, 231.86268615722656, 0.0,
          0.0, 0.0, 1.0, 0.0};

        camera_info_msg.header = msg->header;
      }
      
      if (publish_depth_)
        depth_image_pub_->publish(*msg);

      if (publish_image_)
      {
        if (color_image_received_)
        {
          sensor_msgs::msg::Image rgb_image_;
          rgb_image_ = color_image_;
          rgb_image_.header = msg->header;
          color_image_pub_->publish(color_image_);
        }
      }

      camera_info_pub_->publish(camera_info_msg);
    }


    double k_1_, k_2_, k_3_, p_1_, p_2_;
    std::vector<double> plumb_bob_d_;

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

    sensor_msgs::msg::CameraInfo camera_info_;
    sensor_msgs::msg::Image color_image_;
    size_t count_;
    bool publish_image_;
    bool publish_depth_;
    bool camera_info_received_;
    bool color_image_received_;
    bool depth_image_received_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}