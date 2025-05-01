#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class DisparityToDepthNode : public rclcpp::Node
{
public:
    DisparityToDepthNode()
    : Node("disparity_to_depth_node")
    {
        baseline_ = 0.3; // meters

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/S30/full_resolution/left/disparity/camera_info", 10,
            std::bind(&DisparityToDepthNode::camera_info_callback, this, std::placeholders::_1));

        disparity_sub_ = image_transport::create_subscription(
            this, "/S30/full_resolution/left/disparity",
            std::bind(&DisparityToDepthNode::disparity_callback, this, std::placeholders::_1),
            "raw");

        depth_pub_ = image_transport::create_publisher(this, "/stereo/depth");
    }

private:
    image_transport::Subscriber disparity_sub_;
    image_transport::Publisher depth_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    double focal_length_ = -1.0;
    double baseline_;

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        focal_length_ = msg->k[0];
        
    }

    void disparity_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    if (focal_length_ <= 0.0 || baseline_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Waiting for valid camera intrinsics...");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        std::cout << "Disparity encoding: " << msg->encoding << std::endl;
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    const float scale = focal_length_ * baseline_;

    // Create the output depth image in millimeters (16-bit unsigned)
    cv::Mat depth_mm(msg->height, msg->width, CV_16UC1, cv::Scalar(0));

    // Case 1: Disparity is in 32-bit float format
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        cv::Mat disparity = cv_ptr->image;  // CV_32FC1

        for (int y = 0; y < disparity.rows; ++y) {
            for (int x = 0; x < disparity.cols; ++x) {
                float d = disparity.at<float>(y, x);
                if (d > 0.0f) {
                    float depth_m = scale / d;  // depth in meters
                    uint16_t depth_val_mm = static_cast<uint16_t>(depth_m * 1000.0f);  // convert to mm
                    depth_mm.at<uint16_t>(y, x) = depth_val_mm;
                } else {
                    depth_mm.at<uint16_t>(y, x) = 0;
                }
            }
        }

    // Case 2: Disparity is in 16-bit unsigned format (MONO16)
    } else if (msg->encoding == sensor_msgs::image_encodings::MONO16) {
        cv::Mat disparity_16u = cv_ptr->image;  // CV_16UC1

        // Apply the same scaling logic as the official driver: scale * 16
        const float scale16 = scale * 16.0f;

        for (int y = 0; y < disparity_16u.rows; ++y) {
            for (int x = 0; x < disparity_16u.cols; ++x) {
                uint16_t d = disparity_16u.at<uint16_t>(y, x);
                if (d > 0) {
                    float depth_m = scale16 / static_cast<float>(d);  // depth in meters
                    uint16_t depth_val_mm = static_cast<uint16_t>(depth_m * 1000.0f);  // convert to mm
                    depth_mm.at<uint16_t>(y, x) = depth_val_mm;
                } else {
                    depth_mm.at<uint16_t>(y, x) = 0;
                }
            }
        }

    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported disparity encoding: %s", msg->encoding.c_str());
        return;
    }

    // Publish the resulting depth image
    auto depth_msg = cv_bridge::CvImage(msg->header, "mono16", depth_mm).toImageMsg();
    depth_pub_.publish(depth_msg);
}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisparityToDepthNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
