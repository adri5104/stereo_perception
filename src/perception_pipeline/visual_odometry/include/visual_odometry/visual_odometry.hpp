#ifndef VIS_ODOM_
#define VIS_ODOM_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <deque>
#include <numeric>
#include <cmath>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudafeatures2d.hpp>

namespace perception_pipeline
{
namespace visual_odometry
{
    
  class VisualOdometry
  {
    public:
      
      /**
       * @brief Construct a new Visual Odometry object
       * 
       * @param min_depth 
       * @param max_depth 
       * @param create_debug_image 
       */
      VisualOdometry(double min_depth, double max_depth, bool create_debug_image, bool apply_statistical_filtering, bool apply_expotential_smoothing, double exponential_alpha);

      /**
       * @brief Update the synchronized input images
       * 
       * @param color_image The RGB image
       * @param depth_image The depth image
       * 
       */
      void updateSync(const cv::Mat& color_image, const cv::Mat& depth_image);

      

      /**
       * @brief Get the Output object
       * 
       * @param translation translation vector
       * @param rotation rotation vector
       * @param covariance covariance matrix
       * @param debug debug image
       */
      void getOutput(cv::Mat& translation, cv::Mat& rotation, cv::Mat& covariance, cv::Mat& debug) const;
 
      /**
       * @brief Set the Camera Parameters object
       * 
       * @param fx focal length in x
       * @param fy focal length in y
       * @param cx principal component in x
       * @param cy principal component in y
       */
      void setCameraParameters(double fx, double fy, double cx, double cy);

      void setMinMaxDepth(double min_depth, double max_depth);

    private:

      bool camera_info_arrived_; ///< Flag to check if camera info has arrived
      bool create_debug_image_; //< If true, debug image will be computed
      double min_depth_; ///< Min considered depth of the depth image
      double max_depth_; ///< Max considered depth of the depth image

      cv::Mat camera_matrix_;  ///< Camera intrinsic matrix

      cv::cuda::GpuMat prev_color_image_gpu_;  ///< Previous color image (GPU)
      cv::cuda::GpuMat prev_depth_image_gpu_;  ///< Previous depth image (GPU)

      cv::Mat debug_image_; ///< Image that contain the tracked keypoints
      

      cv::Mat translation_;  ///< Current translation vector
      cv::Mat rotation_;     ///< Current rotation by 
      cv::Mat translation_prev;  ///< Current translation vector
      cv::Mat rotation_prev;     ///< Current rotation by 



      // Sliding windows for linear and angular velocities
      // Sliding windows for translation components
      std::deque<double> translation_x_window_;
      std::deque<double> translation_y_window_;
      std::deque<double> translation_z_window_;

      // Sliding windows for rotation components
      std::deque<double> roll_window_;
      std::deque<double> pitch_window_;
      std::deque<double> yaw_window_;


      // Parameters for statistical filtering
      const size_t window_size_ = 10;            // Number of recent values to track
      const double outlier_threshold_factor_ = 10.0; // Threshold as a multiple of stddev

      // Maximum thresholds for each component
      const double max_translation_threshold_ = 10.0; // Max translation (m)
      const double max_rotation_threshold_ = CV_PI / 5; // Max rotation (rad)

      // Helper methods for statistical calculations and outlier filtering

      /**
       * @brief Helper method to compute mean and standard deviation of a data set
       * 
       * @param data 
       * @param mean 
       * @param stddev 
       */
      void computeStatistics(const std::deque<double>& data, double& mean, double& stddev);
      double filterOutlier(double new_value, std::deque<double>& window, double max_threshold);

      /**
       * @brief Apply moving average filter to a window
       * 
       * @param window 
       * @param new_value 
       * @return double 
       */
      double applyMovingAverage(std::deque<double>& window, double new_value);

      cv::Mat covariance_;
      cv::Ptr<cv::cuda::ORB>  feature_detector_gpu_; ///< CUDA-based ORB feature detector
      cv::Ptr<cv::cuda::DescriptorMatcher> matcher_gpu_; ///< CUDA-based descriptor matcher

      /**
       * @brief Reproject a 2D point to 3D using the depth value
       * 
       * @param point The 2D pixel coordinates
       * @param depth The depth value at the pixel
       * @return cv::Point3f The reprojected 3D point
       */
      cv::Point3f reprojectTo3D(const cv::Point2f& point, float depth) const;

      /**
       * @brief Estimate motion between frames using 3D-2D correspondences
       * 
       * @param prev_points Points in the previous frame
       * @param curr_points Points in the current frame
       * @param prev_depth Depth image of the previous frame
       */
      void estimateMotion(const std::vector<cv::Point2f>& prev_points, const std::vector<cv::Point2f>& curr_points, const cv::Mat& prev_depth);

      /**
       * @brief Detect and match features using CUDA-based ORB and DescriptorMatcher
       * 
       * @param prev_image The previous GPU image
       * @param curr_image The current GPU image
       * @param prev_points Output: Points in the previous frame
       * @param curr_points Output: Corresponding points in the current frame
       */
      void detectAndMatchFeatures(const cv::cuda::GpuMat& prev_image, const cv::cuda::GpuMat& curr_image,
                                  std::vector<cv::Point2f>& prev_points, std::vector<cv::Point2f>& curr_points);

     // ros parameters
    bool apply_statistical_filtering_;
    bool apply_expotential_smoothing_;
    double exponential_alpha_;

  };
  
} // Namespace visual_odometry
} // Namespace perception_pipeline

#endif // VIS_ODOM_
