#ifndef VIS_ODOM_
#define VIS_ODOM_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
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
      VisualOdometry(double min_depth, double max_depth, bool create_debug_image);

      /**
       * @brief Update the synchronized input images
       * 
       * @param color_image The RGB image
       * @param depth_image The depth image
       * 
       */
      void updateSync(const cv::Mat& color_image, const cv::Mat& depth_image);

      

      /**
       * @brief Retrieve the current translation and rotation matrices
       * 
       * @param translation Output translation vector (3x1 matrix)
       * @param rotation Output rotation matrix (3x3 matrix)
       */
      void getOutput(cv::Mat& translation, cv::Mat& rotation, cv::Mat& debug) const;


       // Setters
      
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

     

  };
  
} // Namespace visual_odometry
} // Namespace perception_pipeline

#endif // VIS_ODOM_
