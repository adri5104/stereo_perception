#ifndef KALMAN_FILTER__KALMAN_CORE_HPP_
#define KALMAN_FILTER__KALMAN_CORE_HPP_

#include <opencv2/opencv.hpp>
#include "kalman_filter/worldpoint.hpp"
#include <vector>
#include <string>

using namespace cv;

namespace perception_pipeline
{
namespace kalman_filter
{

/**
 * @class KalmanCore
 * @brief This class implements the pure logic of a Kalman Filter, 
 *        with no dependency on ROS 2.
 */
class KalmanCore
{
public:
  /**
   * @brief Constructor for the KalmanCore class.
   *        Initializes internal state, matrices, etc.
   */
  KalmanCore();

  /**
   * @brief Update step using optical flow data.
   *        
   * @param flow The optical flow image (CV_32FC2 or similar).
   */
  void updateOpticalFlow(const cv::Mat& flow);

  /**
   * @brief Update step using disparity data.
   *       
   * @param disparity The disparity image.
   */
  void updateDisparity(const cv::Mat& disparity);

  /**
   * @brief (Optional) Update step using camera parameters or camera info.
   * @param fx Focal length x
   * @param fy Focal length y
   * @param cx Principal point x
   * @param cy Principal point y
   */
  void setCameraParameters(double fx, double fy, double cx, double cy);

  /**
   * @brief Get the current state vector.
   * @return cv::Mat The internal state of the Kalman Filter.
   */
  cv::Mat getState() const;

private:
  
  // Vector containing references to tracked WorldPoints
  std::vector<WorldPoint*> worldpoints_;

  // Camera parameters
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  Mat projection_matrix_;
  Mat projection_matrix_inv_;

  // Kalman filter parameters
  Mat cov_system; // System covariance matrix
  Mat cov_measurement; // Measurement model covariance matrix
  Mat transition_new; // Transition matrix
  Mat ego_trans_new; // Egomotion translation vector
  Mat ego_rot_new; // Egomotion rotation matrix

};

} // namespace kalman_filter
} // namespace perception_pipeline

#endif  // KALMAN_FILTER__KALMAN_CORE_HPP_
