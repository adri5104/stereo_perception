#ifndef _UTILS_KALMAN_FILTER_HPP_
#define _UTILS_KALMAN_FILTER_HPP_
#include <opencv2/opencv.hpp>

namespace stereo_perception
{
namespace perception_pipeline
{
namespace kalman_filter
{

  struct EgoMotionRotationData {
    cv::Mat rvec; // Rotation vector [3x1]
    cv::Mat R; // Rotation matrix [3x3]
    cv::Mat dRdr; // Derivative of R_k w.r.t. rvec [9x3]
  };

} // namespace kalman_filter
} // namespace perception_pipeline
} // namespace stereo_perception

#endif  // _UTILS_KALMAN_FILTER_HPP_