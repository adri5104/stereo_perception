#include "kalman_filter/kalman_core.hpp"
#include <iostream>

namespace perception_pipeline
{
namespace kalman_filter
{

KalmanCore::KalmanCore()
{
  // Default camera parameters
  fx_ = 0.0; 
  fy_ = 0.0; 
  cx_ = 0.0; 
  cy_ = 0.0;
}


void KalmanCore::updateOpticalFlow(const cv::Mat& flow)
{
  // For demonstration, just print size
  std::cout << "[KalmanCore::updateOpticalFlow] flow size = " 
            << flow.cols << " x " << flow.rows << std::endl;

}

void KalmanCore::updateDisparity(const cv::Mat& disparity)
{
  // For demonstration, just print size
  std::cout << "[KalmanCore::updateDisparity] disparity size = " 
            << disparity.cols << " x " << disparity.rows << std::endl;
}

void KalmanCore::setCameraParameters(double fx, double fy, double cx, double cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  std::cout << "[KalmanCore::setCameraParameters] fx=" << fx_ 
            << ", fy=" << fy_ << ", cx=" << cx_ 
            << ", cy=" << cy_ << std::endl;
}



} // namespace kalman_filter
} // namespace perception_pipeline
