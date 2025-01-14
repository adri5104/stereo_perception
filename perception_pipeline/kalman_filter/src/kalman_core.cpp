#include "kalman_filter/kalman_core.hpp"
#include <iostream>

namespace perception_pipeline
{
namespace kalman_filter
{

KalmanCore::KalmanCore() :
  fx_(0.0), fy_(0.0), cx_(0.0), cy_(0.0),
  first_time_(true),
  C_(Mat::zeros(6, 6, CV_64FC1)),
  T_(Mat::zeros(3, 3, CV_64FC1)),
  sigma_system_(Mat::zeros(3, 3, CV_64FC1)),
  A_new_(Mat::zeros(6, 6, CV_64FC1)),
  u_new_(Mat::zeros(6, 1, CV_64FC1)),
  D_new_(Mat::zeros(6, 6, CV_64FC1)),
  Q_new_w_(Mat::zeros(6, 6, CV_64FC1)),
  G_new_(Mat::zeros(6, 6, CV_64FC1)),
  para_rot_(Mat::zeros(6, 1, CV_64FC1)),
  delta_time(0.0),
  grid_size_worldpoints(10),
  use_var_ego_(false)
{
  worldpoints_.clear();
}

KalmanCore::KalmanCore(
  Mat sigma_system, Mat C, Mat T, double fx, double fy, double cx, double cy, 
  bool useVarEgo, int gridSize) : 

  fx_(fx), fy_(fy), cx_(cx), cy_(cy),
  sigma_system_(sigma_system), C_(C), T_(T),
  first_time_(true),
  A_new_(Mat::zeros(6, 6, CV_64FC1)),
  u_new_(Mat::zeros(6, 1, CV_64FC1)),
  D_new_(Mat::zeros(6, 6, CV_64FC1)),
  Q_new_w_(Mat::zeros(6, 6, CV_64FC1)),
  G_new_(Mat::zeros(6, 6, CV_64FC1)),
  para_rot_(Mat::zeros(6, 1, CV_64FC1)),
  delta_time(0.0),
  grid_size_worldpoints(gridSize),
  use_var_ego_(useVarEgo)
{
  worldpoints_.clear();
}

KalmanCore::~KalmanCore(void)
{
  for (auto wp : worldpoints_)
  {
    delete wp;
  }
}


void KalmanCore::updateOpticalFlow(const cv::Mat& flow)
{
  // For demonstration, just print size
  std::cout << "[KalmanCore::updateOpticalFlow] flow size = " 
            << flow.cols << " x " << flow.rows << std::endl;

}

void KalmanCore::updateDepth(const cv::Mat& disparity)
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

  for (auto wp : worldpoints_)
  {
    wp->setCameraParameters(fx, fy, cx, cy);
  }
}



} // namespace kalman_filter
} // namespace perception_pipeline
