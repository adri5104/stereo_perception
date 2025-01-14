/**
 * @file kalman_core.cpp
 * @author adrian.rieker@tum.de
 * @brief 
 * @version 
 * @date 
 * 
 * 
 * 
 */

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
  grid_size_worldpoints(100),
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

KalmanCoreErrorCode KalmanCore::updateSyncedData(const Mat& optical_flow, const Mat& depth, const Mat& color_image)
{ 
  if (optical_flow.empty() )
  {
    std::cerr << "[KalmanCore::updateSyncedData] Optical flow image is empty!" << std::endl;
    return KalmanCoreErrorCode::BAD_OPTICAL_FLOW_IMAGE_ERROR;
  }

  if (depth.empty() )
  {
    std::cerr << "[KalmanCore::updateSyncedData] Depth image is empty!" << std::endl;
    return KalmanCoreErrorCode::BAD_DEPTH_IMAGE_ERROR;
  }

  if (color_image.empty() )
  {
    std::cerr << "[KalmanCore::updateSyncedData] Color image is empty!" << std::endl;
    return KalmanCoreErrorCode::BAD_COLOR_IMAGE_ERROR;
  }


  input_optical_flow_sync_ = optical_flow;
  input_depth_sync_ = depth;
  input_color_image_sync_ = color_image;

  

  return predict(input_optical_flow_sync_, input_depth_sync_, input_color_image_sync_);
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


KalmanCoreErrorCode KalmanCore::predict(Mat input_optical_flow, Mat input_depth, Mat input_color_image)
{
  std::cout << "[KalmanCore::predict] Predicting next state..." << std::endl;

  // Time measurement
  double time_kalman = (double) cv::getTickCount();

  // Initialization
  output_6d_ = Mat::zeros(input_color_image.rows, input_color_image.cols, CV_MAKETYPE(CV_32F, 6));
  output_debug_image_ = input_color_image;  
  occupancy_grid_ = Mat::zeros(input_color_image.rows / grid_size_worldpoints, 
                               input_color_image.cols / grid_size_worldpoints, CV_8UC1);

  // Draw lines on visual debug output if needed
  for (int row = grid_size_worldpoints-1; row < input_color_image.rows; row += grid_size_worldpoints)
  {
    cv::line(output_debug_image_, 
            cv::Point(0, row), 
            cv::Point(input_color_image.cols-1, row), 
            cv::Scalar(255, 0, 0), 1);
  }
  for (int col = grid_size_worldpoints-1; col < input_color_image.cols; col += grid_size_worldpoints)
  {
    cv::line(output_debug_image_, 
            cv::Point(col, 0), 
            cv::Point(col, input_color_image.rows-1), 
            cv::Scalar(255, 0, 0), 1);
  }

  if(first_time_)
  {
    // Set the world points
    std::cout << "[KalmanCore::predict] Initializating new worldpoints..." << std::endl;
    setNewWorldPoints(occupancy_grid_, output_6d_, output_debug_image_);
    first_time_ = false;
  }

  return KalmanCoreErrorCode::OK;

}

KalmanCoreErrorCode KalmanCore::setNewWorldPoints(Mat &occupancy_grid, Mat &output_6d, Mat &output_debug_image)
{
  std::cout << "[KalmanCore::setNewWorldPoints] Setting new world points..." << std::endl;

  // Temporary buffers
	Mat x	  = Mat::zeros(6,1,CV_64FC1);
	Vec6f tmp = 0;
	Mat z	  = Mat::zeros(3,1,CV_64FC1);
	int age	  = 0;

  // Start iterating over the grid
  int count_valid_depth = 0;
  for(int row = ((grid_size_worldpoints-1) / 2); 
      row < input_depth_sync_.rows ; 
      row+=grid_size_worldpoints)
  {
    float* ptrDepth	= input_depth_sync_.ptr<float>(row);
		Vec6f* ptr6D	= output_6d_.ptr<Vec6f>(row);
    for (int col = ((grid_size_worldpoints-1) / 2);
         col < input_depth_sync_.cols; 
         col += grid_size_worldpoints)
         {
            //// Check if current point has valid depth
            //if (ptrDepth[col] > 0)
            //{
            //  count_valid_depth++;
            //  output_debug_image.at<Vec3b>(row, col) = Vec3b(0, 255, 0);
//
            //  // Create WorldPoints
            //  worldpoints_.push_back(
            //    new WorldPoint(
            //      C_, T_, fx_, fy_, cx_, cy_, use_var_ego_, grid_size_worldpoints));  
//
            //  worldpoints_.back()->initKalmanFilter(
            //    (double) col, 
            //    (double) row, 
            //    (double) ptrDepth[col]);     
//
            //  worldpoints_.back()->getX(x);   
            //  std::cout << "[KalmanCore::setNewWorldPoints] Valid world points:" << count_valid_depth << "..." << std::endl;       
            //}
         }

  }
  std::cout << "[KalmanCore::setNewWorldPoints]AAAAAAAAAAAA" << std::endl;
}

KalmanCoreErrorCode KalmanCore::getOutput(cv::Mat &output_6d, cv::Mat &output_debug_image)
{
  output_6d = output_6d_;
  output_debug_image = output_debug_image_;
  return KalmanCoreErrorCode::OK; 
}

} // namespace kalman_filter
} // namespace perception_pipeline
