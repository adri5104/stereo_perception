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
  sync_input_time_old_(Clock::now()),
  time_diff_(0.0),
  include_ego_motion_(false),
  grid_size_worldpoints_(10),
  fx_(421.37701),
  fy_(421.37701),
  cx_(424.7990),
  cy_(231.86268),
  camera_parameters_set_(false),
  min_depth_(0.0),
  max_depth_(100.0),
  min_height_(0.0),
  max_height_(100.0),
  first_time_(true),
  C_(Mat::zeros(6, 6, CV_64FC1)),
  T_(Mat::zeros(3, 3, CV_64FC1)),
  sigma_system_(Mat::zeros(3, 3, CV_64FC1)),
  A_new_(Mat::zeros(6, 6, CV_64FC1)),
  u_new_(Mat::zeros(6, 1, CV_64FC1)),
  D_new_(Mat::zeros(6, 6, CV_64FC1)),
  Q_new_w_(Mat::zeros(6, 6, CV_64FC1)),
  G_new_(Mat::zeros(6, 6, CV_64FC1)),
  para_rot_(Mat::zeros(6, 1, CV_64FC1))
{
  // Initialize variances
  // Set covariance matrix of system model
  sigma_system_.at<double>(0,0) = 100;         // System: sigma^2 (x dot)
  sigma_system_.at<double>(1,1) = 100;         // System: sigma^2 (y dot)
  sigma_system_.at<double>(2,2) = 100;        // System: sigma^2 (z dot) 

  // Set covariance matrix of measurement model
  T_.at<double>(0,0) = 1000;//3.48152447145948;      // Measurement: sigma^2 (flow x)
  T_.at<double>(1,1) = 1000;//3.32965757670735;      // Measurement: sigma^2 (flow y)
  T_.at<double>(2,2) = 1000 ;                    // Measurement: sigma^2 (disparity)

  // Set covariance matrix of egomotion
  C_ .at<double>(0,0) = 4.240342549164191e-06;  // Egomotion: sigma^2 (t_x)
  C_.at<double>(1,1) = 7.746526425642957e-06;  // Egomotion: sigma^2 (t_y)
  C_.at<double>(2,2) = 7.673502160800497e-05;  // Egomotion: sigma^2 (t_z)
  C_.at<double>(3,3) = 9.636436187737289e-08;  // Egomotion: sigma^2 (Theta)
  C_.at<double>(4,4) = 4.651750796754109e-07;  // Egomotion: sigma^2 (Phi)
  C_.at<double>(5,5) = 2.070712542539731e-08;  // Egomotion: sigma^2 (Psi)
  worldpoints_.clear();
}

KalmanCore::KalmanCore(
  Mat sigma_system, Mat C, Mat T, double min_depth, double max_depth, double min_height, double max_height,
  bool useVarEgo, int gridSize) : 
  sync_input_time_old_(Clock::now()),
  time_diff_(0.0),
  include_ego_motion_(useVarEgo),
  grid_size_worldpoints_(gridSize),
  fx_(421.37701), fy_(421.37701), cx_(424.7990), cy_(231.86268),
  camera_parameters_set_(false),
  min_depth_(min_depth), max_depth_(max_depth),
  min_height_(min_height), max_height_(max_height),
  first_time_(true),
  C_(C), T_(T),sigma_system_(sigma_system), 
  A_new_(Mat::zeros(6, 6, CV_64FC1)),
  u_new_(Mat::zeros(6, 1, CV_64FC1)),
  D_new_(Mat::zeros(6, 6, CV_64FC1)),
  Q_new_w_(Mat::zeros(6, 6, CV_64FC1)),
  G_new_(Mat::zeros(6, 6, CV_64FC1)),
  para_rot_(Mat::zeros(6, 1, CV_64FC1))
{
  cout << "[KalmanCore] =================================================================" << endl;
  cout << "[KalmanCore] =============== KalmanCore object created =======================" << endl;
  cout << "[KalmanCore] =================================================================" << endl << endl;
  cout << "[KalmanCore] System Covariance Matrix = " << sigma_system_ << endl; 
  cout << "[KalmanCore] Measurement Covariance Matrix = " << T_ << endl; 
  cout << "[KalmanCore] Ego motion Covariance Matrix = " << C_ << endl; 
  cout << "[KalmanCore] Camera parameters: [fx fy cx cy] = [" << fx_ << " " << fy_ << " " << cx_ << " " << cy_ << "]" << endl;
  cout << "[KalmanCore] Valid depth [min max] = [" << min_depth_ << " " << max_depth_ << "]" << endl; 
  cout << "[KalmanCore] Valid height [min max] = [" << min_height_ << " " << max_height_ << "]" << endl;
  cout << "[KalmanCore] Grid size = " << grid_size_worldpoints_ << endl; 
  cout << "[KalmanCore] Use ego motion = " << (include_ego_motion_? "Yes" : "No") << endl; 
  

  worldpoints_.clear();
}

KalmanCore::~KalmanCore(void)
{
  for (auto& wp: worldpoints_)
  {
    wp.reset();
  }
}

KalmanCoreErrorCode KalmanCore::updateSyncedData(const Mat& optical_flow, const Mat& depth, const Mat& color_image, const Mat& egomotion)
{ 
  static int counter = 0;
  counter++;

  if (optical_flow.empty() )
  {
    return KalmanCoreErrorCode::BAD_OPTICAL_FLOW_IMAGE_ERROR;
  }

  if (depth.empty() )
  {
    return KalmanCoreErrorCode::BAD_DEPTH_IMAGE_ERROR;
  }

  if (color_image.empty() )
  {
    return KalmanCoreErrorCode::BAD_COLOR_IMAGE_ERROR;
  }

  if (include_ego_motion_)
  {
    if (egomotion.empty() )
    {
      return KalmanCoreErrorCode::BAD_EGOMOTION_ERROR;
    }

    input_egomotion_sync_ = egomotion;
  }

  input_optical_flow_sync_ = optical_flow;
  input_depth_sync_ = depth;
  input_color_image_sync_ = color_image;

  // Do the prediction process
  return predict(input_optical_flow_sync_, input_depth_sync_, input_color_image_sync_);
}

KalmanCoreErrorCode KalmanCore::updateSyncedData(const Mat& optical_flow, const Mat& depth, const Mat& color_image)
{ 
  if (camera_parameters_set_ == false)
  {
    return KalmanCoreErrorCode::CAMERA_PARAMETERS_NOT_SET_ERROR;
  }

  if (optical_flow.empty() )
  {
    return KalmanCoreErrorCode::BAD_OPTICAL_FLOW_IMAGE_ERROR;
  }

  if (depth.empty() )
  {
    return KalmanCoreErrorCode::BAD_DEPTH_IMAGE_ERROR;
  }

  if (color_image.empty() )
  {
    return KalmanCoreErrorCode::BAD_COLOR_IMAGE_ERROR;
  }

  include_ego_motion_ = false;
  input_optical_flow_sync_ = optical_flow;
  input_depth_sync_ = depth;
  input_color_image_sync_ = color_image;

  // Do the prediction process
  return predict(input_optical_flow_sync_, input_depth_sync_, input_color_image_sync_);
}

void KalmanCore::setCameraParameters(double fx, double fy, double cx, double cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  camera_parameters_set_ = true;
}


KalmanCoreErrorCode KalmanCore::predict(Mat input_optical_flow, Mat input_depth, Mat input_color_image)
{ 
  // Create output matrices
  // 6D output matrix (x, y, z, vx, vy, vz, validitiy)
  Mat output_6d		= Mat::zeros(input_color_image.rows,	input_color_image.cols,	CV_MAKETYPE(CV_32F, OUT6D_C));

  // Debug image
	Mat output_debug_image = input_color_image;

  // Occupancy map
  Mat occupancy_grid = Mat::zeros(input_color_image.rows / grid_size_worldpoints_, 
                               input_color_image.cols / grid_size_worldpoints_, CV_8UC1);

  // Draw lines on visual debug output if needed
  //for (int row = grid_size_worldpoints_; row < input_color_image.rows; row += grid_size_worldpoints_)
  //{
  //  cv::line(output_debug_image, 
  //          cv::Point(0, row), 
  //          cv::Point(input_color_image.cols, row), 
  //          cv::Scalar(255, 0, 0), 1);
  //}
  //for (int col = grid_size_worldpoints_; col < input_color_image.cols; col += grid_size_worldpoints_)
  //{
  //  cv::line(output_debug_image, 
  //          cv::Point(col, 0), 
  //          cv::Point(col, input_color_image.rows), 
  //          cv::Scalar(255, 0, 0), 1);
  //}
  time_diff_ = calculateTimeDifference(sync_input_time_old_);
  
  if(first_time_)
  {
    // Set the world points
    KalmanCoreErrorCode result = setNewWorldPoints(occupancy_grid, output_6d, output_debug_image);
    if(result != KalmanCoreErrorCode::OK)
    {
      return result;
    }
    first_time_ = false;
  }
  else
  { 
    // Compute the Kalman matrices
    computeKalmanMatrices();

    // Buffer values
    Mat z		= Mat::zeros(3,1,CV_64FC1);
    Mat x		= Mat::zeros(6,1,CV_64FC1);

    int pos_u	= 0;
    int pos_v	= 0;

    // Iterate over all worldpoints
    WorldPointErrorCode result;
    for(auto it = worldpoints_.begin(); it != worldpoints_.end();)
    { 
      auto& wp = *it;     // Read old measurement value
      wp->getZ(z);
   
      // Predict the state
      result = wp->computeKalmanStep(input_depth, 
                                     input_optical_flow, 
                                     A_new_, 
                                     D_new_, 
                                     Q_new_w_, 
                                     G_new_, 
                                     u_new_, 
                                     para_rot_, 
                                     term1, 
                                     term2, 
                                     term3, 
                                     term4, 
                                     occupancy_grid, 
                                     time_diff_);
      
      // Read current pixel position
			wp->getZ(z);
      pos_u = static_cast<int>(std::floor(z.at<double>(0,0)));
      pos_v = static_cast<int>(std::floor(z.at<double>(1,0)));
    
      switch (result)
      {
        case WorldPointErrorCode::OK:

          // Check if there is already a world point at the new position
          if(output_6d.at<OutVec>(pos_v, pos_u)[OUT6D_VAL_IDX] != 1.0)
          {
            wp->getX(x);
            output_6d.at<OutVec>(pos_v, pos_u) = formatOutput(x, 1);
            output_debug_image.at<Vec3b>(pos_v, pos_u) = Vec3b(0, 255, 0);
            it++;
          }
          else
          {
            it = worldpoints_.erase(it);
          }
        break;
        case WorldPointErrorCode::BAD_DEPTH_VALUE_ERROR:
          // We paint the point yellow
          output_debug_image.at<Vec3b>(pos_v, pos_u) = Vec3b(0, 255, 255);
          it = worldpoints_.erase(it);
        break;
        case WorldPointErrorCode::NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR:
          it = worldpoints_.erase(it);
        break;
        case WorldPointErrorCode::MEASUREMENT_OUT_OF_HEIGHT_BOUNDS_ERROR:
        case WorldPointErrorCode::THREE_SIGMA_TEST_FAILED:
        case WorldPointErrorCode::UNABLE_TO_GET_OPT_FLW_MSMT:
        case WorldPointErrorCode::DIVISION_BY_ZERO_ERROR:
        case WorldPointErrorCode::UNKNOWN_ERROR:
          // We paint the point red
          output_debug_image.at<Vec3b>(pos_v, pos_u) = Vec3b(0, 0, 255);
          it = worldpoints_.erase(it);
        break;

      }
    }

    // Refill gaps with new WorldPoints
    // We iterate over the depth image and create a new WorldPoint for each valid depth value
    for (int row = 0; row < occupancy_grid.rows; row ++) 
    {
      for (int col = 0; col < occupancy_grid.cols; col ++) 
      {
        int points = occupancy_grid.at<uchar>(row, col);
        if(points == 0)
        {
          int new_u = static_cast<int>(col*grid_size_worldpoints_ + grid_size_worldpoints_/2);
          int new_v = static_cast<int>(row*grid_size_worldpoints_ + grid_size_worldpoints_/2);
          if (new_u >= input_depth_sync_.cols) 
          { new_u = input_depth_sync_.cols - 1; }
          if (new_v >= input_depth_sync_.rows)
          { new_v = input_depth_sync_.rows - 1; }

          // Get depth 
          double depth = static_cast<double>(input_depth_sync_.at<u_int16_t>(new_v, new_u));
          depth = depth / 1000;  // Convert to meters
          
          if (depth < max_depth_ && depth > min_depth_)
          {
            //Create a new WorldPoint
            worldpoints_.emplace_back(std::make_unique<WorldPoint>(
                C_, T_,
                min_depth_, max_depth_, 
                min_height_, max_height_,
                fx_, fy_, cx_, cy_, 
                include_ego_motion_, 
                grid_size_worldpoints_));

            worldpoints_.back()->initKalmanFilter(
                static_cast<double>(new_u),
                static_cast<double>(new_v),
                depth,
                occupancy_grid);

            worldpoints_.back()->getX(x);

          // Update the output matrices
          output_debug_image.at<cv::Vec3b>(new_v, new_u) = cv::Vec3b(0, 255, 0);
          output_6d.at<OutVec>(new_v, new_u) = formatOutput(x, 1);
          }  
        }
      }
    }
  }

  output_6d_ = output_6d;
  output_debug_image_ = output_debug_image;
  return KalmanCoreErrorCode::OK;
}

KalmanCoreErrorCode KalmanCore::setNewWorldPoints(Mat &occupancy_grid, Mat &output_6d, Mat &output_debug_image)
{
  // Verify that the input matrices are not empty
  if (input_depth_sync_.empty()) {
    return KalmanCoreErrorCode::SET_NEW_WP_DEPTH_EMPTY_ERROR;
  }
  if (input_optical_flow_sync_.empty()) {
    return KalmanCoreErrorCode::SET_NEW_WP_OPTICAL_FLOW_EMPTY_ERROR;
  }
  if (input_color_image_sync_.empty()) {
    return KalmanCoreErrorCode::SET_NEW_WP_COLOR_IMAGE_EMPTY_ERROR;
  }

  // Validate that the matrices have the same size
  if (input_depth_sync_.size() != output_6d.size() || 
    input_depth_sync_.size() != output_debug_image.size()) {
    return KalmanCoreErrorCode::SET_NEW_WP_WRONG_MAT_SIZE_ERROR;
  }

  // Temporary buffer for world point state
  Mat x = Mat::zeros(6, 1, CV_64FC1);

  // Iterate over the depth image and create a new WorldPoint for each valid depth value
  for (int row = (grid_size_worldpoints_ - 1) / 2; row < input_depth_sync_.rows; row += grid_size_worldpoints_) {
    for (int col = (grid_size_worldpoints_ - 1) / 2; col < input_depth_sync_.cols; col += grid_size_worldpoints_) {
      // Get the depth value
      double depth = static_cast<double>(input_depth_sync_.at<uint16_t>(row, col)) / 1000.0;
      if (depth <= min_depth_ || depth >= max_depth_) {
        continue; // Skip invalid depth values
      }

      // Create a new WorldPoint
      worldpoints_.emplace_back(std::make_unique<WorldPoint>(
        C_, T_,
        min_depth_, max_depth_, 
        min_height_, max_height_,
        fx_, fy_, cx_, cy_, 
        include_ego_motion_, 
        grid_size_worldpoints_));

      worldpoints_.back()->initKalmanFilter(
        static_cast<double>(col),
        static_cast<double>(row),
        depth,
        occupancy_grid);

      worldpoints_.back()->getX(x);

      // Update the output matrices
      output_debug_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0); // Green color
      output_6d.at<OutVec>(row, col) = formatOutput(x, 1);
    }
  }

  output_debug_image_ = output_debug_image;
  output_6d_ = output_6d;
  return KalmanCoreErrorCode::OK;
}

KalmanCoreErrorCode KalmanCore::computeKalmanMatrices()
{
	Mat A_new_w	= Mat::zeros(6, 6, CV_64FC1);

	// --- Precomputations ---

	// Filling A_new_w (Transition matrix)
  A_new_ = Mat::eye(6, 6, CV_64FC1);
  Mat scaled_identity = Mat::eye(3, 3, CV_64FC1) * time_diff_;
  Mat A_new_submatrix = A_new_.colRange(3, 6).rowRange(0, 3);
  scaled_identity.copyTo(A_new_submatrix);
  
  if (include_ego_motion_)
  {
    // Filling D_new_  (Egomotion rotation matrix)
    Mat tmp = D_new_.colRange(0,3).rowRange(0,3);		
    input_egomotion_sync_.colRange(0,3).rowRange(0,3).copyTo(tmp);	
    tmp		= D_new_.colRange(3,6).rowRange(3,6);		
    input_egomotion_sync_.colRange(0,3).rowRange(0,3).copyTo(tmp);	

    // Filling u_new_ (Egomotion translation vector)
    u_new_.at<double>(0,0) = input_egomotion_sync_.at<double>(0,3);
    u_new_.at<double>(1,0) = input_egomotion_sync_.at<double>(1,3);
    u_new_.at<double>(2,0) = input_egomotion_sync_.at<double>(2,3);

    // Compute the new state transition matrix A_new
    A_new_w = A_new_ * D_new_;
  }
  else
  {
    A_new_w = A_new_;
  }

	// Compute and fill Q_new_w
  // Precompute commonly used values for clarity
  Mat scaled_sigma_1 = (1.0 / 3.0) * time_diff_ * time_diff_ * sigma_system_;
  Mat scaled_sigma_2 = 0.5 * time_diff_ * sigma_system_;

  // Fill the top-left block (3x3) of Q_new_w
  Mat Q_top_left = Q_new_w_.colRange(0, 3).rowRange(0, 3);
  scaled_sigma_1.copyTo(Q_top_left);

  // Fill the top-right block (3x3) of Q_new_w
  Mat Q_top_right = Q_new_w_.colRange(3, 6).rowRange(0, 3);
  scaled_sigma_2.copyTo(Q_top_right);

  // Fill the bottom-left block (3x3) of Q_new_w
  Mat Q_bottom_left = Q_new_w_.colRange(0, 3).rowRange(3, 6);
  scaled_sigma_2.copyTo(Q_bottom_left);

  // Fill the bottom-right block (3x3) of Q_new_w
  Mat Q_bottom_right = Q_new_w_.colRange(3, 6).rowRange(3, 6);
  sigma_system_.copyTo(Q_bottom_right);

	// Compute jacobian for statetransformation G 
  if (include_ego_motion_)
  {
    computeJacobianMatrix();
  }

  return KalmanCoreErrorCode::OK;
}

KalmanCoreErrorCode KalmanCore::getOutput(cv::Mat &output_6d, cv::Mat &output_debug_image)
{
  if (!camera_parameters_set_)
  {
    return KalmanCoreErrorCode::CAMERA_PARAMETERS_NOT_SET_ERROR;
  }

  output_6d = output_6d_;
  output_debug_image = output_debug_image_;
  
  return KalmanCoreErrorCode::OK; 
}

double KalmanCore::getDeltaTime()
{ 
  return time_diff_;
}

OutVec KalmanCore::formatOutput(Vec6f x, int validity)
{
  OutVec output;
  output[0] = x[0]; // x
  output[1] = x[1]; // y
  output[2] = x[2]; // z
  output[3] = x[3]; // vx
  output[4] = x[4]; // vy
  output[5] = x[5]; // vz
  output[6] = static_cast<float>(validity); // validity
  return output;
}

double KalmanCore::calculateTimeDifference(TimePoint& lastTime) 
{
  TimePoint currentTime = Clock::now();
  auto duration = std::chrono::duration_cast<Seconds>(currentTime - lastTime).count();
  lastTime = currentTime; // Update the last time
  return duration;
}

void KalmanCore::computeJacobianMatrix()
{
  double Phi		= asin(input_egomotion_sync_.at<double>(2,1));
	double cPhi		= cos(Phi);
	double Psi		= acos(input_egomotion_sync_.at<double>(2,2) / cPhi);
	double Theta	= acos(input_egomotion_sync_.at<double>(1,1) / cPhi);

	double sPhi		= sin(Phi);
	double cTheta	= cos(Theta);
	double sTheta	= sin(Theta);
	double cPsi		= cos(Psi);
	double sPsi		= sin(Psi);

	// Fill vector for egomotion
	para_rot_.at<double>(0,0) = sTheta;
	para_rot_.at<double>(1,0) = cTheta;
	para_rot_.at<double>(2,0) = sPhi;
	para_rot_.at<double>(3,0) = cPhi;
	para_rot_.at<double>(4,0) = sPsi;
	para_rot_.at<double>(5,0) = cPsi;

	// Precalculate terms for forming G_new
	term1 = sPsi*sTheta - cPsi*cTheta*sPhi;
	term2 = cPsi*cTheta - sPhi*sPsi*sTheta;
	term3 = cTheta*sPsi + cPsi*sPhi*sTheta;
	term4 = cPsi*sTheta + cTheta*sPhi*sPsi;
	double term5   = -cPhi * sTheta;	
	double term6   = -cPhi * sPsi;		
	double term7   = cPhi * cTheta;
	double term8   = cPhi * cPsi;

	// Form G_new
	// Row 1
	G_new_.at<double>(0,0) = term2;
	G_new_.at<double>(0,1) = term5;
	G_new_.at<double>(0,2) = term3;
	G_new_.at<double>(0,3) = time_diff_ * term2;
	G_new_.at<double>(0,4) = -time_diff_ * cPhi * sTheta;
	G_new_.at<double>(0,5) = time_diff_ * term3;

	// Row 2
	G_new_.at<double>(1,0) = term4;
	G_new_.at<double>(1,1) = term7;
	G_new_.at<double>(1,2) = term1;
	G_new_.at<double>(1,3) = time_diff_ * term4;
	G_new_.at<double>(1,4) = time_diff_ * cPhi * cTheta;
	G_new_.at<double>(1,5) = time_diff_ * term1;

	// Row 3
	G_new_.at<double>(2,0) = term6;
	G_new_.at<double>(2,1) = sPhi;
	G_new_.at<double>(2,2) = term8;
	G_new_.at<double>(2,3) = -time_diff_ * cPhi * sPsi;
	G_new_.at<double>(2,4) = time_diff_ * sPhi;
	G_new_.at<double>(2,5) = time_diff_ * cPhi * cPsi;

	// Row 4
	G_new_.at<double>(3,3) = term2;
	G_new_.at<double>(3,4) = term5;
	G_new_.at<double>(3,5) = term3;

	// Row 5
	G_new_.at<double>(4,3) = term4;
	G_new_.at<double>(4,4) = term7;
	G_new_.at<double>(4,5) = term1;

	// Row 6
	G_new_.at<double>(5,3) = term6;
	G_new_.at<double>(5,4) = sPhi;
	G_new_.at<double>(5,5) = term8;
}
} // namespace kalman_filter
} // namespace perception_pipeline
