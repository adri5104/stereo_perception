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
  grid_size_worldpoints_(20),
  include_ego_motion_(false),
  time_diff_(0.0),
  sync_input_time_old_(Clock::now())
{
  // Initialize variances
  // Set covariance matrix of system model
  sigma_system_.at<double>(0,0) = 0.1;         // System: sigma^2 (x dot)
  sigma_system_.at<double>(1,1) = 0.1;         // System: sigma^2 (y dot)
  sigma_system_.at<double>(2,2) = 0.01;        // System: sigma^2 (z dot) 

  // Set covariance matrix of measurement model
  T_.at<double>(0,0) = 10;//3.48152447145948;      // Measurement: sigma^2 (flow x)
  T_.at<double>(1,1) = 30;//3.32965757670735;      // Measurement: sigma^2 (flow y)
  T_.at<double>(2,2) = 1;                    // Measurement: sigma^2 (disparity)

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
  grid_size_worldpoints_(gridSize),
  include_ego_motion_(useVarEgo),
  time_diff_(0.0),
  sync_input_time_old_(Clock::now())
{
  worldpoints_.clear();
}

KalmanCore::~KalmanCore(void)
{
  for (auto& wp : worldpoints_)
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

  // Time measurement
  time_diff_ = calculateTimeDifference(sync_input_time_old_);

  cout << "num images received: " << counter << endl;

  
  
  // Do the prediction process
  return predict(input_optical_flow_sync_, input_depth_sync_, input_color_image_sync_);
}

KalmanCoreErrorCode KalmanCore::updateSyncedData(const Mat& optical_flow, const Mat& depth, const Mat& color_image)
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

  include_ego_motion_ = false;

  input_optical_flow_sync_ = optical_flow;
  input_depth_sync_ = depth;
  input_color_image_sync_ = color_image;

  // Time measurement
  time_diff_ = calculateTimeDifference(sync_input_time_old_);
  
  // Do the prediction process
  cout << "[KalmanCore] initializating updateSyncedData iteration number: " << counter << endl;
  return predict(input_optical_flow_sync_, input_depth_sync_, input_color_image_sync_);
}


void KalmanCore::setCameraParameters(double fx, double fy, double cx, double cy)
{
  static bool first_time = true;
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;

  if (first_time)
  {
    for (auto& wp : worldpoints_)
    {
      wp->setCameraParameters(fx, fy, cx, cy);
    }
    first_time = false;
  }
}


KalmanCoreErrorCode KalmanCore::predict(Mat input_optical_flow, Mat input_depth, Mat input_color_image)
{ 

  static int counter = 0;
  counter++;
  cout << "[KalmanCore] initializating predict iteration number: " << counter << endl;

  
  
  // Create some output matrices

  // 6D output matrix (x, y, z, vx, vy, vz)
  Mat output_6d		= Mat::zeros(input_color_image.rows,	input_color_image.cols,	CV_MAKETYPE(CV_32F, 6));

  // Validity matrix (1 if it contains a world point, 0 otherwise)
  Mat output_6d_val	= Mat::zeros(input_color_image.rows,	input_color_image.cols,	CV_8UC1);

  // Debug image
	Mat output_debug_image = Mat::zeros(input_color_image.rows, input_color_image.cols,  CV_8UC3);

  
	
  // Occupancy grid matrix (Matrix that shows how many world points lie in every single part of the grid)
  int occ_h = static_cast<int>(std::ceil(static_cast<double>(input_depth.rows) / grid_size_worldpoints_)); // Always round up
  int occ_w = static_cast<int>(std::ceil(static_cast<double>(input_depth.cols) / grid_size_worldpoints_));  // Always round up
	Mat occupancy_grid = Mat::zeros(occ_h, occ_w, CV_8UC1);

  // Time measurement
  double time_kalman = (double) cv::getTickCount();

  // Initialization
  output_6d = Mat::zeros(input_color_image.rows, input_color_image.cols, CV_MAKETYPE(CV_32F, 6));
  output_debug_image = input_color_image;  
  occupancy_grid = Mat::zeros(input_color_image.rows / grid_size_worldpoints_, 
                               input_color_image.cols / grid_size_worldpoints_, CV_8UC1);

  // Draw lines on visual debug output if needed
  for (int row = grid_size_worldpoints_-1; row < input_color_image.rows; row += grid_size_worldpoints_)
  {
    cv::line(output_debug_image, 
            cv::Point(0, row), 
            cv::Point(input_color_image.cols-1, row), 
            cv::Scalar(255, 0, 0), 1);
  }
  for (int col = grid_size_worldpoints_-1; col < input_color_image.cols; col += grid_size_worldpoints_)
  {
    cv::line(output_debug_image, 
            cv::Point(col, 0), 
            cv::Point(col, input_color_image.rows-1), 
            cv::Scalar(255, 0, 0), 1);
  }

  if(first_time_)
  {
    // Set the world points
    KalmanCoreErrorCode result = setNewWorldPoints(occupancy_grid, output_6d, output_debug_image, output_6d_val);
    if(result != KalmanCoreErrorCode::OK)
    {
      return result;
    }
    first_time_ = false;

    cout << "Number of worldpoints: " << worldpoints_.size() << endl;
   
  }
  else
  { 
    
    // Compute the Kalman matrices
    computeKalmanMatrices();

    
    // Buffer values
    Mat z		= Mat::zeros(3,1,CV_64FC1);
    Mat x		= Mat::zeros(6,1,CV_64FC1);
		Vec6f tmp	= 0;
    int age		= 0;
    int pos_u_old = 0;
    int pos_v_old = 0;
    int pos_u	= 0;
    int pos_v	= 0;

    
    
    
    
    // Iterate over all worldpoints
    WorldPointErrorCode result;
    int i = 0;
    for(auto& wp : worldpoints_)
    {
      i++;
      // Read old measurement value
      wp->getZ(z);
      pos_u_old = static_cast<int>(std::floor(z.at<double>(0,0)));
      pos_v_old = static_cast<int>(std::floor(z.at<double>(1,0)));

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
                                     delta_time);
      
      // Read current pixel position
      
			wp->getZ(z);
      //cout << "New z: " << z << endl;
      pos_u = static_cast<int>(std::floor(z.at<double>(0,0)));
      pos_v = static_cast<int>(std::floor(z.at<double>(1,0)));
      //cout << "Total number of worldpoints: " << worldpoints_.size() << endl;
      //cout << "Point number: " << i << " Result: " << getErrorMessageWorldpoint(result) << endl;
      //
      //cout << "Old position: " << pos_u_old << " " << pos_v_old << " New position: " << pos_u << " " << pos_v << endl;
      
      switch(result)
      { 
        
        // Successful update
        case WorldPointErrorCode::OK:

          // Check if there is an entry at the current pixel position...
          if (output_6d_val.at<uchar>(pos_u, pos_v) == 0)
          {           
            
            // Save state value in output matrix
            wp->getX(x);
            output_6d.at<Vec6f>(pos_v, pos_u) = x;
            

            // Set correspondent validity entry to 1
            output_6d_val.at<uchar>(pos_v, pos_u) = 1;
            //cout << "[KalmanCore] end of iteration number: " << counter << endl; 

            // We paint the pixel in the debug image
            output_debug_image.at<Vec3b>(pos_v, pos_u) = Vec3b(0, 255, 0);	// GREEN
            
          }
          else
          {
            // We delete current worldpoint
            wp.reset();
            

            // Here we could do something more complex like merging the two worldpoints  
          }
          
        break;
      }

    }
  }

  cout << "[KalmanCore] end of predict iteration number: " << counter <<  endl;
  output_6d_ = output_6d;
  output_debug_image_ = output_debug_image;
  return KalmanCoreErrorCode::OK;
}

KalmanCoreErrorCode KalmanCore::setNewWorldPoints(Mat &occupancy_grid, Mat &output_6d, Mat &output_debug_image, Mat &output_6d_val)
{
    // Verify that the matrices are not empty
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

    // Temporary buffers
    cv::Mat x = cv::Mat::zeros(6, 1, CV_64FC1);
    int count_valid_depth = 0;

    // We iterate over the depth image and create a new WorldPoint for each valid depth value
    for (int row = (grid_size_worldpoints_ - 1) / 2; 
         row < input_depth_sync_.rows; 
         row += grid_size_worldpoints_) 
    {
        for (int col = (grid_size_worldpoints_ - 1) / 2; 
             col < input_depth_sync_.cols; 
             col += grid_size_worldpoints_) 
        {
            // Get the depth value
            float depth = input_depth_sync_.at<float>(row, col);
            if (depth <= 0) {
                continue; // Skip invalid depth values
            }

            // Create a new WorldPoint
            worldpoints_.emplace_back(std::make_unique<WorldPoint>(
                C_, T_, fx_, fy_, cx_, cy_, include_ego_motion_, grid_size_worldpoints_));

            worldpoints_.back()->initKalmanFilter(
                static_cast<double>(col),
                static_cast<double>(row),
                static_cast<double>(depth),
                occupancy_grid);

            worldpoints_.back()->getX(x);
            count_valid_depth++;

            // Update the output matrices
            output_debug_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0);
            output_6d_val.at<uchar>(row, col) = 1;

            // Update the 6D output matrix
            output_6d.at<cv::Vec6f>(row, col) = cv::Vec6f(x);
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
  Mat A_new_ = Mat::eye(6, 6, CV_64FC1);
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
  output_6d = output_6d_;
  output_debug_image = output_debug_image_;
  return KalmanCoreErrorCode::OK; 
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
