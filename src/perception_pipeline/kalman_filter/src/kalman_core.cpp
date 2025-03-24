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
  time_diff_(0.0),
  include_ego_motion_(false),
  use_ego_var_(false),
  grid_size_worldpoints_(10),
  debug_image_grid_(false),
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
  A_new_w_(Mat::zeros(6, 6, CV_64FC1)),
  A_new_(Mat::zeros(6, 6, CV_64FC1)),
  u_new_(Mat::zeros(6, 1, CV_64FC1)),
  rot_new_(),
  D_new_(Mat::zeros(6, 6, CV_64FC1)),
  Q_new_w_(Mat::zeros(6, 6, CV_64FC1))
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
  bool include_ego_motion,
  bool use_var_ego,
  int gridSize,
  bool debug_image_grid) : 
  time_diff_(0.0),
  include_ego_motion_(include_ego_motion),
  use_ego_var_(use_var_ego),
  grid_size_worldpoints_(gridSize),
  debug_image_grid_(debug_image_grid),
  fx_(421.37701), fy_(421.37701), cx_(424.7990), cy_(231.86268),
  camera_parameters_set_(false),
  min_depth_(min_depth), max_depth_(max_depth),
  min_height_(min_height), max_height_(max_height),
  first_time_(true),
  C_(C), T_(T),sigma_system_(sigma_system), 
  A_new_w_(Mat::zeros(6, 6, CV_64FC1)),
  A_new_(Mat::zeros(6, 6, CV_64FC1)),
  u_new_(Mat::zeros(6, 1, CV_64FC1)),
  rot_new_(),
  D_new_(Mat::zeros(6, 6, CV_64FC1)),
  Q_new_w_(Mat::zeros(6, 6, CV_64FC1))
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
  cout << "[KalmanCore] Use ego motion covariance = " << (use_ego_var_? "Yes" : "No") << endl;
  cout << "[KalmanCore] =================================================================" << endl << endl;
  

  worldpoints_.clear();
}

KalmanCore::~KalmanCore(void)
{
  for (auto& wp: worldpoints_)
  {
    wp.reset();
  }
}

KalmanCoreErrorCode KalmanCore::updateSyncedData(
  const Mat& optical_flow, 
  const Mat& depth, 
  const Mat& color_image, 
  const Mat& egomotion,
  double time_diff)
{ 
  if (optical_flow.empty()) return KalmanCoreErrorCode::BAD_OPTICAL_FLOW_IMAGE_ERROR;
  if (depth.empty()) return KalmanCoreErrorCode::BAD_DEPTH_IMAGE_ERROR;
  if (color_image.empty()) return KalmanCoreErrorCode::BAD_COLOR_IMAGE_ERROR;
  if (include_ego_motion_ && egomotion.empty()) return KalmanCoreErrorCode::BAD_EGOMOTION_ERROR;

  input_optical_flow_sync_ = optical_flow;
  input_depth_sync_ = depth;
  input_color_image_sync_ = color_image;
  time_diff_ = time_diff;
  if (include_ego_motion_) input_egomotion_sync_ = egomotion;

  return predict(input_optical_flow_sync_, input_depth_sync_, input_color_image_sync_, time_diff_);
}

KalmanCoreErrorCode KalmanCore::updateSyncedData(
  const Mat& optical_flow, 
  const Mat& depth, 
  const Mat& color_image,
  double time_diff)
{ 
  if (!camera_parameters_set_) return KalmanCoreErrorCode::CAMERA_PARAMETERS_NOT_SET_ERROR;
  if (optical_flow.empty()) return KalmanCoreErrorCode::BAD_OPTICAL_FLOW_IMAGE_ERROR;
  if (depth.empty()) return KalmanCoreErrorCode::BAD_DEPTH_IMAGE_ERROR;
  if (color_image.empty()) return KalmanCoreErrorCode::BAD_COLOR_IMAGE_ERROR;

  include_ego_motion_ = false;
  input_optical_flow_sync_ = optical_flow;
  input_depth_sync_ = depth;
  input_color_image_sync_ = color_image;
  time_diff_ = time_diff;

  return predict(input_optical_flow_sync_, input_depth_sync_, input_color_image_sync_, time_diff_);
}

void KalmanCore::setCameraParameters(double fx, double fy, double cx, double cy)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  camera_parameters_set_ = true;
}


KalmanCoreErrorCode KalmanCore::predict(
  const Mat& input_optical_flow, 
  const Mat& input_depth, 
  const Mat& input_color_image, 
  double time_diff)
{ 

  Mat output_6d = Mat::zeros(input_color_image.rows, input_color_image.cols, CV_MAKETYPE(CV_32F, OUT6D_C));
  Mat output_debug_image = input_color_image.clone();  
  Mat occupancy_grid = Mat::zeros(
      input_color_image.rows / grid_size_worldpoints_,
      input_color_image.cols / grid_size_worldpoints_,
      CV_8UC1
  );

  if (debug_image_grid_) {
    for (int row = grid_size_worldpoints_; row < input_color_image.rows; row += grid_size_worldpoints_)
    {
      cv::line(output_debug_image, 
              cv::Point(0, row), 
              cv::Point(input_color_image.cols, row), 
              cv::Scalar(255, 0, 0), 1);
    }
    for (int col = grid_size_worldpoints_; col < input_color_image.cols; col += grid_size_worldpoints_)
    {
      cv::line(output_debug_image, 
              cv::Point(col, 0), 
              cv::Point(col, input_color_image.rows), 
              cv::Scalar(255, 0, 0), 1);
    }
  }
  
  if (first_time_) {
    KalmanCoreErrorCode result = setNewWorldPoints(occupancy_grid, output_6d, output_debug_image);
    if (result != KalmanCoreErrorCode::OK) return result;
    first_time_ = false;
    output_6d_ = output_6d;
    output_debug_image_ = output_debug_image;
    return KalmanCoreErrorCode::OK;
  }

  computeKalmanMatrices();

  // ---------------------------------------------------------------
  //  A: Parallel processing of existing worldpoints
  // ---------------------------------------------------------------

  // Copy of current worldpoints
  std::vector<WorldPoint*> current_wps;
  current_wps.reserve(worldpoints_.size());
  for (auto &wp : worldpoints_) {
    current_wps.push_back(wp.get());
  }

  // Vector for the results
  std::vector<WPResult> results(current_wps.size());

  // Paralell processing of each wp
  #pragma omp parallel for
  for (int i = 0; i < (int)current_wps.size(); i++) {

    //int tid = omp_get_thread_num();
    //std::cout << "[Iteration " << i << "] running in thread " << tid << std::endl;
    auto &wp = *current_wps[i];

    WPResult local;
    local.keep = true;  // default

    // 1) Compute kalman step
    WorldPointErrorCode wperr = wp.computeKalmanStep(
        input_depth,
        input_optical_flow,
        A_new_, D_new_, Q_new_w_, u_new_,
        rot_new_,
        occupancy_grid
    );
    local.error = wperr;

    // 2) store position after predition 
    cv::Mat z = cv::Mat::zeros(3,1, CV_64FC1);
    wp.getZ(z);
    local.pos_u = static_cast<int>(std::floor(z.at<double>(0,0)));
    local.pos_v = static_cast<int>(std::floor(z.at<double>(1,0)));

   

    // 3) Depending on the error we keep or not the wp
    switch (wperr) {
      case WorldPointErrorCode::OK: {
        cv::Mat x = cv::Mat::zeros(6,1, CV_64FC1);
        wp.getX(x);
        local.out_vec = formatOutput(x, 1.0f);
        local.keep = true;   // still havent checked collision
        break;
      }
      case WorldPointErrorCode::BAD_DEPTH_VALUE_ERROR:
      case WorldPointErrorCode::NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR:
      case WorldPointErrorCode::MEASUREMENT_OUT_OF_HEIGHT_BOUNDS_ERROR:
      case WorldPointErrorCode::THREE_SIGMA_TEST_FAILED:
      case WorldPointErrorCode::UNABLE_TO_GET_OPT_FLW_MSMT:
      case WorldPointErrorCode::DIVISION_BY_ZERO_ERROR:
      case WorldPointErrorCode::UNKNOWN_ERROR:
      default: {
        local.keep = false;
      }
    }

    // Store the value in results vector
    results[i] = local;
  } // end #pragma omp parallel for


  // ---------------------------------------------------------------
  // B: Sequential part 
  // ---------------------------------------------------------------
  std::vector<std::unique_ptr<WorldPoint>> new_worldpoints;
  new_worldpoints.reserve(worldpoints_.size());

  // Sequential for
  for (int i = 0; i < (int)current_wps.size(); i++) {
    auto &r = results[i];
    auto &wp_ptr = worldpoints_[i];  // unique_ptr<WorldPoint>&

    const int u = r.pos_u;
    const int v = r.pos_v;

    if (r.keep) {
      if (u < 0 || u >= output_6d.cols || v < 0 || v >= output_6d.rows) {
        continue;
      }

      auto &existing = output_6d.at<OutVec>(v, u);
      if (existing[OUT6D_VAL_IDX] == 1.0f) {
        continue;
      }

      existing = r.out_vec;
      output_debug_image.at<Vec3b>(v, u) = Vec3b(0, 255, 0);

      new_worldpoints.push_back(std::move(wp_ptr));
    }
    else {
      if (u >= 0 && u < output_debug_image.cols && 
          v >= 0 && v < output_debug_image.rows) 
      {
        switch(r.error) {
          case WorldPointErrorCode::BAD_DEPTH_VALUE_ERROR:
            output_debug_image.at<Vec3b>(v, u) = Vec3b(0, 255, 255); 
            break;
          default:
            output_debug_image.at<Vec3b>(v, u) = Vec3b(0, 0, 255);
            break;
        }
      }
    }
  }


  worldpoints_ = std::move(new_worldpoints);

  // ---------------------------------------------------------------
  // Fill gaps (sequential)
  // ---------------------------------------------------------------
  for (int row = 0; row < occupancy_grid.rows; row++)
  {
    for (int col = 0; col < occupancy_grid.cols; col++)
    {
      int points = occupancy_grid.at<uchar>(row, col);
      if (points == 0)
      {
        int new_u = static_cast<int>(col * grid_size_worldpoints_ + grid_size_worldpoints_ / 2);
        int new_v = static_cast<int>(row * grid_size_worldpoints_ + grid_size_worldpoints_ / 2);
        if (new_u >= input_depth_sync_.cols)
          new_u = input_depth_sync_.cols - 1;
        if (new_v >= input_depth_sync_.rows)
          new_v = input_depth_sync_.rows - 1;

        double depth = static_cast<double>(input_depth_sync_.at<u_int16_t>(new_v, new_u)) / 1000.0;
        if (depth < max_depth_ && depth > min_depth_)
        {
          worldpoints_.emplace_back(std::make_unique<WorldPoint>(
              C_, T_,
              min_depth_, max_depth_,
              min_height_, max_height_,
              fx_, fy_, cx_, cy_,
              include_ego_motion_,
              use_ego_var_,
              grid_size_worldpoints_));

          worldpoints_.back()->initKalmanFilter(
              static_cast<double>(new_u),
              static_cast<double>(new_v),
              depth,
              occupancy_grid);

          Mat x = Mat::zeros(6,1,CV_64FC1);
          worldpoints_.back()->getX(x);
          output_debug_image.at<cv::Vec3b>(new_v, new_u) = cv::Vec3b(0, 255, 0);
          output_6d.at<OutVec>(new_v, new_u) = formatOutput(x, 1);
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
        use_ego_var_, 
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
	A_new_= Mat::zeros(6, 6, CV_64FC1); 

	// Filling A_new_w (Transition matrix in world coordinates)
  A_new_w_ = Mat::eye(6, 6, CV_64FC1);
  Mat scaled_identity = Mat::eye(3, 3, CV_64FC1) * time_diff_;
  Mat A_new_submatrix = A_new_w_.colRange(3, 6).rowRange(0, 3);
  scaled_identity.copyTo(A_new_submatrix);
  
  if (include_ego_motion_)
  {
    // Compute rvec from egomotion input
    Mat rvec;
    Rodrigues(input_egomotion_sync_.rowRange(0,3).colRange(0,3), rvec);

    // Compute R and dRdrvec
    Mat R, dRdr;
    Rodrigues(rvec, R, dRdr);

    // Save rotation data 
    rot_new_.rvec = rvec;
    rot_new_.R = R;
    rot_new_.dRdr = dRdr;

    // Build D_new with R
    D_new_ = Mat::zeros(6, 6, CV_64FC1);
    R.copyTo(D_new_.rowRange(0,3).colRange(0,3));
    R.copyTo(D_new_.rowRange(3,6).colRange(3,6)); 
    // Build u_new with input_egomotion_sync_
    u_new_ = Mat::zeros(6, 1, CV_64FC1);
    u_new_.at<double>(0,0) = input_egomotion_sync_.at<double>(0,3);
    u_new_.at<double>(1,0) = input_egomotion_sync_.at<double>(1,3);
    u_new_.at<double>(2,0) = input_egomotion_sync_.at<double>(2,3);

    


    A_new_ = D_new_ * A_new_w_;
  }
  else
  {
    A_new_ = A_new_w_;
  }

  // Compute Q_new_w (Covariance matrix of discrete-time process without egomotion)
  cv::Mat scaled_sigma_1 = (1.0 / 3.0) * time_diff_ * time_diff_ * sigma_system_;
  cv::Mat scaled_sigma_2 = 0.5 * time_diff_ * sigma_system_;

  Q_new_w_ = cv::Mat::zeros(6, 6, CV_64FC1);
  scaled_sigma_1.copyTo(Q_new_w_.rowRange(0, 3).colRange(0, 3));
  scaled_sigma_2.copyTo(Q_new_w_.rowRange(0, 3).colRange(3, 6));
  scaled_sigma_2.copyTo(Q_new_w_.rowRange(3, 6).colRange(0, 3));
  sigma_system_.copyTo(Q_new_w_.rowRange(3, 6).colRange(3, 6));

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

  if (validity == 0)
  {
    output[6] = 0.0;
  }
  else
  {
    output[6] = 1.0;
  }
  
  return output;
}

// Setters
void KalmanCore::setIncludeEgoMotion(bool includeEgoMotion)
{
  include_ego_motion_ = includeEgoMotion;

  for (auto& wp: worldpoints_)
  {
    wp->setUseVarEgo(includeEgoMotion);
  }
}

void KalmanCore::setMinDepth(double minDepth)
{
  min_depth_ = minDepth;

  for (auto& wp: worldpoints_)
  {
    wp->setMinDepth(minDepth);
  }
}

void KalmanCore::setMaxDepth(double maxDepth)
{
  max_depth_ = maxDepth;

  for (auto& wp: worldpoints_)
  {
    wp->setMaxDepth(maxDepth);
  }
}

void KalmanCore::setMinHeight(double minHeight)
{
  min_height_ = minHeight;

  for (auto& wp: worldpoints_)
  {
    wp->setMinHeight(minHeight);
  }
}

void KalmanCore::setMaxHeight(double maxHeight)
{
  max_height_ = maxHeight;

  for (auto& wp: worldpoints_)
  {
    wp->setMaxHeight(maxHeight);
  }
}

void KalmanCore::setC(Mat C)
{
  C_ = C;

  for (auto& wp: worldpoints_)
  {
    wp->setC(C);
  }
}

void KalmanCore::setT(Mat T)
{
  T_ = T;

  for (auto& wp: worldpoints_)
  {
    wp->setT(T);
  }
}

void KalmanCore::setSigmaSystem(Mat sigmaSystem)
{
  sigma_system_ = sigmaSystem;
}


} // namespace kalman_filter
} // namespace perception_pipeline
