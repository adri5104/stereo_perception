/**
 * @file worldpoint.cpp
 * @author adrian.rieker@tum.de
   * @brief This class implements the Kalman filter for a single world point
 *
 */

#include "kalman_filter/worldpoint.hpp"

namespace perception_pipeline {
namespace kalman_filter {

WorldPoint::WorldPoint(
  Mat &C, 
  Mat &T, 
  double min_depth, double max_depth, 
  double min_height, double max_height, 
  double fx, double fy, double cx, double cy,  
  bool &includeEgoMotion,
  bool &useVarEgo, 
  int gridSize)
: C_(C),  T_(T), 
  min_depth_(min_depth),   max_depth_(max_depth),
  min_height_(min_height), max_height_(max_height),
  include_ego_motion_(includeEgoMotion), use_var_ego_(useVarEgo), 
  grid_size_worldpoints(gridSize),
  age_(0),
  f_x_(fx), f_y_(fy), c_x_(cx), c_y_(cy)
{
}

WorldPoint::~WorldPoint(void) {}

WorldPointErrorCode WorldPoint::initKalmanFilter(
  const double &u,      // pixel coordinate u 
  const double &v,      // pixel coordinate v
  const double &depth,  // depth value
  Mat &occupancyGrid)
{
  // Initialize measurement vector
  z_old_ = Mat::zeros(3, 1, CV_64FC1);
  z_old_.at<double>(0,0) = u;
  z_old_.at<double>(1,0) = v;
  z_old_.at<double>(2,0) = depth;

  // Initialize statevector
	x_old_ = Mat::zeros(6, 1, CV_64FC1);				
	Mat tmp = Mat::zeros(4, 1, CV_64FC1);
	projectPixelToWorld(u, v, depth, tmp);							
	tmp.rowRange(0,3).copyTo(x_old_.rowRange(0,3));	
  
  // Initialize variances with 10
  P_old_ = Mat::eye(6, 6, CV_64FC1);
  P_old_.at<double>(0,0) = 1;
  P_old_.at<double>(1,1) = 1;
  P_old_.at<double>(2,2) = 1;
  P_old_.at<double>(3,3) = 1;
  P_old_.at<double>(4,4) = 1;
  P_old_.at<double>(5,5) = 1;
  
  // Increase occupancy grid value
  occupancyGrid.at<uchar>
    (static_cast<int>(std::floor(v / static_cast<double>(grid_size_worldpoints))), 
     static_cast<int>(std::floor(u / static_cast<double>(grid_size_worldpoints)))) += 1;

  // Allocate memory for the Kalman filter matrices
  x_new_pred_ = Mat::zeros(6, 1, CV_64FC1);  // Priori state vector
  x_new_      = Mat::zeros(6, 1, CV_64FC1);  // Posterior state vector
  z_new_pred_ = Mat::zeros(3, 1, CV_64FC1);  // Priori measurement vector
  z_new_      = Mat::zeros(3, 1, CV_64FC1);  // Posterior measurement vector
  s_new_      = Mat::zeros(3, 1, CV_64FC1);  // 3x1 innovation vector
  Q_new_      = Mat::zeros(6, 6, CV_64FC1);  // Process noise covariance matrix
  P_new_pred_ = Mat::zeros(6, 6, CV_64FC1);  // Priori estimate covariance matrix
  P_new_      = Mat::zeros(6, 6, CV_64FC1);  // Posterior estimate covariance matrix
  K_new_      = Mat::zeros(6, 3, CV_64FC1);  // Kalman gain matrix
  J_new_      = Mat::zeros(6, 6, CV_64FC1);  // Jacobian matrix of the egomotion
  H_new_      = Mat::zeros(3, 6, CV_64FC1);  // Jacobian matrix of the measurement model
  S_new_      = Mat::zeros(3, 3, CV_64FC1);  // Innovation covariance matrix
  S_new_inv_  = Mat::zeros(3, 3, CV_64FC1);  // Inverse of the innovation covariance matrix
  
  return WorldPointErrorCode::OK;
}

WorldPointErrorCode WorldPoint::computeKalmanStep(
  const Mat& input_depth,													
  const Mat& input_flow,
  const Mat& A_new,
  const Mat& D_new,
  const Mat& Q_new_w,
  const Mat& u_new,
  const EgoMotionRotationData& rot_new,
  Mat& occupancy_grid)
{
  // Increase age
  age_++;

  // Retrieve pixel optical flow
  try{
    const Vec2f& pixel_flow = 
      input_flow.at<Vec2f>(
        static_cast<int> (floor(z_old_.at<double>(1,0))), 
        static_cast<int> (floor(z_old_.at<double>(0,0))));

    // Update measurement vector
    z_new_.at<double>(0,0) = z_old_.at<double>(0,0) + (double) pixel_flow[0];	// u direction
    z_new_.at<double>(1,0) = z_old_.at<double>(1,0) + (double) pixel_flow[1];	// v direction
  }
  catch(const std::exception& e)
  {
    return WorldPointErrorCode::UNABLE_TO_GET_OPT_FLW_MSMT;
  }

  // New pixel coordinates
  int new_u = static_cast<int> (floor(z_new_.at<double>(0,0)));	// Get pixel coordinates (u)
	int new_v = static_cast<int> (floor(z_new_.at<double>(1,0)));	// Get pixel coordinates (v)

  // Check if new pixel coordinates are within the image
  if (new_u > 0 && new_u < input_depth.cols && new_v > 0 && new_v < input_depth.rows)
  {
    double new_depth = static_cast<double>(input_depth.at<uint16_t>(new_v, new_u)) / 1000.0; // Convert cm to m
  
    if (new_depth > min_depth_ && new_depth < max_depth_)
    {
      // Update measurement vector
      z_new_.at<double>(2,0) = new_depth;
      occupancy_grid.at<uchar>(
        static_cast<int>(std::floor(z_new_.at<double>(1,0) / static_cast<double>(grid_size_worldpoints))), 
        static_cast<int>(std::floor(z_new_.at<double>(0,0) / static_cast<double>(grid_size_worldpoints)))) += 1; // Increase counter value in occupancy grid
    }
    else
    {
      return WorldPointErrorCode::BAD_DEPTH_VALUE_ERROR;
    }
  }
  else
  {
    return WorldPointErrorCode::NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR;
  }

  // Prediction step

  // Compute the process noise covariance matrix
	if (use_var_ego_ && include_ego_motion_)
  {
    // 1. Predicción del estado utilizando la matriz de transición A_new
    Mat v = A_new * x_old_;  // Vector de estado predicho
    Mat v_pos = v.rowRange(0, 3);  // Posición (X, Y, Z)
    Mat v_vel = v.rowRange(3, 6);  // Velocidad (X, Y, Z)

    // 2. Cálculo de la matriz Jacobiana de la egomotion
    // Usamos directamente la matriz derivada dRdr de rot_new.
    Mat J_pos = Mat::zeros(3, 3, CV_64FC1);  // Inicialización de la matriz Jacobiana para posición
    Mat J_vel = Mat::zeros(3, 3, CV_64FC1);  // Inicialización de la matriz Jacobiana para velocidad



    // Asumiendo dRdr es 9x3 (derivadas de la matriz de rotación fila a fila)
    for (int i = 0; i < 3; ++i) {        // filas de R
      for (int j = 0; j < 3; ++j) {      // columnas de R
        int idx = 3 * i + j;            // posición en la matriz R (flattened row-wise)
        for (int k = 0; k < 3; ++k) {   // derivadas respecto a rvec
          double dR = rot_new.dRdr.at<double>(idx, k);
          J_pos.at<double>(i, k) += dR * v_pos.at<double>(j, 0);
          J_vel.at<double>(i, k) += dR * v_vel.at<double>(j, 0);
        }
      }
    }

    // 3. Actualización de la matriz Jacobiana total J_new_
    J_new_ = Mat::zeros(6, 6, CV_64FC1);
    J_new_(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_64FC1);  // Derivada respecto a tvec

    // Añadimos contribución de rvec
    J_new_(Range(0, 3), Range(3, 6)) = J_pos;
    J_new_(Range(3, 6), Range(3, 6)) = J_vel;
    
    
  
    // 4. Calculamos la nueva covarianza del proceso Q_new_
    Q_new_ = D_new * Q_new_w * D_new.t() + J_new_ * C_ * J_new_.t();
   
  }
  else
  {
    if (include_ego_motion_)
    {
      Q_new_ = D_new * Q_new_w * D_new.t() + C_;
    }
    else
    {
      Q_new_ = Q_new_w;
    }
  }

  // A. Prediction step
  if (include_ego_motion_)  
  { 
    x_new_pred_ = A_new * x_old_  + u_new;
  }
  else
  {
    x_new_pred_ = A_new * x_old_;
  }

  P_new_pred_ = A_new * P_old_ * A_new.t() + Q_new_;
  
  
  // B. Update step
  // Compute the Jacobian matrix of the measurement model
  double x_pred = x_new_pred_.at<double>(0,0);	
	double y_pred = x_new_pred_.at<double>(1,0);	
	double z_pred = x_new_pred_.at<double>(2,0);
  
  if(z_pred == 0 || std::isnan(z_pred)) z_pred = 0.00001;

  H_new_ = Mat::zeros(3, 6, CV_64FC1);
  H_new_.at<double>(0,0) = f_x_ / z_pred;
	H_new_.at<double>(0,2) = -(f_x_*x_pred)/(z_pred*z_pred);
	H_new_.at<double>(1,1) = f_y_ / z_pred;
	H_new_.at<double>(1,2) = -(f_y_*y_pred)/(z_pred*z_pred);
	H_new_.at<double>(2,2) = 1.0;		

  // Predict measurement
	projectWorldToPixel(x_pred, y_pred, z_pred, z_new_pred_);

  // Compute innovation vector
  s_new_ = z_new_ - z_new_pred_;

  // Compute innovation covariance matrix and it's inverse
	S_new_	  = T_ + H_new_ * P_new_pred_ * H_new_.t();
	cv::invert(S_new_, S_new_inv_, DECOMP_SVD);

  // Perform 3 sigma test
  Mat tmp = s_new_.t() * S_new_inv_ * s_new_;
	double epsilonSquared = tmp.at<double>(0,0);
	if (sqrt(epsilonSquared) > 3.0) 
  { 
    return WorldPointErrorCode::THREE_SIGMA_TEST_FAILED; 
  }

  // Kalman Gain computation
	K_new_ = P_new_pred_ * H_new_.t() * S_new_inv_;

  // --- Measurement update ---
	x_new_ = x_new_pred_ + K_new_ * s_new_;						
	P_new_ = (Mat::eye(6, 6, CV_64FC1) - K_new_*H_new_) * P_new_pred_;	

  // Check if point is within height bounds
  // In the camera frame, the height is the y coordinate.
  // Y looks down, so the height is negative
  if (x_new_.at<double>(1,0) < min_height_ || x_new_.at<double>(1,0) > max_height_)
  {
    return WorldPointErrorCode::MEASUREMENT_OUT_OF_HEIGHT_BOUNDS_ERROR;
    std::cout << "Height out of bounds" << std::endl;
  }

  // Save new state
  x_old_ = x_new_;
  P_old_ = P_new_;
  z_old_ = z_new_;

  // Print stuff
  //std::cout << "age: " << age_ << std::endl;
  //std::cout << "x: " << x_new_ << std::endl;
  //std::cout << "D_new: " << D_new << std::endl;
  return WorldPointErrorCode::OK;
}

void WorldPoint::projectPixelToWorld(
  const double &u, const double &v, const double &depth, Mat &coordinates)
{
  // Initialize vector for reprojection
	coordinates = Mat::zeros(4, 1, CV_64FC1);

  // Now do the standard pinhole math in meters
  double X = ((u - c_x_) / f_x_) * depth;
  double Y = ((v - c_y_) / f_y_) * depth;

	coordinates.at<double>(0,0) = X;
	coordinates.at<double>(1,0) = Y;
	coordinates.at<double>(2,0) = depth;
	coordinates.at<double>(3,0) =  1.0;
}


void WorldPoint::projectWorldToPixel(
  const double &x, const double &y, const double &z, Mat &pixels)
{
  // Create a 3x1 output
  pixels = cv::Mat::zeros(3, 1, CV_64F);

	double u = (f_x_ * (x / z)) + c_x_; 
  double v = (f_y_ * (y / z)) + c_y_;
  pixels.at<double>(0,0) = u;
	pixels.at<double>(1,0) = v;
	pixels.at<double>(2,0) = z;
}

void WorldPoint::getX(Mat &x)
{
  x_old_.copyTo(x);
}

void WorldPoint::getZ(Mat &z)
{
  z_old_.copyTo(z);
}

int WorldPoint::getAge()
{
  return age_;
}

void WorldPoint::setCameraParameters(double fx, double fy, double cx, double cy)
{
  f_x_ = fx;
  f_y_ = fy;
  c_x_ = cx;
  c_y_ = cy;
}

}  // namespace kalman_filter
}  // namespace perception_pipeline

