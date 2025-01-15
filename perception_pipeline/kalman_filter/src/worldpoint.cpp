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

#include "kalman_filter/worldpoint.hpp"

namespace perception_pipeline {
namespace kalman_filter {

WorldPoint::WorldPoint(Mat &C, Mat &T, double fx, double fy, double cx, double cy,  bool &useVarEgo, int gridSize)
:  C_(C), 
  T_(T), 
  use_var_ego_(useVarEgo), 
  grid_size_worldpoints(gridSize),
  age_(0),
  f_x_(fx),
  f_y_(fy),
  c_x_(cx),
  c_y_(cy)
{
}

WorldPoint::~WorldPoint(void)
{

}

void WorldPoint::initKalmanFilter(const double &u, const double &v, const double &depth,Mat &occupancyGrid)
{
  // Initialize measurement vector
  z_old_ = Mat::zeros(3, 1, CV_64FC1);
  z_old_.at<double>(0,0) = u;
  z_old_.at<double>(1,0) = v;
  z_old_.at<double>(2,0) = depth;

  // Initialize statevector
	x_old_ = Mat::zeros(6, 1, CV_64FC1);					// Initialize with zeros first
	Mat tmp = Mat::zeros(4, 1, CV_64FC1);
	projectPixelToWorld(u, v, depth, tmp);							
	Mat tmp2 = x_old_.rowRange(0,3);						
	tmp.rowRange(0,3).copyTo(tmp2);	

  // Initialize variances with 10 [m^2 respectively m^2/s^2]
  P_old_ = Mat::eye(6, 6, CV_64FC1) * 10;

  // Increase occupancy grid value
  occupancyGrid.at<uchar>
    (static_cast<int>(std::floor(v / static_cast<double>(grid_size_worldpoints))), 
     static_cast<int>(std::floor(u / static_cast<double>(grid_size_worldpoints)))) += 1;
}

WorldPointErrorCode WorldPoint::computeKalmanStep(
  const Mat& input_depth,													///< Method that runs a complete Kalmanstep with all necessary computations. Returns integer depending on the condition of the state vector
  const Mat& input_flow,
  const Mat& A_new,
  const Mat& D_new,
  const Mat& Q_new_w,
  const Mat& G_new,
  const Mat& u_new,
  const Mat& para_rot,
  const double& term1,
  const double& term2,
  const double& term3,
  const double& term4,
  Mat& occupancy_grid,
  const double& delta_time)
{
  // Initializations
  Mat x_new_pred = Mat::zeros(6, 1, CV_64FC1);
  Mat x_new = Mat::zeros(6, 1, CV_64FC1);

  Mat z_new_pred = Mat::zeros(3, 1, CV_64FC1);
  Mat z_new = Mat::zeros(3, 1, CV_64FC1);

  Mat s_new = Mat::zeros(3, 3, CV_64FC1); // Innovation vector
  Mat Q_new = Mat::zeros(6, 6, CV_64FC1); // Process noise covariance matrix
  Mat P_new_pred = Mat::zeros(6, 6, CV_64FC1); // Priori estimate covariance matrix
  Mat P_new = Mat::zeros(6, 6, CV_64FC1); // Posterior estimate covariance matrix
  Mat K_new = Mat::zeros(6, 3, CV_64FC1); // Kalman gain matrix
  Mat J_new = Mat::zeros(3, 3, CV_64FC1); // Jacobian matrix of the egomotion
  Mat H_new = Mat::zeros(3, 6, CV_64FC1); // Jacobian matrix of the measurement model
  Mat S_new = Mat::zeros(3, 3, CV_64FC1); // Innovation covariance matrix
  Mat S_new_inv = Mat::zeros(3, 3, CV_64FC1); // Inverse of the innovation covariance matrix

  // Increase age
  age_++;

  // Get new measurement

  const Vec2f& pixel_flow = 
    input_flow.at<Vec2f>(
      static_cast<int>(floor(z_old_.at<double>(1,0))), 
      static_cast<int> (floor(z_old_.at<double>(0,0))));

  try{
    const Vec2f& pixel_flow = 
      input_flow.at<Vec2f>(
        static_cast<int> (floor(z_old_.at<double>(1,0))), 
        static_cast<int> (floor(z_old_.at<double>(0,0))));

    // Update measurement vector
    z_new.at<double>(0,0) = z_old_.at<double>(0,0) + (double) pixel_flow[0];	// u direction
    z_new.at<double>(1,0) = z_old_.at<double>(1,0) + (double) pixel_flow[1];	// v direction
  }
  catch(const std::exception& e)
  {
    return WorldPointErrorCode::UNABLE_TO_GET_OPT_FLW_MSMT;
  }

  //cout << "[WorldPoint] Old pixel coordinates: " << z_old_.at<double>(0,0) << ", " << z_old_.at<double>(1,0) << endl;
  //cout << "[WorldPoint] Optical flow: " << pixel_flow[0] << ", " << pixel_flow[1] << endl;
  //cout << "[WorldPoint] New pixel coordinates: " << z_new.at<double>(0,0) << ", " << z_new.at<double>(1,0) << endl;

  

  int new_u = static_cast<int> (floor(z_new.at<double>(0,0)));	// Get pixel coordinates (u)
	int new_v = static_cast<int> (floor(z_new.at<double>(1,0)));	// Get pixel coordinates (v)



  // Check if new pixel coordinates are within the image
  if (new_u > 0 && new_u < input_depth.cols && new_v > 0 && new_v < input_depth.rows)
  {
    // Check if depth value has a valid value
    // Convert to meters first
    double new_depth = input_depth.at<double>(new_v, new_u) / 1000.0; // Convert mm to m
    if (new_depth > 0)
    {
      // Update measurement vector
      z_new.at<double>(2,0) = new_depth;
      occupancy_grid.at<uchar>(
        static_cast<int>(std::floor(z_new.at<double>(1,0) / static_cast<double>(grid_size_worldpoints))), 
        static_cast<int>(std::floor(z_new.at<double>(0,0) / static_cast<double>(grid_size_worldpoints)))) += 1; // Increase counter value in occupancy grid
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

	if (use_var_ego_)
  {
    // In progress
  }
  else
  {
		Q_new = D_new * Q_new_w * D_new.t();
  }

  // Kalman core algorithm

  // A. Prediction step
  // Compute the priori estimate
  x_new_pred = A_new * x_old_ + G_new * u_new; 

  

  // Compute the priori estimate covariance matrix
  P_new_pred = A_new * P_old_ * A_new.t() + Q_new;

  // B. Update step
  // Compute the Jacobian matrix of the measurement model
  double x_pred = x_new_pred.at<double>(0,0);	
	double y_pred = x_new_pred.at<double>(1,0);	
	double z_pred = x_new_pred.at<double>(2,0);


  if(z_pred == 0 || std::isnan(z_pred)) z_pred = 0.00001;


  H_new.at<double>(0,0) = f_x_ / z_pred;
	H_new.at<double>(0,2) = -(f_x_*x_pred)/(z_pred*z_pred);
	H_new.at<double>(1,1) = f_y_ / z_pred;
	H_new.at<double>(1,2) = -(f_y_*y_pred)/(z_pred*z_pred);
	H_new.at<double>(2,2) = 1.0;		

  // Predict measurement
	projectWorldToPixel(x_pred, y_pred, z_pred, z_new_pred);

  // Compute innovation vector
  s_new = z_new - z_new_pred;

  // Compute innovation covariance matrix and it's inverse
	S_new	  = T_ + H_new * P_new_pred * H_new.t();
	S_new_inv = S_new.inv();

  // Perform 3 sigma test
  //Mat tmp = s_new.t() * S_new_inv * s_new;
	//double epsilonSquared = tmp.at<double>(0,0);
	//if (sqrt(epsilonSquared) > 3.0) 
  //{ 
  //  return WorldPointErrorCode::THREE_SIGMA_TEST_FAILED; 
  //}

  // Kalman Gain computation
	K_new = P_new_pred * H_new.t() * S_new_inv;

  // --- Measurement update ---
	x_new = x_new_pred + K_new * s_new;								
	P_new = (Mat::eye(6, 6, CV_64FC1) - K_new*H_new) * P_new_pred;	

  // Save new state
  x_old_ = x_new;
  P_old_ = P_new;
  z_old_ = z_new;

  return WorldPointErrorCode::OK;
}


void WorldPoint::projectPixelToWorld(
  const double &u, const double &v, const uint16_t &depth, Mat &coordinates)
{
  // Initialize vector for reprojection
	coordinates = Mat::zeros(4, 1, CV_64FC1);


  // Now do the standard pinhole math in meters
  double X = (u - c_x_ / f_x_) * depth; 
  double Y = (v - c_y_ / f_y_) * depth;
  

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
  x = x_old_;
}

void WorldPoint::getZ(Mat &z)
{
  z = z_old_;
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

