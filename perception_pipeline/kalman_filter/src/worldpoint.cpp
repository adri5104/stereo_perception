#include "kalman_filter/worldpoint.hpp"

namespace perception_pipeline {
namespace kalman_filter {

WorldPoint::WorldPoint(Mat &projectionMatrix, Mat &projectionMatrixInv, Mat &C, Mat &T, bool &useVarEgo, int gridSize)
: projection_matrix_(projectionMatrix), 
  projection_matrix_inv_(projectionMatrixInv),
  C_(C), 
  T_(T), 
  use_var_ego_(useVarEgo), 
  grid_size_worldpoints(gridSize),
  age_(0),
  u_(0.0),
  v_(0.0)
{

	f_x_= projection_matrix_.at<double>(0,0);	// Copy focal length f
	c_x_  = projection_matrix_.at<double>(0,2);	// Copy center of coordinates x-position
	c_y_	= projection_matrix_.at<double>(1,2);	// Copy center of coordinates y-position
}

WorldPoint::~WorldPoint(void)
{

}

void WorldPoint::initKalmanFilter(const double &u, const double &v, const double &d)
{
  // Initialize pixel values
  u_ = u;
  v_ = v; 
  // Initialize measurement vector
  z_old_ = Mat::zeros(3, 1, CV_64FC1);
  z_old_.at<double>(0,0) = u;
  z_old_.at<double>(1,0) = v;
  z_old_.at<double>(2,0) = d;

  // Initialize statevector
	x_old_ = Mat::zeros(6, 1, CV_64FC1);					// Initialize with zeros first
	Mat tmp = Mat::zeros(4, 1, CV_64FC1);
	projectPixelTo3D(u, v, d, tmp);							// Reproject pixel values to 3D coordinates (u,v,d)^T -> (X,Y,Z,1)^T
	Mat tmp2 = x_old_.rowRange(0,3);						// Extract rows 0..2 (X,Y,Z) and copy them to m_x_old(0..2)
	tmp.rowRange(0,3).copyTo(tmp2);	

  // Initialize variances with 10 [m^2 respectively m^2/s^2]
  P_old_ = Mat::eye(6, 6, CV_64FC1) * 10;
}

KalmanErrorCode WorldPoint::computeKalmanStep(const Mat& inputDisp,													///< Method that runs a complete Kalmanstep with all necessary computations. Returns integer depending on the condition of the state vector
						   const Mat& inputFlow,
						   const Mat& A_new,
						   const Mat& D_new,
						   const Mat& Q_new_w,
						   const Mat& G_new,
						   const Mat& u_new,
						   const Mat& paraRot,
						   const double& term1,
						   const double& term2,
						   const double& term3,
						   const double& term4,
						   Mat& occupancyGrid,
						   const double& timediff)
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
    inputFlow.at<Vec2f>((int)floor(m_z_old.at<double>(1,0)), (int)floor(m_z_old.at<double>(0,0)));

  // Update measurement vector
  z_new.at<double>(0,0) = z_old_.at<double>(0,0) + (double) pixel_flow[0];	// u direction
  z_new.at<double>(1,0) = z_old_.at<double>(1,0) + (double) pixel_flow[1];	// v direction
  
  int new_u = (int)floor(z_new.at<double>(0,0));	// Get pixel coordinates (u)
	int new_v = (int)floor(z_new.at<double>(1,0));	// Get pixel coordinates (v)

  // Check if new pixel coordinates are within the image
  if (new_u > 0 && new_u < inputDisp.cols && new_v > 0 && new_v < inputDisp.rows)
  {
    // Check if disparity has a valid value
    if (double new_d = inputDisp.at<double>(new_v, new_u) > 0)
    {
      // Update measurement vector
      z_new.at<double>(2,0) = new_d;
      occupancyGrid.at<uchar>(
        (int)floor(z_new.at<double>(1,0) / (double) m_gridSizeWorldPoints), 
        (int)floor(z_new.at<double>(0,0) / (double) m_gridSizeWorldPoints)) += 1; // Increase counter value in occupancy grid
    }
    else
    {
      return KalmanErrorCode::BAD_DISPARITY_ERROR;
    }
  }
  else
  {
    return KalmanErrorCode::NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR;
  }

	if (m_useVarEgo)
  {
    // In progress
  }
  else
  {
    // Compute covariance matrix of the system model without using J_new and m_C 
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

  // Compute the Jacobian matrix of the measurement model
  H_new.at<double>(0,0) = m_f / z_pred;
	H_new.at<double>(0,2) = -(m_f*x_pred)/(z_pred*z_pred);
	H_new.at<double>(1,1) = m_f / z_pred;
	H_new.at<double>(1,2) = -(m_f*y_pred)/(z_pred*z_pred);
	H_new.at<double>(2,2) = -(m_projectionMatrix.at<double>(2,3)/(z_pred*z_pred));		// Term from reprojection matrix is b*f

  // Predict measurement
	project3DToPixel(x_pred, y_pred, z_pred, z_new_pred);

  // Compute innovation covariance matrix and it's inverse
	S_new	  = m_T + H_new * P_new_pred * H_new.t();
	S_new_inv = S_new.inv();

  // Perform 3 sigma test
  Mat tmp = s_new.t() * S_new_inv * s_new;
	double epsilonSquared = tmp.at<double>(0,0);
	if (sqrt(epsilonSquared) > 3.0) 
  { 
    return KalmanErrorCode::THREE_SIGMA_TEST_FAILED; 
  }

  // Kalman Gain computation
	K_new = P_new_pred * H_new.t() * S_new_inv;

  // --- Measurement update ---
	x_new = x_new_pred + K_new * s_new;								
	P_new = (Mat::eye(6, 6, CV_64FC1) - K_new*H_new) * P_new_pred;	

  // Save new state
  x_old_ = x_new;
  P_old_ = P_new;
  z_old_ = z_new;

  return KalmanErrorCode::OK;
}

void WorldPoint::projectPixelTo3D(
  const double &u, const double &v, const double &d, Mat &coordinates)
{
  // Initialize vector for reprojection
	coordinates = Mat::zeros(4, 1, CV_64FC1);
	coordinates.at<double>(0,0) = u;
	coordinates.at<double>(1,0) = v;
	coordinates.at<double>(2,0) = d;
	coordinates.at<double>(3,0) = (double) 1.0;

	// Reproject to 3D world coordinates
	coordinates = projection_matrix_inv_   * coordinates;	
	coordinates = coordinates * 1/coordinates.at<double>(3,0);
}

void WorldPoint::projectPixelTo3D(const double &u, const double &v, const uint16_t &depth, Mat &coordinates)
{
  // Initialize vector for reprojection
	coordinates = Mat::zeros(4, 1, CV_64FC1);

  // Convert depth to meters
  double depth_m = (double) depth / 1000.0;

  // Now do the standard pinhole math in meters
  double X = (u - c_x_ / f_x_) * depth_m; 
  double Y = (v - c_y_ / f_y_) * depth_m;
  

	coordinates.at<double>(0,0) = X;
	coordinates.at<double>(1,0) = Y;
	coordinates.at<double>(2,0) = depth_m;
	coordinates.at<double>(3,0) =  1.0;
}


void WorldPoint::project3DToPixel(const double &x, const double &y, const double &z, Mat &pixels)
{
  // Create a 3x1 output
  pixels = cv::Mat::zeros(3, 1, CV_64F);

	double u = (f_x_ * (x / z)) + c_x_; 
  double v = (f_y_ * (y / z)) + c_y_;

	// Return result
	//pixels = tmp.rowRange(0,3);
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

}  // namespace kalman_filter
}  // namespace perception_pipeline

