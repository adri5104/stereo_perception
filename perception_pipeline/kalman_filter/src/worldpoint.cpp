#include "kalman_filter/worldpoint.hpp"

namespace perception_pipeline {
namespace kalman_filter {

WorldPoint::WorldPoint(Mat &projectionMatrix, Mat &projectionMatrixInv, Mat &C, Mat &T, bool &useVarEgo, int gridSize)
: projection_matrix_(projectionMatrix), 
  projection_matrix_inv_(projectionMatrixInv),
  cov_system(C), 
  cov_measurement(T), 
  use_ego_motion_(useVarEgo), 
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
  x_old_ = Mat::zeros(6, 1, CV_64FC1);	
  Mat tmp = Mat::zeros(4, 1, CV_64FC1);
  projectPixelTo3D(u, v, d, tmp);
  Mat tmp2 = x_old_.rowRange(0,3);
  tmp.rowRange(0,3).copyTo(tmp2);

  // Initialize variances with 10 [m^2 respectively m^2/s^2]
  p_old_ = Mat::eye(6, 6, CV_64FC1) * 10;
}

int WorldPoint::computeKalmanStep(const Mat& inputDisp,													///< Method that runs a complete Kalmanstep with all necessary computations. Returns integer depending on the condition of the state vector
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
  // Increment age
  age_++;

  
  return age_;
}

void WorldPoint::projectPixelTo3D(const double &u, const double &v, const double &d, Mat &coordinates)
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

void WorldPoint::project3DToPixel(const double &x, const double &y, const double &z, Mat &pixels)
{
  // Initialize vector for projection
	Mat tmp = Mat::zeros(4, 1, CV_64FC1);
	tmp.at<double>(0,0) = x;   
	tmp.at<double>(1,0) = y;
	tmp.at<double>(2,0) = z;
	tmp.at<double>(3,0) = 1;

	// Project to pixel values
	tmp = 1/z * projection_matrix_ * tmp;

	// Return result
	pixels = tmp.rowRange(0,3);
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

