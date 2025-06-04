/**
 * @file kalman_core.cpp
 * @author adrian.rieker@tum.de
 * @brief Class that implements a Kalman filter for 6D state estimation of a world point.
 * 
 */

#ifndef KALMAN_FILTER_WORLDPOINT_HPP
#define KALMAN_FILTER_WORLDPOINT_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <kalman_filter/utils.hpp>

namespace stereo_perception {
namespace perception_pipeline {
namespace kalman_filter {

using namespace std;
using namespace cv;


enum class WorldPointErrorCode {
    OK = 1,
    BAD_DEPTH_VALUE_ERROR,
    MEASUREMENT_OUT_OF_HEIGHT_BOUNDS_ERROR,
    NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR,
    THREE_SIGMA_TEST_FAILED,
    DIVISION_BY_ZERO_ERROR,
    UNABLE_TO_GET_OPT_FLW_MSMT,
    UNKNOWN_ERROR,
};

// Helper function to get error messages
inline  std::string getErrorMessageWorldpoint(WorldPointErrorCode code) {
    static const std::unordered_map<WorldPointErrorCode, std::string> errorMessages = {
        {WorldPointErrorCode::OK, "No error."},
        {WorldPointErrorCode::BAD_DEPTH_VALUE_ERROR, "Invalid depth value."},
        {WorldPointErrorCode::MEASUREMENT_OUT_OF_HEIGHT_BOUNDS_ERROR, "New measurement height out of range."},
        {WorldPointErrorCode::NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR, "New measurement dimensions out of range."},
        {WorldPointErrorCode::THREE_SIGMA_TEST_FAILED, "Three sigma test failed."},
        {WorldPointErrorCode::DIVISION_BY_ZERO_ERROR, "Division by zero."},
        {WorldPointErrorCode::UNABLE_TO_GET_OPT_FLW_MSMT, "Problem getting optical flow measurement"},
        {WorldPointErrorCode::UNKNOWN_ERROR, "Unknown error."}
    };

    auto it = errorMessages.find(code);
    return it != errorMessages.end() ? it->second : "Invalid error code.";
}

/**
 * @class WorldPoint
 * @brief Represents a point in of the scene with its own local state (6D state and covariance).
 *        Provides methods for reprojection between pixel/depth coordinates and world coordinates.
 *        
 *       The state vector is: [x, y, z, vx, vy, vz]
 */
class WorldPoint
{
  public:
   
    /**
     * @brief Construct a new World Point object
     * 
     * @param C Covariance matrix of the egomotion.
     * @param T Covariance matrix of the measurement model.
     * @param min_depth Minimum depth value in meters.
     * @param max_depth Maximum depth value in meters.
     * @param min_height Minimum height value in meters.
     * @param max_height Maximum height value in meters.
     * @param ego_compensation_factor Factor for egomotion compensation.
     * @param fx Focal length x.
     * @param fy Focal length y.
     * @param cx Principal point x.
     * @param cy Principal point y.
     * @param includeEgoMotion Flag for using egomotion compensation.
     * @param useVarEgo Use the covariance matrix of the egomotion.
     * @param gridSize Grid size in pixels
     */
    WorldPoint(
      Mat &C, Mat &T, 
      double min_depth, double max_depth,
      double min_height, double max_height,
      double ego_compensation_factor,
      double fx, double fy, double cx, double cy, 
      bool &includeEgoMotion, bool &useVarEgo,
      int gridSize);

    /**
     * @brief Destructor for the WorldPoint class.
     */
    ~WorldPoint(void);
    
    // Methods

    /**
     * @brief Initialize the Kalman filter with the initial state. Also increases value in the occupancy grid.
     * 
     * @param u The horizontal position in pixels.
     * @param v The initial y position.
     * @param depth The initial depth in meters.
     * @param occupancyGrid The occupancy grid.
     */
    WorldPointErrorCode initKalmanFilter(const double &u, const double &v, const double &depth, Mat &occupancyGrid);

    /**
     * @brief Method that runs a complete Kalmanstep with all necessary computations.
     * 
     * @param input_depth input depth image in mm
     * @param input_flow input optical flow image in pixels
     * @param A_new input state transition matrix in camera coordinates
     * @param D_new input system rotation matrix
     * @param Q_new_w input covariance matrix of discrete-time process in world coordinates
     * @param u_new input egomotion translation vector
     * @param rot_new struct containing the egomotion rotation data (rvec, R, dRdr)
     * @param occupancy_grid The occupancy grid.
     * @return WorldPointErrorCode 
     */
    WorldPointErrorCode	 computeKalmanStep(
              const Mat& input_depth,													///< Method that runs a complete Kalmanstep with all necessary computations. Returns integer depending on the condition of the state vector
              const Mat& input_flow,
              const Mat& A_new,
              const Mat& D_new,
              const Mat& Q_new_w,
              const Mat& u_new,
              const EgoMotionRotationData& rot_new,
              Mat& occupancy_grid);
    /**
     * @brief Function for computing world coordinates out of pixel values (3D reprojection).
     * 
     * @param u The horizontal position in pixels.
     * @param v The vertical position in pixels.
     * @param depth Depth value in m
     * @param coordinates Coordinates of the world point.
     */
    void projectPixelToWorld(const double &u, const double &v, const double &depth, Mat &coordinates);	

    /**
     * @brief Function for computing pixel values out of world coordinates (2D projection).
     * 
     * @param x The x position in the world.
     * @param y The y position in the world.
     * @param z The z position in the world (depth).
     * @param pixels Pixel values.
     */
    void projectWorldToPixel(const double &x, const double &y, const double &z, Mat &pixels);	

    // Getters and setters

    /**
     * @brief Get the current state vector.
     * 
     * @param x state vector.
     */
    void getX(Mat &x);	

    /**
     * @brief Get the current measurement vector.
     * 
     * @param z measurement vector.
     */
	  void getZ(Mat &z);	

    /**
     * @brief Get the current iteration.
     * 
     * @param p state covariance matrix.
     */
	  int	 getAge();	

    /**
     * @brief Set the camera parameters.
     * 
     * @param fx Focal length x
     * @param fy Focal length y
     * @param cx Principal point x
     * @param cy Principal point y
     */
    void setCameraParameters(double fx, double fy, double cx, double cy);

    // Setters
    void setMaxDepth(double max_depth) { max_depth_ = max_depth; }
    void setMinDepth(double min_depth) { min_depth_ = min_depth; }
    void setMaxHeight(double max_height) { max_height_ = max_height; }
    void setMinHeight(double min_height) { min_height_ = min_height; }
    void setUseVarEgo(bool use_var_ego) { use_var_ego_ = use_var_ego; }
    void setC(Mat C) { C_ = C; }
    void setT(Mat T) { T_ = T; }

  private:

    // Attributes
    double max_depth_;
    double min_depth_ ;
    double min_height_;
    double max_height_;
    double ego_compensation_factor_; ///< Factor for egomotion compensation

    Mat	z_old_;	///< Old measurement vector			
	  Mat	x_old_;	///< Old state vector
	  Mat	P_old_;	///< Old state covariance matrix				
	  
	  Mat	C_;	 ///< Reference to the covariance matrix of the egomotion
	  Mat	T_;	 ///< Reference to the covariance matrix of the measurement model
    bool include_ego_motion_;	///< Flag for incorporating the egomotion
	  bool use_var_ego_;	///< Flag for using the covariance matrix of the egomotion
	  double grid_size_worldpoints;	///< Size of the grid for the worldpoints
    int	age_; ///< Number of iterations the object has already been passed through

    // Kalman filter matrices
    Mat x_new_pred_; ///< Prior state vector
    Mat x_new_; ///< Posterior state vector
    Mat z_new_pred_; ///< Prior measurement vector
    Mat z_new_; ///< Posterior measurement vector
    Mat s_new_; ///< Innovation vector
    Mat Q_new_; ///< Process noise covariance matrix
    Mat P_new_pred_; ///< Prior state covariance matrix
    Mat P_new_; ///< Posterior state covariance matrix
    Mat K_new_; ///< Kalman gain matrix
    Mat J_new_; ///< Jacobian matrix of the egomotion
    Mat H_new_; ///< Jacobian matrix of the measurement model
    Mat S_new_; ///< Innovation covariance matrix
    Mat S_new_inv_; ///< Inverse of the innovation covariance matrix

    // Camera parameters
    double f_x_; ///< Focal length x
    double f_y_; ///< Focal length y
    double c_x_; ///< Principal point x
    double c_y_; ///< Principal point y

    WorldPointErrorCode errorCode; // Error code for the Kalman filter
 
};


} // namespace kalman_filter
}  // namespace perception_pipeline
} // namespace stereo_perception

#endif  // KALMAN_FILTER_WORLDPOINT_HPP