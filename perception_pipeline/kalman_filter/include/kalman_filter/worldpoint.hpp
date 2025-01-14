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

#ifndef KALMAN_FILTER_WORLDPOINT_HPP
#define KALMAN_FILTER_WORLDPOINT_HPP

#include <opencv2/opencv.hpp>
namespace perception_pipeline {
namespace kalman_filter {

using namespace std;
using namespace cv;


enum class WorldPointErrorCode {
    OK = 1,
    BAD_DEPTH_VALUE_ERROR,
    NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR,
    THREE_SIGMA_TEST_FAILED,
};

// Helper function to get error messages
inline  std::string getErrorMessage(WorldPointErrorCode code) {
    static const std::unordered_map<WorldPointErrorCode, std::string> errorMessages = {
        {WorldPointErrorCode::OK, "No error."},
        {WorldPointErrorCode::BAD_DEPTH_VALUE_ERROR, "Invalid depth value."},
        {WorldPointErrorCode::NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR, "New measurement dimensions out of range."}
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
     * @brief Constructor for the WorldPoint class.
     *        Initializes internal state, matrices, etc.
     * 
     * @param projectionMatrix The projection matrix of the camera.
     * @param projectionMatrixInv The inverse of the projection matrix.
     * @param C The system covariance matrix.
     * @param T The measurement model covariance matrix.
     * 
     */
    WorldPoint(Mat &projectionMatrix, Mat &projectionMatrixInv, Mat &C, Mat &T, bool &useVarEgo, int gridSize);

    /**
     * @brief Destructor for the WorldPoint class.
     */
    ~WorldPoint(void);
    
    // Methods

    /**
     * @brief Initialize the Kalman filter with the initial state.
     * 
     * @param u The horizontal position in pixels.
     * @param v The initial y position.
     * @param depth The initial depth.
     */
    void initKalmanFilter(const double &u, const double &v, const double &depth);

    /**
     * @brief Method that runs a complete Kalmanstep with all necessary computations.
     * 
     * @param input_depth input depth image in mm
     * @param input_flow input optical flow image in pixels
     * @param A_new input state transition matrix
     * @param D_new 
     * @param Q_new_w 
     * @param G_new 
     * @param u_new 
     * @param para_rot 
     * @param term1 
     * @param term2 
     * @param term3 
     * @param term4 
     * @param occupancy_grid 
     * @param delta_time 
     * @return WorldPointErrorCode 
     */
    WorldPointErrorCode	 computeKalmanStep(
              const Mat& input_depth,													///< Method that runs a complete Kalmanstep with all necessary computations. Returns integer depending on the condition of the state vector
              const Mat& ,
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
              const double& delta_time);


    /**
     * @brief Function for computing world coordinates out of pixel values (3D reprojection).
     * 
     * @param u The horizontal position in pixels.
     * @param v The vertical position in pixels.
     * @param depth Depth value in m
     * @param coordinates Coordinates of the world point.
     */
    void projectPixelToWorld(const double &u, const double &v, const uint16_t &depth, Mat &coordinates);	

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

  private:

    // Attributes
    Mat	z_old_;	// Old measurement vector			
	  Mat	x_old_;	// Old state vector
	  Mat	P_old_;	// Old state covariance matrix				
	  
	  Mat	C_;	 // Reference to the covariance matrix of the egomotion
	  Mat	T_;	 // Reference to the covariance matrix of the measurement model
	  bool use_var_ego_;	// Flag for using the covariance matrix of the egomotion
	  double grid_size_worldpoints;	// Size of the grid for the worldpoints
    int	age_; // Number of iterations the object has already been passed through

    // Camera parameters
    double f_x_; // Focal length x
    double f_y_; // Focal length y
    double c_x_; // Principal point x
    double c_y_; // Principal point y

    WorldPointErrorCode errorCode; // Error code for the Kalman filter
 
};

} // namespace kalman_filter
}  // namespace perception_pipeline

#endif  // KALMAN_FILTER_WORLDPOINT_HPP