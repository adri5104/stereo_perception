#ifndef KALMAN_FILTER_WORLDPOINT_HPP
#define KALMAN_FILTER_WORLDPOINT_HPP

#include <opencv2/opencv.hpp>
namespace perception_pipeline {
namespace kalman_filter {

using namespace std;
using namespace cv;


enum class KalmanErrorCode {
    OK = 1,
    BAD_DISPARITY_ERROR,
    NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR,
    THREE_SIGMA_TEST_FAILED,
};

// Helper function to get error messages
std::string getErrorMessage(KalmanErrorCode code) {
    static const std::unordered_map<KalmanErrorCode, std::string> errorMessages = {
        {KalmanErrorCode::OK, "No error."},
        {KalmanErrorCode::BAD_DISPARITY_ERROR, "Invalid disparity value."},
        {KalmanErrorCode::NEW_MEASUREMENT_OUT_OF_BOUNDS_ERROR, "New measurement dimensions out of range."},
    };

    auto it = errorMessages.find(code);
    return it != errorMessages.end() ? it->second : "Invalid error code.";
}

/**
 * @class WorldPoint
 * @brief Represents a point in of the scene with its own local state (6D state and covariance).
 *        Provides methods for reprojection between pixel/disparity coordinates and world coordinates.
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
     * @param d The initial disparity.
     */
    void initKalmanFilter(const double &u, const double &v, const double &d);

    /**
     * @brief Method that runs a complete Kalmanstep with all necessary computations.
     * 
     * @param inputDisp disparity image
     * @param inputFlow optical flow image
     * @param A_new 
     * @param D_new 
     * @param Q_new_w 
     * @param G_new 
     * @param u_new 
     * @param paraRot 
     * @param term1 
     * @param term2 
     * @param term3 
     * @param term4 
     * @param occupancyGrid 
     * @param timediff 
     * @return int 
     */
    KalmanErrorCode	 computeKalmanStep(const Mat& inputDisp,													///< Method that runs a complete Kalmanstep with all necessary computations. Returns integer depending on the condition of the state vector
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
						   const double& timediff);

    /**
     * @brief Function for computing world coordinates out of pixel values (3D reprojection).
     * 
     * @param u The horizontal position in pixels.
     * @param v The vertical position in pixels.
     * @param d Disparity value.
     * @param coordinates Coordinates of the world point.
     */
    void projectPixelTo3D(const double &u, const double &v, const double &d, Mat &coordinates);

    /**
     * @brief Function for computing world coordinates out of pixel values (3D reprojection).
     * 
     * @param u The horizontal position in pixels.
     * @param v The vertical position in pixels.
     * @param depth Depth value in mm
     * @param coordinates Coordinates of the world point.
     */
    void projectPixelTo3D(const double &u, const double &v, const uint16_t &depth, Mat &coordinates);	

    /**
     * @brief Function for computing pixel values out of world coordinates (2D projection).
     * 
     * @param x The x position in the world.
     * @param y The y position in the world.
     * @param z The z position in the world.
     * @param pixels Pixel values.
     */
    void project3DToPixel(const double &x, const double &y, const double &z, Mat &pixels);	

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

  private:

    // Attributes
    double u_; // Horizontal position in pixels
    double v_; // Vertical position in pixels
    
	  Mat	z_old_;	// Old measurement vector			
	  Mat	x_old_;	// Old state vector
	  Mat	P_old_;	// Old state covariance matrix				
	  
	  Mat	C_;	 // Reference to the covariance matrix of the egomotion
	  Mat	T_;	 // Reference to the covariance matrix of the measurement model
	  bool use_var_ego_;	// Flag for using the covariance matrix of the egomotion
	  double grid_size_worldpoints;	// Size of the grid for the worldpoints
    int	age_; // Number of iterations the object has already been passed through

    // Camera parameters
    Mat	projection_matrix_;		// Variable for holding the reference to the projection matrix
	  Mat	projection_matrix_inv_;	// Variable for holding the reference to the inverse projection matrix
    double f_x_; // Focal length x
    double f_y_; // Focal length y
    double c_x_; // Principal point x
    double c_y_; // Principal point y

    KalmanErrorCode errorCode; // Error code for the Kalman filter
 
};

} // namespace kalman_filter
}  // namespace perception_pipeline

#endif  // KALMAN_FILTER_WORLDPOINT_HPP