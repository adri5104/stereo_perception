#ifndef KALMAN_FILTER_WORLDPOINT_HPP
#define KALMAN_FILTER_WORLDPOINT_HPP

#include <opencv2/opencv.hpp>
namespace perception_pipeline {
namespace kalman_filter {

using namespace std;
using namespace cv;

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
    int	 computeKalmanStep(const Mat& inputDisp,													///< Method that runs a complete Kalmanstep with all necessary computations. Returns integer depending on the condition of the state vector
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
    Mat projection_matrix_; // Projection matrix of the camera 
    Mat projection_matrix_inv_; // Inverse of the projection matrix
    Mat c_; // System covariance matrix
    Mat t_; // Measurement model covariance matrix
    bool use_ego_motion_; // Use ego motion
    double grid_size_worldpoints;	///< Width of a square of pixels, which is initialized with one WorldPoint
    int age_; // Number of iterations the object has passed

    // Kalman filter
    Mat	z_old_;	// Old measurement vector
	  Mat	x_old_;	// Old state vector
	  Mat	p_old_;	// Posterior estimate covariance matrix of previous timestep	

    // Camera parameters
    double f_x_; // Focal length x
    double f_y_; // Focal length y
    double c_x_; // Principal point x
    double c_y_; // Principal point y

};

} // namespace kalman_filter
}  // namespace perception_pipeline

#endif  // KALMAN_FILTER_WORLDPOINT_HPP