  #ifndef KALMAN_FILTER__KALMAN_CORE_HPP_
#define KALMAN_FILTER__KALMAN_CORE_HPP_

#include <opencv2/opencv.hpp>
#include "kalman_filter/worldpoint.hpp"
#include <vector>
#include <string>

using namespace cv;

namespace perception_pipeline
{
namespace kalman_filter
{

/**
 * @class KalmanCore
 * @brief This class implements the pure logic of a Kalman Filter, 
 *        with no dependency on ROS 2.
 */
class KalmanCore
{
public:

  /**
   * @brief Default constructor for the KalmanCore class.
   *        Initializes internal state, matrices, etc.
   */
  KalmanCore();

  /**
   * @brief Constructor for the KalmanCore class.
   *        Initializes internal state, matrices, etc.
   * 
   * @param sigma_system The system covariance matrix.
   * @param C The system covariance matrix.
   * @param T The measurement model covariance matrix.
   * @param fx Focal length x
   * @param fy Focal length y
   * @param cx Principal point x
   * @param cy Principal point y
   * @param useVarEgo Flag for using the covariance matrix of the egomotion
   * @param gridSize Size of the grid for the worldpoints
   */
  KalmanCore(Mat sigma_system, Mat C, Mat T, double fx, double fy, double cx, double cy, bool useVarEgo, int gridSize);

  /**
   * @brief Default destructor for the KalmanCore class.
   */
  KalmanCore(const KalmanCore&) = delete;

  /**
   * @brief Default copy constructor for the KalmanCore class.
   */
  KalmanCore& operator=(const KalmanCore&) = delete;

  /**
   * @brief Default move constructor for the KalmanCore class.
   */
  KalmanCore(KalmanCore&&) = delete;

  /**
   * @brief Default move assignment operator for the KalmanCore class.
   */
  KalmanCore& operator=(KalmanCore&&) = delete;

  /**
   * @brief Default destructor for the KalmanCore class.
   */
  ~KalmanCore(void);



  
  /**
   * @brief Update step using optical flow data.
   *        
   * @param flow The optical flow image (CV_32FC2 or similar).
   */
  void updateOpticalFlow(const cv::Mat& flow);

  /**
   * @brief Update step using disparity data.
   *       
   * @param disparity The disparity image.
   */
  void updateDepth(const cv::Mat& disparity);

  /**
   * @brief Get the current state vector.
   * @return cv::Mat The internal state of the Kalman Filter.
   */
  cv::Mat getState() const;

  // Setters and getters

  /**
   * @brief Set the Camera Parameters object
   * 
   * @param fx Focal length x
   * @param fy Focal length y
   * @param cx Principal point x
   * @param cy Principal point y
   */
  void setCameraParameters(double fx, double fy, double cx, double cy);

private:
  
  // Vector containing references to tracked WorldPoints
  std::vector<WorldPoint*> worldpoints_;

  // Config parameters
  bool use_var_ego_;	// Flag for incorporating the egomotion covariance matrix
  int grid_size_worldpoints;	// Size of the grid cells in pixels

  // Camera parameters
  double fx_;
  double fy_;
  double cx_;
  double cy_;

  // Kalman filter parameters 
  bool first_time_;	// Flag for first time execution  
  Mat C_; // Covariance matrix of the egomotion
  Mat T_ ;// Covariance matrix of the measurement model
  Mat sigma_system_;
  Mat	A_new_;		// Transition matrix
  Mat	u_new_;		// Egomotion translation vector
  Mat	D_new_;		// Matrix containing the rotation matrix of the egomotion
  Mat	Q_new_w_;	// Covariance matrix of discrete-time process
  Mat	G_new_;		// Jacobian for state transformation
  Mat	para_rot_;	// Parametervector with some precomputed values (sin/cos) for rotation of egomotion 
  double delta_time;	// Timedifference between the current frame and the frame before
  double term1;		// Value needed for computation of Jacobimatrices
  double term2;		// Value needed for computation of Jacobimatrices
  double term3;		// Value needed for computation of Jacobimatrices
  double term4;		// Value needed for computation of Jacobimatrices

};

} // namespace kalman_filter
} // namespace perception_pipeline

#endif  // KALMAN_FILTER__KALMAN_CORE_HPP_
