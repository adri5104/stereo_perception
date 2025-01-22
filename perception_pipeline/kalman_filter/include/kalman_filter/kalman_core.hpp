/**
 * @file kalman_core.hpp
 * @author adrian.rieker@tum.de
 * @brief 
 * @version 
 * @date 
 * 
 * 
 * 
 */

#ifndef KALMAN_FILTER__KALMAN_CORE_HPP_
#define KALMAN_FILTER__KALMAN_CORE_HPP_

#include <vector>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "kalman_filter/worldpoint.hpp"

using namespace cv;
using namespace std;

namespace perception_pipeline
{
namespace kalman_filter
{

enum class KalmanCoreErrorCode {
    OK = 1,
    BAD_DEPTH_IMAGE_ERROR,
    BAD_OPTICAL_FLOW_IMAGE_ERROR,
    BAD_COLOR_IMAGE_ERROR,
    BAD_EGOMOTION_ERROR,
    SET_NEW_WP_DEPTH_EMPTY_ERROR,
    SET_NEW_WP_OPTICAL_FLOW_EMPTY_ERROR,
    SET_NEW_WP_COLOR_IMAGE_EMPTY_ERROR,
    SET_NEW_WP_WRONG_MAT_SIZE_ERROR,
    CAMERA_PARAMETERS_NOT_SET_ERROR
};

// Helper function to get error messages
inline  std::string getErrorMessage(KalmanCoreErrorCode code) {
    static const std::unordered_map<KalmanCoreErrorCode, std::string> errorMessages = {
        {KalmanCoreErrorCode::OK, 
        "No error."},
        {KalmanCoreErrorCode::BAD_DEPTH_IMAGE_ERROR,
         "[KalmanCore::updateSyncedData] Depth image is empty!"},
        {KalmanCoreErrorCode::BAD_OPTICAL_FLOW_IMAGE_ERROR,
         "[KalmanCore::updateSyncedData] Optical flow image is empty!"},
        {KalmanCoreErrorCode::BAD_COLOR_IMAGE_ERROR,
         "[KalmanCore::updateSyncedData] Color image is empty!"},
        {KalmanCoreErrorCode::BAD_EGOMOTION_ERROR,
          "[KalmanCore::updateSyncedData] Egomotion is empty!"},
        {KalmanCoreErrorCode::SET_NEW_WP_DEPTH_EMPTY_ERROR,
         "[KalmanCore::setNewWorldPoints] Error: input_depth_sync_ empty"},
        {KalmanCoreErrorCode::SET_NEW_WP_OPTICAL_FLOW_EMPTY_ERROR,
         "[KalmanCore::setNewWorldPoints] Error: input_ input_optical_flow_sync_ empty"},
        {KalmanCoreErrorCode::SET_NEW_WP_COLOR_IMAGE_EMPTY_ERROR,
         "[KalmanCore::setNewWorldPoints] Error: input_color_image_sync_ empty"},
        {KalmanCoreErrorCode::SET_NEW_WP_WRONG_MAT_SIZE_ERROR,
          "[KalmanCore::setNewWorldPoints] Error:  matrices have different sizes."},
        {KalmanCoreErrorCode::CAMERA_PARAMETERS_NOT_SET_ERROR,
          "[KalmanCore::setCameraParameters] Camera parameters not set."}
    };

    auto it = errorMessages.find(code);
    return it != errorMessages.end() ? it->second : "Invalid error code.";
}

// Define type aliases for convenience
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Seconds = std::chrono::duration<double>;

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
   * @brief Construct a new Kalman Core object
   * 
   * @param sigma_system 
   * @param C 
   * @param T 
   * @param min_depth 
   * @param max_depth 
   * @param useVarEgo 
   * @param gridSize 
   */
  KalmanCore(Mat sigma_system, Mat C, Mat T, double min_depth, double max_depth, bool useVarEgo, int gridSize);

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
   * @brief Update step using synchronized optical flow and depth data.
   * 
   * @param optical_flow The optical flow image (CV_32FC2 or similar).
   * @param depth The depth image.
   * @param color_image The color image.
   * @param egomotion The egomotion matrix.
   * @return KalmanCoreErrorCode Error code.
   */
  KalmanCoreErrorCode updateSyncedData(
    const Mat& optical_flow, const Mat& depth, const Mat& color_image, const Mat& egomotion);
  
  /**
   * @brief Update step using synchronized optical flow and depth data.
   * 
   * @param optical_flow The optical flow image (CV_32FC2 or similar).
   * @param depth The depth image.
   * @param color_image The color image.
   * @return KalmanCoreErrorCode Error code.
   */
  KalmanCoreErrorCode updateSyncedData(
    const Mat& optical_flow, const Mat& depth, const Mat& color_image);
  

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

  /**
   * @brief Get the Output object
   * 
   * @param output_6d output 6D matrix
   * @param output_debug_image output debug image
   * @return KalmanCoreErrorCode 
   */
  KalmanCoreErrorCode getOutput(cv::Mat &output_6d, cv::Mat &output_6d_val, cv::Mat &output_debug_image);

  /**
   * @brief getter for the current timedifference between frames
   * 
   * @return double time difference 
   */
  double getDeltaTime();

  

private:

  /**
   * @brief Predict the next state of the Kalman Filter.
   */
  KalmanCoreErrorCode predict(Mat input_optical_flow, Mat input_depth, Mat input_color_image);

  /**
   * @brief Set and initialize the WorldPoints according to the grid defined by the user.
   * 
   * @param occupancy_grid The occupancy grid.
   * @param output_6d The output 6D matrix.
   * @param output_debug_image The output debug image.
   */
  KalmanCoreErrorCode setNewWorldPoints(Mat &occupancy_grid, Mat &output_6d, Mat &output_debug_image, Mat &output_6d_val);


  /**
   * @brief Compute the Kalman matrices.
   */
  KalmanCoreErrorCode computeKalmanMatrices();

  /**
   * @brief Static method to calculate and update the time difference given a reference to lastTime.
   *        Automatically updates the lastTime reference.
   * 
   * @param lastTime TimePoint reference to the last time.
   * @return elapsed time in seconds. 
   */
  static double calculateTimeDifference(TimePoint& lastTime);

  void computeJacobianMatrix();
  
  // Vector containing references to tracked WorldPoints
  std::vector<unique_ptr<WorldPoint>> worldpoints_;

  // Input attributes
  Mat input_optical_flow_sync_;
  Mat input_depth_sync_;
  Mat input_color_image_sync_;
  Mat input_egomotion_sync_;

  // Time attributes
  TimePoint sync_input_time_old_;
  double time_diff_;

  // Output attributes
  Mat output_6d_; // Output matrix with 
  Mat output_6d_val_; //
  Mat output_debug_image_; // Debug image



  // Config parameters
  bool include_ego_motion_;	// Flag for incorporating the egomotion covariance matrix
  int grid_size_worldpoints_;	// Size of the grid cells in pixels

  // Camera parameters
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  bool camera_parameters_set_;
  double min_depth_;
  double max_depth_;

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
  bool currently_processing_;	// Flag for indicating that the KalmanCore is currently processing data

};

} // namespace kalman_filter
} // namespace perception_pipeline

#endif  // KALMAN_FILTER__KALMAN_CORE_HPP_
