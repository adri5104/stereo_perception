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
#include <unordered_set>
#include <opencv2/opencv.hpp>
#include <omp.h>


#include "kalman_filter/worldpoint.hpp"
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

using namespace cv;
using namespace std;

namespace perception_pipeline
{
namespace kalman_filter
{

/// 6D image pixel value
typedef Vec<float, 7> OutVec;

/// 6D image channel number
const int OUT6D_VAL_IDX = 6;
const int OUT6D_C = 7;
const int OUT6D_TYPE = CV_32FC(OUT6D_C);

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


// Define an alias for the auxiliary struct to store the result of each wp
using WPResult = struct {
  bool keep;           // erase or not
  int pos_u, pos_v;    // final pos
  cv::Vec<float, 7> out_vec;  // x,y,z,vx,vy,vz, validity
  WorldPointErrorCode error;  // to assign color in debug image
};

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
   * @param sigma_system covariance matrix of the system model
   * @param C covariance matrix of the egomotion
   * @param T covariance matrix of the measurement model
   * @param min_depth minimum depth value of the depth image
   * @param max_depth maximum depth value of the depth image
   * @param min_height minimum height value considered for the worldpoints
   * @param max_height maximum height value considered for the worldpoints
   * @param include_ego_motion flag for using egomotion compensation
   * @param use_var_ego flag for using the covariance matrix of the egomotion
   * @param gridSize grid size for the worldpoints in pixels
   * @param debug_image_grid flag for drawing grid lines on the debug image
   */
  KalmanCore(
    Mat sigma_system, 
    Mat C, 
    Mat T, 
    double min_depth,  double max_depth, 
    double min_height, double max_height,
    bool include_ego_motion, 
    bool use_var_ego,
    int gridSize,
    bool debug_image_grid
  );
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
   * @param time_diff The time difference between frames.
   * @return KalmanCoreErrorCode Error code.
   */
  KalmanCoreErrorCode updateSyncedData(
    const Mat& optical_flow, 
    const Mat& depth, 
    const Mat& color_image, 
    const Mat& egomotion, 
    double time_diff);
  
  /**
   * @brief Update step using synchronized optical flow and depth data.
   * 
   * @param optical_flow The optical flow image (CV_32FC2 or similar).
   * @param depth The depth image.
   * @param color_image The color image.
   * @param time_diff The time difference between frames.
   * @return KalmanCoreErrorCode Error code.
   */
  KalmanCoreErrorCode updateSyncedData(
    const Mat& optical_flow,
    const Mat& depth, 
    const Mat& color_image, 
    double time_diff);
  

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
   * @param output_6d output 6D image with (x,y,z,vx,vy,vz,valid) format
   * @param output_debug_image output debug image
   * @return KalmanCoreErrorCode 
   */
  KalmanCoreErrorCode getOutput(cv::Mat &output_6d, cv::Mat &output_debug_image);

  /**
   * @brief getter for the current timedifference between frames
   * 
   * @return double time difference 
   */
  double getDeltaTime();

  void setIncludeEgoMotion(bool include_ego_motion);
  void setMinDepth(double min_depth);
  void setMaxDepth(double max_depth);
  void setMinHeight(double min_height);
  void setMaxHeight(double max_height);
  void setC(Mat C);
  void setT(Mat T);
  void setSigmaSystem(Mat sigma_system);
  

private:

  /**
   * @brief Predict the next state of the Kalman Filter.
   */
  KalmanCoreErrorCode predict(
    Mat input_optical_flow, 
    Mat input_depth, 
    Mat input_color_image,
    double time_diff);

  /**
   * @brief Set and initialize the WorldPoints according to the grid defined by the user.
   * 
   * @param occupancy_grid The occupancy grid.
   * @param output_6d The output 6D matrix.
   * @param output_debug_image The output debug image.
   */
  KalmanCoreErrorCode setNewWorldPoints(Mat &occupancy_grid, Mat &output_6d, Mat &output_debug_image);


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

  /**
   * @brief Returns the 6D state vector of a WorldPoint in the format (x, y, z, vx, vy, vz, validity).
   *    
   * @param x  6D state vector
   * @param validity 1 if the pixel contains a valid WorldPoint and 0 otherwise
   * @return Vec7f 
   */
  OutVec formatOutput(Vec6f x, int validity);

  std::vector<unique_ptr<WorldPoint>> worldpoints_; ///< Current WorldPoints

  Mat input_optical_flow_sync_;    ///< Input optical flow image
  Mat input_depth_sync_;           ///< Input depth image
  Mat input_color_image_sync_;     ///< Input color image
  Mat input_egomotion_sync_;       ///< Input egomotion matrix

  double time_diff_;              ///< Time difference between frames

  Mat output_6d_;          ///< 6D output image
  Mat output_debug_image_; ///< output Debug image

  bool include_ego_motion_;	  ///< Flag for incorporating the egomotion 
  bool use_ego_var_;           ///< Flag for using the covariance matrix of the egomotion
  int grid_size_worldpoints_;	///< Size of the grid cells in pixels
  bool debug_image_grid_;     ///< Flag for drawing grid lines on the debug image

  double fx_;                  ///< Focal length x
  double fy_;                  ///< Focal length y
  double cx_;                  ///< Principal point x
  double cy_;                  ///< Principal point y
  bool camera_parameters_set_; ///< Flag for camera parameters set
  double min_depth_;           ///< Minimum depth value in meters
  double max_depth_;           ///< Maximum depth value in meters
  double min_height_;          ///< Minimum height value in meters
  double max_height_;          ///< Maximum height value in meters

  bool first_time_;	 ///< Flag for first time execution
  Mat C_;            ///< Covariance matrix of the egomotion  
  Mat T_ ;           ///< Covariance matrix of the measurement model
  Mat sigma_system_; ///< Covariance matrix of the system model
  Mat	A_new_;		     ///< Transition matrix
  Mat	u_new_;		     ///< Egomotion translation vector
  Mat	D_new_;		     ///< Matrix containing the rotation matrix of the egomotion
  Mat	Q_new_w_;	     ///< Covariance matrix of discrete-time process
  Mat	para_rot_;     ///< Parametervector with some precomputed values (sin/cos) for rotation of egomotion 
  double term1;		   ///< sPsi*sTheta - cPsi*cTheta*sPhi;
  double term2;		   ///< cPsi*cTheta - sPhi*sPsi*sTheta;
  double term3;		   ///< cTheta*sPsi + cPsi*sPhi*sTheta;
  double term4;		   ///< cPsi*sTheta + cTheta*sPhi*sPsi;

};

} // namespace kalman_filter
} // namespace perception_pipeline

#endif  // KALMAN_FILTER__KALMAN_CORE_HPP_
