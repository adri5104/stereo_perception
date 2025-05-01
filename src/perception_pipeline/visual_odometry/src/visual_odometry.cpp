#include "visual_odometry/visual_odometry.hpp"
#include <iostream>

namespace perception_pipeline
{
namespace visual_odometry
{
  VisualOdometry::VisualOdometry(double min_depth, double max_depth, 
    bool create_debug_image, bool apply_statistical_filtering, 
    bool apply_expotential_smoothing, double exponential_alpha):
                                  camera_info_arrived_(false),
                                  create_debug_image_(create_debug_image),
                                  min_depth_(min_depth), 
                                  max_depth_(max_depth),
                                  covariance_(cv::Mat::zeros(6, 6, CV_64F)),
                                  translation_(cv::Mat::zeros(3, 1, CV_64F)),
                                  rotation_(cv::Mat::eye(3, 3, CV_64F)),
                                  translation_prev(cv::Mat::zeros(3, 1, CV_64F)),
                                  rotation_prev(cv::Mat::eye(3, 3, CV_64F)),
                                  translation_x_window_(std::deque<double>(window_size_, 0.0)),
                                  translation_y_window_(std::deque<double>(window_size_, 0.0)),
                                  translation_z_window_(std::deque<double>(window_size_, 0.0)),
                                  roll_window_(std::deque<double>(window_size_, 0.0)),
                                  pitch_window_(std::deque<double>(window_size_, 0.0)),
                                  yaw_window_(std::deque<double>(window_size_, 0.0)),
                                  apply_expotential_smoothing_(apply_expotential_smoothing),
                                  exponential_alpha_(exponential_alpha),
                                  apply_statistical_filtering_(apply_statistical_filtering)
  {
    // Initialize the camera intrinsic matrix
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<  1.0, 0.0, 1.0,
                                                 0.0, 1.0, 1.0,
                                                 0.0, 0.0, 1.0);

    translation_ = cv::Mat::zeros(3, 1, CV_64F);
    rotation_ = cv::Mat::eye(3, 3, CV_64F);

    // Initialize CUDA-based feature detector and matcher
    feature_detector_gpu_ = cv::cuda::ORB::create(10000, 1.1f, 8, 31, 0, 2, 
                                       cv::ORB::HARRIS_SCORE, 31, 20);
    
    matcher_gpu_ = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
  }

  void VisualOdometry::updateSync(const cv::Mat& color_image, const cv::Mat& depth_image)
  {
    if (color_image.empty() || depth_image.empty())
        throw std::runtime_error("Input images must not be empty");

    if (!camera_info_arrived_)
        throw std::runtime_error("Camera parameters have not been set.");

    // Convert color_image to grayscale
    cv::Mat gray_image;
    if (color_image.type() == CV_8UC3)
    {
        cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
    }
    else if (color_image.type() == CV_8UC1)
    {
        gray_image = color_image;
    }
    else
    {
        throw std::runtime_error("color_image must be CV_8UC3 or CV_8UC1!");
    }

    // Convert depth_image to CV_32FC1
    cv::Mat float_depth_image;
    if (depth_image.type() == CV_16UC1)
    {
        depth_image.convertTo(float_depth_image, CV_32FC1, 1.0 / 1000.0); // Convert mm to meters
    }
    else if (depth_image.type() == CV_32FC1)
    {
        float_depth_image = depth_image;
    }
    else
    {
        throw std::runtime_error("depth_image must be CV_16UC1 or CV_32FC1!");
    }

    if (prev_color_image_gpu_.empty())
    {
        prev_color_image_gpu_.upload(gray_image);
        prev_depth_image_gpu_.upload(float_depth_image);
        
        return;
    }

    // Detect and match features using GPU
    std::vector<cv::Point2f> prev_points_features, curr_points_features;
    detectAndMatchFeatures(prev_color_image_gpu_, cv::cuda::GpuMat(gray_image), prev_points_features, curr_points_features);


    // Download depth image from GPU
    cv::Mat prev_depth_image;
    prev_depth_image_gpu_.download(prev_depth_image);

    // Estimate motion
    estimateMotion(prev_points_features, curr_points_features, prev_depth_image);

    // Update previous frames
    prev_color_image_gpu_.upload(gray_image);
    prev_depth_image_gpu_.upload(float_depth_image);

  }

void VisualOdometry::detectAndMatchFeatures(const cv::cuda::GpuMat& prev_image, const cv::cuda::GpuMat& curr_image,
                                            std::vector<cv::Point2f>& prev_points, std::vector<cv::Point2f>& curr_points)
{
    // Check input images
    if (prev_image.empty() || curr_image.empty())
        throw std::runtime_error("Input images for feature detection must not be empty.");

    // Initialize CUDA stream for asynchronous operations
    cv::cuda::Stream stream;

    // Allocate GPU matrices for descriptors
    cv::cuda::GpuMat keypoints_prev, keypoints_curr;
    cv::cuda::GpuMat descriptors_prev, descriptors_curr;

    // Detect features and compute descriptors for the previous image
    feature_detector_gpu_->detectAndComputeAsync(prev_image, cv::noArray(), keypoints_prev, descriptors_prev, false, stream);

    // Detect features and compute descriptors for the current image
    feature_detector_gpu_->detectAndComputeAsync(curr_image, cv::noArray(), keypoints_curr, descriptors_curr, false, stream);

    // Wait for all CUDA operations to complete
    stream.waitForCompletion();

    // Check if descriptors are empty
    if (descriptors_prev.empty() || descriptors_curr.empty())
        throw std::runtime_error("Descriptors are empty after detectAndComputeAsync.");

    // Match descriptors using GPU matcher
    std::vector<cv::DMatch> matches_host;
    matcher_gpu_->match(descriptors_prev, descriptors_curr, matches_host);

    // Check if matches are empty
    if (matches_host.empty())
    {
        std::cerr << "No matches found between frames." << std::endl;
        return;
    }

    // Allocate host keypoints
    std::vector<cv::KeyPoint> keypoints_prev_host, keypoints_curr_host;

    // Download keypoints from GPU to host
    feature_detector_gpu_->convert(keypoints_prev, keypoints_prev_host);
    feature_detector_gpu_->convert(keypoints_curr, keypoints_curr_host);

    // Extract matched points
    for (const auto& match : matches_host)
    {
        prev_points.emplace_back(keypoints_prev_host[match.queryIdx].pt);
        curr_points.emplace_back(keypoints_curr_host[match.trainIdx].pt);
    }  
}

void VisualOdometry::estimateMotion(const std::vector<cv::Point2f>& prev_points, 
  const std::vector<cv::Point2f>& curr_points, 
  const cv::Mat& prev_depth)
{
  std::vector<cv::Point3f> points3D;
  std::vector<cv::Point2f> valid_curr_points;

  // Debug image (optional)
  if (create_debug_image_)
  {
    debug_image_ = cv::Mat::zeros(prev_depth.size(), CV_8UC3);
  }

  // Reproject points to 3D, filtering by min/max depth
  for (size_t i = 0; i < prev_points.size(); ++i)
  {
    const cv::Point2f& pt2d = prev_points[i];

    // Safety check in case keypoints lie out of bounds
    if (pt2d.x < 0 || pt2d.x >= prev_depth.cols || pt2d.y < 0 || pt2d.y >= prev_depth.rows)
      continue;

    float depth = prev_depth.at<float>(static_cast<int>(pt2d.y), static_cast<int>(pt2d.x));
      if (depth > min_depth_ && depth < max_depth_)
      {
        // Valid depth: reproject to 3D
        cv::Point3f pt3d = reprojectTo3D(pt2d, depth);
        points3D.push_back(pt3d);
        valid_curr_points.push_back(curr_points[i]);

        // (Optional) Draw valid keypoints in green
        if (create_debug_image_)
        {
          cv::circle(debug_image_, curr_points[i], 2, cv::Scalar(0, 255, 0), -1);
        }
      }
      else
      {
        // (Optional) Draw invalid keypoints in red
        if (create_debug_image_)
        {
          cv::circle(debug_image_, curr_points[i], 2, cv::Scalar(0, 0, 255), -1);
        }
      }
  }

  // Early exit if not enough points to even attempt solvePnP
  if (points3D.size() < 6)
  {
    std::cerr << "[VisualOdometry.cpp] Not enough valid points for motion estimation.\n";
    return;
  }

  // (1) First Pass: Use solvePnPRansac to get initial pose + inlier mask
  cv::Mat rvec, tvec, inliers;
  bool success = cv::solvePnPRansac(
    points3D,          // 3D points
    valid_curr_points, // 2D points
    camera_matrix_,
    cv::noArray(),     // no distortion
    rvec,
    tvec,
    false,             // useExtrinsicGuess = false
    500,               // iterationsCount
    5.0,               // reprojectionError threshold (pixels)
    0.99,              // confidence
    inliers
  );

  if (!success || inliers.rows < 6)
  {
    // If solvePnPRansac fails or inliers are too few, skip updating pose
    std::cerr << "[VisualOdometry.cpp] solvePnPRansac failed or not enough inliers.\n";
    return;
  }

  // (2) Gather inliers for a second pass solvePnP
  std::vector<cv::Point3f> inlier_3D; 
  std::vector<cv::Point2f> inlier_2D;
  inlier_3D.reserve(inliers.rows);
  inlier_2D.reserve(inliers.rows);

  for (int i = 0; i < inliers.rows; ++i)
  {
    int idx = inliers.at<int>(i, 0);
    inlier_3D.push_back(points3D[idx]);
    inlier_2D.push_back(valid_curr_points[idx]);
  }

  // (3) Second Pass: Refine the pose with a standard solvePnP using only inliers
  //     Provide the previous rvec, tvec as the initial guess
  cv::solvePnP(
    inlier_3D, 
    inlier_2D, 
    camera_matrix_, 
    cv::noArray(), 
    rvec, 
    tvec, 
    true,                   // useExtrinsicGuess = true
    cv::SOLVEPNP_ITERATIVE // e.g. Levenberg–Marquardt
  );

  // Covariance Estimation
  cv::Mat residuals(inliers.rows, 1, CV_64F);
  for (int i = 0; i < inliers.rows; ++i)
  {
    int idx = inliers.at<int>(i, 0);
    std::vector<cv::Point3f> single_point{ points3D[idx] };
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(single_point, rvec, tvec, camera_matrix_, cv::noArray(), projected_points);
    cv::Point2f projected_point = projected_points[0];
    residuals.at<double>(i) = cv::norm(projected_point - valid_curr_points[idx]);
  }
  double var_translation = cv::mean(residuals)[0];
  double var_rotation = var_translation / 10.0; // simple heuristic
  cv::Mat cov = cv::Mat::zeros(6, 6, CV_64F);
  cov.at<double>(0, 0) = var_translation; // tx
  cov.at<double>(1, 1) = var_translation; // ty
  cov.at<double>(2, 2) = var_translation; // tz
  cov.at<double>(3, 3) = var_rotation;    // roll
  cov.at<double>(4, 4) = var_rotation;    // pitch
  cov.at<double>(5, 5) = var_rotation;    // yaw
  covariance_ = cov.clone();

  // (Optional) Statistical Filtering & Smoothing
  if (apply_statistical_filtering_)
  {
    // Decompose translation vector
    double tx = tvec.at<double>(0);
    double ty = tvec.at<double>(1);
    double tz = tvec.at<double>(2);

    // Decompose rotation vector
    double rx = rvec.at<double>(0);
    double ry = rvec.at<double>(1);
    double rz = rvec.at<double>(2);

    // Filter each component
    tx = filterOutlier(tx, translation_x_window_, max_translation_threshold_);
    ty = filterOutlier(ty, translation_y_window_, max_translation_threshold_);
    tz = filterOutlier(tz, translation_z_window_, max_translation_threshold_);

    rx = filterOutlier(rx, roll_window_, max_rotation_threshold_);
    ry = filterOutlier(ry, pitch_window_, max_rotation_threshold_);
    rz = filterOutlier(rz, yaw_window_, max_rotation_threshold_);

    // Apply moving average or other smoothing
    tvec.at<double>(0) = applyMovingAverage(translation_x_window_, tx);
    tvec.at<double>(1) = applyMovingAverage(translation_y_window_, ty);
    tvec.at<double>(2) = applyMovingAverage(translation_z_window_, tz);

    rvec.at<double>(0) = applyMovingAverage(roll_window_, rx);
    rvec.at<double>(1) = applyMovingAverage(pitch_window_, ry);
    rvec.at<double>(2) = applyMovingAverage(yaw_window_, rz);
  }

  // Convert rotation vector to rotation matrix
  cv::Rodrigues(rvec, rotation_);
  //std::cout << "Rotation matrix: " << rotation_ << std::endl;
  //std::cout << "Rotation vector " << rvec << std::endl;
  translation_ = tvec.clone();

  // If exponential smoothing is enabled, apply it directly
  if (apply_expotential_smoothing_)
  {
  // Note: direct matrix blending for rotation can be sub-optimal.
  // For now, we do as in your code. Alternatively, apply smoothing on rvec or quaternion.
  translation_ = exponential_alpha_ * translation_ + (1.0 - exponential_alpha_) * translation_prev;
  rotation_ = exponential_alpha_ * rotation_ + (1.0 - exponential_alpha_) * rotation_prev;
  }

  // Update previous translation and rotation
  translation_prev = translation_.clone();
  rotation_prev = rotation_.clone();
}


void VisualOdometry::computeStatistics(const std::deque<double>& data, double& mean, double& stddev)
{
  if (data.empty())
  {
      mean = 0.0;
      stddev = 0.0;
      return;
  }

  // Compute mean
  mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

  // Compute variance and standard deviation
  double variance = 0.0;
  for (const auto& value : data)
  {
      variance += (value - mean) * (value - mean);
  }
  variance /= data.size();
  stddev = std::sqrt(variance);
}

// Filter outliers based on statistical criteria and maximum thresholds

double VisualOdometry::filterOutlier(double new_value, std::deque<double>& window, double max_threshold)
{
  // Initialize the window if empty
  if (window.empty())
  {
      window.push_back(new_value);
      return new_value; // Return the first value
  }

  // Compute mean and standard deviation
  double mean, stddev;
  computeStatistics(window, mean, stddev);

  if (stddev == 0.0)
    {
        //std::cerr << "Standard deviation is zero. Accepting new value: " << new_value << std::endl;

        // Add the new value to the window
        window.push_back(new_value);
        if (window.size() > window_size_)
        {
            window.pop_front(); // Maintain the sliding window size
        }

        return new_value; // Return the new value as valid
    }



  // Detect and handle huge spikes
  if (std::abs(new_value) > max_threshold)
  {
      //std::cerr << "Huge spike detected: " << new_value << " (exceeds max threshold: " << max_threshold << ")" << std::endl;
      return mean; // Replace with the mean
  }

  // Detect and handle statistical outliers
  if (std::abs(new_value - mean) > outlier_threshold_factor_ * stddev)
  {
      //std::cerr << "Statistical outlier detected: " << new_value 
      //          << " (mean: " << mean << ", stddev: " << stddev << ")" << std::endl;
      return mean; // Replace with the mean
  }

  // Add the new value to the window
  window.push_back(new_value);

  // Remove the oldest value if the window exceeds the maximum size
  if (window.size() > window_size_)
  {
      window.pop_front();
  }

  return new_value; // Return the filtered value
}



  cv::Point3f VisualOdometry::reprojectTo3D(const cv::Point2f& point, float depth) const
  {
    float x = (point.x - camera_matrix_.at<double>(0, 2)) * depth / camera_matrix_.at<double>(0, 0);
    float y = (point.y - camera_matrix_.at<double>(1, 2)) * depth / camera_matrix_.at<double>(1, 1);
    return cv::Point3f(x, y, depth);
  }

  void VisualOdometry::getOutput(cv::Mat& translation, cv::Mat& rotation, cv::Mat& covariance, cv::Mat& debug) const
  { 
    translation = translation_.clone();
    rotation = rotation_.clone();
    debug = debug_image_.clone();
    covariance = covariance_.clone();
  }

  // setters
  void VisualOdometry::setCameraParameters(double fx, double fy, double cx, double cy)
  {
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<  fx, 0.0, cx,
                                                 0.0, fy, cy,
                                                 0.0, 0.0, 1.0);
    camera_info_arrived_ = true;
  }

  void VisualOdometry::setMinMaxDepth(double min_depth, double max_depth)
  {
    min_depth_ = min_depth;
    max_depth_ = max_depth;
  }

  double VisualOdometry::applyMovingAverage(std::deque<double>& window, double new_value)
{
    window.push_back(new_value);

    if (window.size() > window_size_)
    {
        window.pop_front();
    }

    return std::accumulate(window.begin(), window.end(), 0.0) / window.size();
}


} // Namespace visual_odometry
} // Namespace perception_pipeline
