#include "visual_odometry/visual_odometry.hpp"
#include <iostream>

namespace perception_pipeline
{
namespace visual_odometry
{

  VisualOdometry::VisualOdometry(double min_depth, double max_depth, bool create_debug_image):
                                  camera_info_arrived_(false),
                                  create_debug_image_(create_debug_image),
                                  min_depth_(min_depth), 
                                  max_depth_(max_depth),
                                  covariance_(cv::Mat::zeros(6, 6, CV_64F))
                              
  {
    // Initialize the camera intrinsic matrix
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<  1.0, 0.0, 1.0,
                                                 0.0, 1.0, 1.0,
                                                 0.0, 0.0, 1.0);

    translation_ = cv::Mat::zeros(3, 1, CV_64F);
    rotation_ = cv::Mat::eye(3, 3, CV_64F);

    // Initialize CUDA-based feature detector and matcher
    feature_detector_gpu_ = cv::cuda::ORB::create(
        1500,               // nfeatures
        1.1f,               // scaleFactor
        12,                 // nlevels
        15,                 // edgeThreshold
        0,                  // firstLevel
        2,                  // WTA_K
        cv::ORB::HARRIS_SCORE, // scoreType
        10                  // patchSize
    );
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

void VisualOdometry::estimateMotion(const std::vector<cv::Point2f>& prev_points, const std::vector<cv::Point2f>& curr_points, 
                                    const cv::Mat& prev_depth)
{
    std::vector<cv::Point3f> points3D;
    std::vector<cv::Point2f> valid_curr_points;

    // Debug image (optional)
    if (create_debug_image_)
    {
        debug_image_ = cv::Mat::zeros(prev_depth.size(), CV_8UC3);
    }

    // Reproject points to 3D
    for (size_t i = 0; i < prev_points.size(); ++i)
    {
        const cv::Point2f& point = prev_points[i];
        float depth = prev_depth.at<float>(static_cast<int>(point.y), static_cast<int>(point.x));

        if (depth > min_depth_ && depth < max_depth_)
        {
            // Valid depth: reproject to 3D
            points3D.push_back(reprojectTo3D(point, depth));
            valid_curr_points.push_back(curr_points[i]);

            // Draw valid keypoints in green
            if (create_debug_image_)
            {
                cv::circle(debug_image_, curr_points[i], 3, cv::Scalar(0, 255, 0), -1);
            }
        }
        else
        {
            // Draw invalid keypoints in red
            if (create_debug_image_)
            {
                cv::circle(debug_image_, curr_points[i], 3, cv::Scalar(0, 0, 255), -1);
            }
        }
    }

    // Early exit if not enough points
    if (points3D.size() < 6)
    {
        std::cerr << "Not enough valid points for motion estimation." << std::endl;
        return;
    }

    // Estimate motion using solvePnPRansac
    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(points3D, valid_curr_points, camera_matrix_, cv::noArray(), rvec, tvec, false, 100, 8.0, 0.99, inliers);

    // **Covariance Estimation**
    cv::Mat residuals(inliers.rows, 1, CV_64F);
    for (int i = 0; i < inliers.rows; ++i)
    {
        int idx = inliers.at<int>(i, 0);
        std::vector<cv::Point3f> single_point{points3D[idx]};
        std::vector<cv::Point2f> projected_points;
        
        cv::projectPoints(single_point, rvec, tvec, camera_matrix_, cv::noArray(), projected_points);
        cv::Point2f projected_point = projected_points[0];

        residuals.at<double>(i) = cv::norm(projected_point - valid_curr_points[idx]);
    }

    double var_translation = cv::mean(residuals)[0];
    double var_rotation = var_translation / 10.0; // Heuristic: rotation uncertainty smaller than translation

    cv::Mat cov = cv::Mat::zeros(6, 6, CV_64F);
    cov.at<double>(0, 0) = var_translation; // tx
    cov.at<double>(1, 1) = var_translation; // ty
    cov.at<double>(2, 2) = var_translation; // tz
    cov.at<double>(3, 3) = var_rotation;    // roll
    cov.at<double>(4, 4) = var_rotation;    // pitch
    cov.at<double>(5, 5) = var_rotation;    // yaw

    covariance_ = cov.clone();
    

    // get rotation matrix
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Transform rotation matrix to Euler angles
    double roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    double pitch = asin(-R.at<double>(2, 0));
    double yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));

    // Update rotation vector
    rotation_.at<double>(0) = roll;
    rotation_.at<double>(1) = pitch;
    rotation_.at<double>(2) = yaw;

    // Update translation vector
    translation_ = tvec.clone();

    std::cout << "Motion estimated: " << inliers.rows << " inliers used." << std::endl;
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



} // Namespace visual_odometry
} // Namespace perception_pipeline
