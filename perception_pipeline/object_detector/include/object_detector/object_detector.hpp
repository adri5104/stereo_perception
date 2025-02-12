/**
 * @file object_detector.hpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief This file contains the ObjectDetector class, which is responsible for detecting object clusters in a 6D image using DBSCAN clustering.
 */

#ifndef OBJECT_DETECTOR_HPP
#define OBJECT_DETECTOR_HPP

#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include "mlpack/core.hpp"
#include "mlpack/core/util/log.hpp"
#include "mlpack/core/util/timers.hpp"
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <mlpack/methods/range_search/range_search.hpp>

#ifdef ROS_VERSION_HUMBLE
#include <mlpack/core/tree/cover_tree.hpp>
#endif
#include "object_detector/object_detector.hpp"
#include "object_detector/world_entity.hpp"
#include "object_detector/object_detector_error_codes.hpp"
#include "object_detector/point_distance.hpp"

using namespace cv;

namespace perception_pipeline
{
namespace object_detector
{


#ifdef ROS_VERSION_JAZZY
/// @brief  Range search type for DBSCAN clustering
using RSType = mlpack::RangeSearch<
    WeightedPosVelDistance,
    arma::mat,
    mlpack::StandardCoverTree
>;
#endif
#ifdef ROS_VERSION_HUMBLE
/// @brief  Range search type for DBSCAN clustering
using RSType = mlpack::range::RangeSearch<
    WeightedPosVelDistance,
    arma::mat,
    mlpack::tree::StandardCoverTree
>;
#endif

class ObjectDetector
{
  public: 
    
    /**
     * @brief Construct a new Object Detector object
     * 
     * @param eps min distance between two points to be considered in the same cluster
     * @param minPts min number of points to form a cluster
     * @param pos_weight weight factor for position in distance computation
     * @param vel_weight weight factor for velocity in distance computation
     * @param vel_threshold velocity threshold for filtering
     */
    ObjectDetector(float eps, int minPts, float pos_weight, float vel_weight, float vel_threshold);

    /**
     * @brief update the 6D image stored in GPU memory using OpenCV's CUDA GPU matrix, 
     *        and apply DBSCAN clustering on the 6D image using CUDA and mlpack.
     * 
     * @param image_6d 6D image to be updated
     * @return objectDetectorErrorCode 
     */
    objectDetectorErrorCode update(const cv::Mat& image_6d);

    


    /**
     * @brief Get the clusters detected by the object detector.
     * 
     * @return std::vector<WorldEntity> List of detected object clusters.
     */
    std::vector<WorldEntity> getClusters() const { return clusters_; }
    
  private:

    /**
     * @brief Apply DBSCAN clustering on the 6D image using CUDA.
     * 
     * @param eps Clustering epsilon threshold.
     * @param minPts Minimum number of points to form a cluster.
     * @param pos_weight Weight factor for position in distance computation.
     * @param vel_weight Weight factor for velocity in distance computation.
     * @return std::vector<WorldEntity> List of detected object clusters.
     */
    std::vector<WorldEntity> applyDBSCAN(float eps, int minPts, float pos_weight, float vel_weight);

    /**
     * @brief Extracts world points from the 6D image and filters them based on the velocity threshold using CUDA.
     * 
     * @param image_6d The input 6D image.
     * @param vel_threshold The velocity threshold for filtering.
     * 
     * @return arma::mat The filtered worldpoints.
     */
    arma::mat extractAndFilterCUDA(const cv::cuda::GpuMat& image_6d, float vel_threshold);

    /**
     * @brief Converts a CUDA pointer to an Armadillo matrix.
     * 
     * @param d_data The CUDA pointer.
     * @param channels The number of channels.
     * @param total_points The total number of points.
     * 
     * @return arma::mat The Armadillo matrix.
     */
    arma::mat cudaPtrToArmaMat(float* d_data, int channels, int total_points);

    /// DBSCAN object
    RSType rs_;
    mlpack::dbscan::DBSCAN<RSType> dbscan_;
    
    /// The current 6D image stored as cuda matrix
    cv::cuda::GpuMat input_6d_image_; 

    /// Detected object clusters
    std::vector<WorldEntity> clusters_;

    float eps_;
    int minPts_;
    float pos_weight_;
    float vel_weight_;
    float vel_threshold_;

};

// CUDA kernel launchers
void filterValidPointsKernelLauncher(const float* d_input_6d_image, float* d_valid_points, int* d_valid_count, int total_points);
void filterPointsVelKernelLauncher(const float* d_data, float* d_filtered_data, int* d_valid_count, int total_points, float vel_th);


} // namespace object_detector
} // namespace perception_pipeline

#endif // OBJECT_DETECTOR_HPP
