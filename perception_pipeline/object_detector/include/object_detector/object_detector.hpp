#ifndef OBJECT_DETECTOR_HPP
#define OBJECT_DETECTOR_HPP

#include <opencv2/opencv.hpp>  
#include <opencv2/core/cuda.hpp>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

#include "object_detector/object_detector.hpp"  
#include "object_detector/world_entity.hpp"     

using namespace cv;

namespace perception_pipeline
{
namespace object_detector
{

  /// 6D image pixel value
typedef Vec<float, 7> OutVec;

/// 6D image channel number
const int OUT6D_VAL_IDX = 6;
const int OUT6D_C = 7;
const int OUT6D_TYPE = CV_32FC(OUT6D_C);

/**
 * @brief Error codes for ObjectDetector operations.
 */
enum class objectDetectorErrorCode {
    OK = 1,                    ///< Operation successful.
    INVALID_6D_IMAGE_ERROR,     ///< Input 6D image is invalid or empty.
};
inline std::string getErrorMessageObjectDetector(objectDetectorErrorCode code) {
    static const std::unordered_map<objectDetectorErrorCode, std::string> errorMessages = {
        {objectDetectorErrorCode::OK, "No error."},
        {objectDetectorErrorCode::INVALID_6D_IMAGE_ERROR, "Invalid 6D image."}
    };
    auto it = errorMessages.find(code);
    return it != errorMessages.end() 
        ? it->second 
        : "Invalid error code.";
}

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
     */
    ObjectDetector(float eps, int minPts, float pos_weight, float vel_weight);

    /**
     * @brief update the 6D image stored in GPU memory using OpenCV's CUDA GPU matrix
     * 
     * @param image_6d 6D image to be updated
     * @return objectDetectorErrorCode 
     */
    objectDetectorErrorCode update(const cv::Mat& image_6d);

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


    std::vector<WorldEntity> getClusters() const { return clusters_; }
    
  private:
    /// The current 6D image stored in GPU memory using OpenCV's CUDA GPU matrix
    cv::cuda::GpuMat input_6d_image_; 

    /// Detected object clusters
    std::vector<WorldEntity> clusters_; ///< Detected object clusters 

    float eps_;
    int minPts_;
    float pos_weight_;
    float vel_weight_;

};

/**
 * @brief Function to launch the CUDA-based DBSCAN kernel.
 * 
 * @param d_data Pointer to device memory containing 6D image data.
 * @param d_labels Pointer to device memory storing cluster labels.
 * @param total_points Total number of points in the image.
 * @param eps Clustering epsilon threshold.
 * @param minPts Minimum number of points to form a cluster.
 * @param pos_w Weight for positional distance.
 * @param vel_w Weight for velocity similarity.
 */
extern void launchDbscanKernel(float* d_data, int* d_labels, int total_points, float eps, int minPts, float pos_w, float vel_w);

extern void filterValidPointsKernelLauncher(const float* d_data,
                                     float* d_valid_data,  
                                     int* d_valid_count, int total_points);

} // namespace object_detector
} // namespace perception_pipeline

#endif // OBJECT_DETECTOR_HPP
