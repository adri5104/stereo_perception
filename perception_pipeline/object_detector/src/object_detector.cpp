#include "object_detector/object_detector.hpp"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <unordered_map>
#include <cuda_runtime.h>

namespace perception_pipeline
{
namespace object_detector
{

ObjectDetector::ObjectDetector(float eps, int minPts, float pos_weight, float vel_weight) :
  input_6d_image_(cv::cuda::GpuMat()),
  input_6d_image_val_(cv::cuda::GpuMat()),
  eps_(eps),
  minPts_(minPts),
  pos_weight_(pos_weight),
  vel_weight_(vel_weight)
{
}

objectDetectorErrorCode ObjectDetector::updateSync(const cv::Mat& image_6d, const cv::Mat& image_6d_val)
{
  if (image_6d.empty() || image_6d_val.empty()) {
    return objectDetectorErrorCode::INVALID_6D_IMAGE_ERROR;
  }

  input_6d_image_.upload(image_6d);
  input_6d_image_val_.upload(image_6d_val);

  clusters_.clear();
  clusters_ = applyDBSCAN(eps_, minPts_, pos_weight_, vel_weight_);

  return objectDetectorErrorCode::OK;
}

std::vector<WorldEntity> ObjectDetector::applyDBSCAN(float eps, int minPts, float pos_weight, float vel_weight)
{
    std::vector<WorldEntity> clusters; 

    std::cout << "Applying DBSCAN clustering..." << std::endl;

    if (input_6d_image_.empty() || input_6d_image_val_.empty()) return clusters;

    int rows = input_6d_image_.rows;
    int cols = input_6d_image_.cols;
    int total_points = rows * cols;

    // Allocate GPU memory
    float* d_valid_points;
    int* d_valid_indices;
    int* d_labels;
    int* d_valid_count; 

    cudaMalloc(&d_valid_points, total_points * 6 * sizeof(float)); 
    cudaMalloc(&d_valid_indices, total_points * sizeof(int)); 
    cudaMalloc(&d_labels, total_points * sizeof(int)); 
    cudaMalloc(&d_valid_count, sizeof(int));

    cudaMemset(d_valid_count, 0, sizeof(int));

    // Call the CUDA filtering function 
    //filterValidPointsKernelLauncher(
    //    (float*)input_6d_image_.ptr<float>(), 
    //    (uint8_t*)input_6d_image_val_.ptr<uint8_t>(), 
    //    d_valid_points, d_valid_indices, 
    //    d_valid_count, total_points
    //);

    cudaDeviceSynchronize();

    // Retrieve the number of valid points
    int valid_points_number = 0;
    cudaMemcpy(&valid_points_number, d_valid_count, sizeof(int), cudaMemcpyDeviceToHost);

    

    if (valid_points_number == 0) {
        std::cerr << "[ERROR] No valid points for clustering!" << std::endl;
        cudaFree(d_valid_points);
        cudaFree(d_valid_indices);
        cudaFree(d_labels);
        cudaFree(d_valid_count);
        return clusters;
    }

    cudaMemset(d_labels, -1, valid_points_number * sizeof(int));

    // Run DBSCAN
    launchDbscanKernel(d_valid_points, d_labels, valid_points_number, eps, minPts, pos_weight, vel_weight);
    cudaDeviceSynchronize();

    // Retrieve cluster labels
    std::vector<int> labels(valid_points_number);
    cudaMemcpy(labels.data(), d_labels, valid_points_number * sizeof(int), cudaMemcpyDeviceToHost);


    // Retrieve valid indices
    std::vector<int> valid_indices(valid_points_number);
    cudaMemcpy(valid_indices.data(), d_valid_indices, valid_points_number * sizeof(int), cudaMemcpyDeviceToHost);

    // Free GPU memory
    cudaFree(d_valid_points);
    cudaFree(d_valid_indices);
    cudaFree(d_labels);
    cudaFree(d_valid_count);

    // Map clusters
    std::unordered_map<int, WorldEntity> cluster_map;
    for (int i = 0; i < valid_points_number; ++i) {
        if (labels[i] == -1) continue;

        int index = valid_indices[i]; 
        int x = index % cols;
        int y = index / cols;
        cv::Mat input_6d_image_host;
        input_6d_image_.download(input_6d_image_host);
        cv::Vec6f pixel = input_6d_image_host.at<cv::Vec6f>(y, x);

        pcl::PointXYZ point(pixel[0], pixel[1], pixel[2]); 
        Eigen::Vector3f velocity(pixel[3], pixel[4], pixel[5]); 

        cluster_map[labels[i]].addPoint(point, velocity);
    }

    std::cout << "[DEBUG] Total Clusters Detected: " << clusters.size() << std::endl;
    for (size_t i = 0; i < clusters.size(); ++i) {
      std::cout << "[Cluster " << i << "] has " << clusters[i].getPoints().size()<< " points." << std::endl;
}


    return clusters;
}

} // namespace object_detector
} // namespace perception_pipeline
