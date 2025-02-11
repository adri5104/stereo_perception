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
  eps_(eps),
  minPts_(minPts),
  pos_weight_(pos_weight),
  vel_weight_(vel_weight)
{
}

objectDetectorErrorCode ObjectDetector::update(const cv::Mat& image_6d)
{
  if (image_6d.empty()) {
      return objectDetectorErrorCode::INVALID_6D_IMAGE_ERROR;
  }

  // Allocate continuous memory for the 6D image
  cv::cuda::createContinuous(image_6d.size(), image_6d.type(), input_6d_image_);

  // Upload the 6D image to the GPU
  input_6d_image_.upload(image_6d);

  // Apply DBSCAN clustering
  clusters_ = applyDBSCAN(eps_, minPts_, pos_weight_, vel_weight_);

  return objectDetectorErrorCode::OK;
}

std::vector<WorldEntity> ObjectDetector::applyDBSCAN(float eps, int minPts, float pos_weight, float vel_weight)
{
  std::vector<WorldEntity> clusters; 

  // Check if the input 6D image is empty
  if (input_6d_image_.empty()) return clusters;

  // Get the image dimensions and total number of points
  int rows = input_6d_image_.rows;
  int cols = input_6d_image_.cols;
  int total_points = rows * cols;

  // Allocate GPU memory
  float* d_valid_points;
  int* d_labels;
  int* d_valid_count;

  cudaMalloc(&d_valid_points, total_points * 6 * sizeof(float));  
  cudaMalloc(&d_labels, total_points * sizeof(int)); 
  cudaMalloc(&d_valid_count, sizeof(int));

  // Initialize valid count
  cudaMemset(d_valid_count, 0, sizeof(int)); 

  // Call CUDA filtering function 
  filterValidPointsKernelLauncher(
    input_6d_image_.ptr<float>(), 
    d_valid_points,  
    d_valid_count, 
    total_points
  );

  // Retrieve the number of valid points
  int valid_points_number = 0;
  cudaMemcpy(&valid_points_number, d_valid_count, sizeof(int), cudaMemcpyDeviceToHost);
  cudaDeviceSynchronize();  // Ensure all previous CUDA ops complete

  if (valid_points_number == 0) {
      cudaFree(d_valid_points);
      cudaFree(d_labels);
      cudaFree(d_valid_count);
      return clusters;
  }

  // Properly initialize labels for valid points only
  std::vector<int> init_labels(valid_points_number, -2);
  cudaMemcpy(d_labels, init_labels.data(), valid_points_number * sizeof(int), cudaMemcpyHostToDevice);
  
  // Run DBSCAN kernel
  launchDbscanKernel(d_valid_points, d_labels, valid_points_number, eps, minPts, pos_weight, vel_weight);
  cudaDeviceSynchronize();  // Ensure DBSCAN is completed before proceeding

  // Retrieve cluster labels from GPU
  std::vector<int> labels(valid_points_number);
  cudaMemcpy(labels.data(), d_labels, valid_points_number * sizeof(int), cudaMemcpyDeviceToHost);

  // Retrieve the valid 6D points **in one go** (batch memory copy)
  std::vector<float> host_valid_points(valid_points_number * 6);
  cudaMemcpy(host_valid_points.data(), d_valid_points, valid_points_number * 6 * sizeof(float), cudaMemcpyDeviceToHost);

  // Free GPU memory (only after data has been copied)
  cudaFree(d_valid_points);
  cudaFree(d_labels);
  cudaFree(d_valid_count);

  // Map clusters
  std::unordered_map<int, WorldEntity> cluster_map;
  for (int i = 0; i < valid_points_number; ++i) {
    //std::cout << "Label: " << labels[i] << std::endl;

    if (labels[i] == -1) continue;  // Skip noise

    float* pixel = &host_valid_points[i * 6];

    // Create a new point
    pcl::PointXYZ point(pixel[0], pixel[1], pixel[2]); 
    Eigen::Vector3f velocity(pixel[3], pixel[4], pixel[5]); 

    cluster_map[labels[i]].addPoint(point, velocity);
  }

  // Convert clusters to output vector
  for (auto& [label, cluster] : cluster_map) {
    clusters.push_back(cluster);
  }

  // Print cluster number
  std::cout << "Number of clusters: " << clusters.size() << std::endl;

  return clusters;
}

} // namespace object_detector
} // namespace perception_pipeline
