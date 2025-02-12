/**
 * @file object_detector.hpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief This file contains the implementation of the ObjectDetector class, which is responsible for detecting object clusters in a 6D image using DBSCAN clustering.
 */


#include "object_detector/object_detector.hpp"

#include <iostream>
#include <cmath>
#include <unordered_map>
#include <queue>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include <cuda_runtime.h>

namespace perception_pipeline
{
namespace object_detector
{

ObjectDetector::ObjectDetector(float eps, int minPts, float pos_weight, float vel_weight, float vel_threshold) :
  input_6d_image_(cv::cuda::GpuMat()),
  eps_(eps),
  minPts_(minPts),
  pos_weight_(pos_weight),
  vel_weight_(vel_weight),
  vel_threshold_(vel_threshold),
  rs_(true, false, WeightedPosVelDistance(pos_weight, vel_weight)),
  dbscan_(eps, 
          minPts, 
          false, 
          rs_, 
          mlpack::OrderedPointSelection()){}

objectDetectorErrorCode ObjectDetector::update(const cv::Mat& image_6d)
{
  if (image_6d.empty()) {
      return objectDetectorErrorCode::INVALID_6D_IMAGE_ERROR;
  }

  // Allocate continuous memory for the 6D image on the GPU
  cv::cuda::createContinuous(image_6d.size(), image_6d.type(), input_6d_image_);

  // Upload to the GPU (so you can do further GPU processing if needed)
  input_6d_image_.upload(image_6d);

  // Now do CPU-based DBSCAN
  clusters_ = applyDBSCAN(eps_, minPts_, pos_weight_, vel_weight_);

  return objectDetectorErrorCode::OK;
}


std::vector<WorldEntity> ObjectDetector::applyDBSCAN(float eps, int minPts, float pos_weight, float vel_weight)
{
  std::vector<WorldEntity> clusters; 
  arma::mat points = extractAndFilterCUDA(input_6d_image_, vel_threshold_);
  if(points.n_cols == 0) {
    return clusters;
  }
  arma::Row<size_t> assignments;
  size_t cluster_count = dbscan_.Cluster(points, assignments);
    
  std::vector<int> labels(points.n_cols, -1);
  for (size_t i = 0; i < points.n_cols; i++) {
    if (assignments[i] == SIZE_MAX) {
      labels[i] = -1; // noise
    } 
    else 
    {
      labels[i] = static_cast<int>(assignments[i]);
    }
  }
  std::unordered_map<int, WorldEntity> cluster_map;
  for(size_t i=0; i<points.n_cols; i++){
    int lbl = labels[i];
    if (lbl < 0) continue; // skip noise

    double x  = points(0, i);
    double y  = points(1, i);
    double z  = points(2, i);
    double vx = points(3, i);
    double vy = points(4, i);
    double vz = points(5, i);

    pcl::PointXYZ point(x,y,z);
    Eigen::Vector3f velocity(vx,vy,vz);
    cluster_map[lbl].addPoint(point, velocity);
  }
  for(auto& [cid, entity] : cluster_map) {
    clusters.push_back(entity);
  }
  return clusters;
}

arma::mat ObjectDetector::extractAndFilterCUDA(const cv::cuda::GpuMat& image_6d, float vel_threshold)
{
  if (image_6d.empty()) 
    return arma::mat();
  
  int rows = image_6d.rows;
  int cols = image_6d.cols;
  int total_points = rows * cols;

  // Allocate GPU memory
  float* d_valid_points;
  int* d_valid_count;
  cudaMalloc(&d_valid_points, total_points * 6 * sizeof(float));  
  cudaMalloc(&d_valid_count, sizeof(int));
  cudaMemset(d_valid_count, 0, sizeof(int)); 

  filterValidPointsKernelLauncher(
    input_6d_image_.ptr<float>(), 
    d_valid_points,  
    d_valid_count, 
    total_points
  );

  // Retrieve the number of valid points
  int valid_points_number = 0;
  cudaMemcpy(&valid_points_number, d_valid_count, sizeof(int), cudaMemcpyDeviceToHost);

  // Allocate GPU memory for the filtered points
  float* d_filtered_points;
  int* d_filtered_count;
  cudaMalloc(&d_filtered_points, valid_points_number * 6 * sizeof(float));
  cudaMalloc(&d_filtered_count, sizeof(int));
  cudaMemset(d_filtered_count, 0, sizeof(int));

  filterPointsVelKernelLauncher(
    d_valid_points, 
    d_filtered_points, 
    d_filtered_count, 
    valid_points_number, 
    vel_threshold
  );

  // Retrieve the number of filtered points
  int filtered_points_number = 0;
  cudaMemcpy(&filtered_points_number, d_filtered_count, sizeof(int), cudaMemcpyDeviceToHost);

  // Convert the filtered points to an Armadillo matrix
  arma::mat valid_points = cudaPtrToArmaMat(d_filtered_points, 6, filtered_points_number);

  // Free GPU memory (only after data has been copied)
  cudaFree(d_valid_points);
  cudaFree(d_valid_count);
  cudaFree(d_filtered_points);
  cudaFree(d_filtered_count);

  return valid_points;
}

arma::mat ObjectDetector::cudaPtrToArmaMat(float* d_data, int channels, int total_points)
{

  float* h_data = new float[channels * total_points];
  cudaMemcpy(h_data, d_data, channels * total_points * sizeof(float), cudaMemcpyDeviceToHost);
  double* h_data_double = new double[channels * total_points];
  for (int i = 0; i < channels * total_points; ++i) {
    h_data_double[i] = static_cast<double>(h_data[i]);
  }

  // Convert to Armadillo matrix
  arma::mat mat(
    h_data_double, // Pointer to the data
    channels, // Number of rows
    total_points, // Number of columns
    true, //  copy
    true // Transpose
    );

  // Free host memory
  delete[] h_data;
  delete[] h_data_double;
  
  // Return the Armadillo matrix
  return mat;
}

} // namespace object_detector
} // namespace perception_pipeline

