#include "object_detector/object_detector.hpp"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <unordered_map>
#include <queue>       // For BFS
#include <iostream>
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
  #ifdef ROS_VERSION_JAZZY
  dbscan_(eps, 
          minPts, 
          false, 
          rs_, 
          mlpack::dbsOrderedPointSelection())
  #endif
  #ifdef ROS_VERSION_HUMBLE
  dbscan_(eps, 
          minPts, 
          false, 
          rs_, 
          mlpack::dbscan::OrderedPointSelection())
  #endif

{
}

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
  std::cout << "Applying DBSCAN clustering on the 6D image..." << std::endl;
  std::vector<WorldEntity> clusters; 

  arma::mat points = extractAndFilterCUDA(input_6d_image_, vel_threshold_);

  // print the arma matrix dimension
  std::cout << "points.n_rows: " << points.n_rows << std::endl;
  std::cout << "points.n_cols: " << points.n_cols << std::endl;

  // If there are no points, simply return an empty vector.
  if(points.n_cols == 0) {
    std::cout << "No valid points found, returning empty clusters." << std::endl;
    return clusters;
  }

  arma::Row<size_t> assignments;
  size_t cluster_count = dbscan_.Cluster(points, assignments);

  // Print the number of clusters
  std::cout << "Number of clusters: " << cluster_count << std::endl;

    
  std::vector<int> labels(points.n_cols, -1);
  for (size_t i = 0; i < points.n_cols; i++) {
    if (assignments[i] == SIZE_MAX) {
      labels[i] = -1; // noise
    } else {
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
  // Check if the input image is empty
  if (image_6d.empty()) {
      return arma::mat();
  }

  // Get the image dimensions and total number of points
  int rows = image_6d.rows;
  int cols = image_6d.cols;
  int total_points = rows * cols;

  // Allocate GPU memory
  float* d_valid_points;
  int* d_valid_count;
  cudaMalloc(&d_valid_points, total_points * 6 * sizeof(float));  
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

  // Allocate GPU memory for the filtered points
  float* d_filtered_points;
  int* d_filtered_count;
  cudaMalloc(&d_filtered_points, valid_points_number * 6 * sizeof(float));
  cudaMalloc(&d_filtered_count, sizeof(int));

  // Initialize filtered count
  cudaMemset(d_filtered_count, 0, sizeof(int));

  // Call CUDA filtering function
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
  // Allocate memory on the host
  float* h_data = new float[channels * total_points];

  // Copy data from device to host
  cudaMemcpy(h_data, d_data, channels * total_points * sizeof(float), cudaMemcpyDeviceToHost);

  // Convert float data to double data
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

