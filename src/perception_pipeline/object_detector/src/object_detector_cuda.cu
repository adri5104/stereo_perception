/**
 * @file object_detector_cuda.cu
 * @author Adrian Rieker
 * @brief CUDA Kernel implementation for the object detector
 */

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>
#include <cstdint>  
#include <stdio.h>


namespace stereo_perception
{
namespace perception_pipeline
{
namespace object_detector
{

  __device__ int global_cluster_counter = 0;

/**
 * @brief Computes a custom distance metric for DBSCAN
 * 
 * @param a First point (6D: XYZ + Velocity XYZ)
 * @param b Second point (6D: XYZ + Velocity XYZ)
 * @param pos_w Weight for position distance
 * @param vel_w Weight for velocity similarity
 * @return Distance metric based on position and velocity
 */
__device__ float customDistance(const float* a, const float* b, float pos_w, float vel_w) {
    float pos_dist = sqrtf(powf(a[0] - b[0], 2) + powf(a[1] - b[1], 2) + powf(a[2] - b[2], 2));
    
    float dot_product = a[3] * b[3] + a[4] * b[4] + a[5] * b[5];
    float norm_product = sqrtf(a[3] * a[3] + a[4] * a[4] + a[5] * a[5]) * 
                         sqrtf(b[3] * b[3] + b[4] * b[4] + b[5] * b[5]);

    float vel_similarity = (norm_product > 0) ? acosf(dot_product / norm_product) : M_PI;
    
    return pos_w * pos_dist + vel_w * vel_similarity;
}

__device__ float getSpeed(const float* a) {
    return sqrtf(a[3] * a[3] + a[4] * a[4] + a[5] * a[5]);
}

/**
 * @brief Kernel to filter valid points from the input 6D image
 */
__global__ void filterValidPointsKernel(
  const float* data, float* filtered_data, int total_points, int* valid_count) 
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total_points) return;

    int input_offset = idx * 7;  
    float valid_flag = data[input_offset + 6];

    if (valid_flag >= 0.5f) {
        int output_idx = atomicAdd(valid_count, 1);
        for (int i = 0; i < 6; ++i) {
            filtered_data[output_idx * 6 + i] = data[input_offset + i];
        }
    }
}

/**
 * @brief Kernel to filter valid points depending on the velocity
 */
__global__ void filterPointsKernel(
  const float* data, float* filtered_data, int total_points, int* valid_count, float vel_th) 
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= total_points) return;

  int input_offset = idx * 6;  
  float speed = getSpeed(&data[input_offset]);

  if (speed >= vel_th) 
  {
    int output_idx = atomicAdd(valid_count, 1);
    for (int i = 0; i < 6; ++i) {
      filtered_data[output_idx * 6 + i] = data[input_offset + i];
    }
  }
}

/**
 * @brief DBSCAN Kernel for clustering
 */
__global__ void dbscanKernel(float* data, int* labels, int total_points, 
                             float eps, int minPts, float pos_w, float vel_w, 
                             int* neighbors, int* neighbor_counts) 
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total_points) return;

    if (labels[idx] != -2) return;

    int* my_neighbors = &neighbors[idx * 1024];  
    int count = 0;

    // Find neighbors
    for (int j = 0; j < total_points; ++j) {
        if (j == idx) continue;

        float dist = customDistance(&data[idx * 6], &data[j * 6], pos_w, vel_w);
        if (dist < eps && count < 1024) {
            my_neighbors[count++] = j;
        }
    }

    neighbor_counts[idx] = count;

    if (count < minPts) {
        labels[idx] = -1;
        return;
    }

    int cluster_id = atomicAdd(&global_cluster_counter, 1);
    labels[idx] = cluster_id;

    // **Iteratively Expand Clusters**
    for (int iter = 0; iter < 10; ++iter) 
    {
        __syncthreads();

        for (int i = 0; i < count; ++i) {
            int neighbor_idx = my_neighbors[i];

            if (labels[neighbor_idx] == -2 || labels[neighbor_idx] == -1) {
                labels[neighbor_idx] = cluster_id;
            }
        }
    }
}

/**
 * @brief Function to launch the DBSCAN kernel
 */
void launchDbscanKernel(float* d_data, int* d_labels, int total_points,
                        float eps, int minPts, float pos_w, float vel_w) 
{
    int blockSize = 256;
    int gridSize = (total_points + blockSize - 1) / blockSize;

    int zero = 0;
    cudaMemcpyToSymbol(global_cluster_counter, &zero, sizeof(int));
    cudaDeviceSynchronize();

    int* d_neighbors;
    int* d_neighbor_counts;
    cudaMalloc(&d_neighbors, total_points * 1024 * sizeof(int));
    cudaMalloc(&d_neighbor_counts, total_points * sizeof(int));
    cudaMemset(d_neighbor_counts, 0, total_points * sizeof(int));
  
    dbscanKernel<<<gridSize, blockSize>>>(d_data, d_labels, total_points, eps, minPts, pos_w, vel_w,
                                          d_neighbors, d_neighbor_counts);
    cudaDeviceSynchronize();

    cudaFree(d_neighbors);
    cudaFree(d_neighbor_counts);
}

/**
 * @brief Filters valid points using CUDA (valid if 7th element is >= 0.5)
 * 
 * @param d_data Input 6D data
 * @param d_filtered_data Output 6D data
 * @param d_valid_count Number of valid points
 * @param total_points Total number of points
 */
void filterValidPointsKernelLauncher(
  const float* d_data, float* d_filtered_data, int* d_valid_count, int total_points) 
{
    int blockSize = 256;
    int gridSize = (total_points + blockSize - 1) / blockSize;
    filterValidPointsKernel<<<gridSize, blockSize>>>(
      d_data, d_filtered_data, total_points, d_valid_count);

    cudaDeviceSynchronize();
}

/**
 * @brief Filters points based on velocity 
 * 
 * @param d_data Input 6D data
 * @param d_filtered_data Output 6D data
 * @param d_valid_count Number of valid points
 * @param total_points Total number of points of d_data
 * @param vel_th Velocity threshold
 */
void filterPointsVelKernelLauncher(
  const float* d_data, float* d_filtered_data, int* d_valid_count, int total_points, float vel_th) 
{
    int blockSize = 256;
    int gridSize = (total_points + blockSize - 1) / blockSize;
    filterPointsKernel<<<gridSize, blockSize>>>(
      d_data, d_filtered_data, total_points, d_valid_count, vel_th);

    cudaDeviceSynchronize();
}

} // namespace object_detector
} // namespace perception_pipeline
} // namespace stereo_perception
