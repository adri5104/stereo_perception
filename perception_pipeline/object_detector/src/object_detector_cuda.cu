#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>
#include <cstdint>  // Ensure uint8_t is defined
#include <stdio.h>

namespace perception_pipeline
{
namespace object_detector
{

// CUDA custom distance function for DBSCAN
__device__ float customDistance(const float* a, const float* b, float pos_w, float vel_w) {
    // Position distance (Euclidean)
    float pos_dist = sqrtf(powf(a[0] - b[0], 2) + powf(a[1] - b[1], 2) + powf(a[2] - b[2], 2));
    
    // Velocity angular similarity
    float dot_product = a[3] * b[3] + a[4] * b[4] + a[5] * b[5];
    float norm_product = sqrtf(a[3] * a[3] + a[4] * a[4] + a[5] * a[5]) * 
                          sqrtf(b[3] * b[3] + b[4] * b[4] + b[5] * b[5]);
    float vel_similarity = (norm_product > 0) ? acosf(dot_product / norm_product) : M_PI;

   
    return pos_w * pos_dist + vel_w * vel_similarity;
}

// CUDA Kernel: Filters valid points
__global__ void filterValidPointsKernel(const float* data, const uint8_t* valid_mask, float* filtered_data, int* valid_indices, int total_points, int* valid_count) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total_points) return;

    if (valid_mask[idx] == 1) {
        int out_idx = atomicAdd(valid_count, 1);
        for (int j = 0; j < 6; ++j) {
            filtered_data[out_idx * 6 + j] = data[idx * 6 + j];
        }
        valid_indices[out_idx] = idx;
    }
}



__global__ void dbscanKernel(float* data, int* labels, int total_points, float eps, int minPts, float pos_w, float vel_w) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total_points) return;

    if (idx < 10) {  // Print first 10 points
        printf("[DEBUG] CUDA Point %d: [%f, %f, %f, %f, %f, %f]\n",
               idx, data[idx * 6], data[idx * 6 + 1], data[idx * 6 + 2],
               data[idx * 6 + 3], data[idx * 6 + 4], data[idx * 6 + 5]);
    }

    if (labels[idx] != -1) return; // Already classified

    int count = 0;
    int neighbors[1024]; // Store indices of neighbors (max 1024 for now)
    
    // **Find Neighbors**
    for (int j = 0; j < total_points; ++j) {
        if (idx != j && customDistance(&data[idx * 6], &data[j * 6], pos_w, vel_w) < eps) {
            if (count < 1024) neighbors[count] = j; // Store neighbor index
            count++;
        }
    }

    if (count < minPts) {
        labels[idx] = -1; // Noise
        return;
    }

    // **Expand Cluster (Using Shared Memory for Efficiency)**
    __shared__ int cluster_id;
    if (threadIdx.x == 0) cluster_id = idx; // Assign unique cluster ID
    __syncthreads();

    labels[idx] = cluster_id; // Set label for the starting point

    for (int i = 0; i < count; ++i) {
        int neighbor_idx = neighbors[i];

        if (labels[neighbor_idx] == -1) { // If noise, convert to cluster point
            labels[neighbor_idx] = cluster_id;
        } else if (labels[neighbor_idx] == -2) { // If unclassified, assign cluster
            labels[neighbor_idx] = cluster_id;
        }
    }
}


// Function to launch CUDA DBSCAN
void launchDbscanKernel(float* d_data, int* d_labels, int total_points, float eps, int minPts, float pos_w, float vel_w) {
    int blockSize = 256;
    int gridSize = (total_points + blockSize - 1) / blockSize;
    dbscanKernel<<<gridSize, blockSize>>>(d_data, d_labels, total_points, eps, minPts, pos_w, vel_w);
    cudaDeviceSynchronize();
}

// Function to launch CUDA filtering of valid points
void filterValidPointsKernelLauncher(const float* d_data, const uint8_t* d_valid_mask, float* d_filtered_data, int* d_valid_indices, int* d_valid_count, int total_points) {
    int blockSize = 256;
    int gridSize = (total_points + blockSize - 1) / blockSize;
    filterValidPointsKernel<<<gridSize, blockSize>>>(d_data, d_valid_mask, d_filtered_data, d_valid_indices, total_points, d_valid_count);
    cudaDeviceSynchronize();
}

} // namespace object_detector
} // namespace perception_pipeline
