#include "object_detector/world_entity.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

namespace perception_pipeline
{
namespace object_detector
{
  WorldEntity::WorldEntity() :
    centroid_(Eigen::Vector3f::Zero()),
    velocity_(Eigen::Vector3f::Zero())
  { 
    // Do nothing
  }

  void WorldEntity::addPoint(const pcl::PointXYZ& point, const Eigen::Vector3f& velocity)
  {
    points_.push_back(point);
    velocities_.push_back(velocity);
  }

  void WorldEntity::computeCentroidAndVelocity()
  {
      if (points_.empty()) return;
      
      Eigen::Vector3f sum_position(0.0, 0.0, 0.0);
      Eigen::Vector3f sum_velocity(0.0, 0.0, 0.0);
      
      for (size_t i = 0; i < points_.size(); ++i)
      {
          sum_position += Eigen::Vector3f(points_[i].x, points_[i].y, points_[i].z);
          sum_velocity += velocities_[i];
      }
      
      centroid_ = sum_position / static_cast<float>(points_.size());
      velocity_ = sum_velocity / static_cast<float>(points_.size());
  }

  float WorldEntity::computeAngularSimilarity(const WorldEntity& other) const
  { 
    float dot_product = velocity_.dot(other.velocity_);
    float norm_product = velocity_.norm() * other.velocity_.norm();

    if (norm_product == 0.0f) return 0.0f;
    return std::acos(dot_product / norm_product);
  }

  // getters
  Eigen::Vector3f WorldEntity::getCentroid() const
  {
    return centroid_;
  }

  Eigen::Vector3f WorldEntity::getVelocity() const
  {
    return velocity_;
  }
} // namespace object_detector
} // namespace perception_pipeline