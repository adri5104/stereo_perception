/**
 * @file world_entity.hpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief This file contains the WorldEntity class, which represents a cluster of points in the world.
 */

#ifndef WORLD_ENTITY_HPP
#define WORLD_ENTITY_HPP

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

namespace perception_pipeline
{
namespace object_detector
{

using namespace std;

class WorldEntity {
public:

  /**
   * @brief Construct a new World Entity object
   * 
   */
  WorldEntity();

  /**
   * @brief add a new point to the entity 
   * 
   * @param point point to add
   * @param velocity velocity of the point
   */
  void addPoint(const pcl::PointXYZ& point, const Eigen::Vector3f& velocity);


  // Getters
  Eigen::Vector3f getCentroid();
  Eigen::Vector3f getVelocity();
  std::vector<pcl::PointXYZ> getPoints();
  std::vector<pcl::PointXYZ> getBoundingBox();
  

private:
  void computeCentroidAndVelocity();
  void computeBoundingBox();

  /// points that belong to the entity
  std::vector<pcl::PointXYZ> points_;

  /// velocities of the points
  std::vector<Eigen::Vector3f> velocities_;

  Eigen::Vector3f centroid_;
  Eigen::Vector3f velocity_;
  std::vector<pcl::PointXYZ> bounding_box_; 
  bool bounding_box_computed_;
  bool centroid_velocity_computed_;
};



} // namespace object_detector
} // namespace perception_pipeline


#endif // WORLD_OBJECT_HPP