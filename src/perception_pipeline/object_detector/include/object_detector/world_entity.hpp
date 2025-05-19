/**
 * @file world_entity.hpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief This file contains the WorldEntity class, which represents a cluster of points in the world.
 */

#ifndef WORLD_ENTITY_HPP
#define WORLD_ENTITY_HPP

#include <vector>
#include<set>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
namespace stereo_perception
{
namespace perception_pipeline
{
namespace object_detector
{

using namespace std;

class WorldEntity {
public:

  /**
   * @brief Construct a new World Entity object
   */
  WorldEntity();

  /**
   * @brief add a new point to the entity 
   * 
   * @param point point to add
   * @param velocity velocity of the point
   */
  void addPoint(const pcl::PointXYZ& point, const Eigen::Vector3f& velocity);


  /**
   * @brief Compute the centroid, velocity and bounding box of the entity
   * 
   */
  void compute();


  // Getters
  Eigen::Vector3f getCentroid() const;
  Eigen::Vector3f getVelocity() const;
  std::vector<pcl::PointXYZ> getPoints() const;
  std::vector<pcl::PointXYZ> getBoundingBox() const;
  int getId() const;

  // Setters
  void setId(int id);
  

private:
  void computeCentroidAndVelocity();
  void computeBoundingBox();

  /// ID of the entity
  int id_;

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
} // namespace stereo_perception

#endif // WORLD_OBJECT_HPP