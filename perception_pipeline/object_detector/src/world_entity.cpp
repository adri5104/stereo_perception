#include "object_detector/world_entity.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

namespace perception_pipeline
{
namespace object_detector
{
  WorldEntity::WorldEntity() :
    id_(-1),
    points_(),
    velocities_(),
    centroid_(Eigen::Vector3f::Zero()),
    velocity_(Eigen::Vector3f::Zero()),
    bounding_box_(),
    bounding_box_computed_(false),
    centroid_velocity_computed_(false){
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
    centroid_velocity_computed_ = true;
  }

  void WorldEntity::computeBoundingBox() 
  {
    if(points_.empty()) return;

    // 1. Compute centroid
    if (!centroid_velocity_computed_)
      computeCentroidAndVelocity();

    // 2. Build a data matrix where each column is a point (centroid subtracted)
    Eigen::MatrixXf data(3, points_.size());
    for(size_t i = 0; i < points_.size(); i++)
    {
      data(0, i) = points_[i].x;
      data(1, i) = points_[i].y;
      data(2, i) = points_[i].z;
    }
    data.colwise() -= centroid_;

    // 3. Compute the covariance matrix
    Eigen::Matrix3f cov = data * data.transpose() / static_cast<float>(points_.size());

    // 4. Compute the eigenvectors of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    Eigen::Matrix3f eig_vecs = solver.eigenvectors();

    // 5. Transform the points to the eigenvector space
    Eigen::MatrixXf transformed = eig_vecs.transpose() * data; // each column is a transformed point

    // 6. Determine minimum and maximum values of the transformed points
    Eigen::Vector3f min_pt = transformed.rowwise().minCoeff();
    Eigen::Vector3f max_pt = transformed.rowwise().maxCoeff();

    // 7. Define the 8 corners in the local (PCA) coordinate system.
    std::vector<Eigen::Vector3f> corners_local;
    corners_local.push_back(Eigen::Vector3f(min_pt.x(), min_pt.y(), min_pt.z()));
    corners_local.push_back(Eigen::Vector3f(min_pt.x(), min_pt.y(), max_pt.z()));
    corners_local.push_back(Eigen::Vector3f(min_pt.x(), max_pt.y(), min_pt.z()));
    corners_local.push_back(Eigen::Vector3f(min_pt.x(), max_pt.y(), max_pt.z()));
    corners_local.push_back(Eigen::Vector3f(max_pt.x(), min_pt.y(), min_pt.z()));
    corners_local.push_back(Eigen::Vector3f(max_pt.x(), min_pt.y(), max_pt.z()));
    corners_local.push_back(Eigen::Vector3f(max_pt.x(), max_pt.y(), min_pt.z()));
    corners_local.push_back(Eigen::Vector3f(max_pt.x(), max_pt.y(), max_pt.z()));

    // 8. Transform the corners back to the original coordinate system
    for (const auto & corner_local : corners_local) 
    {
      Eigen::Vector3f world_corner = eig_vecs * corner_local + centroid_;
      bounding_box_.push_back(pcl::PointXYZ(world_corner.x(), world_corner.y(), world_corner.z()));
    }
    bounding_box_computed_ = true;
  }

  void WorldEntity::compute() 
  {
    computeCentroidAndVelocity();
    computeBoundingBox();
  }

  // getters
  Eigen::Vector3f WorldEntity::getCentroid() const
  {
    if (!centroid_velocity_computed_)
    {
      std::cerr << "Centroid and velocity not computed yet!" << std::endl;
      return Eigen::Vector3f::Zero();
    }

    return centroid_;
  }

  Eigen::Vector3f WorldEntity::getVelocity() const 
  { 
    if (!centroid_velocity_computed_)
    {
      std::cerr << "Centroid and velocity not computed yet!" << std::endl;
      return Eigen::Vector3f::Zero();
    }
    return velocity_;
  }

  std::vector<pcl::PointXYZ> WorldEntity::getPoints() const 
  {
    return points_;
  }

  std::vector<pcl::PointXYZ> WorldEntity::getBoundingBox() const 
  { 
    if(!bounding_box_computed_)
    {
      std::cerr << "Bounding box not computed yet!" << std::endl;
      return std::vector<pcl::PointXYZ>();
    }
    return bounding_box_;
  }

  int WorldEntity::getId() const  
  {
    return id_;
  }

  // setters
  void WorldEntity::setId(int id) 
  {
    id_ = id;
  }

} // namespace object_detector
} // namespace perception_pipeline