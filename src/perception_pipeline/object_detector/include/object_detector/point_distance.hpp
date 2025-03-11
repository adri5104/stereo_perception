/**
 * @file point_distance.hpp
 * @author Adrian Rieker (adrian.rieker@tum.de)
 * @brief This file contains the WeightedPosVelDistance struct, which is used to compute the distance between two 6D points.
 * 
 *     The distance between two 6D points is computed using the following formula:
 * 
 *    distance = pos_w * pos_dist + vel_w * angle
 *    
 *    where:
 *    - pos_dist is the Euclidean distance between the position components (x, y, z) of the points.
 *    - angle is the angle between the velocity components (vx, vy, vz) of the points.
 *    - pos_w is the weight for the position distance.
 *    - vel_w is the weight for the velocity angle.
 *    
 *    The Euclidean distance (pos_dist) is calculated as:
 *    
 *    pos_dist = sqrt((a[0] - b[0])^2 + (a[1] - b[1])^2 + (a[2] - b[2])^2)
 *    
 *    The angle between the velocity components is calculated using 
 *     the dot product and magnitudes of the velocity vectors:
 *    
 *    dot = a[3] * b[3] + a[4] * b[4] + a[5] * b[5]
 *    na = sqrt(a[3]^2 + a[4]^2 + a[5]^2)
 *    nb = sqrt(b[3]^2 + b[4]^2 + b[5]^2)
 *    
 *    angle = acos(dot / (na * nb))
 *    
 *    If either velocity vector has a near-zero magnitude, the angle is set to π (180 degrees).
 */
 

#ifndef OBJECT_DETECTOR_POINT_DISTANCE_HPP
#define OBJECT_DETECTOR_POINT_DISTANCE_HPP

#include <mlpack/core.hpp>
#include <armadillo>
#include <cmath>

/**
  * @brief Struct to compute the distance between two 6D points.
 */
struct WeightedPosVelDistance
{
  double pos_w;
  double vel_w;

  WeightedPosVelDistance() : pos_w(1.0), vel_w(1.0) {}
  WeightedPosVelDistance(double pw, double vw)
    : pos_w(pw), vel_w(vw) {}

  double Evaluate(const arma::vec& a, const arma::vec& b) const
  {
    // Expected format: [x, y, z, vx, vy, vz]
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    double dz = a[2] - b[2];
    double pos_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    double dot = a[3] * b[3] + a[4] * b[4] + a[5] * b[5];
    double na = std::sqrt(a[3]*a[3] + a[4]*a[4] + a[5]*a[5]);
    double nb = std::sqrt(b[3]*b[3] + b[4]*b[4] + b[5]*b[5]);

    double angle = M_PI; // default if one velocity is nearly zero.
    if(na > 1e-8 && nb > 1e-8)
    {
      double c = dot / (na * nb);
      if(c > 1.0) c = 1.0;
      if(c < -1.0) c = -1.0;
      angle = std::acos(c);
    }
    return pos_w * pos_dist + vel_w * angle;
  }
};

#endif // OBJECT_DETECTOR_POINT_DISTANCE_HPP  
