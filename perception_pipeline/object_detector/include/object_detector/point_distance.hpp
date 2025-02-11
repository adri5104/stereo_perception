#pragma once

// Include mlpack core before armadillo.
#include <mlpack/core.hpp>
#include <armadillo>
#include <cmath>


struct WeightedPosVelDistance
{
  double pos_w;
  double vel_w;

  // Default constructor (needed by our adapter).
  WeightedPosVelDistance() : pos_w(1.0), vel_w(1.0) {}

  // Custom constructor.
  WeightedPosVelDistance(double pw, double vw)
    : pos_w(pw), vel_w(vw) {}

  // Compute the distance between two 6D points (arma::vec).
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

  
