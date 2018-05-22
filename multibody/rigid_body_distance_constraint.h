#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Dense>

/// Defines a "relative distance constraint" that models a constraint between
/// points on two different rigid bodies. The relative distance constraint is
/// specified by two indexes for two different `RigidBody` objects, two
/// `Vector3d` objects representing a point on each `RigidBody` object, and a
/// distance between these two points.
///
class RigidBodyDistCon {
 public:
  /// The constructor
  ///
  /// @param[in] bodyA_ind Index of first Rigid Body involved in constraint
  ///
  /// @param[in] pointA Point on first Rigid Body to measure distance from
  ///
  /// @param[in] bodyB_ind Index of second Rigid Body involved in constraint
  ///
  /// @param[in] pointB Point on second Rigid Body to measure distance from
  ///
  /// @param[in] dist Distance the two points should be separated by
  RigidBodyDistCon(int bodyA_ind, const Eigen::Vector3d& pointA,
                  int bodyB_ind, const Eigen::Vector3d& pointB,
                  double dist);

  const int from_body, to_body;
  const Eigen::Vector3d from_point, to_point;
  const double distance;

  // friend std::ostream& operator<<(std::ostream& os,
  //                                 const RigidBodyDistCon& obj);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
