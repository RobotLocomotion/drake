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
class RigidBodyDistanceConstraint {
 public:
  /// The constructor
  ///
  /// @param[in] bodyA_index Index of first Rigid Body involved in constraint.
  ///
  /// @param[in] pointA Point on first Rigid Body to measure distance from.
  ///
  /// @param[in] bodyB_index Index of second Rigid Body involved in constraint.
  ///
  /// @param[in] pointB Point on second Rigid Body to measure distance from.
  ///
  /// @param[in] dist Distance the two points should be separated by.
  RigidBodyDistanceConstraint(int bodyA_index, const Eigen::Vector3d& pointA,
                              int bodyB_index, const Eigen::Vector3d& pointB,
                              double distance_);

  const int from_body, to_body;
  const Eigen::Vector3d from_point, to_point;
  const double distance;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
