#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Dense>

/// Defines a "relative distance constraint" that models a constraint between
/// points on two different rigid bodies. The relative distance constraint is
/// specified by the indexes of two different `RigidBody` objects (bodyA and
/// body B), two `Vector3d` objects representing a point relative to the origin
/// on each `RigidBody` object (r_AP and r_BQ), and a distance between these
/// two points (distance).
///
class RigidBodyDistanceConstraint {
 public:
  /// The constructor
  ///
  /// @param[in] bodyA_index Index of first RigidBody involved in constraint.
  ///
  /// @param[in] r_AP Point on first RigidBody to measure distance from.
  ///
  /// @param[in] bodyB_index Index of second RigidBody involved in constraint.
  ///
  /// @param[in] r_BQ Point on second RigidBody to measure distance from.
  ///
  /// @param[in] distance Distance the two points should be separated by.
  RigidBodyDistanceConstraint(int bodyA_index, const Eigen::Vector3d& r_AP,
                              int bodyB_index, const Eigen::Vector3d& r_BQ,
                              double distance);

  const int bodyA_index_, bodyB_index_;
  const Eigen::Vector3d r_AP_, r_BQ_;
  const double distance_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
