#pragma once

#include <Eigen/Dense>

/// Defines a "relative distance constraint" that models a constraint between
/// points on two different rigid bodies. The relative distance constraint is
/// specified by the indexes of two different `RigidBody` objects (bodyA and
/// bodyB), two `Vector3d` objects representing a point relative to the origin
/// on each `RigidBody` object (r_AP and r_BQ), and a distance between these
/// two points (distance).
///
struct RigidBodyDistanceConstraint {
 public:
  /// The constructor
  ///
  /// @param[in] bodyA_index_in Index of first RigidBody involved in constraint.
  ///
  /// @param[in] r_AP_in Point on first RigidBody to measure distance from.
  ///
  /// @param[in] bodyB_index_in Index of second RigidBody involved in
  /// constraint.
  ///
  /// @param[in] r_BQ_in Point on second RigidBody to measure distance from.
  ///
  /// @param[in] distance_in Distance the two points should be separated by.
  RigidBodyDistanceConstraint(int bodyA_index_in,
                              const Eigen::Vector3d& r_AP_in,
                              int bodyB_index_in,
                              const Eigen::Vector3d& r_BQ_in,
                              double distance_in);

  const int bodyA_index;
  const int bodyB_index;
  const Eigen::Vector3d r_AP;
  const Eigen::Vector3d r_BQ;
  const double distance;
};
