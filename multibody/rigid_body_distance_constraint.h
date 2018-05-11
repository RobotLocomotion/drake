#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Dense>

/// Defines a "loop joint" that models a kinematic loop formed by a chain of
/// rigid bodies and their regular joints. The loop joint is specified by two
/// `RigidBodyFrame` objects that must be attached to two different `RigidBody`
/// objects. The coordinate frames defined by the two `RigidBodyFrame` objects
/// are constrained to have the same origin. The orientations of the two frames
/// are partially constrained based on the axis of rotation (i.e., the two
/// frames are only allowed to rotate relative to each other along the axis of
/// rotation).
///
/// @tparam T The type being integrated. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
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
