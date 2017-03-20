#pragma once

#include <memory>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// This system essentially is a double pendulum consisting of two links.
/// Link 1 is connected to the world by a "shoulder" revolute joint parametrized
/// by angle theta1 and Link 2 is connected to Link 1 by an "elbow" revolute
/// joint parametrized by angle theta2.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template <typename T>
class Acrobot {
 public:
  /// Creates an acrobot model in a plane passing through the world's origin
  /// and normal to @p normal. Vector @p up defines the upwards direction on
  /// this plane. Both @p normal and @p up are expressed in the world's frame.
  /// Essentially the two dimensional equations of the acrobot are described
  /// in a model frame D within a x-y plane with y the vertical direction
  /// and graviting pointing downwards.
  /// Thefore the axes defining the model frame D are:
  /// z_W = normal_W.normalized();
  /// y_W = (up - up.dot(z_W) * z_W).normalized();
  /// x_W = y_W.cross(z_W);
  Acrobot(const Vector3<T>& normal, const Vector3<T>& up);

  Matrix2<T> CalcMassMatrix(const T& theta1, const T& theta2) const;

  /// Computes the pose of the center of mass of link 1 measured and expressed
  /// in the world frame.
  /// @param theta1 The shoulder angle in radians.
  /// @param theta2 The elbow angle in radians.
  /// @returns X_WL1 the pose of link 1 measured and expressed in the world
  /// frame.
  Isometry3<T> CalcLink1PoseInWorldFrame(
      const T& theta1, const T& theta2) const;

  /// Computes the pose of the center of mass of link 2 measured and expressed
  /// in the world frame.
  /// @param theta1 The shoulder angle in radians.
  /// @param theta2 The elbow angle in radians.
  /// @returns X_WL2 the pose of link 2 measured and expressed in the world
  /// frame.
  Isometry3<T> CalcLink2PoseInWorldFrame(
      const T& theta1, const T& theta2) const;

  /// Computes the spatial velocity of the center of mass of link 1 expressed
  /// in the world frame.
  /// @param theta1 The shoulder angle in radians.
  /// @param theta2 The elbow angle in radians.
  /// @param theta1dot The shoulder angular velocity in radians per second.
  /// @param theta2dot The elbow angular velocity in radians per second.
  /// @returns V_WL1_W the spatial velocity of the center of mass of link 1 with
  /// respect to the world and expressed in the world frame.
  Vector6<T> CalcLink1SpatialVelocityInWorldFrame(
      const T& theta1, const T& theta2,
      const T& theta1dot, const T& theta2dot) const;

  /// Computes the spatial velocity of the center of mass of link 2 expressed
  /// in the world frame.
  /// @param theta1 The shoulder angle in radians.
  /// @param theta2 The elbow angle in radians.
  /// @param theta1dot The shoulder angular velocity in radians per second.
  /// @param theta2dot The elbow angular velocity in radians per second.
  /// @returns V_WL2_W the spatial velocity of the center of mass of link 2 with
  /// respect to the world and expressed in the world frame.
  Vector6<T> CalcLink2SpatialVelocityInWorldFrame(
      const T& theta1, const T& theta2,
      const T& theta1dot, const T& theta2dot) const;

 private:
  const T
      m1{1.0},    // Mass of link 1 (kg).
      m2{1.0},    // Mass of link 2 (kg).
      l1{1.0},    // Length of link 1 (m).
      l2{1.0},    // Length of link 2 (m).
      lc1{0.5},   // Vertical distance from shoulder joint to center of mass of
                  // link 1 (m).
      lc2{0.5},   // Vertical distance from elbox joint to center of mass of
                  // link 2 (m).
      Ic1{.083},  // Inertia of link 1 about the center of mass of link 1
                  // (kg*m^2).
      Ic2{.33},   // Inertia of link 2 about the center of mass of link 2
                  // (kg*m^2).
      b1{0.1},    // Damping coefficient of the shoulder joint (kg*m^2/s).
      b2{0.1},    // Damping coefficient of the elbow joint (kg*m^2/s).
      g{9.81};    // Gravitational constant (m/s^2).

  // Transformation from the model frame D to the world frame W.
  Isometry3<T> X_WD_{Isometry3<T>::Identity()};
};

}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
