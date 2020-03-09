#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace benchmarks {

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// This system essentially is a double pendulum consisting of two links.
/// Link 1 is connected to the world by a "shoulder" revolute joint
/// parameterized by angle theta1 and Link 2 is connected to Link 1 by an
/// "elbow" revolute joint parameterized by angle theta2.
///
/// @tparam_nonsymbolic_scalar
template <typename T>
class Acrobot {
 public:
  /// Creates an acrobot model in a plane passing through the world's origin
  /// and normal to @p normal. Vector @p up defines the upwards direction on
  /// this plane. Both @p normal and @p up are expressed in the world's frame.
  /// Essentially the two dimensional equations of the acrobot are described
  /// in a model frame D within a x-y plane with y the vertical direction
  /// and gravity pointing downwards.
  /// Therefore the axes defining the model frame D are: <pre>
  ///   z_W = normal_W.normalized()
  ///   y_W = (up - up.dot(z_W) * z_W).normalized()
  ///   x_W = y_W.cross(z_W)
  /// </pre>
  /// The remaining arguments define the properties of the double pendulum
  /// system:
  ///
  /// - m1: mass of the first link.
  /// - m2: mass of the second link.
  /// - l1: length of the first link.
  /// - l2: length of the second link.
  /// - lc1: length from the shoulder to the center of mass of the first link.
  /// - lc2: length from the elbow to the center of mass of the second link.
  /// - Ic1: moment of inertia about the center of mass for the first link.
  /// - Ic2: moment of inertia about the center of mass for the second link.
  /// - b1: damping coefficient of the shoulder joint.
  /// - b2: damping coefficient of the elbow joint.
  /// - g: acceleration of gavity.
  Acrobot(const Vector3<T>& normal, const Vector3<T>& up,
          double m1 = 1.0,
          double m2 = 1.0,
          double l1 = 1.0,
          double l2 = 1.0,
          double lc1 = 0.5,
          double lc2 = 0.5,
          double Ic1 = .083,
          double Ic2 = .33,
          double b1 = 0.1,
          double b2 = 0.1,
          double g = 9.81);

  /// Computes the mass matrix `H(q)` for the double pendulum system. It turns
  /// out that for this system the mass matrix is independent of the shoulder
  /// angle `theta1`.
  Matrix2<T> CalcMassMatrix(const T& theta2) const;

  /// Computes the bias term `C(q, v) * v` containing Coriolis and gyroscopic
  /// effects as a function of the state of the pendulum.
  Vector2<T> CalcCoriolisVector(const T& theta1, const T& theta2,
                                const T& theta1dot, const T& theta2dot) const;

  /// Computes the effective joint-space torques induced by gravity `tau_g(q)`
  /// containing the effect of gravity as a function of the configuration of
  /// the pendulum.
  /// Unlike http://underactuated.mit.edu/underactuated.html?chapter=3, cited in
  /// this class's documentation, we define `tau_g(q)` to be on the right hand
  /// side of the equations of motion, that is, `Mv̇ + C(q, v)v = tau_g(q)`.
  Vector2<T> CalcGravityVector(const T& theta1, const T& theta2) const;

  /// Computes the pose of the center of mass of link 1 measured and expressed
  /// in the world frame.
  /// @param theta1 The shoulder angle in radians.
  /// @param theta2 The elbow angle in radians.
  /// @returns X_WL1 the pose of link 1 measured and expressed in the world
  /// frame.
  math::RigidTransform<T> CalcLink1PoseInWorldFrame(const T& theta1) const;

  /// Computes the pose of the center of mass of link 2 measured and expressed
  /// in the world frame.
  /// @param theta1 The shoulder angle in radians.
  /// @param theta2 The elbow angle in radians.
  /// @returns X_WL2 the pose of link 2 measured and expressed in the world
  /// frame.
  math::RigidTransform<T> CalcLink2PoseInWorldFrame(
      const T& theta1, const T& theta2) const;

  /// Computes the pose of the elbow outboard frame `Eo` in the world frame W.
  /// @param theta1 The shoulder angle in radians.
  /// @param theta2 The elbow angle in radians.
  /// @returns X_WEo the pose of the elbow frame Eo in the world frame W.
  math::RigidTransform<T> CalcElbowOutboardFramePoseInWorldFrame(
      const T& theta1, const T& theta2) const;

  /// Computes the spatial velocity of the center of mass of link 1 expressed
  /// in the world frame.
  /// @param theta1 The shoulder angle in radians.
  /// @param theta1dot The shoulder angular velocity in radians per second.
  /// @returns V_WL1_W the spatial velocity of the center of mass of link 1 with
  /// respect to the world and expressed in the world frame.
  Vector6<T> CalcLink1SpatialVelocityInWorldFrame(
      const T& theta1, const T& theta1dot) const;

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

  /// Computes the spatial acceleration of the center of mass of link 1
  /// expressed in the world frame.
  /// @param theta1
  ///   The shoulder angle in radians.
  /// @param theta1dot
  ///   The shoulder angular velocity in radians per second.
  /// @param theta1dotdot
  ///   The elbow angular acceleration in radians per second squared.
  /// @retval A_WL1_W
  ///   the spatial acceleration of the center of mass of link 1 with respect to
  ///   the world and expressed in the world frame.
  Vector6<T> CalcLink1SpatialAccelerationInWorldFrame(
      const T& theta1, const T& theta1dot, const T& theta1dotdot) const;

  /// Computes the spatial acceleration of the center of mass of link 2
  /// expressed in the world frame.
  /// @param theta1
  ///   The shoulder angle in radians.
  /// @param theta2
  ///   The elbow angle in radians.
  /// @param theta1dot
  ///   The shoulder angular velocity in radians per second.
  /// @param theta2dot
  ///   The elbow angular velocity in radians per second.
  /// @param theta1dotdot
  ///   The shoulder angular acceleration in radians per second squared.
  /// @param theta2dotdot
  ///   The elbow angular acceleration in radians per second squared.
  /// @retval A_WL2_W
  ///   the spatial acceleration of the center of mass of link 2 with respect to
  ///   the world and expressed in the world frame.
  Vector6<T> CalcLink2SpatialAccelerationInWorldFrame(
      const T& theta1, const T& theta2,
      const T& theta1dot, const T& theta2dot,
      const T& theta1dotdot, const T& theta2dotdot) const;

  /// Computes the total potential energy due to gravity of the acrobot system
  /// for the state given by angles `theta1` and `theta2`.
  /// The zero potential energy is defined for `y = 0`.
  T CalcPotentialEnergy(const T& theta1, const T& theta2) const;

 private:
  const T
      m1_{1.0},    // Mass of link 1 (kg).
      m2_{1.0},    // Mass of link 2 (kg).
      l1_{1.0},    // Length of link 1 (m).
      l2_{1.0},    // Length of link 2 (m).
      lc1_{0.5},   // Vertical distance from shoulder joint to center of mass of
                   // link 1 (m).
      lc2_{0.5},   // Vertical distance from elbox joint to center of mass of
                   // link 2 (m).
      Ic1_{.083},  // Inertia of link 1 about the center of mass of link 1
                   // (kg*m^2).
      Ic2_{.33},   // Inertia of link 2 about the center of mass of link 2
                   // (kg*m^2).
      b1_{0.1},    // Damping coefficient of the shoulder joint (kg*m^2/s).
      b2_{0.1},    // Damping coefficient of the elbow joint (kg*m^2/s).
      g_{9.81};    // Gravitational constant (m/s^2).

  // Transformation from the model frame D to the world frame W.
  math::RigidTransform<T> X_WD_;  // Default is identity transform.
};

}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
