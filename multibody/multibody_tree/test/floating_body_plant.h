#pragma once

#include <memory>

#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {

/// This plant models the free motion of a torque free body in space.
/// This body is axially symmetric with rotational inertia about its axis of
/// revolution J and with a rotational inertia I about any axis perpendicular to
/// its axis of revolution. This particular case has a nice closed form
/// analytical solution which we have implemented in
/// drake::multibody::benchmarks::free_body::FreeBody.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template<typename T>
class AxiallySymmetricFreeBodyPlant final :
    public multibody_plant::MultibodyPlant<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AxiallySymmetricFreeBodyPlant)

  /// Constructor from known rotational inertia values.
  /// Rotational inertia values have units of kg⋅m².
  /// @param mass
  ///   The mass of the body.
  /// @param I
  ///   rotational inertia about any axis perpendicular to the axis of
  ///   revolution of the body.
  /// @param J
  ///   rotational inertia about the axis of revolution of the body.
  /// @param g
  ///   Acceleration of gravity. In this model if `g > 0` the gravity vector
  ///   points downward in the z-axis direction.
  AxiallySymmetricFreeBodyPlant(double mass, double I, double J, double g);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit AxiallySymmetricFreeBodyPlant(
      const AxiallySymmetricFreeBodyPlant<U>&);

  /// Sets `state` to a default value corresponding to a configuration in which
  /// the free body frame B is coincident with the world frame W and the angular
  /// and translational velocities have a value as returned by
  /// get_default_initial_angular_velocity() and
  /// get_default_initial_translational_velocity(), respectively.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

  /// Returns the angular velocity `w_WB` stored in `context` of the free body B
  /// in the world frame W.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const;

  /// Returns the translational velocity `v_WB` stored in `context` of the free
  /// body B in the world frame W.
  Vector3<T> get_translational_velocity(
      const systems::Context<T>& context) const;

  /// Computes the pose `X_WB` of the body in the world frame.
  Isometry3<T> CalcPoseInWorldFrame(
      const systems::Context<T>& context) const;

  /// Computes the spatial velocity `V_WB` of the body in the world frame.
  SpatialVelocity<T> CalcSpatialVelocityInWorldFrame(
      const systems::Context<T>& context) const;

  /// Returns the default value of the angular velocity set by default by
  /// SetDefaultState(). Currently a non-zero value.
  static Vector3<double> get_default_initial_angular_velocity();

  /// Returns the default value of the translational velocity set by default by
  /// SetDefaultState(). Currently a non-zero value.
  static Vector3<double> get_default_initial_translational_velocity();

  /// Returns a constant reference to the free body model of this plant.
  const RigidBody<T>& body() const { return *body_; }

 private:
  double mass_{0};
  double I_{0};
  double J_{0};
  double g_{0};
  const RigidBody<T>* body_{nullptr};
};

}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
