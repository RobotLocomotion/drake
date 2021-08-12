#pragma once

#include <memory>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace test {

// TODO(eric.cousineau): Change this to not inherit from `MultibodyPlant`.
/// This plant models the free motion of a torque free body in space.
/// This body is axially symmetric with rotational inertia about its axis of
/// revolution J and with a rotational inertia I about any axis perpendicular to
/// its axis of revolution. This particular case has a nice closed form
/// analytical solution which we have implemented in
/// drake::multibody::benchmarks::free_body::FreeBody.
///
/// @tparam_nonsymbolic_scalar
template<typename T>
class AxiallySymmetricFreeBodyPlant final : public MultibodyPlant<T> {
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
  /// @param time_step [optional]
  ///   See MultibodyPlant::MultibodyPlant.
  AxiallySymmetricFreeBodyPlant(double mass, double I, double J, double g,
                                double time_step = 0.0);

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
  math::RigidTransform<T> CalcPoseInWorldFrame(
      const systems::Context<T>& context) const;

  /// Calculates V_WB, `this` body B's spatial velocity in the world frame W.
  /// @param[in] context Contains the state of the model.
  /// @retval V_WB_W `this` free-body B's spatial velocity in the world
  /// frame W, expressed in W (for point Bo, the body frame's origin).
  /// @param[in] context Contains the state of the model.
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
  const internal::MultibodyTree<T>& tree() const {
    return internal::GetInternalTree(*this);
  }

  double mass_{0};
  double I_{0};
  double J_{0};
  double g_{0};
  const RigidBody<T>* body_{nullptr};
};

}  // namespace test
}  // namespace multibody
}  // namespace drake
