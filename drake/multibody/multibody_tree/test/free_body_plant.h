#pragma once

#include <memory>

#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/space_xyz_mobilizer.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {

/// This plant models the rotational motion of a torque free body in space.
/// This body is axially symmetric with rotational inertia about its axis of
/// revolution J and with a rotational inertia I about any axis perpendicular to
/// its axis of revolution.
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
class FreeRotatingBodyPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FreeRotatingBodyPlant)

  /// Constructor from known rotational inertia values.
  /// Rotational inertia values have units of kg⋅m².
  /// @param I
  ///   rotational inertia about any axis perpendicular to the axis of
  ///   revolution of the body.
  /// @param J
  ///   rotational inertia about the axis of revolution of the body.
  FreeRotatingBodyPlant(double I, double J);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit FreeRotatingBodyPlant(const FreeRotatingBodyPlant<U>&);

  /// Stores in `context` the value of the angular velocity `w_WB` of the body
  /// in the world frame W.
  void set_angular_velocity(
      systems::Context<T>* context, const Vector3<T>& w_WB) const;

  /// Computes the pose `X_WB` of the body in the world frame.
  Isometry3<T> CalcPoseInWorldFrame(
      const systems::Context<T>& context) const;

  /// Computes the spatial velocity `V_WB` of the body in the world frame.
  SpatialVelocity<T> CalcSpatialVelocityInWorldFrame(
      const systems::Context<T>& context) const;

 private:
  // Override of context construction so that we can delegate it to
  // MultibodyTree.
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T> &context,
      systems::ContinuousState<T> *derivatives) const override;

  void DoMapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      systems::VectorBase<T>* generalized_velocity) const override;

  void DoMapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      systems::VectorBase<T>* qdot) const override;

  // Helper method to build the MultibodyTree model for this plant.
  void BuildMultibodyTreeModel();

  double I_{0};
  double J_{0};
  MultibodyTree<T> model_;
  const RigidBody<T>* body_{nullptr};
  const SpaceXYZMobilizer<T>* mobilizer_{nullptr};
};

}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
