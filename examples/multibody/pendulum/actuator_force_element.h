#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/force_element.h"
#include "drake/multibody/multibody_tree/joints/joint.h"
#include "drake/multibody/multibody_tree/multibody_tree_forcing.h"

namespace drake {
namespace examples {
namespace multibody {
namespace pendulum {

// Forward declaration.
template<typename T> class PendulumPlant;

/// This ForceElement allows modeling the effect of a uniform gravity field as
/// felt by bodies on the surface of the Earth.
/// This gravity fields acts on all bodies in the MultibodyTree model.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class ActuatorForceElement : public multibody::ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ActuatorForceElement)

  /// Constructs a custom forcing element from the provided callback.
  // TODO(amcastro-tri): Update constructor to also take callbacks for
  // calculating energy budgets.
  explicit ActuatorForceElement(
      const PendulumPlant<T>* plant, const multibody::Joint<T>* joint);

  T CalcPotentialEnergy(
      const multibody::MultibodyTreeContext<T>& context,
      const multibody::PositionKinematicsCache<T>& pc) const final;

  T CalcConservativePower(
      const multibody::MultibodyTreeContext<T>& context,
      const multibody::PositionKinematicsCache<T>& pc,
      const multibody::VelocityKinematicsCache<T>& vc) const final;

  T CalcNonConservativePower(
      const multibody::MultibodyTreeContext<T>& context,
      const multibody::PositionKinematicsCache<T>& pc,
      const multibody::VelocityKinematicsCache<T>& vc) const final;

 protected:
  void DoCalcAndAddForceContribution(
      const multibody::MultibodyTreeContext<T>& context,
      const multibody::PositionKinematicsCache<T>& pc,
      const multibody::VelocityKinematicsCache<T>& vc,
      std::vector<SpatialForce<T>>* F_B_W,
      EigenPtr<VectorX<T>> tau) const final;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  const PendulumPlant<T>* plant_{nullptr};
  const multibody::Joint<T>* joint_;
};

}  // namespace pendulum
}  // namespace multibody
}  // namespace examples
}  // namespace drake