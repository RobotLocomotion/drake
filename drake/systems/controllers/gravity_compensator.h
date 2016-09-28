#pragma once

#include <memory>

#include "drake/drakeSystemControllers_export.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {

/// A gravity compensator block with input as the partial state of a
/// `RigidBodyPlant` `[q]` and output `y = G(q)`, where, for a given
/// `RigidBodyTree`, `G(q)` is a vector of generalised gravity forces
/// corresponding to a given joint configuration `q`.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class DRAKESYSTEMCONTROLLERS_EXPORT GravityCompensator : public LeafSystem<T> {
 public:
  /// Constructs a gravity compensator (corresponding to a `RigidBodyTree`)
  /// with the dimension of the input and output port equaling the dimension
  /// of the Degrees of Freedom of the `RigidBodyTree` (positions of a
  /// `RigidBodyPlant`)
  /// @param rigid_body_tree_ a constant reference to a `RigidBodyTree` object
  /// corresponding to the `RigidBodyPlant` that is to be controlled.
  explicit GravityCompensator(const RigidBodyTree& rigid_body_tree);

  /// Sets the output port value to the generalised gravity forces
  /// corresponding to a joint configuration as specified in the input.
  /// If the number of connected input or output ports differs from one or the
  /// input ports are not of size length_, a std::runtime_error will be thrown.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  const RigidBodyTree& rigid_body_tree_;
};

}  // namespace systems
}  // namespace drake
