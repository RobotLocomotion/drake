#pragma once

#include <memory>
#include <vector>

#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitives_common.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/sparsity_matrix.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace monolithic_pick_and_place {

/// This class is the base of an `ActionPrimitive`. `ActionPrimitive` in the
/// sense of the pick-and-place demo are systems that can be used to
/// independently perform some kind of action on the simulated robot / gripper
/// system. The functionality they offer is to decouple the state transition
/// style logic of high level planners and the implementation level logic of
/// how long an action needs to run, or what kind of LCM message an action
/// must be encoded in etc. Each `ActionPrimitive` can maintain internal state
/// as well as timing information that are task specific. Every
/// `ActionPrimitive` has some default features,
/// as defined by this base class :
/// 1. They have at least 1 internal state variable, the `ActionPrimitiveState`.
/// 2. They have at least one Abstract output port that sends the current
/// `ActionPrimitiveState`.
/// 3. They have a declared periodic unrestricted update.
/// 4. They are not direct feedthrough (child classes must continue to enforce
/// this in their DoExtendedCalcOutput).
class ActionPrimitive : public systems::LeafSystem<double> {
 public:
  /// The constructor to an action primitive.
  /// @param desired_update_interval The update period for the unrestricted
  /// state update.
  /// @param action_primitive_state_index The state index for the abstract
  /// state belonging to this base class (index of the ActionPrimitiveState).
  /// The derived classes need to set this appropriately.
  ActionPrimitive(double desired_update_interval = 0.01,
                  unsigned int action_primitive_state_index = 1);
  // TODO(naveenoid) : Modify constructor and design of this class hierarchy
  // using
  // DeclareAbstractState (#5819).

  /// This gets the abstract output port corresponding to the
  /// `ActionPrimitive` status. Child classes may implement additional getters
  /// as needed.
  const systems::OutputPort<double>& get_status_output_port() const {
    return this->get_output_port(status_output_port_);
  }

  // LeafSystem override.
  std::unique_ptr<systems::AbstractValues> AllocateAbstractState() const final;

  // LeafSystem override.
  void SetDefaultState(const systems::Context<double>& context,
                       systems::State<double>* state) const final;

  // LeafSystem override.
  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const final {
    DoExtendedCalcUnrestrictedUpdate(context, state);
  }

  // This kind of a system is not direct feedthrough.
  bool DoHasDirectFeedthrough(const systems::SparsityMatrix*,
                              int, int) const final {
    return false;
  }

 protected:
  /// Derived class need to implement this. This method is used to set the
  /// default state unique to the class.
  virtual void SetExtendedDefaultState(const systems::Context<double>& context,
                                       systems::State<double>* state) const = 0;

  /// Derived class need to implement this. This method is used to specify the
  /// additional `AbstractState` of the derived class.
  virtual std::vector<std::unique_ptr<systems::AbstractValue>>
  AllocateExtendedAbstractState() const = 0;

  /// Derived class need to implement this. This method can be used to compute
  /// the unrestricted update unique to the derived class.
  virtual void DoExtendedCalcUnrestrictedUpdate(
      const systems::Context<double>& context,
      systems::State<double>* state) const = 0;

  unsigned int get_action_primitive_state_index() const {
    return action_primitive_state_index_;
  }

 private:
  void OutputPrimitiveState(const systems::Context<double>& context,
                            ActionPrimitiveState* output) const;

  const unsigned int action_primitive_state_index_{0};
  int status_output_port_{-1};
  const double update_interval_{0.01};
};

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
