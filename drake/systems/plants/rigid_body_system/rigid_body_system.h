#pragma once

#include <memory>
#include <string>

#include "drake/drake_rbs_export.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace tinyxml2 {
class XMLElement;
}

namespace drake {
namespace systems {

// TODO(amcastro-tri): Make RBS inherit from Diagram<T> once #3215 is solved.
// RigidBodySystem is a diagram containing the multibody dynamics engine system
// connected to forcing systems and sensor systems.
template<typename T>
class DRAKE_RBS_EXPORT RigidBodySystem : public LeafSystem<T> {
 public:
  /// Doc.
  RigidBodySystem(std::unique_ptr<const RigidBodyTree> mbd_world);

  virtual ~RigidBodySystem() override;

  const RigidBodyTree& get_multibody_world() const;

  int get_num_generalized_positions() const;

  int get_num_generalized_velocities() const;

  int get_num_states() const;

  int get_num_actuators() const;

  int get_num_inputs() const;

  int get_num_outputs() const;

  void set_position(ContextBase<T>* context, int position_index, T position) const {
    context->get_mutable_state()->continuous_state->get_mutable_generalized_position()->SetAtIndex(position_index, position);
  }

  void set_velocity(ContextBase<T>* context, int velocity_index, T position) const {
    context->get_mutable_state()->continuous_state->get_mutable_generalized_velocity()->SetAtIndex(velocity_index, position);
  }

  /// Sets the state in @p context so that generalized positions and velocities
  /// are zero. For quaternion based joints the quaternion is set to be the
  /// identity or zero rotation quaternion.
  void ObtainZeroConfiguration(ContextBase<T>* context) const {
    VectorX<T> x0 = VectorX<T>::Zero(get_num_states());
    x0.head(get_num_generalized_positions()) =
        mbd_world_->getZeroConfiguration();
    context->get_mutable_xc()->SetFromVector(x0);
  }

 private:
  bool has_any_direct_feedthrough() const override;

  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;

  void EvalTimeDerivatives(const ContextBase<T>& context,
                           ContinuousState<T>* derivatives) const override;

  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

  // some parameters defining the contact.
  // TODO(amcastro-tri): Implement contact materials for the RBT engine.
  T penetration_stiffness_{150.0};  // An arbitrarily large number.
  T penetration_damping_{0};
  T friction_coefficient_{0};

 private:
  std::unique_ptr<const RigidBodyTree> mbd_world_;
};

}  // namespace systems
}  // namespace drake
