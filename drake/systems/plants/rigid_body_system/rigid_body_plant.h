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

template<typename T>
class DRAKE_RBS_EXPORT RigidBodyPlant : public LeafSystem<T> {
 public:
  /// Instantiates a %RigidBodyPlant from a RigidBodyTree model of the world.
  /// A %RigidBodyPlant has a vector valued input port for external actuation
  /// with size equal to the number of actuators in the RigidBodyTree.
  /// A %RigidBodyPlant outputs the state of the system in a vector valued port.
  explicit RigidBodyPlant(std::unique_ptr<const RigidBodyTree> mbd_world);

  ~RigidBodyPlant() override;

  /// Returns a constant reference to the multibody dynamics model
  /// of the world.
  const RigidBodyTree& get_multibody_world() const;

  /// Returns the number of generalized coordinates of the model.
  int get_num_positions() const;

  /// Returns the number of generalized velocities of the model.
  int get_num_velocities() const;

  /// Returns the size of the continuous state of the system.
  int get_num_states() const;

  /// Returns the number of actuators.
  int get_num_actuators() const;

  /// Returns the size of the input vector to the system. This equals the
  /// number of actuators.
  int get_num_inputs() const;

  /// Returns the size of the output vector of the system. This equals the size
  /// of the continuous state vector.
  int get_num_outputs() const;

  /// Sets the generalized coordinate @p position_index to the value
  /// @p position.
  void set_position(ContextBase<T>* context,
                    int position_index, T position) const;

  /// Sets the generalized velocity @p velocity_index to the value
  /// @p velocity.
  void set_velocity(ContextBase<T>* context,
                    int velocity_index, T velocity) const;

  /// Sets the continuous state vector of the system to be @p x.
  void set_state_vector(ContextBase<T>* context,
                        const Eigen::Ref<const VectorX<T>> x) const;

  /// Sets the state in @p context so that generalized positions and velocities
  /// are zero. For quaternion based joints the quaternion is set to be the
  /// identity (or equivalently a zero rotation).
  void ObtainZeroConfiguration(ContextBase<T>* context) const {
    VectorX<T> x0 = VectorX<T>::Zero(get_num_states());
    x0.head(get_num_positions()) =
        mbd_world_->getZeroConfiguration();
    context->get_mutable_xc()->SetFromVector(x0);
  }

  // System<T> overrides
  bool has_any_direct_feedthrough() const override;
  void EvalTimeDerivatives(const ContextBase<T>& context,
                           ContinuousState<T>* derivatives) const override;
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

 protected:
  // LeafSystem<T> override
  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;

 private:
  // Some parameters defining the contact.
  // TODO(amcastro-tri): Implement contact materials for the RBT engine.
  T penetration_stiffness_{150.0};  // An arbitrarily large number.
  T penetration_damping_{0};
  T friction_coefficient_{0};

  std::unique_ptr<const RigidBodyTree> mbd_world_;
};

}  // namespace systems
}  // namespace drake
