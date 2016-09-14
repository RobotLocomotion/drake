#pragma once

#include <memory>
#include <string>

#include "drake/drake_rbp_export.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace tinyxml2 {
class XMLElement;
}

namespace drake {
namespace systems {

/// This class provides a System interface around a multibody dynamics model
/// of the world represented by a RigidBodyTree.
///
/// <B>%System input</B>: A %RigidBodyPlant has a vector valued input port for
/// external actuation with size equal to the number of RigidBodyActuator's in
/// the RigidBodyTree. Each RigidBodyActuator maps to a single DOF joint
/// (currently actuation cannot be applied to multiple DOF's joints). The units
/// of the actuation are the same as the units of the generalized force on the
/// joint. In addition, actuators allow for a gear box reduction factor and for
/// actuation limits which are only used by controllers; the RigidBodyPlant
/// does not apply these limits. The gear box factor effectively is a
/// multiplier on the input actuation to the RigidBodyPlant.
///
/// <B>%System output</B>: A %RigidBodyPlant outputs the state of the system in
/// a vector valued port.
///
/// The multibody model consists of a set of rigid bodies connected through
/// joints in a tree structure. Bodies may have a collision model in which case
/// collisions are considered. In addition the model may contain loop
/// constraints described by RigidBodyLoop's in the multibody model. Even though
/// loop constraints are a particular case of holonomic constrants, general
/// holonomic constrants are not yet supported.
///
/// The system dynamics is given by the set of multibody equations written in
/// generalized coordinates including loop joints as a set of holonomic
/// constraints like so:
/// <pre>
///   H(q) * vdot + C(q, v) = tau_actuators + tau_constraints.
/// </pre>
/// where `q` is the vector of generalized coordinates (or positions), `v` is
/// the vector of generalized velocities, `C` includes the velocity-dependent
/// Coriolis and gyroscopic forces, `tau_actuators` is the vector of externally
/// applied generalized forces and finally `tau_constraints` is the vector of
/// generalized forces due to constraints.
/// `tau_constraints` is computed as
/// <pre>
///   tau_constraints = -J^T * lambda
/// </pre>
/// where `lambda` is the vector of Lagrange multipliers representing the
/// constraint forces and `J` is the constraint Jacobian.
/// The time derivative of the generalized coordinates is then obtained from the
/// generalized velocities as
/// <pre>
///   qdot = N(q) * v
/// </pre>
/// where `N(q)` is a transformation matrix only dependent on the positions.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class DRAKE_RBP_EXPORT RigidBodyPlant : public LeafSystem<T> {
 public:
  /// Instantiates a %RigidBodyPlant from a Multi-Body Dynamics (MBD) model of
  /// the world in @p tree.
  explicit RigidBodyPlant(std::unique_ptr<const RigidBodyTree> tree);

  ~RigidBodyPlant() override;

  /// Returns a constant reference to the multibody dynamics model
  /// of the world.
  const RigidBodyTree& get_multibody_world() const;

  /// Returns the number of generalized coordinates of the model.
  int get_num_positions() const;

  /// Returns the number of generalized velocities of the model.
  int get_num_velocities() const;

  /// Returns the size of the continuous state of the system which equals
  /// get_num_positions() plus get_num_velocities().
  int get_num_states() const;

  /// Returns the number of actuators.
  int get_num_actuators() const;

  /// Returns the size of the input vector to the system. This equals the
  /// number of actuators.
  int get_input_size() const;

  /// Returns the size of the output vector of the system. This equals the size
  /// of the continuous state vector.
  int get_output_size() const;

  /// Sets the generalized coordinate @p position_index to the value
  /// @p position.
  void set_position(Context<T>* context,
                    int position_index, T position) const;

  /// Sets the generalized velocity @p velocity_index to the value
  /// @p velocity.
  void set_velocity(Context<T>* context,
                    int velocity_index, T velocity) const;

  /// Sets the continuous state vector of the system to be @p x.
  void set_state_vector(Context<T>* context,
                        const Eigen::Ref<const VectorX<T>> x) const;

  /// Sets the state in @p context so that generalized positions and velocities
  /// are zero. For quaternion based joints the quaternion is set to be the
  /// identity (or equivalently a zero rotation).
  void SetZeroConfiguration(Context<T> *context) const {
    VectorX<T> x0 = VectorX<T>::Zero(get_num_states());
    x0.head(get_num_positions()) = tree_->getZeroConfiguration();
    context->get_mutable_continuous_state()->SetFromVector(x0);
  }

  // System<T> overrides.
  bool has_any_direct_feedthrough() const override;
  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override;
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

  void MapVelocityToConfigurationDerivatives(
      const Context<T>& context, const VectorBase<T>& generalized_velocity,
      VectorBase<T>* positions_derivative) const override;

  // System<T> overrides to track energy conservation.
  // TODO(amcastro-tri): provide proper implementations for these methods to
  // track energy conservation.
  // TODO(amcastro-tri): provide a method to track applied actuator power.
  T EvalPotentialEnergy(const Context<T>& context) const override {
    return T(NAN);
  }

  T EvalKineticEnergy(const Context<T>& context) const override {
    return T(NAN);
  }

  T EvalConservativePower(const Context<T>& context) const override {
    return T(NAN);
  }

  T EvalNonConservativePower(const Context<T>& context) const override {
    return T(NAN);
  }

 protected:
  // LeafSystem<T> override
  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;

 private:
  // Some parameters defining the contact.
  // TODO(amcastro-tri): Implement contact materials for the RBT engine.
  T penetration_stiffness_{150.0};  // An arbitrarily large number.
  T penetration_damping_{0};
  T friction_coefficient_{0};

  std::unique_ptr<const RigidBodyTree> tree_;
};

}  // namespace systems
}  // namespace drake
