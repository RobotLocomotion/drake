#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// This class provides a System interface around a multibody dynamics model
/// of the world represented by a RigidBodyTree.
///
/// <B>%System input</B>: A %RigidBodyPlant has a vector valued input
/// port for external actuation with size equal to the number of
/// RigidBodyActuator's in the RigidBodyTree. Each RigidBodyActuator
/// maps to a single DOF joint (currently actuation cannot be applied
/// to multiple DOF's joints). The units of the actuation are the same
/// as the units of the generalized force on the joint. In addition,
/// actuators allow for a gear box reduction factor and for actuation
/// limits which are only used by controllers; the RigidBodyPlant does
/// not apply these limits. The gear box factor effectively is a
/// multiplier on the input actuation to the RigidBodyPlant.
/// Alternately, individual input ports for each model instance which
/// has actuators are provided by model_input_port().  If any port
/// returned by model_input_port() is connected, the input port for
/// the full tree must not be used.
///
/// The %RigidBodyPlant's state consists of a vector containing the generalized
/// positions followed by the generalized velocities of the system.
///
/// <B>%System output</B>:
/// - Port 0: The state vector of the system in a vector valued port.
/// - Port 1: A KinematicsResults class allowing access to the results from
/// kinematics computations for each RigidBody.
/// - Port 2: A ContactsResults class allowing access to the results
/// from contact computations.
/// - Ports 3-N: Individual output ports containing the state
/// (positions + velocities) for each model instance, accessed
/// through model_state_output_port().
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
/// @ingroup rigid_body_systems
template <typename T>
class RigidBodyPlant : public LeafSystem<T> {
 public:
  /// Instantiates a %RigidBodyPlant from a Multi-Body Dynamics (MBD) model of
  /// the world in @p tree.  @p tree must not be `nullptr`.
  // TODO(SeanCurtis-TRI): It appears that the tree has to be "compiled"
  // already.  Confirm/deny and document that result.
  explicit RigidBodyPlant(std::unique_ptr<const RigidBodyTree<T>> tree);

  ~RigidBodyPlant() override;

  // TODO(liang.fok) Remove this method once a more advanced contact modeling
  // framework is available.
  /// Sets the contact parameters.
  void set_contact_parameters(double penetration_stiffness,
      double penetration_damping, double friction_coefficient);

  /// Returns a constant reference to the multibody dynamics model
  /// of the world.
  const RigidBodyTree<T>& get_rigid_body_tree() const;

  /// Returns the number of bodies in the world.
  int get_num_bodies() const;

  /// Returns the number of generalized coordinates of the model.
  int get_num_positions() const;

  /// Returns the number of generalized coordinates for a specific
  /// model instance.
  int get_num_positions(int model_instance_id) const;

  /// Returns the number of generalized velocities of the model.
  int get_num_velocities() const;

  /// Returns the number of generalized velocities for a specific
  /// model instance.
  int get_num_velocities(int model_instance_id) const;

  /// Returns the size of the continuous state of the system which equals
  /// get_num_positions() plus get_num_velocities().
  int get_num_states() const;

  /// Returns the size of the continuous state of a specific model
  /// instance which equals get_num_positions() plus
  /// get_num_velocities().
  int get_num_states(int model_instance_id) const;

  /// Returns the number of actuators.
  int get_num_actuators() const;

  /// Returns the number of actuators for a specific model instance.
  int get_num_actuators(int model_instance_id) const;

  /// Returns the number of model instances in the world.
  int get_num_model_instances() const;

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

  /// Sets the continuous state vector of the system to be @p x.
  void set_state_vector(State<T>* state,
                        const Eigen::Ref<const VectorX<T>> x) const;

  /// Sets the state in @p context so that generalized positions and velocities
  /// are zero. For quaternion based joints the quaternion is set to be the
  /// identity (or equivalently a zero rotation).
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override {
    // Extract a pointer to continuous state from the context.
    DRAKE_DEMAND(state != nullptr);
    ContinuousState<T>* xc = state->get_mutable_continuous_state();
    DRAKE_DEMAND(xc != nullptr);

    // Write the zero configuration into the continuous state.
    VectorX<T> x0 = VectorX<T>::Zero(get_num_states());
    x0.head(get_num_positions()) = tree_->getZeroConfiguration();
    xc->SetFromVector(x0);
  }

  // System<T> overrides.
  /// Allocates two output ports, one for the RigidBodyPlant state and one for
  /// KinematicsResults.
  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override;

  bool has_any_direct_feedthrough() const override;

  /// Computes the force exerted by the stop when a joint hits its limit,
  /// using a linear stiffness model.
  /// Exposed for unit testing of the formula.
  ///
  /// Linear stiffness formula (and definition of "dissipation") from:
  /// https://simtk.org/api_docs/simbody/latest/classSimTK_1_1Force_1_1MobilityLinearStop.html#details
  static T JointLimitForce(const DrakeJoint& joint,
                           const T& position, const T& velocity);

  /// Returns a descriptor of state output port.
  const SystemPortDescriptor<T>& state_output_port() const {
    return System<T>::get_output_port(state_output_port_id_);
  }

  /// Returns a descriptor of KinematicsResults output port.
  const SystemPortDescriptor<T>& kinematics_results_output_port() const {
    return System<T>::get_output_port(kinematics_output_port_id_);
  }

  /// Returns a descriptor of ContactResults output port.
  const SystemPortDescriptor<T>& contact_results_output_port() const {
    return System<T>::get_output_port(contact_output_port_id_);
  }

  /// Returns a descriptor of the input port for a specific model
  /// instance.
  const SystemPortDescriptor<T>& model_input_port(
      int model_instance_id) const {
    return System<T>::get_input_port(input_map_.at(model_instance_id));
  }

  /// Returns a descriptor of the output port for a specific model
  /// instance.
  const SystemPortDescriptor<T>& model_state_output_port(
      int model_instance_id) const {
    return System<T>::get_output_port(output_map_.at(model_instance_id));
  }

  /// Returns the index into the output port for @p model_instance_id
  /// which corresponds to the world position index of @p
  /// world_position_index, or throws if the position index does not
  /// correspond to the model id.
  int FindInstancePositionIndexFromWorldIndex(
      int model_instance_id, int world_position_index);

  /// Creates a right-handed local basis from a z-axis. Defines an arbitrary x-
  /// and y-axis such that the basis is orthonormal.  The basis is R_WL, where W
  /// is the frame in which the z-axis is expressed and L is a local basis such
  /// that v_W = R_WL * v_L.
  /// @param[in] z_axis_W   The vector defining the basis's z-axis expressed
  ///                       in frame W.
  /// @retval R_WL          The computed basis.
  static Matrix3<T> ComputeBasisFromZ(const Vector3<T>& z_axis_W);

 protected:
  // LeafSystem<T> override.
  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;

  // System<T> overrides.

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;
  void DoCalcOutput(const Context<T>& context,
                    SystemOutput<T>* output) const override;

  // TODO(amcastro-tri): provide proper implementations for these methods to
  // track energy conservation.
  // TODO(amcastro-tri): provide a method to track applied actuator power.
  T DoCalcPotentialEnergy(const Context<T>& context) const override {
    return T(NAN);
  }

  T DoCalcKineticEnergy(const Context<T>& context) const override {
    return T(NAN);
  }

  T DoCalcConservativePower(const Context<T>& context) const override {
    return T(NAN);
  }

  T DoCalcNonConservativePower(const Context<T>& context) const override {
    return T(NAN);
  }

  void DoMapVelocityToQDot(
      const Context<T> &context,
      const Eigen::Ref<const VectorX<T>> &generalized_velocity,
      VectorBase<T> *positions_derivative) const override;

  void DoMapQDotToVelocity(
      const Context<T> &context,
      const Eigen::Ref<const VectorX<T>> &configuration_dot,
      VectorBase<T> *generalized_velocity) const override;

 private:
  // Computes the contact results for feeding the corresponding output port.
  void ComputeContactResults(const Context<T>& context,
                             ContactResults<T>* contacts) const;

  // Computes the generalized forces on all bodies due to contact.
  //
  // @param kinsol         The kinematics of the rigid body system at the time
  //                       of contact evaluation.
  // @param[out] contacts  The optional contact results.  If non-null, stores
  //                       the contact information for consuming on the output
  //                       port.
  // @return               The generalized forces across all the bodies due to
  //                       contact response.
  VectorX<T> ComputeContactForce(const KinematicsCache<T>& kinsol,
                                 ContactResults<T>* contacts = nullptr) const;

  // Some parameters defining the contact.
  // TODO(amcastro-tri): Implement contact materials for the RBT engine.
  T penetration_stiffness_{150.0};  // An arbitrarily large number.
  T penetration_damping_{penetration_stiffness_ / 10.0};
  T friction_coefficient_{1.0};

  std::unique_ptr<const RigidBodyTree<T>> tree_;
  int state_output_port_id_{};
  int kinematics_output_port_id_{};
  int contact_output_port_id_{};

  // Maps model instance ids to input port indices.  A value of -1
  // indicates that a model instance has no actuators, and thus no
  // corresponding input port.
  std::vector<int> input_map_;
  // Maps model instance ids to actuator indices and number of
  // actuators in the RigidBodyTree.  Values are stored as a pair of
  // (index, count).
  std::vector<std::pair<int, int>> actuator_map_;

  // Maps model instance ids to output port indices.  A value of -1
  // indicates that a model instance has no state and thus no
  // corresponding output port.
  std::vector<int> output_map_;
  // Maps model instance ids to position indices and number of
  // position states in the RigidBodyTree.  Values are stored as a
  // pair of (index, count).
  std::vector<std::pair<int, int>> position_map_;
  // Maps model instance ids to velocity indices and number of
  // velocity states in the RigidBodyTree.  Values are stored as a
  // pair of (index, count).
  std::vector<std::pair<int, int>> velocity_map_;
};

}  // namespace systems
}  // namespace drake
