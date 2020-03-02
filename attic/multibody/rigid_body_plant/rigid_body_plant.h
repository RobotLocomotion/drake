#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/constraint/constraint_solver.h"
#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// This class provides a System interface around a multibody dynamics model
/// of the world represented by a RigidBodyTree.
///
/// The %RigidBodyPlant provides a number of input and output ports. The number
/// and types of port accessors depends on the number of model instances within
/// the RigidBodyTree with actuators. The following lists the accessors for
/// obtaining the input and output ports of the %RigidBodyPlant. These accessors
/// are typically used when "wiring up" a RigidBodyPlant within a Diagram using
/// DiagramBuilder. See, for example, DiagramBuilder::Connect(),
/// DiagramBuilder::ExportInput(), and DiagramBuilder::ExportOutput().
///
/// <B>Plant-Centric Port Accessors:</B>
///
/// - actuator_command_input_port(): Contains the command vector for the
///   RigidBodyTree's actuators. This method can only be called when there is
///   only one model instance in the RigidBodyTree, as
///   determined by get_num_model_instances(), and this model instance has at
///   least one actuator. The size of this vector equals the number of
///   RigidBodyActuator's in the RigidBodyTree. Each RigidBodyActuator maps to a
///   single-DOF joint (currently actuation cannot be applied to multiple-DOF
///   joints). The units of the actuation are the same as the units of the
///   generalized force on the joint. In addition, actuators allow for a gear
///   box reduction factor and for actuation limits which are only used by
///   controllers; the RigidBodyPlant does not apply these limits. The gear box
///   factor effectively is a multiplier on the input actuation to the
///   RigidBodyPlant.
///
/// - state_output_port(): A vector-valued port containing the state vector,
///   `x`, of the system. This is useful for downstream systems that require
///   `x`, which includes DrakeVisualizer. The state vector, `x`, consists of
///   generalized positions followed by generalized velocities. Semantics of `x`
///   can be obtained using the following methods:
///
///   - RigidBodyPlant<T>::get_num_states()
///   - RigidBodyTree<T>::get_num_positions()
///   - RigidBodyTree<T>::get_num_velocities()
///   - RigidBodyTree<T>::get_position_name()
///   - RigidBodyTree<T>::get_velocity_name()
///
/// - state_derivative_output_port(): A vector-valued port containing the time
///   derivative `xcdot` of the state vector.  The order of indices within the
///   vector is identical to state_output_port() as explained above.
///
/// - kinematics_results_output_port(): An abstract-valued port containing a
///   KinematicsResults object allowing access to the results from kinematics
///   computations for each RigidBody in the RigidBodyTree.
///
/// - contact_results_output_port(): An abstract-valued port containing a
///   ContactsResults object allowing access to the results from contact
///   computations.
///
/// <B>Model-Instance-Centric Port Accessors:</B>
///
/// - model_instance_actuator_command_input_port(): Contains the command vector
///   for the actuators belonging to a particular model instance within the
///   RigidBodyTree. This method can only be called using the model instance ID
///   of a model with actuators. To determine if a model instance possesses
///   actuators, use model_instance_has_actuators().
///
/// - model_instance_state_output_port(): A vector-valued port containing the
///   state vector for a particular model instance in the RigidBodyTree.
///
/// The %RigidBodyPlant's state consists of a vector containing the generalized
/// positions followed by the generalized velocities of the system. This state
/// is applied to a RigidBodyTree, which is a multibody model that consists of a
/// set of rigid bodies connected through joints in a tree structure. Bodies may
/// have a collision model, in which case, collisions are considered. In
/// addition, the model may contain loop constraints described by
/// RigidBodyLoop instances in the multibody model. Even though loop constraints
/// are a particular case of holonomic constraints, general holonomic
/// constraints are not yet supported. For simulating discretized
/// %RigidBodyPlant systems, an additional (discrete) scalar state variable
/// stores the last time that the system's state was updated.
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
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
///
/// @throws std::runtime_error  The AutodiffXd implementation has some
/// restrictions:
/// - The collision detection code does not yet support AutoDiff, and calls
/// to RigidBodyTree that would have required that gradient information will
/// throw a std::runtime_error.  Currently, the implication is that AutoDiff
/// calls for RigidBodyPlants with Context that do not require gradients of
/// the contact forces will succeed, but calls where Context results in
/// non-zero contact forces will throw.
/// - DoCalcDiscreteVariableUpdates does not yet support AutoDiff, and will
/// throw if called.  In practice this means that AutoDiff of the dynamics
/// are only available if the RigidBodyPlant is constructed with `timestep=0`.
///
/// @ingroup rigid_body_systems
template <typename T>
class RigidBodyPlant : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidBodyPlant)

  /// Instantiates a %RigidBodyPlant from a Multi-Body Dynamics (MBD) model of
  /// the world in `tree`.  `tree` must not be `nullptr`.
  ///
  /// @param[in] tree the dynamic model to use with this plant.
  /// @param[in] timestep a non-negative value specifying the update period of
  ///   the model; 0.0 implies continuous-time dynamics with derivatives, and
  ///   values > 0.0 result in discrete-time dynamics implementing a
  ///   discretization of the dynamics equation.  @default 0.0.
  // TODO(SeanCurtis-TRI): It appears that the tree has to be "compiled"
  // already.  Confirm/deny and document that result.
  explicit RigidBodyPlant(std::unique_ptr<const RigidBodyTree<double>> tree,
                          double timestep = 0.0);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit RigidBodyPlant(const RigidBodyPlant<U>& other);

  ~RigidBodyPlant() override;

  /// Sets the parameters of the compliance _model_. To set material parameters,
  /// use the CompliantMaterial instance associated with the collision element.
  void set_contact_model_parameters(
      const CompliantContactModelParameters& parameters);

  /// Sets the compliant material values to use for default-configured material
  /// properties on collision elements (see CompliantMaterial for details).
  void set_default_compliant_material(const CompliantMaterial& material);

  /// Returns a constant reference to the multibody dynamics model
  /// of the world.
  const RigidBodyTree<double>& get_rigid_body_tree() const;

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

  /// Returns the number of model instances in the world, not including the
  /// world.
  int get_num_model_instances() const;

  /// Returns the size of the input vector to the system. This equals the
  /// number of actuators.
  int get_input_size() const;

  /// Returns the size of the output vector of the system. This equals the size
  /// of the continuous state vector.
  int get_output_size() const;

  /// Sets the generalized coordinate `position_index` to the value
  /// `position`.
  void set_position(Context<T>* context, int position_index, T position) const;

  /// Sets the generalized coordinates of the model instance specified by
  /// `model_instance_id` to the values in `position`.
  void SetModelInstancePositions(
      Context<T>* context, int model_instance_id,
      const Eigen::Ref<const VectorX<T>> positions) const;

  /// Sets the generalized velocity `velocity_index` to the value
  /// `velocity`.
  void set_velocity(Context<T>* context, int velocity_index, T velocity) const;

  /// Sets the continuous state vector of the system to be `x`.
  void set_state_vector(Context<T>* context,
                        const Eigen::Ref<const VectorX<T>> x) const;

  /// Sets the continuous state vector of the system to be `x`.
  void set_state_vector(State<T>* state,
                        const Eigen::Ref<const VectorX<T>> x) const;

  /// Sets the state in `context` so that generalized positions and velocities
  /// are zero. For quaternion based joints the quaternion is set to be the
  /// identity (or equivalently a zero rotation).
  void SetDefaultState(const Context<T>&,
                       State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);

    VectorX<T> x0 = VectorX<T>::Zero(get_num_states());
    x0.head(get_num_positions()) = tree_->getZeroConfiguration();

    if (is_state_discrete()) {
      // Extract a reference to the discrete state from the context.
      BasicVector<T>& xd =
          state->get_mutable_discrete_state().get_mutable_vector(0);

      // Write the zero configuration into the discrete state.
      xd.SetFromVector(x0);

      // Set the initial time.
      state->get_mutable_discrete_state().get_mutable_vector(1)[0] = 0;
    } else {
      // Extract a reference to continuous state from the context.
      ContinuousState<T>& xc = state->get_mutable_continuous_state();

      // Write the zero configuration into the continuous state.
      xc.SetFromVector(x0);
    }
  }

  /// Computes the force exerted by the stop when a joint hits its limit,
  /// using a linear stiffness model.
  /// Exposed for unit testing of the formula.
  ///
  /// Linear stiffness formula (and definition of "dissipation") from:
  /// https://simtk.org/api_docs/simbody/latest/classSimTK_1_1Force_1_1MobilityLinearStop.html#details
  static T JointLimitForce(const DrakeJoint& joint, const T& position,
                           const T& velocity);

  /// Returns the index into the output port for `model_instance_id`
  /// which corresponds to the world position index of
  /// `world_position_index`, or throws if the position index does not
  /// correspond to the model id.
  int FindInstancePositionIndexFromWorldIndex(int model_instance_id,
                                              int world_position_index);

  /// @name System input port accessors.
  /// These are accessors for obtaining this RigidBodyPlant's input ports. See
  /// this class's description for details about these ports and how these
  /// accessors are typically used.
  ///@{

  /// Returns a the actuator command input port. This method can only be
  /// called when there is only one model instance in the RigidBodyTree.
  /// Otherwise, a std::runtime_error will be thrown. It returns the same port
  /// as model_instance_actuator_command_input_port() using input
  /// parameter RigidBodyTreeConstants::kFirstNonWorldModelInstanceId.
  const InputPort<T>& actuator_command_input_port() const {
    if (get_num_model_instances() != 1) {
      throw std::runtime_error(
          "RigidBodyPlant::actuator_command_input_port(): "
          "ERROR: This method can only called when there is only one model "
          "instance in the RigidBodyTree. There are currently " +
          std::to_string(get_num_model_instances()) +
          " model instances in the "
          "RigidBodyTree.");
    }
    return model_instance_actuator_command_input_port(
        RigidBodyTreeConstants::kFirstNonWorldModelInstanceId);
  }

  /// Returns true if and only if the model instance with the provided
  /// `model_instance_id` has actuators. This is useful when trying to determine
  /// whether it's safe to call model_instance_actuator_command_input_port().
  bool model_instance_has_actuators(int model_instance_id) const;

  /// Returns the input port for a specific model instance. This method can
  /// only be called when this class is instantiated with constructor
  /// parameter `export_model_instance_centric_ports` equal to `true`.
  const InputPort<T>& model_instance_actuator_command_input_port(
      int model_instance_id) const;

  ///@}

  /// @name System output port accessors.
  /// These are accessors for obtaining this RigidBodyPlant's
  /// output ports. See this class's description for details about these ports
  /// and how these accessors are typically used.
  ///@{

  /// Returns the plant-centric state output port. The size of
  /// this port is equal to get_num_states().
  const OutputPort<T>& state_output_port() const {
    return System<T>::get_output_port(state_output_port_index_);
  }

  /// Returns the plant-centric state derivative output port. The size of
  /// this port is equal to get_num_states().
  /// @pre This %RigidBodyPlant is using continuous-time dynamics.
  const OutputPort<T>& state_derivative_output_port() const {
    DRAKE_DEMAND(state_derivative_output_port_index_.has_value());
    return System<T>::get_output_port(*state_derivative_output_port_index_);
  }

  /// Returns the output port containing the state of a
  /// particular model with instance ID equal to `model_instance_id`.
  /// @throws std::runtime_error if `model_instance_id` does not exist.
  ///
  /// This method can only be called when this class is instantiated with
  /// constructor parameter `export_model_instance_centric_ports` equal to
  /// `true`.
  const OutputPort<T>& model_instance_state_output_port(
      int model_instance_id) const;

  /// Returns the output port containing measured joint torques.
  /// @throws std::runtime_error if this RigidBodyTree contains more than one
  /// model instances.
  const OutputPort<T>& torque_output_port() const {
    if (get_num_model_instances() != 1) {
      throw std::runtime_error(
          "RigidBodyPlant::torque_output_port(): "
          "ERROR: This method can only called when there is only one model "
          "instance in the RigidBodyTree. There are currently " +
          std::to_string(get_num_model_instances()) +
          " model instances in the "
          "RigidBodyTree.");
    }
    return model_instance_torque_output_port(
        RigidBodyTreeConstants::kFirstNonWorldModelInstanceId);
  }

  /// Returns the output port containing the measured joint torques of a
  /// particular model with @p model_instance_id.
  const OutputPort<T>& model_instance_torque_output_port(
      int model_instance_id) const;

  /// Returns the KinematicsResults output port.
  const OutputPort<T>& kinematics_results_output_port() const {
    return System<T>::get_output_port(kinematics_output_port_index_);
  }

  /// Returns the ContactResults output port.
  const OutputPort<T>& contact_results_output_port() const {
    return System<T>::get_output_port(contact_output_port_index_);
  }
  ///@}

  // Gets a constant reference to the state vector, irrespective of whether
  // the state is continuous or discrete.
  Eigen::VectorBlock<const VectorX<T>> GetStateVector(
      const Context<T>& context) const;

  /// Gets whether this system is modeled using discrete state.
  bool is_state_discrete() const { return timestep_ > 0.0; }

  /// Get the time step used to construct the plant. If the step is zero, the
  /// system is continuous. Otherwise, the step corresponds to the update rate
  /// (seconds per update).
  double get_time_step() const { return timestep_; }

 protected:
  // Constructor for derived classes to support system scalar conversion, as
  // mandated in the doxygen `system_scalar_conversion` documentation.
  explicit RigidBodyPlant(SystemScalarConverter converter,
                          std::unique_ptr<const RigidBodyTree<double>> tree,
                          double timestep = 0.0);

  // Evaluates the actuator command input ports and throws a std::runtime_error
  // exception if at least one of the ports is not connected.
  VectorX<T> EvaluateActuatorInputs(const Context<T>& context) const;

  // System<T> overrides.

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<T>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
      drake::systems::DiscreteValues<T>* updates) const override {
    // Pass to SFINAE compatible implementation.
    DoCalcDiscreteVariableUpdatesImpl(context, events, updates);
  }

  // TODO(amcastro-tri): provide proper implementations for these methods to
  // track energy conservation.
  // TODO(amcastro-tri): provide a method to track applied actuator power.
  T DoCalcPotentialEnergy(const Context<T>&) const override {
    return T(NAN);
  }

  T DoCalcKineticEnergy(const Context<T>&) const override {
    return T(NAN);
  }

  T DoCalcConservativePower(const Context<T>&) const override {
    return T(NAN);
  }

  T DoCalcNonConservativePower(const Context<T>&) const override {
    return T(NAN);
  }

  void DoMapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* positions_derivative) const override;

  void DoMapQDotToVelocity(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& configuration_dot,
      VectorBase<T>* generalized_velocity) const override;

 private:
  friend class RigidBodyPlantTimeSteppingDataTest_NormalJacobian_Test;
  friend class RigidBodyPlantTimeSteppingDataTest_TangentJacobian_Test;

  // Common logic only intended to be called from the (multiple) constructors.
  void initialize(void);

  template <typename U = T>
  std::enable_if_t<std::is_same<U, double>::value, void>
  DoCalcDiscreteVariableUpdatesImpl(
      const drake::systems::Context<U>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<U>*>& events,
      drake::systems::DiscreteValues<U>* updates) const;

  template <typename U = T>
  std::enable_if_t<!std::is_same<U, double>::value, void>
  DoCalcDiscreteVariableUpdatesImpl(
      const drake::systems::Context<U>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<U>*>& events,
      drake::systems::DiscreteValues<U>* updates) const;

  OutputPortIndex DeclareContactResultsOutputPort();

  // These five are the output port calculator methods.
  void CopyStateToOutput(const Context<T>& context,
                         BasicVector<T>* state_output_vector) const;

  void CalcStateDerivativeOutput(const Context<T>& context,
                                 BasicVector<T>*) const;

  void CalcInstanceOutput(int instance_id,
                          const Context<T>& context,
                          BasicVector<T>* instance_output) const;

  // Measured torque is currently directly copied from the inputs.
  void CopyInstanceTorqueOutput(int instance_id,
                                const Context<T>& context,
                                BasicVector<T>* instance_output) const;

  void CalcKinematicsResultsOutput(const Context<T>& context,
                                   KinematicsResults<T>* output) const;

  void CalcContactResultsOutput(const Context<T>& context,
                                ContactResults<T>* output) const;

  void ExportModelInstanceCentricPorts();

  void CalcContactStiffnessDampingMuAndNumHalfConeEdges(
      const drake::multibody::collision::PointPair<T>& contact,
      double* stiffness, double* damping, double* mu,
      int* num_cone_edges) const;

  Vector3<T> CalcRelTranslationalVelocity(
      const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
      const Vector3<T>& p_W) const;

  void UpdateGeneralizedForce(
      const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
      const Vector3<T>& p, const Vector3<T>& f, VectorX<T>* gf) const;

  VectorX<T> ContactNormalJacobianMult(
      const std::vector<drake::multibody::collision::PointPair<T>>& contacts,
      const VectorX<T>& q, const VectorX<T>& v) const;

  VectorX<T> TransposedContactNormalJacobianMult(
      const std::vector<drake::multibody::collision::PointPair<T>>& contacts,
      const KinematicsCache<T>& kcache, const VectorX<T>& f) const;

  VectorX<T> ContactTangentJacobianMult(
      const std::vector<drake::multibody::collision::PointPair<T>>& contacts,
      const VectorX<T>& q, const VectorX<T>& v,
      const std::vector<int>& half_num_cone_edges) const;

  VectorX<T> TransposedContactTangentJacobianMult(
      const std::vector<drake::multibody::collision::PointPair<T>>& contacts,
      const KinematicsCache<T>& kcache, const VectorX<T>& f,
      const std::vector<int>& half_num_cone_edges) const;

  // Note: The templated ScalarTypes are used in the KinematicsCache, but all
  // KinematicsResults use RigidBodyTree<double>.  This effectively implies
  // that we can e.g. AutoDiffXd with respect to the configurations, but not
  // the RigidBodyTree parameters.
  std::unique_ptr<const RigidBodyTree<double>> tree_;

  // Object that performs all constraint computations.
  multibody::constraint::ConstraintSolver<double> constraint_solver_;

  OutputPortIndex state_output_port_index_{};
  std::optional<OutputPortIndex> state_derivative_output_port_index_;
  OutputPortIndex kinematics_output_port_index_{};
  OutputPortIndex contact_output_port_index_{};

  // timestep == 0.0 implies continuous-time dynamics,
  // timestep > 0.0 implies a discrete-time dynamics approximation.
  const double timestep_{0.0};

  // Maps model instance ids to input port indices.  A value of
  // kInvalidPortIdentifier indicates that a model instance has no actuators,
  // and thus no corresponding input port.
  std::vector<int> input_map_;
  // Maps model instance ids to actuator indices and number of
  // actuators in the RigidBodyTree.  Values are stored as a pair of
  // (index, count).
  std::vector<std::pair<int, int>> actuator_map_;

  // Maps model instance ids to output port indices.  A value of
  // kInvalidPortIdentifier indicates that a model instance has no state and
  // thus no corresponding output port.
  std::vector<int> output_map_;

  // Maps model instance ids to output port indices. A value of
  // kInvalidPortIdentifier indicates that a model instance has no actuators,
  // and thus no corresponding output port.
  std::vector<int> torque_output_map_;

  // Maps model instance ids to position indices and number of
  // position states in the RigidBodyTree.  Values are stored as a
  // pair of (index, count).
  std::vector<std::pair<int, int>> position_map_;
  // Maps model instance ids to velocity indices and number of
  // velocity states in the RigidBodyTree.  Values are stored as a
  // pair of (index, count).
  std::vector<std::pair<int, int>> velocity_map_;

  // Pointer to the class that encapsulates all the contact computations.
  const std::unique_ptr<CompliantContactModel<T>> compliant_contact_model_;

  // Structure for storing joint limit data for the discretized version of the
  // plant.
  struct JointLimit {
    // The index for the joint limit.
    int v_index{-1};

    // Whether the limit is a lower limit or upper limit.
    bool lower_limit{false};

    // The signed distance from the limit. Negative signed distances correspond
    // to joint limit violations.
    T signed_distance{0};
  };

  template <typename U>
  friend class RigidBodyPlant;  // For scalar-converting copy constructor.
};

// Explicitly disable symbolic::Expression (for now).
namespace scalar_conversion {
template <>
struct Traits<RigidBodyPlant> : public NonSymbolicTraits {};
}  // namespace scalar_conversion

}  // namespace systems
}  // namespace drake
