#pragma once

#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

/** @defgroup systems Modeling Dynamical Systems
 * @{
 * @brief Drake uses a Simulink-inspired description of dynamical systems.
 *
 * Includes basic building blocks (adders, integrators, delays, etc),
 * physics models of mechanical systems, and a growing list of sensors,
 * actuators, controllers, planners, estimators.
 *
 * All dynamical systems derive from the System base class, and must
 * explicitly  declare all State, parameters, and noise/disturbances inputs.
 * The Diagram class permits modeling complex systems from libraries of parts.
 * @}
 */

/// A description of a discrete-time event, which is passed from the simulator
/// to the recipient System's HandleEvent method.
template <typename T>
struct DiscreteEvent {
  enum ActionType {
    kUnknownAction = 0,  // A default value that causes the handler to abort.
    kPublishAction = 1,  // On a publish action, state does not change.
    kUpdateAction = 2,   // On an update action, discrete state may change.
  };

  /// The system that receives the event.
  const System<T>* recipient{nullptr};
  /// The type of action the system must take in response to the event.
  ActionType action{kUnknownAction};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kPublishAction. If nullptr, Publish will be used.
  std::function<void(const Context<T>&)> do_publish{nullptr};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kUpdateAction. If nullptr, DoEvalDifferenceUpdates will be used.
  std::function<void(const Context<T>&, DifferenceState<T>*)> do_update{
    nullptr};
};

/// A token that identifies the next sample time at which a System must
/// perform some actions, and the actions that must be performed.
template <typename T>
struct UpdateActions {
  /// When the System next requires a discrete action. If the System is
  /// not discrete, time should be set to infinity.
  T time{std::numeric_limits<T>::quiet_NaN()};

  /// The events that should occur when the sample time arrives.
  std::vector<DiscreteEvent<T>> events;
};

/// A superclass template for systems that receive input, maintain state, and
/// produce output of a given mathematical type T.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class System {
 public:
  virtual ~System() {}

  /// Evaluates to `true` if any of the inputs to the system is directly
  /// fed through to any of its outputs and `false` otherwise.
  ///
  /// By default we assume that there is direct feedthrough of values from
  /// every input port to every output port. This is a conservative assumption
  /// that ensures we detect and can prevent the formation of algebraic loops
  /// (implicit computations) in system diagrams. Any System for which none of
  /// the input ports ever feeds through to any of the output ports should
  /// override this method to return false.
  // TODO(amcastro-tri): Provide a more descriptive mechanism to specify
  // pairwise (input_port, output_port) feedthrough.
  virtual bool has_any_direct_feedthrough() const { return true; }

  /// Returns the number of input ports of the system.
  int get_num_input_ports() const {
    return static_cast<int>(input_ports_.size());
  }

  /// Returns the number of output ports of the system.
  int get_num_output_ports() const {
    return static_cast<int>(output_ports_.size());
  }

  /// Returns descriptors for all the input ports of this system.
  const std::vector<SystemPortDescriptor<T>>& get_input_ports() const {
    return input_ports_;
  }

  /// Returns the input port @p input_port.
  const SystemPortDescriptor<T>& get_input_port(int port_number) const {
    if (port_number >= get_num_input_ports()) {
      throw std::out_of_range("port number out of range.");
    }
    return input_ports_[port_number];
  }

  /// Returns the output port @p output_port.
  const SystemPortDescriptor<T>& get_output_port(int port_number) const {
    if (port_number >= get_num_output_ports()) {
      throw std::out_of_range("port number out of range.");
    }
    return output_ports_[port_number];
  }

  /// Returns descriptors for all the output ports of this system.
  const std::vector<SystemPortDescriptor<T>>& get_output_ports() const {
    return output_ports_;
  }

  /// Checks that @p output is consistent with the number and size of output
  /// ports declared by the system.
  /// @throw exception unless `output` is non-null and valid for this system.
  void CheckValidOutput(const SystemOutput<T>* output) const {
    DRAKE_THROW_UNLESS(output != nullptr);

    // Checks that the number of output ports in the system output is consistent
    // with the number of output ports declared by the System.
    DRAKE_THROW_UNLESS(output->get_num_ports() == get_num_output_ports());

    // Checks the validity of each output port.
    for (int i = 0; i < get_num_output_ports(); ++i) {
      // TODO(amcastro-tri): add appropriate checks for kAbstractValued ports
      // once abstract ports are implemented in 3164.
      if (get_output_port(i).get_data_type() == kVectorValued) {
        const VectorBase<T>* output_vector = output->get_vector_data(i);
        DRAKE_THROW_UNLESS(output_vector != nullptr);
        DRAKE_THROW_UNLESS(output_vector->size() ==
                           get_output_port(i).get_size());
      }
    }
  }

  /// Checks that @p context is consistent for this system.
  /// @throw exception unless `context` is valid for this system.
  void CheckValidContext(const Context<T>& context) const {
    // Checks that the number of input ports in the context is consistent with
    // the number of ports declared by the System.
    DRAKE_THROW_UNLESS(context.get_num_input_ports() ==
                       this->get_num_input_ports());

    // Checks that the size of the input ports in the context matches the
    // declarations made by the system.
    for (int i = 0; i < this->get_num_input_ports(); ++i) {
      context.VerifyInputPort(this->get_input_port(i));
    }
  }

  // Returns a copy of the continuous state vector into an Eigen vector.
  VectorX<T> CopyContinuousStateVector(const Context<T>& context) const {
    DRAKE_ASSERT(context.get_continuous_state() != nullptr);
    return context.get_continuous_state()->CopyToVector();
  }

  /// Returns a default context, initialized with the correct
  /// numbers of concrete input ports and state variables for this System.
  /// Since input port pointers are not owned by the context, they should
  /// simply be initialized to nullptr.
  virtual std::unique_ptr<Context<T>> CreateDefaultContext() const = 0;

  /// Returns a default output, initialized with the correct number of
  /// concrete output ports for this System. @p context is provided as
  /// an argument to support some specialized use cases. Most typical
  /// System implementations should ignore it.
  virtual std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const = 0;

  /// Generates side-effect outputs such as terminal output, visualization,
  /// logging, plotting, and network messages. Other than computational cost,
  /// publishing has no effect on the progress of a simulation.
  /// Dispatches to DoPublish.
  ///
  /// This method is invoked by the Simulator at the start of each continuous
  /// integration step, after discrete variables have been updated to the values
  /// they will hold throughout the step. It will always be called at the start
  /// of the first step of a simulation (after initialization) and after the
  /// final simulation step (after a final update to discrete variables).
  void Publish(const Context<T>& context) const {
    DiscreteEvent<T> event;
    event.action = DiscreteEvent<T>::kPublishAction;
    Publish(context, event);
  }

  /// Generates the particular side-effect outputs requested by @p event,
  /// because the sample time requested by @p event has arrived. Dispatches to
  /// DoPublish by default, or to `event.do_publish` if provided.
  ///
  /// @note When publishing is scheduled at particular times, those times likely
  /// will not coincide with integrator step times. A Simulator may interpolate
  /// to generate a suitable Context, or it may adjust the integrator step size
  /// so that a step begins exactly at the next publication time. In the latter
  /// case the change in step size may affect the numerical result somewhat
  /// since a smaller integrator step produces a more accurate solution.
  void Publish(const Context<T>& context, const DiscreteEvent<T>& event) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_DEMAND(event.action == DiscreteEvent<T>::kPublishAction);
    if (event.do_publish == nullptr) {
      DoPublish(context);
    } else {
      event.do_publish(context);
    }
  }

  /// This method is called to update discrete variables in the @p context
  /// because the given @p event has arrived.  Dispatches to
  /// DoEvalDifferenceUpdates by default, or to `event.do_update` if provided.
  void EvalDifferenceUpdates(const Context<T>& context,
                             const DiscreteEvent<T>& event,
                             DifferenceState<T>* difference_state) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_DEMAND(event.action == DiscreteEvent<T>::kUpdateAction);
    if (event.do_update == nullptr) {
      DoEvalDifferenceUpdates(context, difference_state);
    } else {
      event.do_update(context, difference_state);
    }
  }

  /// This method is called by a Simulator during its calculation of the size of
  /// the next continuous step to attempt. The System returns the next time at
  /// which some discrete action must be taken, and records what those actions
  /// ought to be in the given UpdateActions object, which must not be null.
  /// Upon reaching that time, the Simulator invokes either a publication
  /// action (with a const Context) or an update action (with a mutable
  /// Context). The UpdateAction object is retained and returned to the System
  /// when it is time to take the action.
  T CalcNextUpdateTime(const Context<T>& context,
                       UpdateActions<T>* actions) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_ASSERT(actions != nullptr);
    DoCalcNextUpdateTime(context, actions);
    return actions->time;
  }

  /// Computes the output for the given context, possibly updating values
  /// in the cache.
  virtual void EvalOutput(const Context<T>& context,
                          SystemOutput<T>* output) const = 0;

  /// Returns the potential energy currently stored in the configuration
  /// provided in the given @p context. Non-physical Systems will return 0.
  virtual T EvalPotentialEnergy(const Context<T>& context) const {
    return T(0);
  }

  /// Returns the kinetic energy currently present in the motion provided in
  /// the given @p context. Non-physical Systems will return 0.
  virtual T EvalKineticEnergy(const Context<T>& context) const { return T(0); }

  /// Returns the rate at which mechanical energy is being converted *from*
  /// potential energy *to* kinetic energy by this system in the given Context.
  /// This quantity will be positive when potential energy is decreasing. Note
  /// that kinetic energy will also be affected by non-conservative forces so we
  /// can't say which direction it is moving, only whether the conservative
  /// power is increasing or decreasing the kinetic energy. Power is in watts
  /// (J/s).
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  virtual T EvalConservativePower(const Context<T>& context) const {
    return T(0);
  }

  /// Returns the rate at which mechanical energy is being generated (positive)
  /// or dissipated (negative) *other than* by conversion between potential and
  /// kinetic energy (in the given Context). Integrating this quantity yields
  /// work W, and the total energy `E=PE+KE-W` should be conserved by any
  /// physically-correct model, to within integration accuracy of W. Power is in
  /// watts (J/s). (Watts are abbreviated W but not to be confused with work!)
  /// This method is meaningful only for physical systems; others return 0.
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  virtual T EvalNonConservativePower(const Context<T>& context) const {
    return T(0);
  }

  /// Returns a ContinuousState of the same size as the continuous_state
  /// allocated in CreateDefaultContext. The simulator will provide this state
  /// as the output argument to EvalTimeDerivatives.
  ///
  /// By default, allocates no derivatives. Systems with continuous state
  /// variables should override.
  virtual std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const {
    return nullptr;
  }

  /// Returns a DifferenceState of the same dimensions as the difference_state
  /// allocated in CreateDefaultContext. The simulator will provide this state
  /// as the output argument to Update.
  /// By default, allocates nothing. Systems with discrete state variables
  /// should override.
  virtual std::unique_ptr<DifferenceState<T>> AllocateDifferenceVariables()
      const {
    return nullptr;
  }

  /// Produces the derivatives of the continuous state xc with respect to time.
  /// The @p derivatives vector will correspond elementwise with the state
  /// vector Context.state.continuous_state.get_state(). Thus, if the state in
  /// the Context has second-order structure, that same structure applies to
  /// the derivatives.
  ///
  /// Implementations may assume that the given @p derivatives argument has the
  /// same constituent structure as was produced by AllocateTimeDerivatives.
  ///
  /// By default, this function does nothing if the @p derivatives are empty,
  /// and aborts otherwise.
  ///
  /// @param context The context in which to evaluate the derivatives.
  ///
  /// @param derivatives The output vector. Will be the same size as the
  ///                    state vector Context.state.continuous_state.
  virtual void EvalTimeDerivatives(const Context<T>& context,
                                   ContinuousState<T>* derivatives) const {
    // This default implementation is only valid for Systems with no continuous
    // state. Other Systems must override this method!
    DRAKE_DEMAND(derivatives->size() == 0);
    return;
  }

  /// Transforms the velocity to the derivative of the configuration.
  /// Generalized velocities (v) and configuration derivatives (qdot) are
  /// related linearly by `qdot = N(q) * v` (where `N` may be the identity
  /// matrix). See the alternate signature if you already have the generalized
  /// velocity in an Eigen VectorX object; this signature will copy the
  /// VectorBase into an Eigen object before performing the computation.
  void MapVelocityToConfigurationDerivatives(
      const Context<T>& context, const VectorBase<T>& generalized_velocity,
      VectorBase<T>* configuration_derivatives) const {
    MapVelocityToConfigurationDerivatives(context,
                                          generalized_velocity.CopyToVector(),
                                          configuration_derivatives);
  }

  /// Transforms the generalized velocity to the derivative of configuration.
  /// See alternate signature of MapVelocityToConfigurationDerivatives() for
  /// more information.
  void MapVelocityToConfigurationDerivatives(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* configuration_derivatives) const {
    DoMapVelocityToConfigurationDerivatives(context, generalized_velocity,
                                            configuration_derivatives);
  }

  /// Transforms the time derivative of configuration to generalized
  /// velocities. Generalized velocities (v) and configuration
  /// derivatives (qdot) are related linearly by `qdot = N(q) * v` (where `N`
  /// may be the identity matrix). Although `N` is not necessarily square,
  /// its left pseudo-inverse `N+` can be used to invert that relationship
  /// without residual error. Using the configuration `q` from the given
  /// context this method calculates `v = N+ * qdot` (where `N+=N+(q)`) for
  /// a given `qdot` (@p configuration_derivatives). This method does not
  /// take `qdot` from the context. See the alternate signature if you
  /// already have `qdot` in an Eigen VectorX object; this signature will
  /// copy the VectorBase into an Eigen object before performing the
  /// computation.
  // TODO(edrumwri): Evan to verify that N is always left-invertible.
  void MapConfigurationDerivativesToVelocity(
      const Context<T>& context, const VectorBase<T>& configuration_derivatives,
      VectorBase<T>* generalized_velocity) const {
    MapConfigurationDerivativesToVelocity(
        context, configuration_derivatives.CopyToVector(),
        generalized_velocity);
  }

  /// Transforms the time derivative of configuration to generalized velocity.
  /// This signature allows using an Eigen VectorX object for faster speed.
  /// See the other signature of MapConfigurationDerivativesToVelocity() for
  /// additional information.
  void MapConfigurationDerivativesToVelocity(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& configuration_derivatives,
      VectorBase<T>* generalized_velocity) const {
    DoMapConfigurationDerivativesToVelocity(context, configuration_derivatives,
                                            generalized_velocity);
  }

  // TODO(david-german-tri): MapAccelerationToConfigurationSecondDerivatives.

  // Sets the name of the system. It is recommended that the name not include
  // the character ':', since that the path delimiter. is "::".
  virtual void set_name(const std::string& name) { name_ = name; }
  virtual std::string get_name() const { return name_; }

  /// Writes the full path of this System in the tree of Systems to @p output.
  /// The path has the form (::ancestor_system_name)*::this_system_name.
  virtual void GetPath(std::stringstream* output) const {
    // If this System has a parent, that parent's path is a prefix to this
    // System's path. Otherwise, this is the root system and there is no prefix.
    if (parent_ != nullptr) {
      parent_->GetPath(output);
    }
    *output << "::";
    *output << (get_name().empty() ? "<unnamed System>" : get_name());
  }

  // Returns the full path of the System in the tree of Systems.
  std::string GetPath() const {
    std::stringstream path;
    GetPath(&path);
    return path.str();
  }

  /// Declares that @p parent is the immediately enclosing Diagram. The
  /// enclosing Diagram is needed to evaluate inputs recursively. Aborts if
  /// the parent has already been set to something else.
  ///
  /// This is a dangerous implementation detail. Conceptually, a System
  /// ought to be completely ignorant of its parent Diagram. However, we
  /// need this pointer so that we can cause our inputs to be evaluated.
  /// See https://github.com/RobotLocomotion/drake/pull/3455.
  void set_parent(const detail::InputPortEvaluatorInterface<T>* parent) {
    DRAKE_DEMAND(parent_ == nullptr || parent_ == parent);
    parent_ = parent;
  }

 protected:
  System() {}

  /// Adds a port with the specified @p descriptor to the input topology.
  void DeclareInputPort(const SystemPortDescriptor<T>& descriptor) {
    DRAKE_ASSERT(descriptor.get_index() == get_num_input_ports());
    DRAKE_ASSERT(descriptor.get_face() == kInputPort);
    input_ports_.emplace_back(descriptor);
  }

  /// Adds a port with the specified @p type, @p size, and @p sampling
  /// to the input topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareInputPort(PortDataType type, int size,
                                                  SamplingSpec sampling) {
    int port_number = get_num_input_ports();
    input_ports_.emplace_back(this, kInputPort, port_number, type, size,
                              sampling);
    return input_ports_.back();
  }

  /// Adds an abstract-valued port with the specified @p sampling to the
  /// input topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareAbstractInputPort(
      SamplingSpec sampling) {
    return DeclareInputPort(kAbstractValued, 0 /* size */, sampling);
  }

  /// Adds a port with the specified @p descriptor to the output topology.
  void DeclareOutputPort(const SystemPortDescriptor<T>& descriptor) {
    DRAKE_ASSERT(descriptor.get_index() == get_num_output_ports());
    DRAKE_ASSERT(descriptor.get_face() == kOutputPort);
    output_ports_.emplace_back(descriptor);
  }

  /// Adds a port with the specified @p type, @p size, and @p sampling
  /// to the output topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareOutputPort(PortDataType type, int size,
                                                   SamplingSpec sampling) {
    int port_number = get_num_output_ports();
    output_ports_.emplace_back(this, kOutputPort, port_number, type, size,
                               sampling);
    return output_ports_.back();
  }

  /// Adds an abstract-valued port with the specified @p sampling to the
  /// output topology.
  /// @return descriptor of declared port.
  const SystemPortDescriptor<T>& DeclareAbstractOutputPort(
      SamplingSpec sampling) {
    return DeclareOutputPort(kAbstractValued, 0 /* size */, sampling);
  }

  /// Causes the vector-valued port with the given @p port_index to become
  /// up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's value, or nullptr if the port is not connected.
  ///
  /// Throws std::bad_cast if the port is not vector-valued.
  /// Aborts if the port does not exist.
  const BasicVector<T>* EvalVectorInput(const Context<T>& context,
                                        int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    return context.EvalVectorInput(parent_, get_input_port(port_index));
  }

  /// Causes the vector-valued port with the given @p port_index to become
  /// up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's value as an Eigen expression.
  Eigen::VectorBlock<const VectorX<T>> EvalEigenVectorInput(
      const Context<T>& context, int port_index) const {
    const BasicVector<T>* input_vector = EvalVectorInput(context, port_index);
    DRAKE_ASSERT(input_vector != nullptr);
    DRAKE_ASSERT(input_vector->size() == get_input_port(port_index).get_size());
    return input_vector->get_value();
  }

  /// Causes the abstract-valued port with the given @p port_index to become
  /// up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's abstract value pointer, or nullptr if the port is not
  /// connected.
  const AbstractValue* EvalAbstractInput(const Context<T>& context,
                                         int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    return context.EvalAbstractInput(parent_, get_input_port(port_index));
  }

  /// Causes the abstract-valued port with the given @p port_index to become
  /// up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's abstract value, or nullptr if the port is not connected.
  ///
  /// @tparam V The type of data expected.
  template <typename V>
  const V* EvalInputValue(const Context<T>& context, int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    return context.template EvalInputValue<V>(parent_,
                                              get_input_port(port_index));
  }

  /// Returns a mutable Eigen expression for a vector valued output port with
  /// index @p port_index in this system. All InputPorts that directly depend
  /// on this OutputPort will be notified that upstream data has changed, and
  /// may invalidate cache entries as a result.
  Eigen::VectorBlock<VectorX<T>> GetMutableOutputVector(SystemOutput<T>* output,
                                                        int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_output_ports());

    BasicVector<T>* output_vector = output->GetMutableVectorData(port_index);
    DRAKE_ASSERT(output_vector != nullptr);
    DRAKE_ASSERT(output_vector->size() ==
                 get_output_port(port_index).get_size());

    return output_vector->get_mutable_value();
  }

  /// Implement this in your concrete System if you want it to take some action
  /// when the Simulator calls the Publish() method. This can be used for
  /// sending messages, producing console output, debugging, logging, saving the
  /// trajectory to a file, etc. You may assume that the `context` has already
  /// been validated before it is passed to you here.
  virtual void DoPublish(const Context<T>& context) const {}

  /// Updates the @p difference_state on sample events.
  /// Override it, along with DoCalcNextUpdateTime, if your System has any
  /// difference variables.
  ///
  /// @p difference_state is not a pointer into @p context. It is a separate
  /// buffer, which the Simulator is responsible for writing back to the @p
  /// context later.
  virtual void DoEvalDifferenceUpdates(
      const Context<T>& context, DifferenceState<T>* difference_state) const {}

  /// Computes the next time at which this System must perform a discrete
  /// action.
  ///
  /// Override this method if your System has any discrete actions which must
  /// interrupt the continuous simulation. You may assume that the context
  /// has already been validated and the `actions` pointer is not nullptr.
  ///
  /// The default implementation returns with `actions` having a next sample
  /// time of Infinity and no actions to take.  If you declare actions, you may
  /// specify custom do_publish and do_update handlers.  If you do not,
  /// DoPublish and DoEvalDifferenceUpdates will be used by default.
  virtual void DoCalcNextUpdateTime(const Context<T>& context,
                                    UpdateActions<T>* actions) const {
    actions->time = std::numeric_limits<T>::infinity();
  }

  /// Provides the substantive implementation of
  /// MapConfigurationDerivativesToVelocity(). This signature can work directly
  /// with an Eigen vector object for faster performance. See the other
  /// DoMapConfigurationDerivativesToVelocity() signature for additional
  /// information.
  ///
  /// The default implementation uses the identity mapping. It throws
  /// std::runtime_error if the @p generalized_velocity and
  /// @p configuration_derivatives are not the same size. Child classes must
  /// override this function if qdot != v (even if they are the same size).
  ///
  /// Implementations may assume that @p generalized_velocity is of
  /// the same size as the generalized velocity allocated in
  /// AllocateTimeDerivatives(). Implementations that are not
  /// second-order systems may simply do nothing.
  virtual void DoMapConfigurationDerivativesToVelocity(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& configuration_derivatives,
      VectorBase<T>* generalized_velocity) const {
    // In the particular case where generalized velocity and generalized
    // configuration are not even the same size, we detect this error and abort.
    // This check will thus not identify cases where the generalized velocity
    // and time derivative of generalized configuration are identically sized
    // but not identical!
    const int n = configuration_derivatives.size();
    // You need to override System<T>::DoMapConfigurationDerivativestoVelocity!
    DRAKE_THROW_UNLESS(generalized_velocity->size() == n);
    generalized_velocity->SetFromVector(configuration_derivatives);
  }

  /**
   * Provides the substantive implementation of
   * MapVelocityToConfigurationDerivatives(). This signature can work
   * directly with an Eigen vector object for faster performance. See
   * the other DoMapVelocityToConfigurationDerivatives() signature for
   * additional information.
   */
  virtual void DoMapVelocityToConfigurationDerivatives(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* configuration_derivatives) const {
    // In the particular case where generalized velocity and generalized
    // configuration are not even the same size, we detect this error and abort.
    // This check will thus not identify cases where the generalized velocity
    // and time derivative of generalized configuration are identically sized
    // but not identical!
    const int n = generalized_velocity.size();
    // You need to override System<T>::DoMapVelocityToConfigurationDerivatives!
    DRAKE_THROW_UNLESS(configuration_derivatives->size() == n);
    configuration_derivatives->SetFromVector(generalized_velocity);
  }

  /// Causes an InputPort in the @p context to become up-to-date, delegating to
  /// the parent Diagram if necessary.
  ///
  /// This is a framework implementation detail. User code should never call it.
  void EvalInputPort(const Context<T>& context, int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    context.EvalInputPort(parent_, get_input_port(port_index));
  }

 private:
  // SystemInterface objects are neither copyable nor moveable.
  System(const System<T>& other) = delete;
  System& operator=(const System<T>& other) = delete;
  System(System<T>&& other) = delete;
  System& operator=(System<T>&& other) = delete;

  std::string name_;
  std::vector<SystemPortDescriptor<T>> input_ports_;
  std::vector<SystemPortDescriptor<T>> output_ports_;
  const detail::InputPortEvaluatorInterface<T>* parent_{nullptr};
};

}  // namespace systems
}  // namespace drake
