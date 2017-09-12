#pragma once

#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/symbolic.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event_collection.h"
#include "drake/systems/framework/input_port_descriptor.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace systems {

/** @cond */
// Private helper class for System.
class SystemImpl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemImpl)
  SystemImpl() = delete;

  // The implementation of System<T>::GetMemoryObjectName.
  static std::string GetMemoryObjectName(const std::string&, int64_t);
};
/** @endcond */

/// A superclass template for systems that receive input, maintain state, and
/// produce output of a given mathematical type T.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class System {
 public:
  // System objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(System)

  virtual ~System() {}

  //----------------------------------------------------------------------------
  /// @name           Resource allocation and initialization
  /// These methods are used to allocate and initialize Context resources.
  //@{

  /// Allocates a context, initialized with the correct numbers of concrete
  /// input ports and state variables for this System.  Since input port
  /// pointers are not owned by the context, they should simply be initialized
  /// to nullptr.
  virtual std::unique_ptr<Context<T>> AllocateContext() const = 0;

  /// Allocates a CompositeEventCollection for this system. The allocated
  /// instance is used for registering events; for example, Simulator passes
  /// this object to System::CalcNextUpdateTime() to allow the system to
  /// register upcoming events.
  virtual std::unique_ptr<CompositeEventCollection<T>>
      AllocateCompositeEventCollection() const = 0;

  /// Given a port descriptor, allocates the vector storage.  The default
  /// implementation in this class allocates a BasicVector.  Subclasses must
  /// override the NVI implementation of this function, DoAllocateInputVector,
  /// to return input vector types other than BasicVector. The @p descriptor
  /// must match a port declared via DeclareInputPort.
  std::unique_ptr<BasicVector<T>> AllocateInputVector(
      const InputPortDescriptor<T>& descriptor) const {
    DRAKE_ASSERT(descriptor.get_data_type() == kVectorValued);
    const int index = descriptor.get_index();
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    DRAKE_ASSERT(get_input_port(index).get_data_type() == kVectorValued);
    return std::unique_ptr<BasicVector<T>>(DoAllocateInputVector(descriptor));
  }

  /// Given a port descriptor, allocates the abstract storage. Subclasses with a
  /// abstract input ports must override the NVI implementation of this
  /// function, DoAllocateInputAbstract, to return an appropriate AbstractValue.
  /// The @p descriptor must match a port declared via DeclareInputPort.
  std::unique_ptr<AbstractValue> AllocateInputAbstract(
      const InputPortDescriptor<T>& descriptor) const {
    DRAKE_ASSERT(descriptor.get_data_type() == kAbstractValued);
    const int index = descriptor.get_index();
    DRAKE_ASSERT(index >= 0 && index < get_num_input_ports());
    DRAKE_ASSERT(get_input_port(index).get_data_type() == kAbstractValued);
    return std::unique_ptr<AbstractValue>(DoAllocateInputAbstract(descriptor));
  }

  /// Returns a container that can hold the values of all of this System's
  /// output ports. It is sized with the number of output ports and uses each
  /// output port's allocation method to provide an object of the right type
  /// for that port. A Context is provided as
  /// an argument to support some specialized use cases. Most typical
  /// System implementations should ignore it.
  virtual std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const = 0;

  /// Returns a ContinuousState of the same size as the continuous_state
  /// allocated in CreateDefaultContext. The simulator will provide this state
  /// as the output argument to EvalTimeDerivatives.
  ///
  /// By default, allocates no derivatives. Systems with continuous state
  /// variables should override.
  virtual std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const {
    return nullptr;
  }

  /// Returns a DiscreteState of the same dimensions as the discrete_state
  /// allocated in CreateDefaultContext. The simulator will provide this state
  /// as the output argument to Update.
  /// By default, allocates nothing. Systems with discrete state variables
  /// should override.
  virtual std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables() const {
    return nullptr;
  }

  /// This convenience method allocates a context using AllocateContext() and
  /// sets its default values using SetDefaults().
  std::unique_ptr<Context<T>> CreateDefaultContext() const {
    std::unique_ptr<Context<T>> context = AllocateContext();
    SetDefaults(context.get());
    return context;
  }

  /// Assigns default values to all elements of the state. Overrides must not
  /// change the number of state variables.
  virtual void SetDefaultState(const Context<T>& context,
                               State<T>* state) const = 0;

  // Sets Context fields to their default values.  User code should not
  // override.
  virtual void SetDefaults(Context<T>* context) const = 0;

  /// For each input port, allocates a freestanding input of the concrete type
  /// that this System requires, and binds it to the port, disconnecting any
  /// prior input. Does not assign any values to the freestanding inputs.
  void AllocateFreestandingInputs(Context<T>* context) const {
    for (const auto& port : input_ports_) {
      if (port->get_data_type() == kVectorValued) {
        context->FixInputPort(port->get_index(), AllocateInputVector(*port));
      } else {
        DRAKE_DEMAND(port->get_data_type() == kAbstractValued);
        context->FixInputPort(port->get_index(), AllocateInputAbstract(*port));
      }
    }
  }

  /// Reports all direct feedthroughs from input ports to output ports. For
  /// a system with m input ports: `I = i₀, i₁, ..., iₘ₋₁`, and n output ports,
  /// `O = o₀, o₁, ..., oₙ₋₁`, the return map will contain pairs (u, v) such
  /// that
  ///     - 0 ≤ u < m,
  ///     - 0 ≤ v < n,
  ///     - and there _might_ be a direct feedthrough from input iᵤ to each
  ///       output oᵥ.
  virtual std::multimap<int, int> GetDirectFeedthroughs() const = 0;

  /// Returns `true` if any of the inputs to the system might be directly
  /// fed through to any of its outputs and `false` otherwise.
  bool HasAnyDirectFeedthrough() const {
    return GetDirectFeedthroughs().size() > 0;
  }

  /// Returns true if there might be direct-feedthrough from any input port to
  /// the given @p output_port, and false otherwise.
  bool HasDirectFeedthrough(int output_port) const {
    std::multimap<int, int> pairs = GetDirectFeedthroughs();
    for (const auto& pair : pairs) {
      if (pair.second == output_port) return true;
    }
    return false;
  }

  /// Returns true if there might be direct-feedthrough from the given
  /// @p input_port to the given @p output_port, and false otherwise.
  bool HasDirectFeedthrough(int input_port, int output_port) const {
    std::multimap<int, int> pairs = GetDirectFeedthroughs();
    auto range = pairs.equal_range(input_port);
    for (auto i = range.first; i != range.second; ++i) {
      if (i->second == output_port) return true;
    }
    return false;
  }

  //@}

  //----------------------------------------------------------------------------
  /// @name                        Publishing
  /// Publishing is the primary mechanism for a %System to communicate with
  /// the world outside the %System abstraction during a simulation. Publishing
  /// occurs at user-specified times or events and can generate side-effect
  /// results such as terminal output, visualization, logging, plotting, and
  /// network messages. Other than computational cost, publishing has no effect
  /// on the progress of a simulation.
  //@{

  /// This method is the public entry point for dispatching all publish event
  /// handlers. It checks the validity of @p context, and directly calls
  /// DispatchPublishHandler. @p events is a homogeneous collection of publish
  /// events, which is typically the publish portion of the heterogeneous
  /// event collection generated by CalcNextUpdateTime or GetPerStepEvents.
  ///
  /// @note When publishing is scheduled at particular times, those times likely
  /// will not coincide with integrator step times. A Simulator may interpolate
  /// to generate a suitable Context, or it may adjust the integrator step size
  /// so that a step begins exactly at the next publication time. In the latter
  /// case the change in step size may affect the numerical result somewhat
  /// since a smaller integrator step produces a more accurate solution.
  void Publish(const Context<T>& context,
               const EventCollection<PublishEvent<T>>& events) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DispatchPublishHandler(context, events);
  }

  /// Forces a publish on the system, given a @p context. The publish event will
  /// have a trigger type of kForced, with no additional data, attribute or
  /// custom callback. The Simulator can be configured to call this in
  /// Simulator::Initialize() and at the start of each continuous integration
  /// step. See the Simulator API for more details.
  void Publish(const Context<T>& context) const {
    Publish(context, this->get_forced_publish_events());
  }
  //@}

  //----------------------------------------------------------------------------
  /// @name                     Cached evaluations
  /// Given the values in a Context, a Drake %System must be able to provide
  /// the results of particular computations needed for analysis and simulation
  /// of the %System. These results are maintained in a mutable cache within
  /// the Context so that a result need be computed only once, the first time
  /// it is requested after a change to one of its prerequisite values.
  ///
  /// The `Eval` methods in this group return a reference to the
  /// already-computed result in the given Context's cache. If the current value
  /// is out of date, they first update the cache entry using the corresponding
  /// `Calc` method from the "Calculations" group. Evaluations of input ports
  /// instead delegate to the containing Diagram, which arranges to have the
  /// appropriate subsystem evaluate the source output port.
  //@{

  /// Returns a reference to the cached value of the conservative power. If
  /// necessary the cache will be updated first using CalcConservativePower().
  /// @see CalcConservativePower()
  const T& EvalConservativePower(const Context<T>& context) const {
    // TODO(sherm1) Replace with an actual cache entry.
    fake_cache_conservative_power_ = CalcConservativePower(context);
    return fake_cache_conservative_power_;
  }

  /// Returns a reference to the cached value of the non-conservative power. If
  /// necessary the cache will be updated first using
  /// CalcNonConservativePower().
  /// @see CalcNonConservativePower()
  const T& EvalNonConservativePower(const Context<T>& context) const {
    // TODO(sherm1) Replace with an actual cache entry.
    fake_cache_nonconservative_power_ = CalcNonConservativePower(context);
    return fake_cache_nonconservative_power_;
  }

  /// Causes the vector-valued input port with the given `port_index` to become
  /// up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's value, or nullptr if the port is not connected.
  ///
  /// Throws std::bad_cast if the port is not vector-valued. Returns nullptr if
  /// the port is vector valued, but not of type Vec. Aborts if the port
  /// does not exist.
  ///
  /// @tparam Vec The template type of the input vector, which must be a
  ///             subclass of BasicVector.
  template <template <typename> class Vec = BasicVector>
  const Vec<T>* EvalVectorInput(const Context<T>& context,
                                int port_index) const {
    static_assert(
        std::is_base_of<BasicVector<T>, Vec<T>>::value,
        "In EvalVectorInput<Vec>, Vec must be a subclass of BasicVector.");
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    return dynamic_cast<const Vec<T>*>(
        context.EvalVectorInput(parent_, get_input_port(port_index)));
  }

  /// Causes the vector-valued input port with the given `port_index` to become
  /// up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's value as an %Eigen expression.
  Eigen::VectorBlock<const VectorX<T>> EvalEigenVectorInput(
      const Context<T>& context, int port_index) const {
    const BasicVector<T>* input_vector = EvalVectorInput(context, port_index);
    DRAKE_ASSERT(input_vector != nullptr);
    DRAKE_ASSERT(input_vector->size() == get_input_port(port_index).size());
    return input_vector->get_value();
  }

  /// Causes the abstract-valued input port with the given `port_index` to
  /// become up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's abstract value pointer, or nullptr if the port is not
  /// connected.
  const AbstractValue* EvalAbstractInput(const Context<T>& context,
                                         int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    return context.EvalAbstractInput(parent_, get_input_port(port_index));
  }

  /// Causes the abstract-valued input port with the given `port_index` to
  /// become up-to-date, delegating to our parent Diagram if necessary. Returns
  /// the port's abstract value, or nullptr if the port is not connected.
  ///
  /// @tparam V The type of data expected.
  template <typename V>
  const V* EvalInputValue(const Context<T>& context, int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    return context.template EvalInputValue<V>(parent_,
                                              get_input_port(port_index));
  }
  //@}

  //----------------------------------------------------------------------------
  /// @name               Constraint-related functions.
  ///
  // @{

  /// Gets the number of constraint equations for this system using the given
  /// context (useful in case the number of constraints is dependent upon the
  /// current state (as might be the case with a system modeled using piecewise
  /// differential algebraic equations).
  int get_num_constraint_equations(const Context<T>& context) const {
    return do_get_num_constraint_equations(context);
  }

  /// Evaluates the constraint equations for the system at the generalized
  /// coordinates and generalized velocity specified by the context. The context
  /// allows the set of constraints to be dependent upon the current system
  /// state (as might be the case with a system modeled using piecewise
  /// differential algebraic equations).
  /// @returns a vector of dimension get_num_constraint_equations(); the
  ///          zero vector indicates that the algebraic constraints are all
  ///          satisfied.
  Eigen::VectorXd EvalConstraintEquations(const Context<T>& context) const {
    return DoEvalConstraintEquations(context);
  }

  /// Computes the time derivative of each constraint equation, evaluated at
  /// the generalized coordinates and generalized velocity specified by the
  /// context. The context allows the set of constraints to be dependent upon
  /// the current system state (as might be the case with a system modeled using
  /// piecewise differential algebraic equations).
  /// @returns a vector of dimension get_num_constraint_equations().
  Eigen::VectorXd EvalConstraintEquationsDot(const Context<T>& context) const {
    return DoEvalConstraintEquationsDot(context);
  }

  /// Computes the change in velocity from applying the given constraint forces
  /// to the system at the given context.
  /// @param context the current system state, provision of which also yields
  ///        the ability of the constraints to be dependent upon the current
  ///        system state (as might be the case with a piecewise differential
  ///        algebraic equation).
  /// @param J a m × n constraint Jacobian matrix of the `m` constraint
  ///          equations `g()` differentiated with respect to the `n`
  ///          configuration variables `q` (i.e., `J` should be `∂g/∂q`). If
  ///          the time derivatives of the generalized coordinates of the system
  ///          are not identical to the generalized velocity (in general they
  ///          need not be, e.g., if generalized coordinates use unit
  ///          unit quaternions to represent 3D orientation), `J` should instead
  ///          be defined as `∂g/∂q⋅N`, where `N ≡ ∂q/∂ꝗ` is the Jacobian matrix
  ///          (dependent on `q`) of the generalized coordinates with respect
  ///          to the quasi-coordinates (ꝗ, pronounced "qbar", where dꝗ/dt are
  ///          the generalized velocities).
  /// @param lambda the vector of constraint forces (of same dimension as the
  ///        number of rows in the Jacobian matrix, @p J)
  /// @returns a `n` dimensional vector, where `n` is the dimension of the
  ///          quasi-coordinates.
  Eigen::VectorXd CalcVelocityChangeFromConstraintImpulses(
      const Context<T>& context, const Eigen::MatrixXd& J,
      const Eigen::VectorXd& lambda) const {
    DRAKE_ASSERT(lambda.size() == get_num_constraint_equations(context));
    DRAKE_ASSERT(J.rows() == get_num_constraint_equations(context));
    DRAKE_ASSERT(
        J.cols() ==
        context.get_continuous_state()->get_generalized_velocity().size());
    return DoCalcVelocityChangeFromConstraintImpulses(context, J, lambda);
  }

  /// Computes the norm on constraint error (used as a metric for comparing
  /// errors between the outputs of algebraic equations applied to two
  /// different state variable instances). This norm need be neither continuous
  /// nor differentiable.
  /// @throws std::logic_error if the dimension of @p err is not equivalent to
  ///         the output of get_num_constraint_equations().
  double CalcConstraintErrorNorm(const Context<T>& context,
                                 const Eigen::VectorXd& error) const {
    if (error.size() != get_num_constraint_equations(context))
      throw std::logic_error("Error vector is mis-sized.");
    return DoCalcConstraintErrorNorm(context, error);
  }

  //@}

  //----------------------------------------------------------------------------
  /// @name                        Calculations
  /// A Drake %System defines a set of common computations that are understood
  /// by the framework. Most of these are embodied in a `Calc` method that
  /// unconditionally performs the calculation into an output argument of the
  /// appropriate type, using only values from the given Context. These are
  /// paired with an `Eval` method that returns a reference to an
  /// already-calculated result residing in the cache; if needed that result is
  /// first obtained using the `Calc` method. See the "Evaluations" group for
  /// more information.
  ///
  /// This group also includes additional %System-specific operations that
  /// depend on both Context and additional input arguments.
  //@{

  /// Calculates the time derivatives `xcdot` of the continuous state `xc`.
  /// The `derivatives` vector will correspond elementwise with the continuous
  /// state in the given Context. Thus, if the state in
  /// the Context has second-order structure `xc=[q v z]`, that same structure
  /// applies to the derivatives so we will have `xcdot=[qdot vdot zdot]`.
  ///
  /// @param context The Context whose time, input port, parameter, and state
  /// values are used to evaluate the derivatives.
  ///
  /// @param derivatives The time derivatives `xcdot`. Must be the same size as
  ///                    the continuous state vector in `context`.
  void CalcTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const {
    DRAKE_DEMAND(derivatives != nullptr);
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DoCalcTimeDerivatives(context, derivatives);
  }

  void ProjectQ(Context<T>* context) const {
    DRAKE_DEMAND(context != nullptr);
    DRAKE_ASSERT_VOID(CheckValidContext(*context));
    DoProjectQ(context);
  }

  /// This method is the public entry point for dispatching all discrete
  /// variable update event handlers. Using all the discrete update handlers in
  /// @p events, the method calculates the update `xd(n+1)` to discrete
  /// variables `xd(n)` in @p context and outputs the results to @p
  /// discrete_state. See documentation for
  /// DispatchDiscreteVariableUpdateHandler() for more details.
  void CalcDiscreteVariableUpdates(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));

    DispatchDiscreteVariableUpdateHandler(context, events, discrete_state);
  }

  /// This method forces a discrete update on the system given a @p context,
  /// and the updated discrete state is stored in @p discrete_state. The
  /// discrete update event will have a trigger type of kForced, with no
  /// attribute or custom callback.
  void CalcDiscreteVariableUpdates(const Context<T>& context,
                                   DiscreteValues<T>* discrete_state) const {
    CalcDiscreteVariableUpdates(
        context, this->get_forced_discrete_update_events(), discrete_state);
  }

  /// This method is the public entry point for dispatching all unrestricted
  /// update event handlers. Using all the unrestricted update handers in
  /// @p events, it updates *any* state variables in the @p context, and
  /// outputs the results to @p state. It does not allow the dimensionality
  /// of the state variables to change. See the documentation for
  /// DispatchUnrestrictedUpdateHandler() for more details.
  ///
  /// @throws std::logic_error if the dimensionality of the state variables
  ///         changes in the callback.
  void CalcUnrestrictedUpdate(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    const int continuous_state_dim = state->get_continuous_state()->size();
    const int discrete_state_dim = state->get_discrete_state()->num_groups();
    const int abstract_state_dim = state->get_abstract_state()->size();

    // Copy current state to the passed-in state, as specified in the
    // documentation for DoCalcUnrestrictedUpdate().
    state->CopyFrom(context.get_state());

    DispatchUnrestrictedUpdateHandler(context, events, state);

    if (continuous_state_dim != state->get_continuous_state()->size() ||
        discrete_state_dim != state->get_discrete_state()->num_groups() ||
        abstract_state_dim != state->get_abstract_state()->size())
      throw std::logic_error(
          "State variable dimensions cannot be changed "
          "in CalcUnrestrictedUpdate().");
  }

  /// This method forces an unrestricted update on the system given a
  /// @p context, and the updated state is stored in @p discrete_state. The
  /// unrestricted update event will have a trigger type of kForced, with no
  /// additional data, attribute or custom callback.
  ///
  /// @sa CalcUnrestrictedUpdate(const Context<T>&, const
  /// EventCollection<UnrestrictedUpdateEvent<T>>*, State<T>* state)
  ///     for more information.
  void CalcUnrestrictedUpdate(const Context<T>& context,
                              State<T>* state) const {
    CalcUnrestrictedUpdate(
        context, this->get_forced_unrestricted_update_events(), state);
  }

  /// This method is called by a Simulator during its calculation of the size of
  /// the next continuous step to attempt. The System returns the next time at
  /// which some discrete action must be taken, and records what those actions
  /// ought to be in @p events. Upon reaching that time, the simulator will
  /// merge @p events with the other CompositeEventCollection instances
  /// scheduled through mechanisms (e.g. GetPerStepEvents()), and the merged
  /// CompositeEventCollection will be passed to all event handling mechanisms.
  ///
  /// @p events cannot be null. @p events will be cleared on entry.
  T CalcNextUpdateTime(const Context<T>& context,
                       CompositeEventCollection<T>* events) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_DEMAND(events != nullptr);
    events->Clear();
    T time;
    DoCalcNextUpdateTime(context, events, &time);
    return time;
  }

  /// This method is called by Simulator::Initialize() to gather all
  /// update and publish events that are to be handled in StepTo() at the point
  /// before Simulator integrates continuous state. It is assumed that these
  /// events remain constant throughout the simulation. The "step" here refers
  /// to the major time step taken by the Simulator. During every simulation
  /// step, the simulator will merge @p events with the other
  /// CompositeEventCollection instances generated by other types of event
  /// triggering mechanism (e.g., CalcNextUpdateTime()), and the merged
  /// CompositeEventCollection objects will be passed to the appropriate
  /// handlers before Simulator integrates the continuous state.
  ///
  /// @p events cannot be null. @p events will be cleared on entry.
  void GetPerStepEvents(const Context<T>& context,
                        CompositeEventCollection<T>* events) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_DEMAND(events != nullptr);
    events->Clear();
    DoGetPerStepEvents(context, events);
  }

  /// Utility method that computes for _every_ output port i the value y(i) that
  /// should result from the current contents of the given Context. Note that
  /// individual output port values can be calculated using
  /// `get_output_port(i).Calc()`; this method invokes that for each output port
  /// in index order. The result may depend on time and the current values of
  /// input ports, parameters, and state variables. The result is written to
  /// `outputs` which must already have been allocated to have the right number
  /// of entries of the right types.
  void CalcOutput(const Context<T>& context, SystemOutput<T>* outputs) const {
    DRAKE_DEMAND(outputs != nullptr);
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_ASSERT_VOID(CheckValidOutput(outputs));
    for (OutputPortIndex i(0); i < get_num_output_ports(); ++i) {
      get_output_port(i).Calc(
          context, outputs->get_mutable_port_value(i)->GetMutableData());
    }
  }

  /// Calculates and returns the potential energy current stored in the
  /// configuration provided in `context`. Non-physical Systems will return
  /// zero.
  /// @see EvalPotentialEnergy()
  T CalcPotentialEnergy(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcPotentialEnergy(context);
  }

  /// Calculates and returns the kinetic energy currently present in the motion
  /// provided in the given Context. Non-physical Systems will return zero.
  /// @see EvalKineticEnergy()
  T CalcKineticEnergy(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcKineticEnergy(context);
  }

  /// Calculates and returns the rate at which mechanical energy is being
  /// converted *from* potential energy *to* kinetic energy by this system in
  /// the given Context. This quantity will be positive when potential energy is
  /// decreasing. Note that kinetic energy will also be affected by
  /// non-conservative forces so we can't say whether it is increasing or
  /// decreasing in an absolute sense, only whether the conservative
  /// power is increasing or decreasing the kinetic energy. Power is in watts
  /// (J/s).Non-physical Systems will return zero.
  /// @see EvalConservativePower()
  T CalcConservativePower(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcConservativePower(context);
  }

  /// Calculates and returns the rate at which mechanical energy is being
  /// generated (positive) or dissipated (negative) *other than* by conversion
  /// between potential and kinetic energy (in the given Context). Integrating
  /// this quantity yields work W, and the total energy `E=PE+KE-W` should be
  /// conserved by any physically-correct model, to within integration accuracy
  /// of W. Power is in watts (J/s). (Watts are abbreviated W but not to be
  /// confused with work!) This method is meaningful only for physical systems;
  /// others return zero.
  /// @see EvalNonConservativePower()
  T CalcNonConservativePower(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcNonConservativePower(context);
  }

  /// Transforms a given generalized velocity `v` to the time derivative `qdot`
  /// of the generalized configuration `q` taken from the supplied Context.
  /// `v` and `qdot` are related linearly by `qdot = N(q) * v`, where `N` is a
  /// block diagonal matrix. For example, in a multibody system there will be
  /// one block of `N` per tree joint. This computation requires only `O(nq)`
  /// time where `nq` is the size of `qdot`. Note that `v` is *not* taken from
  /// the Context; it is given as an argument here.
  ///
  /// See the alternate signature if you already have the generalized
  /// velocity in an Eigen VectorX object; this signature will copy the
  /// VectorBase into an Eigen object before performing the computation.
  /// @see MapQDotToVelocity()
  void MapVelocityToQDot(const Context<T>& context,
                         const VectorBase<T>& generalized_velocity,
                         VectorBase<T>* qdot) const {
    MapVelocityToQDot(context, generalized_velocity.CopyToVector(), qdot);
  }

  /// Transforms the given generalized velocity to the time derivative of
  /// generalized configuration. See the other signature of MapVelocityToQDot()
  /// for more information.
  void MapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* qdot) const {
    DoMapVelocityToQDot(context, generalized_velocity, qdot);
  }

  /// Transforms the time derivative `qdot` of the generalized configuration `q`
  /// to generalized velocities `v`. `v` and `qdot` are related linearly by
  /// `qdot = N(q) * v`, where `N` is a block diagonal matrix. For example, in a
  /// multibody system there will be one block of `N` per tree joint. Although
  /// `N` is not necessarily square, its left pseudo-inverse `N+` can be used to
  /// invert that relationship without residual error, provided that `qdot` is
  /// in the range space of `N` (that is, if it *could* have been produced as
  /// `qdot=N*v` for some `v`). Using the configuration `q` from the given
  /// Context this method calculates `v = N+ * qdot` (where `N+=N+(q)`) for
  /// a given `qdot`. This computation requires only `O(nq)` time where `nq` is
  /// the size of `qdot`. Note that this method does not take `qdot` from the
  /// Context.
  ///
  /// See the alternate signature if you already have `qdot` in an %Eigen
  /// VectorX object; this signature will copy the VectorBase into an %Eigen
  /// object before performing the computation.
  /// @see MapVelocityToQDot()
  void MapQDotToVelocity(const Context<T>& context, const VectorBase<T>& qdot,
                         VectorBase<T>* generalized_velocity) const {
    MapQDotToVelocity(context, qdot.CopyToVector(), generalized_velocity);
  }

  /// Transforms the given time derivative `qdot` of generalized configuration
  /// `q` to generalized velocity `v`. This signature takes `qdot` as an %Eigen
  /// VectorX object for faster speed. See the other signature of
  /// MapQDotToVelocity() for additional information.
  void MapQDotToVelocity(const Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         VectorBase<T>* generalized_velocity) const {
    DoMapQDotToVelocity(context, qdot, generalized_velocity);
  }

  // TODO(edrumwri): MapAccelerationToQDotDot.
  //@}

  //----------------------------------------------------------------------------
  /// @cond
  // Functions to avoid RTTI in Diagram. Conceptually, these should be protected
  // and should not be directly called, so they are hidden from doxygen.

  // TODO(siyuan): change all target_system to reference.

  // Returns @p context if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual Context<T>* DoGetMutableTargetSystemContext(
      const System<T>& target_system, Context<T>* context) const {
    if (&target_system == this) return context;
    return nullptr;
  }

  // Returns @p context if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual const Context<T>* DoGetTargetSystemContext(
      const System<T>& target_system, const Context<T>* context) const {
    if (&target_system == this) return context;
    return nullptr;
  }

  // Returns @p state if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual State<T>* DoGetMutableTargetSystemState(
      const System<T>& target_system, State<T>* state) const {
    if (&target_system == this) return state;
    return nullptr;
  }

  /// Returns @p state if @p target_system equals `this`, nullptr otherwise.
  /// Should not be directly called.
  virtual const State<T>* DoGetTargetSystemState(const System<T>& target_system,
                                                 const State<T>* state) const {
    if (&target_system == this) return state;
    return nullptr;
  }

  // Returns @p events if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual CompositeEventCollection<T>*
  DoGetMutableTargetSystemCompositeEventCollection(
      const System<T>& target_system,
      CompositeEventCollection<T>* events) const {
    if (&target_system == this) return events;
    return nullptr;
  }

  // Returns @p events if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual const CompositeEventCollection<T>*
  DoGetTargetSystemCompositeEventCollection(
      const System<T>& target_system,
      const CompositeEventCollection<T>* events) const {
    if (&target_system == this) return events;
    return nullptr;
  }

  // The derived class implementation should provide exactly one event of the
  // appropriate type with a kForced trigger type.
  // Consumers of this class should never need to call the three methods below.
  // These three methods would ideally be designated as "protected", but
  // Diagram::AllocateForcedXEventCollection() needs to call these methods and,
  // perhaps surprisingly, is not able to access these methods when they are
  // protected. See: https://stackoverflow.com/questions/16785069/why-cant-a-derived-class-call-protected-member-function-in-this-code.
  // To address this problem, we keep the methods "public" and
  // (1) Make the overriding methods in LeafSystem and Diagram "final" and
  // (2) Use the doxygen cond/endcond tags so that these methods are hidden
  //     from the user (in the doxygen documentation).
  virtual std::unique_ptr<EventCollection<PublishEvent<T>>>
  AllocateForcedPublishEventCollection() const = 0;

  virtual std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
  AllocateForcedDiscreteUpdateEventCollection() const = 0;

  virtual std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
  AllocateForcedUnrestrictedUpdateEventCollection() const = 0;
  /// @endcond

  //----------------------------------------------------------------------------
  /// @name                      Utility methods
  //@{

  /// Sets the name of the system. It is recommended that the name not include
  /// the character ':', since the path delimiter is "::". When creating a
  /// Diagram, names of sibling subsystems should be unique.
  void set_name(const std::string& name) { name_ = name; }

  /// Returns the name last supplied to set_name(), or empty if set_name() was
  /// never called.  Systems created through transmogrification have by default
  /// an identical name to the system they were created from.
  std::string get_name() const { return name_; }

  /// Returns a name for this %System based on a stringification of its type
  /// name and memory address.  This is intended for use in diagnostic output
  /// and should not be used for behavioral logic, because the stringification
  /// of the type name may produce differing results across platforms and
  /// because the address can vary from run to run.
  std::string GetMemoryObjectName() const {
    return SystemImpl::GetMemoryObjectName(NiceTypeName::Get(*this),
                                           GetGraphvizId());
  }

  /// Writes the full path of this System in the tree of Systems to @p output.
  /// The path has the form (::ancestor_system_name)*::this_system_name.
  void GetPath(std::stringstream* output) const {
    // If this System has a parent, that parent's path is a prefix to this
    // System's path. Otherwise, this is the root system and there is no prefix.
    if (parent_ != nullptr) {
      parent_->GetPath(output);
    }
    *output << "::" << (get_name().empty() ? "_" : get_name());
  }

  // Returns the full path of the System in the tree of Systems.
  std::string GetPath() const {
    std::stringstream path;
    GetPath(&path);
    return path.str();
  }

  /// Returns the number of input ports of the system.
  int get_num_input_ports() const {
    return static_cast<int>(input_ports_.size());
  }

  /// Returns the number of output ports of the system.
  int get_num_output_ports() const {
    return static_cast<int>(output_ports_.size());
  }

  /// Returns the descriptor of the input port at index @p port_index.
  const InputPortDescriptor<T>& get_input_port(int port_index) const {
    if (port_index < 0 || port_index >= get_num_input_ports()) {
      throw std::out_of_range(
          "System " + get_name() + ": Port index " +
          std::to_string(port_index) + " is out of range. There are only " +
          std::to_string(get_num_input_ports()) + " input ports.");
    }
    return *input_ports_[port_index];
  }

  /// Returns the output port at index @p port_index.
  const OutputPort<T>& get_output_port(int port_index) const {
    if (port_index < 0 || port_index >= get_num_output_ports()) {
      throw std::out_of_range(
          "System " + get_name() + ": Port index " +
          std::to_string(port_index) + " is out of range. There are only " +
          std::to_string(get_num_output_ports()) + " output ports.");
    }
    return *output_ports_[port_index];
  }

  /// Returns the number of constraints specified for the system.
  int get_num_constraints() const {
    return static_cast<int>(constraints_.size());
  }

  /// Returns the constraint at index @p constraint_index.
  /// @throws std::out_of_range for an invalid constraint_index.
  const SystemConstraint<T>& get_constraint(
      SystemConstraintIndex constraint_index) const {
    if (constraint_index < 0 || constraint_index >= get_num_constraints()) {
      throw std::out_of_range("System " + get_name() + ": Constraint index " +
                              std::to_string(constraint_index) +
                              " is out of range. There are only " +
                              std::to_string(get_num_constraints()) +
                              " constraints.");
    }
    return *constraints_[constraint_index];
  }

  /// Returns the total dimension of all of the input ports (as if they were
  /// muxed).
  int get_num_total_inputs() const {
    int count = 0;
    for (const auto& in : input_ports_) count += in->size();
    return count;
  }

  /// Returns the total dimension of all of the output ports (as if they were
  /// muxed).
  int get_num_total_outputs() const {
    int count = 0;
    for (const auto& out : output_ports_) count += out->size();
    return count;
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
        DRAKE_THROW_UNLESS(output_vector->size() == get_output_port(i).size());
      }
    }
  }

  /// Checks that @p context is consistent for this System template. Supports
  /// any scalar type, but expects T by default.
  ///
  /// @throw exception unless `context` is valid for this system.
  /// @tparam T1 the scalar type of the Context to check.
  template <typename T1 = T>
  void CheckValidContext(const Context<T1>& context) const {
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

  /// Returns a copy of the continuous state vector `xc` into an Eigen vector.
  VectorX<T> CopyContinuousStateVector(const Context<T>& context) const {
    DRAKE_ASSERT(context.get_continuous_state() != nullptr);
    return context.get_continuous_state()->CopyToVector();
  }

  /// Declares that `parent` is the immediately enclosing Diagram. The
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
  //@}

  //----------------------------------------------------------------------------
  /// @name                      Graphviz methods
  //@{

  /// Returns a Graphviz string describing this System.  To render the string,
  /// use the Graphviz tool, ``dot``.
  /// http://www.graphviz.org/Documentation/dotguide.pdf
  std::string GetGraphvizString() const {
    std::stringstream dot;
    dot << "digraph _" << this->GetGraphvizId() << " {" << std::endl;
    dot << "rankdir=LR" << std::endl;
    GetGraphvizFragment(&dot);
    dot << "}" << std::endl;
    return dot.str();
  }

  /// Appends a Graphviz fragment to the @p dot stream.  The fragment must be
  /// valid Graphviz when wrapped in a `digraph` or `subgraph` stanza.  Does
  /// nothing by default.
  virtual void GetGraphvizFragment(std::stringstream* dot) const {
    unused(dot);
  }

  /// Appends a fragment to the @p dot stream identifying the graphviz node
  /// representing @p port. Does nothing by default.
  virtual void GetGraphvizInputPortToken(const InputPortDescriptor<T>& port,
                                         std::stringstream* dot) const {
    unused(port, dot);
  }

  /// Appends a fragment to the @p dot stream identifying the graphviz node
  /// representing @p port. Does nothing by default.
  virtual void GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                          std::stringstream* dot) const {
    unused(port, dot);
  }

  /// Returns an opaque integer that uniquely identifies this system in the
  /// Graphviz output.
  int64_t GetGraphvizId() const { return reinterpret_cast<int64_t>(this); }

  //@}

  //----------------------------------------------------------------------------
  /// @name                Automatic differentiation
  /// From a %System templatized by `double`, you can obtain an identical system
  /// templatized by an automatic differentation scalar providing
  /// machine-precision computation of partial derivatives of any numerical
  /// result of the %System with respect to any of the numerical values that
  /// can be contained in a Context (time, inputs, parameters, and state).

  // This group appears as a top-level heading in Doxygen because it contains
  // both static and non-static member functions.
  //@{

  /// Creates a deep copy of this System, transmogrified to use the autodiff
  /// scalar type, with a dynamic-sized vector of partial derivatives.  The
  /// result is never nullptr.
  /// @throw exception if this System does not support autodiff
  std::unique_ptr<System<AutoDiffXd>> ToAutoDiffXd() const {
    return System<T>::ToAutoDiffXd(*this);
  }

  /// Creates a deep copy of `from`, transmogrified to use the autodiff scalar
  /// type, with a dynamic-sized vector of partial derivatives.  The result is
  /// never nullptr.
  /// @throw exception if `from` does not support autodiff
  ///
  /// Usage: @code
  ///   MySystem<double> plant;
  ///   std::unique_ptr<MySystem<AutoDiffXd>> ad_plant =
  ///       systems::System<double>::ToAutoDiffXd(plant);
  /// @endcode
  ///
  /// @tparam S The specific System type to accept and return.
  template <template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<AutoDiffXd>> ToAutoDiffXd(const S<T>& from) {
    using U = AutoDiffXd;
    const System<T>& from_system = from;  // Upcast to unlock protected methods.
    std::unique_ptr<System<U>> base_result{from_system.DoToAutoDiffXd()};
    if (!base_result) {
      std::stringstream ss;
      ss << "The object named [" << from.get_name() << "] of type"
         << NiceTypeName::Get(from) << " does not support ToAutoDiffXd.";
      throw std::logic_error(ss.str().c_str());
    }

    // Downcast to the derived type S (throwing on error), and then transfer
    // ownership to a correctly-typed unique_ptr.
    // NOLINTNEXTLINE(runtime/casting)
    std::unique_ptr<S<U>> result{&dynamic_cast<S<U>&>(*base_result)};
    base_result.release();

    // Match the result's name to its originator.
    result->set_name(from.get_name());
    return result;
  }

  /// Creates a deep copy of this system exactly like ToAutoDiffXd(), but
  /// returns nullptr if this System does not support autodiff, instead of
  /// throwing an exception.
  std::unique_ptr<System<AutoDiffXd>> ToAutoDiffXdMaybe() const {
    std::unique_ptr<System<AutoDiffXd>> result{DoToAutoDiffXd()};
    if (result) {
      // Match the result's name to its originator.
      result->set_name(this->get_name());
    }
    return result;
  }
  //@}

  //----------------------------------------------------------------------------
  /// @name                Symbolics
  /// From a %System templatized by `double`, you can obtain an identical system
  /// templatized by a symbolic expression scalar.

  // This group appears as a top-level heading in Doxygen because it contains
  // both static and non-static member functions.
  //@{

  /// Creates a deep copy of this System, transmogrified to use the symbolic
  /// scalar type. The result is never nullptr.
  /// @throw exception if this System does not support symbolic
  std::unique_ptr<System<symbolic::Expression>> ToSymbolic() const {
    return System<T>::ToSymbolic(*this);
  }

  /// Creates a deep copy of `from`, transmogrified to use the symbolic scalar
  /// type. The result is never nullptr.
  /// @throw exception if this System does not support symbolic
  ///
  /// Usage: @code
  ///   MySystem<double> plant;
  ///   std::unique_ptr<MySystem<symbolic::Expression>> sym_plant =
  ///       systems::System<double>::ToSymbolic(plant);
  /// @endcode
  ///
  /// @tparam S The specific System pointer type to return.
  template <template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<symbolic::Expression>> ToSymbolic(const S<T>& from) {
    using U = symbolic::Expression;
    const System<T>& from_system = from;  // Upcast to unlock protected methods.
    std::unique_ptr<System<U>> base_result{from_system.DoToSymbolic()};
    if (!base_result) {
      std::stringstream ss;
      ss << "The object named [" << from.get_name() << "] of type"
         << NiceTypeName::Get(from) << " does not support ToSymbolic.";
      throw std::logic_error(ss.str().c_str());
    }

    // Downcast to the derived type S (throwing on error), and then transfer
    // ownership to a correctly-typed unique_ptr.
    // NOLINTNEXTLINE(runtime/casting)
    std::unique_ptr<S<U>> result{&dynamic_cast<S<U>&>(*base_result)};
    base_result.release();

    // Match the result's name to its originator.
    result->set_name(from.get_name());
    return result;
  }

  /// Creates a deep copy of this system exactly like ToSymbolic(), but returns
  /// nullptr if this System does not support symbolic, instead of throwing an
  /// exception.
  std::unique_ptr<System<symbolic::Expression>> ToSymbolicMaybe() const {
    std::unique_ptr<System<symbolic::Expression>> result{DoToSymbolic()};
    if (result) {
      // Match the result's name to its originator.
      result->set_name(this->get_name());
    }
    return result;
  }
  //@}

  //----------------------------------------------------------------------------
  /// @name                Transmogrification utilities

  /// Fixes all of the input ports in @p target_context to their current values
  /// in @p other_context, as evaluated by @p other_system. Throws an exception
  /// unless `other_context` and `target_context` both have the same shape as
  /// this System, and the `other_system`. Ignores disconnected inputs.
  void FixInputPortsFrom(const System<double>& other_system,
                         const Context<double>& other_context,
                         Context<T>* target_context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(other_context));
    DRAKE_ASSERT_VOID(CheckValidContext(*target_context));
    DRAKE_ASSERT_VOID(other_system.CheckValidContext(other_context));
    DRAKE_ASSERT_VOID(other_system.CheckValidContext(*target_context));

    for (int i = 0; i < get_num_input_ports(); ++i) {
      const auto& descriptor = get_input_port(i);

      if (descriptor.get_data_type() == kVectorValued) {
        // For vector-valued input ports, we placewise initialize a fixed input
        // vector using the explicit conversion from double to T.
        const BasicVector<double>* other_vec =
            other_system.EvalVectorInput(other_context, i);
        if (other_vec == nullptr) continue;
        auto our_vec = this->AllocateInputVector(descriptor);
        for (int j = 0; j < our_vec->size(); ++j) {
          our_vec->SetAtIndex(j, T(other_vec->GetAtIndex(j)));
        }
        target_context->FixInputPort(i, std::move(our_vec));
      } else if (descriptor.get_data_type() == kAbstractValued) {
        // For abstract-valued input ports, we just clone the value and fix
        // it to the port.
        const AbstractValue* other_value =
            other_system.EvalAbstractInput(other_context, i);
        if (other_value == nullptr) continue;
        target_context->FixInputPort(i, other_value->Clone());
      } else {
        DRAKE_ABORT_MSG("Unknown descriptor type.");
      }
    }
  }

  //@}

  /// Gets the witness functions active at the beginning of a continuous time
  /// interval. DoGetWitnessFunctions() does the actual work.
  /// @param context a valid context for the System (aborts if not true).
  /// @param[out] w a valid pointer to an empty vector that will store
  ///             pointers to the witness functions active at the beginning of
  ///             the continuous time interval. The method aborts if witnesses
  ///             is null or non-empty.
  void GetWitnessFunctions(const Context<T>& context,
                           std::vector<const WitnessFunction<T>*>* w) const {
    DRAKE_DEMAND(w);
    DRAKE_DEMAND(w->empty());
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DoGetWitnessFunctions(context, w);
  }

  /// Evaluates a witness function at the given context.
  T EvaluateWitness(const Context<T>& context,
                    const WitnessFunction<T>& witness_func) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoEvaluateWitness(context, witness_func);
  }

  /// Add @p witness_func to @p events. @p events cannot be nullptr. @p events
  /// should be allocated with this system's AllocateCompositeEventCollection.
  /// The system associated with @p witness_func has to be either `this` or a
  /// subsystem of `this` depending on whether `this` is a LeafSystem or
  /// a Diagram.
  virtual void AddTriggeredWitnessFunctionToCompositeEventCollection(
      const WitnessFunction<T>& witness_func,
      CompositeEventCollection<T>* events) const = 0;

  /// Returns a string suitable for identifying this particular %System in
  /// error messages, when it is a subsystem of a larger Diagram. This method
  /// captures human-readable subsystem identification best practice; the
  /// specifics of that are likely to change over time. However it will always
  /// be formatted like "System xxx" or "adjective System xxx" so that the
  /// remainder of the error message will continue to make sense. Currently it
  /// returns "system_type_name System subsystem_pathname".
  // TODO(sherm1) Remove the system type noise once the subsystem path is
  // a fully reliable identifier.
  std::string GetSystemIdString() const {
    return NiceTypeName::Get(*this) + " System " + GetPath();
  }

 protected:
  /// Derived classes will implement this method to evaluate a witness function
  /// at the given context.
  virtual T DoEvaluateWitness(const Context<T>& context,
                              const WitnessFunction<T>& witness_func) const = 0;

  /// Derived classes can override this method to provide witness functions
  /// active at the beginning of a continuous time interval. The default
  /// implementation does nothing. On entry to this function, the context will
  /// have already been validated and the vector of witness functions will have
  /// been validated to be both empty and non-null.
  virtual void DoGetWitnessFunctions(const Context<T>&,
      std::vector<const WitnessFunction<T>*>*) const {
  }

  //----------------------------------------------------------------------------
  /// @name                 Event handler dispatch mechanism
  /// For a LeafSystem (or user implemented equivalent classes), these functions
  /// need to call the appropriate LeafSystem::DoX event handler. E.g.
  /// LeafSystem::DispatchPublishHandler() calls LeafSystem::DoPublish(). User
  /// supplied custom event callbacks embedded in each individual event need to
  /// be further dispatched in the LeafSystem::DoX handlers if desired. For a
  /// LeafSystem, the pseudo code of the complete default publish event handler
  /// dispatching is roughly:
  /// <pre>
  ///   leaf_sys.Publish(context, event_collection)
  ///   -> leaf_sys.DispatchPublishHandler(context, event_collection)
  ///      -> leaf_sys.DoPublish(context, event_collection.get_events())
  ///         -> for (event : event_collection_events):
  ///              if (event.has_handler)
  ///                event.handler(context)
  /// </pre>
  /// Discrete update events and unrestricted update events are dispatched
  /// similarly for a LeafSystem.
  ///
  /// For a Diagram (or user implemented equivalent classes), these functions
  /// must iterate through all subsystems, extract their corresponding
  /// subcontext and subevent collections from @p context and @p events,
  /// and pass those to the subsystems' public non-virtual event handlers if
  /// the subevent collection is nonempty (e.g. System::Publish() for publish
  /// events).
  ///
  /// All of these functions are only called from their corresponding public
  /// non-virtual event dispatchers, where @p context is error checked. The
  /// derived implementations can assume that @p context is valid. See, e.g.,
  /// LeafSystem::DispatchPublishHandler() and Diagram::DispatchPublishHandler()
  /// for more details.

  //@{
  /// This function dispatches all publish events to the appropriate handlers.
  virtual void DispatchPublishHandler(
      const Context<T>& context,
      const EventCollection<PublishEvent<T>>& events) const = 0;

  /// This function dispatches all discrete update events to the appropriate
  /// handlers. @p discrete_state cannot be null.
  virtual void DispatchDiscreteVariableUpdateHandler(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state) const = 0;

  /// This function dispatches all unrestricted update events to the appropriate
  /// handlers. @p state cannot be null.
  virtual void DispatchUnrestrictedUpdateHandler(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state) const = 0;
  //@}

  //----------------------------------------------------------------------------
  /// @name                 System construction
  /// Authors of derived %Systems can use these methods in the constructor
  /// for those %Systems.
  //@{
  /// Constructs an empty %System base class object.
  System() {}

  /// Adds a port with the specified @p type and @p size to the input topology.
  /// @return descriptor of declared port.
  const InputPortDescriptor<T>& DeclareInputPort(PortDataType type, int size) {
    int port_index = get_num_input_ports();
    input_ports_.push_back(
        std::make_unique<InputPortDescriptor<T>>(this, port_index, type, size));
    return *input_ports_.back();
  }

  /// Adds an abstract-valued port to the input topology.
  /// @return descriptor of declared port.
  const InputPortDescriptor<T>& DeclareAbstractInputPort() {
    return DeclareInputPort(kAbstractValued, 0 /* size */);
  }

  /// Adds an already-created output port to this System. Insists that the port
  /// already contains a reference to this System, and that the port's index is
  /// already set to the next available output port index for this System.
  void CreateOutputPort(std::unique_ptr<OutputPort<T>> port) {
    DRAKE_DEMAND(port != nullptr);
    DRAKE_DEMAND(&port->get_system() == this);
    DRAKE_DEMAND(port->get_index() == this->get_num_output_ports());
    output_ports_.push_back(std::move(port));
  }
  //@}

  /// Adds an already-created constraint to the list of constraints for this
  /// System.  Ownership of the SystemConstraint is transferred to this system.
  SystemConstraintIndex AddConstraint(
      std::unique_ptr<SystemConstraint<T>> constraint) {
    DRAKE_DEMAND(constraint != nullptr);
    constraints_.push_back(std::move(constraint));
    return SystemConstraintIndex(constraints_.size() - 1);
  }

  //----------------------------------------------------------------------------
  /// @name               Virtual methods for input allocation
  /// Authors of derived %Systems should override these methods to self-describe
  /// acceptable inputs to the %System.

  /// Allocates an input vector of the leaf type that the System requires on
  /// the port specified by @p descriptor. Caller owns the returned memory.
  virtual BasicVector<T>* DoAllocateInputVector(
      const InputPortDescriptor<T>& descriptor) const = 0;

  /// Allocates an abstract input of the leaf type that the System requires on
  /// the port specified by @p descriptor. Caller owns the returned memory.
  virtual AbstractValue* DoAllocateInputAbstract(
      const InputPortDescriptor<T>& descriptor) const = 0;
  //@}

  //----------------------------------------------------------------------------
  /// @name               Virtual methods for calculations
  /// These virtuals allow concrete systems to implement the calculations
  /// defined by the `Calc` methods in the public interface. Most have default
  /// implementations that are usable for simple systems, but you are likely
  /// to need to override some or all of these in your concrete system to
  /// produce meaningful calculations.
  ///
  /// These methods are invoked by the corresponding method in the public
  /// interface that has the same name with `Do` removed. The public method
  /// performs error checking on the arguments so you do not need to do so in
  /// your implementation. Users cannot invoke these directly since they are
  /// protected. You should place your overrides in the protected or private
  /// sections of your concrete class.
  //@{

  /// Override this if you have any continuous state variables `xc` in your
  /// concrete %System to calculate their time derivatives.
  /// The `derivatives` vector will correspond elementwise with the state
  /// vector Context.state.continuous_state.get_state(). Thus, if the state in
  /// the Context has second-order structure `xc=[q,v,z]`, that same structure
  /// applies to the derivatives.
  ///
  /// This method is called only from the public non-virtual
  /// CalcTimeDerivatives() which will already have error-checked
  /// the parameters so you don't have to.
  /// In particular, implementations may assume that the given Context is valid
  /// for this %System; that the `derivatives` pointer is non-null, and that
  /// the referenced object has the same constituent structure as was
  /// produced by AllocateTimeDerivatives().
  ///
  /// The default implementation does nothing if the `derivatives` vector is
  /// size zero and aborts otherwise.
  virtual void DoCalcTimeDerivatives(const Context<T>& context,
                                     ContinuousState<T>* derivatives) const {
    // This default implementation is only valid for Systems with no continuous
    // state. Other Systems must override this method!
    unused(context);
    DRAKE_DEMAND(derivatives->size() == 0);
  }

  virtual void DoProjectQ(Context<T>* context) const {
    // This default implementation is only valid for Systems with nothing to
    // project.
    unused(context);
    const int num_positions =
        context->get_continuous_state()->get_generalized_position().size();
    const int num_velocities =
        context->get_continuous_state()->get_generalized_velocity().size();
    DRAKE_DEMAND(num_positions == num_velocities);
  }

  /// Computes the next time at which this System must perform a discrete
  /// action.
  ///
  /// Override this method if your System has any discrete actions which must
  /// interrupt the continuous simulation. This method is called only from the
  /// public non-virtual CalcNextUpdateTime() which will already have
  /// error-checked the parameters so you don't have to. You may assume that
  /// @p context has already been validated and @p events pointer is not
  /// null.
  ///
  /// The default implementation returns with the next sample time being
  /// Infinity and no events added to @p events.
  virtual void DoCalcNextUpdateTime(const Context<T>& context,
                                    CompositeEventCollection<T>* events,
                                    T* time) const {
    unused(context, events);
    *time = std::numeric_limits<T>::infinity();
  }

  /// Implement this method to return any events to be handled before the
  /// simulator integrates the system's continuous state at each time step.
  /// @p events is cleared in the public non-virtual GetPerStepEvents()
  /// before that method calls this function. An overriding implementation
  /// of this method should not clear @p events, and only append to it. You
  /// may assume that @p context has already been validated and that
  /// @p events is not null. @p events can be changed freely by the overriding
  /// implementation.
  ///
  /// The default implementation returns without changing @p events.
  /// @sa GetPerStepEvents()
  virtual void DoGetPerStepEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* events) const {
    unused(context, events);
  }

  /// Override this method for physical systems to calculate the potential
  /// energy currently stored in the configuration provided in the given
  /// Context. The default implementation returns 0 which is correct for
  /// non-physical systems. You may assume that `context` has already
  /// been validated before it is passed to you here.
  virtual T DoCalcPotentialEnergy(const Context<T>& context) const {
    unused(context);
    return T(0);
  }

  /// Override this method for physical systems to calculate the kinetic
  /// energy currently present in the motion provided in the given
  /// Context. The default implementation returns 0 which is correct for
  /// non-physical systems. You may assume that `context` has already
  /// been validated before it is passed to you here.
  virtual T DoCalcKineticEnergy(const Context<T>& context) const {
    unused(context);
    return T(0);
  }

  /// Override this method to return the rate at which mechanical energy is
  /// being converted *from* potential energy *to* kinetic energy by this system
  /// in the given Context. This quantity must be positive when potential energy
  /// is *decreasing*. Power is in watts (J/s).
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  /// You may assume that `context` has already been validated before it is
  /// passed to you here.
  virtual T DoCalcConservativePower(const Context<T>& context) const {
    unused(context);
    return T(0);
  }

  /// Override this method to return the rate at which mechanical energy is
  /// being generated (positive) or dissipated (negative) *other than* by
  /// conversion between potential and kinetic energy (in the given Context).
  /// Integrating this quantity yields work W, and the total energy `E=PE+KE-W`
  /// should be conserved by any physically-correct model, to within integration
  /// accuracy of W. Power is in watts (J/s). (Watts are abbreviated W but not
  /// to be confused with work!) This method is meaningful only for physical
  /// systems; others return zero.
  ///
  /// By default, returns zero. Continuous, physical systems should override.
  /// You may assume that `context` has already been validated before it is
  /// passed to you here.
  virtual T DoCalcNonConservativePower(const Context<T>& context) const {
    unused(context);
    return T(0);
  }

  /// Provides the substantive implementation of MapQDotToVelocity().
  ///
  /// The default implementation uses the identity mapping, and correctly does
  /// nothing if the %System does not have second-order state variables. It
  /// throws std::runtime_error if the `generalized_velocity` and
  /// `qdot` are not the same size, but that is not enough to guarantee that
  /// the default implementation is adequate. Child classes must
  /// override this function if qdot != v (even if they are the same size).
  /// This occurs, for example, if a joint uses roll-pitch-yaw rotation angles
  /// for orientation but angular velocity for rotational rate rather than
  /// rotation angle derivatives.
  ///
  /// If you implement this method you are required to use no more than `O(nq)`
  /// time where `nq` is the size of `qdot`, so that the %System can meet the
  /// performance guarantee made for the public interface, and you must also
  /// implement DoMapVelocityToQDot(). Implementations may assume that `qdot`
  /// has already been validated to be the same size as `q` in the given
  /// Context, and that `generalized_velocity` is non-null.
  virtual void DoMapQDotToVelocity(const Context<T>& context,
                                   const Eigen::Ref<const VectorX<T>>& qdot,
                                   VectorBase<T>* generalized_velocity) const {
    unused(context);
    // In the particular case where generalized velocity and generalized
    // configuration are not even the same size, we detect this error and abort.
    // This check will thus not identify cases where the generalized velocity
    // and time derivative of generalized configuration are identically sized
    // but not identical!
    const int n = qdot.size();
    // You need to override System<T>::DoMapQDottoVelocity!
    DRAKE_THROW_UNLESS(generalized_velocity->size() == n);
    generalized_velocity->SetFromVector(qdot);
  }

  /// Provides the substantive implementation of MapVelocityToQDot().
  ///
  /// The default implementation uses the identity mapping, and correctly does
  /// nothing if the %System does not have second-order state variables. It
  /// throws std::runtime_error if the `generalized_velocity` (`v`) and
  /// `qdot` are not the same size, but that is not enough to guarantee that
  /// the default implementation is adequate. Child classes must
  /// override this function if `qdot != v` (even if they are the same size).
  /// This occurs, for example, if a joint uses roll-pitch-yaw rotation angles
  /// for orientation but angular velocity for rotational rate rather than
  /// rotation angle derivatives.
  ///
  /// If you implement this method you are required to use no more than `O(nq)`
  /// time where `nq` is the size of `qdot`, so that the %System can meet the
  /// performance guarantee made for the public interface, and you must also
  /// implement DoMapQDotToVelocity(). Implementations may assume that
  /// `generalized_velocity` has already been validated to be the same size as
  /// `v` in the given Context, and that `qdot` is non-null.
  virtual void DoMapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* qdot) const {
    unused(context);
    // In the particular case where generalized velocity and generalized
    // configuration are not even the same size, we detect this error and abort.
    // This check will thus not identify cases where the generalized velocity
    // and time derivative of generalized configuration are identically sized
    // but not identical!
    const int n = generalized_velocity.size();
    // You need to override System<T>::DoMapVelocityToQDot!
    DRAKE_THROW_UNLESS(qdot->size() == n);
    qdot->SetFromVector(generalized_velocity);
  }

  /// NVI implementation of ToAutoDiffXdMaybe. Caller takes ownership of the
  /// returned pointer.
  /// @return nullptr if this System does not support autodiff
  virtual System<AutoDiffXd>* DoToAutoDiffXd() const { return nullptr; }

  /// NVI implementation of ToSymbolicMaybe. Caller takes ownership of the
  /// returned pointer.
  /// @return nullptr if this System does not support symbolic form
  virtual System<symbolic::Expression>* DoToSymbolic() const { return nullptr; }
  //@}

//----------------------------------------------------------------------------
/// @name             Constraint-related functions (protected).
///
// @{

  /// Gets the number of constraint equations for this system from the given
  /// context. The context is supplied in case the number of constraints is
  /// dependent upon the current state (as might be the case with a piecewise
  /// differential algebraic equation). Derived classes can override this
  /// function, which is called by get_num_constraint_equations().
  /// @sa get_num_constraint_equations() for parameter documentation.
  /// @returns zero by default
  virtual int do_get_num_constraint_equations(const Context<T>& context) const {
    unused(context);
    return 0;
  }

  /// Evaluates the constraint equations for the system at the generalized
  /// coordinates and generalized velocity specified by the context. The context
  /// allows the set of constraints to be dependent upon the current
  /// system state (as might be the case with a piecewise differential algebraic
  /// equation). The default implementation of this function returns a
  /// zero-dimensional vector. Derived classes can override this function,
  /// which is called by EvalConstraintEquations().
  /// @sa EvalConstraintEquations() for parameter documentation.
  /// @returns a vector of dimension get_num_constraint_equations(); the
  ///          zero vector indicates that the algebraic constraints are all
  ///          satisfied.
  virtual Eigen::VectorXd DoEvalConstraintEquations(
      const Context<T>& context) const {
    DRAKE_DEMAND(get_num_constraint_equations(context) == 0);
    return Eigen::VectorXd();
  }

  /// Computes the time derivative of each constraint equation, evaluated at
  /// the generalized coordinates and generalized velocity specified by the
  /// context.  The context allows the set of constraints to be dependent upon
  /// the current system state (as might be the case with a piecewise
  /// differential algebraic equation). The default implementation of this
  /// function returns a zero-dimensional vector. Derived classes can override
  /// this function, which is called by EvalConstraintEquationsDot().
  /// @returns a vector of dimension get_num_constraint_equations().
  /// @sa EvalConstraintEquationsDot() for parameter documentation.
  virtual Eigen::VectorXd DoEvalConstraintEquationsDot(
      const Context<T>& context) const {
    DRAKE_DEMAND(get_num_constraint_equations(context) == 0);
    return Eigen::VectorXd();
  }

  /// Computes the change in velocity from applying the given constraint forces
  /// to the system at the given context. Derived classes can override this
  /// function, which is called by CalcVelocityChangeFromConstraintImpulses().
  /// @returns the zero vector of dimension of the dimension of the
  ///          quasi-coordinates, by default.
  /// @sa CalcVelocityChangeFromConstraintImpulses() for parameter
  ///     documentation.
  virtual Eigen::VectorXd DoCalcVelocityChangeFromConstraintImpulses(
      const Context<T>& context, const Eigen::MatrixXd& J,
      const Eigen::VectorXd& lambda) const {
    unused(J, lambda);
    DRAKE_DEMAND(get_num_constraint_equations(context) == 0);
    const auto& gv = context.get_continuous_state()->get_generalized_velocity();
    return Eigen::VectorXd::Zero(gv.size());
  }

  /// Computes the norm of the constraint error. This default implementation
  /// computes a Euclidean norm of the error. Derived classes can override this
  /// function, which is called by CalcConstraintErrorNorm(). This norm need be
  /// neither continuous nor differentiable.
  /// @sa CalcConstraintErrorNorm() for parameter documentation.
  virtual double DoCalcConstraintErrorNorm(const Context<T>& context,
                                           const Eigen::VectorXd& error) const {
    unused(context);
    return error.norm();
  }

  //----------------------------------------------------------------------------
  /// @name                 Utility methods (protected)
  //@{

  /// Returns a mutable Eigen expression for a vector valued output port with
  /// index @p port_index in this system. All input ports that directly depend
  /// on this output port will be notified that upstream data has changed, and
  /// may invalidate cache entries as a result.
  Eigen::VectorBlock<VectorX<T>> GetMutableOutputVector(SystemOutput<T>* output,
                                                        int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_output_ports());

    BasicVector<T>* output_vector = output->GetMutableVectorData(port_index);
    DRAKE_ASSERT(output_vector != nullptr);
    DRAKE_ASSERT(output_vector->size() == get_output_port(port_index).size());

    return output_vector->get_mutable_value();
  }

  /// Causes an InputPortValue in the @p context to become up-to-date,
  /// delegating to the parent Diagram if necessary.
  ///
  /// This is a framework implementation detail. User code should never call it.
  void EvalInputPort(const Context<T>& context, int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < get_num_input_ports());
    context.EvalInputPort(parent_, get_input_port(port_index));
  }
  //@}

  const EventCollection<PublishEvent<T>>&
  get_forced_publish_events() const {
    return *forced_publish_;
  }

  const EventCollection<DiscreteUpdateEvent<T>>&
  get_forced_discrete_update_events() const {
    return *forced_discrete_update_;
  }

  const EventCollection<UnrestrictedUpdateEvent<T>>&
  get_forced_unrestricted_update_events() const {
    return *forced_unrestricted_update_;
  }

  void set_forced_publish_events(
  std::unique_ptr<EventCollection<PublishEvent<T>>> forced) {
    forced_publish_ = std::move(forced);
  }

  void set_forced_discrete_update_events(
  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>> forced) {
    forced_discrete_update_ = std::move(forced);
  }

  void set_forced_unrestricted_update_events(
  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>> forced) {
    forced_unrestricted_update_ = std::move(forced);
  }

 private:
  std::string name_;
  // input_ports_ and output_ports_ are vectors of unique_ptr so that references
  // to the descriptors will remain valid even if the vector is resized.
  std::vector<std::unique_ptr<InputPortDescriptor<T>>> input_ports_;
  std::vector<std::unique_ptr<OutputPort<T>>> output_ports_;
  const detail::InputPortEvaluatorInterface<T>* parent_{nullptr};

  std::vector<std::unique_ptr<SystemConstraint<T>>> constraints_;

  // These are only used to dispatch forced event handling. For a LeafSystem,
  // all of these have exactly one kForced triggered event. For a Diagram, they
  // are DiagramEventCollection, whose leafs are LeafEventCollection with
  // exactly one kForced triggered event.
  std::unique_ptr<EventCollection<PublishEvent<T>>> forced_publish_{nullptr};
  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
      forced_discrete_update_{nullptr};
  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
      forced_unrestricted_update_{nullptr};

  // TODO(sherm1) Replace these fake cache entries with real cache asap.
  // These are temporaries and hence uninitialized.
  mutable T fake_cache_pe_;
  mutable T fake_cache_ke_;
  mutable T fake_cache_conservative_power_;
  mutable T fake_cache_nonconservative_power_;
};

}  // namespace systems
}  // namespace drake
