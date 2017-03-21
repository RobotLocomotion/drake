#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

/// A description of a discrete-time event, which is passed from the Simulator
/// to the recipient System's appropriate event handling method. Different
/// DiscreteEvent ActionTypes trigger different event handlers, which are
/// permitted to modify different state fields. For instance, publish events may
/// not modify any state at all, discrete update events may modify the discrete
/// state only, and unrestricted update events may modify any state. The
/// event handlers do not apply state updates to the Context directly. Instead,
/// they write the updates into a separate buffer that the Simulator provides.

template <typename T>
struct DiscreteEvent {
  typedef std::function<void(const Context<T>&)> PublishCallback;
  typedef std::function<void(const Context<T>&, DiscreteState<T>*)>
      DiscreteUpdateCallback;
  typedef std::function<void(const Context<T>&, State<T>*)>
      UnrestrictedUpdateCallback;

  /// These enumerations represent an indication of the type of event that
  /// triggered the event handler, toward obviating the need to redetermine
  /// the reason that the event handler is called.
  enum ActionType {
    /// A default value that causes the handler to abort.
    kUnknownAction = 0,

    /// On publish actions, state does not change.
    kPublishAction = 1,

    /// On discrete updates, discrete state may change.
    kDiscreteUpdateAction = 2,

    /// On unrestricted updates, the state variables may change arbitrarily.
    kUnrestrictedUpdateAction = 3,
  };

  /// The type of action the system must take in response to the event.
  ActionType action{kUnknownAction};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kPublishAction. If nullptr, Publish() will be used.
  PublishCallback do_publish{nullptr};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kDiscreteUpdateAction. If nullptr, DoCalcDiscreteVariableUpdates() will
  /// be used.
  DiscreteUpdateCallback do_calc_discrete_variable_update{nullptr};

  /// An optional callback, supplied by the recipient, to carry out a
  /// kUpdateUnrestrictedAction. If nullptr, DoCalcUnrestrictedUpdate() will be
  /// used.
  UnrestrictedUpdateCallback do_unrestricted_update{nullptr};
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

  /// Returns a default output, initialized with the correct number of
  /// concrete output ports for this System. @p context is provided as
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
  virtual std::unique_ptr<DiscreteState<T>> AllocateDiscreteVariables()
  const {
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

  // This method is DEPRECATED. Legacy overrides will be respected, but should
  // migrate to override LeafSystem::DoHasDirectFeedthrough.
  virtual bool has_any_direct_feedthrough() const {
    return (get_num_input_ports() > 0) && (get_num_output_ports() > 0);
  }

  /// Returns `true` if any of the inputs to the system might be directly
  /// fed through to any of its outputs and `false` otherwise.
  ///
  /// This method is virtual for framework purposes only. User code should not
  /// override it.
  virtual bool HasAnyDirectFeedthrough() const = 0;

  /// Returns true if there might be direct-feedthrough from any input port to
  /// the given @p output_port, and false otherwise.
  ///
  /// This method is virtual for framework purposes only. User code should not
  /// override it.
  virtual bool HasDirectFeedthrough(int output_port) const = 0;

  /// Returns true if there might be direct-feedthrough from the given
  /// @p input_port to the given @p output_port, and false otherwise.
  ///
  /// This method is virtual for framework purposes only. User code should not
  /// override it.
  virtual bool HasDirectFeedthrough(int input_port, int output_port) const = 0;

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

  /// This method is invoked by the Simulator when every-time step publishing
  /// is enabled, at the start of each continuous integration step, after
  /// discrete variables have been updated to the values
  /// they will hold throughout the step. It will always be called at the start
  /// of the first step of a simulation (after initialization) and after the
  /// final simulation step (after a final update to discrete variables).
  /// Dispatches to DoPublish().
  void Publish(const Context<T>& context) const {
    DiscreteEvent<T> event;
    event.action = DiscreteEvent<T>::kPublishAction;
    Publish(context, event);
  }

  /// This method publishes as a result of a specified `event`, such as the
  /// arrival of the sample time requested by `event`. Dispatches to
  /// DoPublish() by default, or to `event.do_publish()` if provided.
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
  template <template<typename> class Vec = BasicVector>
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
  /// @name                        Constraint-related functions.
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
  Eigen::VectorXd EvalConstraintEquations(
      const Context<T>& context) const {
    return DoEvalConstraintEquations(context);
  }

  /// Computes the time derivative of each constraint equation, evaluated at
  /// the generalized coordinates and generalized velocity specified by the
  /// context. The context allows the set of constraints to be dependent upon
  /// the current system state (as might be the case with a system modeled using
  /// piecewise differential algebraic equations).
  /// @returns a vector of dimension get_num_constraint_equations().
  Eigen::VectorXd EvalConstraintEquationsDot(
      const Context<T>& context) const {
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
  Eigen::VectorXd
      CalcVelocityChangeFromConstraintImpulses(const Context<T>& context,
                                               const Eigen::MatrixXd& J,
                                               const Eigen::VectorXd& lambda)
                                                   const {
    DRAKE_ASSERT(lambda.size() == get_num_constraint_equations(context));
    DRAKE_ASSERT(J.rows() == get_num_constraint_equations(context));
    DRAKE_ASSERT(J.cols() ==
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

  /// This method is called to calculate the correct update `xd(n+1)` to
  /// discrete variables `xd` given a Context containing their current values
  /// `xd(n)`, because the given `event` has arrived.  Dispatches to
  /// DoCalcDiscreteVariableUpdates by default, or to `event.do_update` if
  /// provided.
  void CalcDiscreteVariableUpdates(const Context<T>& context,
                                   const DiscreteEvent<T>& event,
                                   DiscreteState<T> *discrete_state) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_DEMAND(event.action == DiscreteEvent<T>::kDiscreteUpdateAction);
    if (event.do_calc_discrete_variable_update == nullptr) {
      DoCalcDiscreteVariableUpdates(context, discrete_state);
    } else {
      event.do_calc_discrete_variable_update(context, discrete_state);
    }
  }

  /// This method is called to update *any* state variables in the @p context
  /// because the given @p event has arrived. Dispatches to
  /// DoCalcUnrestrictedUpdate() by default, or to
  /// `event.do_unrestricted_update` if provided. Does not allow the
  /// dimensionality of the state variables to change.
  /// @throws std::logic_error if the dimensionality of the state variables
  ///         changes in the callback.
  void CalcUnrestrictedUpdate(const Context<T>& context,
                              const DiscreteEvent<T>& event,
                              State<T>* state) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_DEMAND(event.action == DiscreteEvent<T>::kUnrestrictedUpdateAction);
    const int continuous_state_dim =
        state->get_continuous_state()->size();
    const int discrete_state_dim = state->get_discrete_state()->size();
    const int abstract_state_dim = state->get_abstract_state()->size();

    // Copy current state to the passed-in state, as specified in the
    // documentation for DoCalclUnrestrictedUpdate().
    state->CopyFrom(context.get_state());

    if (event.do_unrestricted_update == nullptr) {
      DoCalcUnrestrictedUpdate(context, state);
    } else {
      event.do_unrestricted_update(context, state);
    }
    if (continuous_state_dim != state->get_continuous_state()->size() ||
        discrete_state_dim != state->get_discrete_state()->size() ||
        abstract_state_dim != state->get_abstract_state()->size())
      throw std::logic_error("State variable dimensions cannot be changed "
                                 "in CalcUnrestrictedUpdate().");
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
    actions->events.clear();
    DoCalcNextUpdateTime(context, actions);
    return actions->time;
  }

  /// Computes the output values that should result from the current contents
  /// of the given Context. The result may depend on time and the current values
  /// of input ports, parameters, and state variables.
  void CalcOutput(const Context<T>& context, SystemOutput<T>* output) const {
    DRAKE_DEMAND(output != nullptr);
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_ASSERT_VOID(CheckValidOutput(output));
    DoCalcOutput(context, output);
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
  /// @name                      Utility methods
  //@{

  /// Sets the name of the system. It is recommended that the name not include
  /// the character ':', since the path delimiter is "::". When creating a
  /// Diagram, names of sibling subsystems should be unique.
  void set_name(const std::string& name) { name_ = name; }

  /// Retrieves the name last supplied to set_name().
  std::string get_name() const { return name_; }

  /// Writes the full path of this System in the tree of Systems to @p output.
  /// The path has the form (::ancestor_system_name)*::this_system_name.
  void GetPath(std::stringstream* output) const {
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
      throw std::out_of_range("System " + get_name() + ": Port index " +
          std::to_string(port_index) + " is out of range. There are only " +
          std::to_string(get_num_input_ports()) + " input ports.");
    }
    return *input_ports_[port_index];
  }

  /// Returns the descriptor of the output port at index @p port_index.
  const OutputPortDescriptor<T>& get_output_port(int port_index) const {
    if (port_index < 0 || port_index >= get_num_output_ports()) {
      throw std::out_of_range("System " + get_name() + ": Port index " +
          std::to_string(port_index) + " is out of range. There are only " +
          std::to_string(get_num_output_ports()) + " output ports.");
    }
    return *output_ports_[port_index];
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
        DRAKE_THROW_UNLESS(output_vector->size() ==
            get_output_port(i).size());
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
  virtual void GetGraphvizFragment(std::stringstream *dot) const {}

  /// Appends a fragment to the @p dot stream identifying the graphviz node
  /// representing @p port. Does nothing by default.
  virtual void GetGraphvizInputPortToken(const InputPortDescriptor<T> &port,
                                         std::stringstream *dot) const {}

  /// Appends a fragment to the @p dot stream identifying the graphviz node
  /// representing @p port. Does nothing by default.
  virtual void GetGraphvizOutputPortToken(const OutputPortDescriptor<T> &port,
                                          std::stringstream *dot) const {}

  /// Returns an opaque integer that uniquely identifies this system in the
  /// Graphviz output.
  int64_t GetGraphvizId() const {
    return reinterpret_cast<int64_t>(this);
  }

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
  /// scalar type, with a dynamic-sized vector of partial derivatives.
  /// Concrete Systems may shadow this with a more specific return type.
  std::unique_ptr<System<AutoDiffXd>> ToAutoDiffXd() const {
    return std::unique_ptr<System<AutoDiffXd>>(DoToAutoDiffXd());
  }

  /// Creates a deep copy of `from`, transmogrified to use the autodiff
  /// scalar type, with a dynamic-sized vector of partial derivatives. Returns
  /// `nullptr` if the template parameter `S` is not the type of the concrete
  /// system, or a superclass thereof.
  ///
  /// Usage: @code
  ///   MySystem<double> plant;
  ///   std::unique_ptr<MySystem<AutoDiffXd>> ad_plant =
  ///       systems::System<double>::ToAutoDiffXd<MySystem>(plant);
  /// @endcode
  ///
  /// @tparam S The specific System pointer type to return.
  template <template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<AutoDiffXd>> ToAutoDiffXd(
      const System<double>& from) {
    // Capture the copy as System<AutoDiffXd>.
    std::unique_ptr<System<AutoDiffXd>> clone(from.DoToAutoDiffXd());
    // Attempt to downcast to S<AutoDiffXd>.
    S<AutoDiffXd>* downcast = dynamic_cast<S<AutoDiffXd>*>(clone.get());
    // If the downcast fails, return nullptr, letting the copy be deleted.
    if (downcast == nullptr) {
      return nullptr;
    }
    // If the downcast succeeds, redo it, taking ownership this time.
    return std::unique_ptr<S<AutoDiffXd>>(
        dynamic_cast<S<AutoDiffXd>*>(clone.release()));
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
  /// scalar type. Returns `nullptr` if DoToSymbolic has not been implemented.
  ///
  /// Concrete Systems may shadow this with a more specific return type.
  std::unique_ptr<System<symbolic::Expression>> ToSymbolic() const {
    return std::unique_ptr<System<symbolic::Expression>>(DoToSymbolic());
  }

  /// Creates a deep copy of `from`, transmogrified to use the symbolic
  /// scalar type. Returns `nullptr` if the template parameter `S` is not the
  /// type of the concrete system, or a superclass thereof. Returns `nullptr`
  /// if DoToSymbolic has not been implemented.
  ///
  /// Usage: @code
  ///   MySystem<double> plant;
  ///   std::unique_ptr<MySystem<symbolic::Expression>> ad_plant =
  ///       systems::System<double>::ToSymbolic<MySystem>(plant);
  /// @endcode
  ///
  /// @tparam S The specific System pointer type to return.
  template <template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<symbolic::Expression>> ToSymbolic(
      const System<double>& from) {
    // Capture the copy as System<symbolic::Expression>.
    std::unique_ptr<System<symbolic::Expression>> clone(from.DoToSymbolic());
    // Attempt to downcast to S<symbolic::Expression>.
    S<symbolic::Expression>* downcast =
        dynamic_cast<S<symbolic::Expression>*>(clone.get());
    // If the downcast fails, return nullptr, letting the copy be deleted.
    if (downcast == nullptr) {
      return nullptr;
    }
    // If the downcast succeeds, redo it, taking ownership this time.
    return std::unique_ptr<S<symbolic::Expression>>(
        dynamic_cast<S<symbolic::Expression>*>(clone.release()));
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
        const AbstractValue* other_value = other_system.EvalAbstractInput(
            other_context, i);
        if (other_value == nullptr) continue;
        target_context->FixInputPort(i, other_value->Clone());
      } else {
        DRAKE_ABORT_MSG("Unknown descriptor type.");
      }
    }
  }

  //@}

 protected:
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
    input_ports_.push_back(std::make_unique<InputPortDescriptor<T>>(
    this, port_index, type, size));
    return *input_ports_.back();
  }

  /// Adds an abstract-valued port to the input topology.
  /// @return descriptor of declared port.
  const InputPortDescriptor<T>& DeclareAbstractInputPort() {
    return DeclareInputPort(kAbstractValued, 0 /* size */);
  }

  /// Adds a port with the specified @p type and @p size to the output topology.
  /// @return descriptor of declared port.
  const OutputPortDescriptor<T>& DeclareOutputPort(PortDataType type,
                                                   int size) {
    int port_index = get_num_output_ports();
    output_ports_.push_back(std::make_unique<OutputPortDescriptor<T>>(
        this, port_index, type, size));
    return *output_ports_.back();
  }

  /// Adds an abstract-valued port with to the output topology.
  /// @return descriptor of declared port.
  const OutputPortDescriptor<T>& DeclareAbstractOutputPort() {
    return DeclareOutputPort(kAbstractValued, 0 /* size */);
  }
  //@}

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

  /// You must override this method to calculate values for output ports into
  /// the supplied argument, based on the contents of the given Context.
  ///
  /// This method is called only from the public non-virtual CalcOutput()
  /// which will already have error-checked the parameters so you don't have to.
  /// In particular, implementations may assume that the given Context is valid
  /// for this %System; that the `output` pointer is non-null, and that
  /// the referenced object is valid for this %System.
  virtual void DoCalcOutput(const Context<T>& context,
                            SystemOutput<T>* output) const = 0;

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
    DRAKE_DEMAND(derivatives->size() == 0);
    return;
  }

  /// Implement this in your concrete System if you want it to take some action
  /// when the Simulator calls the Publish() method. This can be used for
  /// sending messages, producing console output, debugging, logging, saving the
  /// trajectory to a file, etc.
  ///
  /// This method is called only from the public non-virtual Publish() which
  /// will have already error-checked `context` so you may assume that it is
  /// valid for this %System.
  virtual void DoPublish(const Context<T>& context) const {}

  /// Updates the @p discrete_state on sample events.
  /// Override it, along with DoCalcNextUpdateTime(), if your System has any
  /// discrete variables.
  ///
  /// This method is called only from the public non-virtual
  /// CalcDiscreteVariableUpdates() which will already have error-checked the
  /// parameters so you don't have to. In particular, implementations may assume
  /// that the given Context is valid for this %System; that the
  /// `discrete_state` pointer is non-null, and that the referenced object
  /// has the same constituent structure as was produced by
  /// AllocateDiscreteVariables().
  virtual void DoCalcDiscreteVariableUpdates(
      const Context<T>& context, DiscreteState<T>* discrete_state) const {}

  /// Updates the @p state *in an unrestricted fashion* on unrestricted update
  /// events. Override this function if you need your System to update
  /// abstract variables or generally make changes to state that cannot be
  /// made using CalcDiscreteVariableUpdates() or via integration of continuous
  /// variables.
  ///
  /// This method is called only from the public non-virtual
  /// CalcUnrestrictedUpdate() which will already have error-checked the
  /// parameters so you don't have to. In particular, implementations may assume
  /// that the given Context is valid for this %System; that the `state` pointer
  /// is non-null, and that the referenced object has the same constituent
  /// structure as the state in `context`.
  ///
  /// @param[in]     context The "before" state that is to be used to calculate
  ///                        the returned state update.
  /// @param[in,out] state   The current state of the system on input; the
  ///                        desired state of the system on return.
  // TODO(sherm1) Shouldn't require preloading of the output state; better to
  //              note just the changes since usually only a small subset will
  //              be changed by this method.
  virtual void DoCalcUnrestrictedUpdate(const Context<T>& context,
                                        State<T>* state) const {}

  /// Computes the next time at which this System must perform a discrete
  /// action.
  ///
  /// Override this method if your System has any discrete actions which must
  /// interrupt the continuous simulation. This method is called only from the
  /// public non-virtual CalcNextUpdateTime() which will already have
  /// error-checked the parameters so you don't have to. You may assume that
  /// `context` has already been validated and the `actions` pointer is
  /// not `nullptr`.
  ///
  /// The default implementation returns with `actions` having a next sample
  /// time of Infinity and no actions to take.  If you declare actions, you may
  /// specify custom do_publish and do_update handlers.  If you do not,
  /// DoPublish and DoCalcDifferenceUpdates will be used by default.
  virtual void DoCalcNextUpdateTime(const Context<T>& context,
                                    UpdateActions<T>* actions) const {
    actions->time = std::numeric_limits<T>::infinity();
  }


  /// Override this method for physical systems to calculate the potential
  /// energy currently stored in the configuration provided in the given
  /// Context. The default implementation returns 0 which is correct for
  /// non-physical systems. You may assume that `context` has already
  /// been validated before it is passed to you here.
  virtual T DoCalcPotentialEnergy(const Context<T>& context) const {
    return T(0);
  }

  /// Override this method for physical systems to calculate the kinetic
  /// energy currently present in the motion provided in the given
  /// Context. The default implementation returns 0 which is correct for
  /// non-physical systems. You may assume that `context` has already
  /// been validated before it is passed to you here.
  virtual T DoCalcKineticEnergy(const Context<T>& context) const {
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

  /// NVI implementation of ToAutoDiffXd. Caller takes ownership of the returned
  /// pointer. Overrides should return a more specific covariant type.
  /// Templated overrides may assume that they are subclasses of System<double>.
  ///
  /// No default implementation is provided in LeafSystem, since the member data
  /// of a particular concrete leaf system is not knowable to the framework.
  /// A default implementation is provided in Diagram, which Diagram subclasses
  /// with member data should override.
  virtual System<AutoDiffXd>* DoToAutoDiffXd() const {
    DRAKE_ABORT_MSG("Override DoToAutoDiffXd before using ToAutoDiffXd.");
    return nullptr;
  }

  /// NVI implementation of ToSymbolic. Caller takes ownership of the returned
  /// pointer. Overrides should return a more specific covariant type.
  /// Templated overrides may assume that they are subclasses of System<double>.
  ///
  /// Returns `nullptr` by default.  Direct-feedthrough detection relies on
  /// this behavior.
  ///
  /// No default implementation is provided in LeafSystem, since the member data
  /// of a particular concrete leaf system is not knowable to the framework.
  /// A default implementation is provided in Diagram, which Diagram subclasses
  /// with member data should override.
  virtual System<symbolic::Expression>* DoToSymbolic() const {
    return nullptr;
  }
  //@}

//----------------------------------------------------------------------------
/// @name                        Constraint-related functions (protected).
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
  virtual Eigen::VectorXd
    DoCalcVelocityChangeFromConstraintImpulses(const Context<T>& context,
                                               const Eigen::MatrixXd& J,
                                               const Eigen::VectorXd& lambda)
                                                   const {
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
    DRAKE_ASSERT(output_vector->size() ==
        get_output_port(port_index).size());

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

 private:
  std::string name_;
  // input_ports_ and output_ports_ are vectors of unique_ptr so that references
  // to the descriptors will remain valid even if the vector is resized.
  std::vector<std::unique_ptr<InputPortDescriptor<T>>> input_ports_;
  std::vector<std::unique_ptr<OutputPortDescriptor<T>>> output_ports_;
  const detail::InputPortEvaluatorInterface<T>* parent_{nullptr};

  // TODO(sherm1) Replace these fake cache entries with real cache asap.
  // These are temporaries and hence uninitialized.
  mutable T fake_cache_pe_;
  mutable T fake_cache_ke_;
  mutable T fake_cache_conservative_power_;
  mutable T fake_cache_nonconservative_power_;
};

}  // namespace systems
}  // namespace drake
