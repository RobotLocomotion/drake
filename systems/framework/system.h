#pragma once

#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/random.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event_collection.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/system_base.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_scalar_converter.h"
#include "drake/systems/framework/system_visitor.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace systems {

/** Base class for all System functionality that is dependent on the templatized
scalar type T for input, state, parameters, and outputs.

@tparam_default_scalar */
template <typename T>
class System : public SystemBase {
 public:
  // System objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(System)

  ~System() override;

  /// Implements a visitor pattern.  @see SystemVisitor<T>.
  virtual void Accept(SystemVisitor<T>* v) const;

  //----------------------------------------------------------------------------
  /** @name           Resource allocation and initialization
  These methods are used to allocate and initialize Context resources. */
  //@{

  // This is just an intentional shadowing of the base class method to return
  // a more convenient type.
  /** Returns a Context<T> suitable for use with this System<T>. */
  std::unique_ptr<Context<T>> AllocateContext() const;

  /** Allocates a CompositeEventCollection for this system. The allocated
  instance is used for populating collections of triggered events; for
  example, Simulator passes this object to System::CalcNextUpdateTime() to
  allow the system to identify and handle upcoming events. */
  std::unique_ptr<CompositeEventCollection<T>>
  AllocateCompositeEventCollection() const;

  /** Given an input port, allocates the vector storage.  The @p input_port
  must match a port declared via DeclareInputPort. */
  std::unique_ptr<BasicVector<T>> AllocateInputVector(
      const InputPort<T>& input_port) const;

  /** Given an input port, allocates the abstract storage.  The @p input_port
  must match a port declared via DeclareInputPort. */
  std::unique_ptr<AbstractValue> AllocateInputAbstract(
      const InputPort<T>& input_port) const;

  /** Returns a container that can hold the values of all of this System's
  output ports. It is sized with the number of output ports and uses each
  output port's allocation method to provide an object of the right type
  for that port. */
  std::unique_ptr<SystemOutput<T>> AllocateOutput() const;

  /** Returns a ContinuousState of the same size as the continuous_state
  allocated in CreateDefaultContext. The simulator will provide this state
  as the output argument to EvalTimeDerivatives. */
  virtual std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const
      = 0;

  /** Returns an Eigen VectorX suitable for use as the output argument to
  the CalcImplicitTimeDerivativesResidual() method. The returned VectorX
  will have size implicit_time_derivatives_residual_size() with the
  elements uninitialized. This is just a convenience method -- you are free
  to use any properly-sized mutable Eigen object as the residual vector. */
  VectorX<T> AllocateImplicitTimeDerivativesResidual() const {
    return VectorX<T>(implicit_time_derivatives_residual_size());
  }

  /** Returns a DiscreteValues of the same dimensions as the discrete_state
  allocated in CreateDefaultContext. The simulator will provide this state
  as the output argument to Update. */
  virtual std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables() const
      = 0;

  /** This convenience method allocates a context using AllocateContext() and
  sets its default values using SetDefaultContext(). */
  std::unique_ptr<Context<T>> CreateDefaultContext() const;

  /** Assigns default values to all elements of the state. Overrides must not
  change the number of state variables. */
  virtual void SetDefaultState(const Context<T>& context,
                               State<T>* state) const = 0;

  /** Assigns default values to all parameters. Overrides must not
  change the number of parameters. */
  virtual void SetDefaultParameters(const Context<T>& context,
                                    Parameters<T>* parameters) const = 0;

  /** Sets Context fields to their default values.  User code should not
  override. */
  void SetDefaultContext(Context<T>* context) const;

  /** Assigns random values to all elements of the state.
  This default implementation calls SetDefaultState; override this method to
  provide random initial conditions using the stdc++ random library, e.g.:
  @code
    std::normal_distribution<T> gaussian();
    state->get_mutable_continuous_state()->get_mutable_vector()
         ->SetAtIndex(0, gaussian(*generator));
  @endcode
  Overrides must not change the number of state variables.

  @see @ref stochastic_systems */
  virtual void SetRandomState(const Context<T>& context, State<T>* state,
                              RandomGenerator* generator) const;

  /** Assigns random values to all parameters.
  This default implementation calls SetDefaultParameters; override this
  method to provide random parameters using the stdc++ random library, e.g.:
  @code
    std::uniform_real_distribution<T> uniform();
    parameters->get_mutable_numeric_parameter(0)
              ->SetAtIndex(0, uniform(*generator));
  @endcode
  Overrides must not change the number of state variables.

  @see @ref stochastic_systems */
  virtual void SetRandomParameters(const Context<T>& context,
                                   Parameters<T>* parameters,
                                   RandomGenerator* generator) const;

  /** Sets Context fields to random values.  User code should not
  override. */
  void SetRandomContext(Context<T>* context, RandomGenerator* generator) const;

  /** For each input port, allocates a fixed input of the concrete type
  that this System requires, and binds it to the port, disconnecting any
  prior input. Does not assign any values to the fixed inputs. */
  void AllocateFixedInputs(Context<T>* context) const;

  /** Returns `true` if any of the inputs to the system might be directly
  fed through to any of its outputs and `false` otherwise. */
  bool HasAnyDirectFeedthrough() const;

  /** Returns true if there might be direct-feedthrough from any input port to
  the given @p output_port, and false otherwise. */
  bool HasDirectFeedthrough(int output_port) const;

  /** Returns true if there might be direct-feedthrough from the given
  @p input_port to the given @p output_port, and false otherwise. */
  bool HasDirectFeedthrough(int input_port, int output_port) const;

  using SystemBase::GetDirectFeedthroughs;
  //@}

  //----------------------------------------------------------------------------
  /** @name                        Publishing
  Publishing is the primary mechanism for a %System to communicate with
  the world outside the %System abstraction during a simulation. Publishing
  occurs at user-specified times or events and can generate side-effect
  results such as terminal output, visualization, logging, plotting, and
  network messages. Other than computational cost, publishing has no effect
  on the progress of a simulation. */
  //@{

  /** This method is the public entry point for dispatching all publish event
  handlers. It checks the validity of @p context, and directly calls
  DispatchPublishHandler. @p events is a homogeneous collection of publish
  events.

  @note When publishing is triggered at particular times, those times likely
  will not coincide with integrator step times. A Simulator may interpolate
  to generate a suitable Context, or it may adjust the integrator step size
  so that a step begins exactly at the next publication time. In the latter
  case the change in step size may affect the numerical result somewhat
  since a smaller integrator step produces a more accurate solution. */
  void Publish(const Context<T>& context,
               const EventCollection<PublishEvent<T>>& events) const;

  /** Forces a publish on the system, given a @p context. The publish event will
  have a trigger type of kForced, with no additional data, attribute or
  custom callback. The Simulator can be configured to call this in
  Simulator::Initialize() and at the start of each continuous integration
  step. See the Simulator API for more details. */
  void Publish(const Context<T>& context) const;
  //@}

  //----------------------------------------------------------------------------
  /** @name                     Cached evaluations
  Given the values in a Context, a Drake %System must be able to provide
  the results of particular computations needed for analysis and simulation
  of the %System. These results are maintained in a mutable cache within
  the Context so that a result need be computed only once, the first time
  it is requested after a change to one of its prerequisite values.

  The `Eval` methods in this group return a reference to the
  already-computed result in the given Context's cache. If the current value
  is out of date, they first update the cache entry using the corresponding
  `Calc` method from the "Calculations" group. Evaluations of input ports
  instead delegate to the containing Diagram, which arranges to have the
  appropriate subsystem evaluate the source output port.

  Methods in this group that specify preconditions operate as follows:
  The preconditions will be checked in Debug builds but some or all might
  not be checked in Release builds for performance reasons. If we do check
  and a precondition is violated, an std::logic_error will be thrown with
  a helpful message. */
  //@{

  /** Returns a reference to the cached value of the continuous state variable
  time derivatives, evaluating first if necessary using CalcTimeDerivatives().

  This method returns the time derivatives ẋ꜀ of the continuous state
  x꜀. The referenced return object will correspond elementwise with the
  continuous state in the given Context. Thus, if the state in the Context
  has second-order structure `x꜀ = [q v z]`, that same structure applies to
  the derivatives so we will have `ẋ꜀ = [q̇ ̇v̇ ż]`.

  @param context The Context whose time, input port, parameter, state, and
  accuracy values may be used to evaluate the derivatives.

  @retval xcdot Time derivatives ẋ꜀ of x꜀ returned as a reference to an object
                of the same type and size as `context`'s continuous state.
  @see CalcTimeDerivatives(), CalcImplicitTimeDerivativesResidual(),
       get_time_derivatives_cache_entry() */
  const ContinuousState<T>& EvalTimeDerivatives(
      const Context<T>& context) const {
    ValidateContext(context);
    const CacheEntry& entry = get_time_derivatives_cache_entry();
    return entry.Eval<ContinuousState<T>>(context);
  }

  /** (Advanced) Returns the CacheEntry used to cache time derivatives for
  EvalTimeDerivatives(). */
  const CacheEntry& get_time_derivatives_cache_entry() const {
    return this->get_cache_entry(time_derivatives_cache_index_);
  }

  /** Returns a reference to the cached value of the potential energy (PE),
  evaluating first if necessary using CalcPotentialEnergy().

  By definition here, potential energy depends only on "configuration"
  (e.g. orientation and position), which includes a subset of the state
  variables, and parameters that affect configuration or conservative
  forces (such as lengths and masses). The calculated value may also be
  affected by the accuracy value supplied in the Context. PE cannot depend
  explicitly on time (∂PE/∂t = 0), velocities (∂PE/∂v = 0), or input port
  values (∂PE/∂u = 0).

  Non-physical systems where PE is not meaningful will return PE = 0.

  @param context The Context whose configuration variables may be used to
                 evaluate potential energy.
  @retval PE The potential energy in joules (J) represented by the
             configuration given in `context`.
  @see CalcPotentialEnergy() */
  const T& EvalPotentialEnergy(const Context<T>& context) const;

  /** Returns a reference to the cached value of the kinetic energy (KE),
  evaluating first if necessary using CalcKineticEnergy().

  By definition here, kinetic energy depends only on "configuration" and
  "velocity" (e.g. angular and translational velocity) of moving masses
  which includes a subset of the state variables, and parameters that affect
  configuration, velocities, or mass properties. The calculated value may
  also be affected by the accuracy value supplied in the Context. KE cannot
  depend explicitly on time (∂KE/∂t = 0) or input port values (∂KE/∂u = 0).

  Non-physical systems where KE is not meaningful will return KE = 0.

  @param context The Context whose configuration and velocity variables may
                 be used to evaluate kinetic energy.
  @retval KE The kinetic energy in joules (J) represented by the
             configuration and velocity given in `context`.
  @see CalcKineticEnergy() */
  const T& EvalKineticEnergy(const Context<T>& context) const;

  /** Returns a reference to the cached value of the conservative power (Pc),
  evaluating first if necessary using CalcConservativePower().

  The returned Pc represents the rate at which mechanical energy is being
  converted _from_ potential energy (PE) _to_ kinetic energy (KE) by this
  system in the given Context. This quantity will be _positive_ when PE
  is _decreasing_. By definition here, conservative power may depend only
  on quantities that explicitly contribute to PE and KE. See
  EvalPotentialEnergy() and EvalKineticEnergy() for details.

  Power due to non-conservative forces (e.g. dampers) can contribute to the
  rate of change of KE. Therefore this method alone cannot be used to
  determine whether KE is increasing or decreasing, only whether the
  conservative power is adding or removing kinetic energy.
  EvalNonConservativePower() can be used in conjunction with this method to
  find the total rate of change of KE.

  Non-physical systems where Pc is not meaningful will return Pc = 0.

  @param context The Context whose contents may be used to evaluate
                 conservative power.
  @retval Pc The conservative power in watts (W or J/s) represented by the
             contents of the given `context`.
  @see CalcConservativePower(), EvalNonConservativePower(),
       EvalPotentialEnergy(), EvalKineticEnergy() */
  const T& EvalConservativePower(const Context<T>& context) const;

  /** Returns a reference to the cached value of the non-conservative power
  (Pnc), evaluating first if necessary using CalcNonConservativePower().

  The returned Pnc represents the rate at which work W is done on the system
  by non-conservative forces. Pnc is _negative_ if the non-conservative
  forces are _dissipative_, positive otherwise. Time integration of Pnc
  yields work W, and the total mechanical energy `E = PE + KE − W` should be
  conserved by any physically-correct model, to within integration accuracy
  of W. Power is in watts (J/s). (Watts are abbreviated W but not to be
  confused with work!) Any values in the supplied Context (including time
  and input ports) may contribute to the computation of non-conservative
  power.

  Non-physical systems where Pnc is not meaningful will return Pnc = 0.

  @param context The Context whose contents may be used to evaluate
                 non-conservative power.
  @retval Pnc The non-conservative power in watts (W or J/s) represented by
              the contents of the given `context`.
  @see CalcNonConservativePower(), EvalConservativePower() */
  const T& EvalNonConservativePower(const Context<T>& context) const;

  // TODO(jwnimmer-tri) Deprecate me.
  /** Returns the value of the vector-valued input port with the given
  `port_index` as a BasicVector or a specific subclass `Vec` derived from
  BasicVector. Causes the value to become up to date first if necessary. See
  EvalAbstractInput() for more information.

  The result is returned as a pointer to the input port's value of type
  `Vec<T>` or nullptr if the port is not connected.

  @pre `port_index` selects an existing input port of this System.
  @pre the port must have been declared to be vector-valued.
  @pre the port's value must be of type Vec<T>.

  @tparam Vec The template type of the input vector, which must be a
              subclass of BasicVector. */
  template <template <typename> class Vec = BasicVector>
  const Vec<T>* EvalVectorInput(const Context<T>& context,
                                int port_index) const {
    static_assert(
        std::is_base_of_v<BasicVector<T>, Vec<T>>,
        "In EvalVectorInput<Vec>, Vec must be a subclass of BasicVector.");

    ValidateContext(context);

    // The API allows an int but we'll use InputPortIndex internally.
    if (port_index < 0)
      ThrowNegativePortIndex(__func__, port_index);
    const InputPortIndex iport_index(port_index);

    const BasicVector<T>* const basic_value =
        EvalBasicVectorInputImpl(__func__, context, iport_index);
    if (basic_value == nullptr)
      return nullptr;  // An unconnected port.

    // It's a BasicVector, but we're fussy about the subtype here.
    const Vec<T>* const value = dynamic_cast<const Vec<T>*>(basic_value);
    if (value == nullptr) {
      ThrowInputPortHasWrongType(__func__, iport_index,
                                 NiceTypeName::Get<Vec<T>>(),
                                 NiceTypeName::Get(*basic_value));
    }

    return value;
  }

  /** (Deprecated) Returns the value of the vector-valued input port with the
  given `port_index` as an %Eigen vector. Causes the value to become up to date
  first if necessary. See EvalAbstractInput() for more information.

  @pre `port_index` selects an existing input port of this System.
  @pre the port must have been declared to be vector-valued.
  @pre the port must be evaluable (connected or fixed).

  @see InputPort::Eval() */
  DRAKE_DEPRECATED("2021-03-01",
      "Use get_input_port(index).Eval(context) instead.")
  Eigen::VectorBlock<const VectorX<T>> EvalEigenVectorInput(
      const Context<T>& context, int port_index) const;
  //@}

  //----------------------------------------------------------------------------
  /** @name               Constraint-related functions */
  //@{

  /** Adds an "external" constraint to this System.

  This method is intended for use by applications that are examining this
  System to add additional constraints based on their particular situation
  (e.g., that a velocity state element has an upper bound); it is not
  intended for declaring intrinsic constraints that some particular System
  subclass might always impose on itself (e.g., that a mass parameter is
  non-negative).  To that end, this method should not be called by
  subclasses of `this` during their constructor.

  The `constraint` will automatically persist across system scalar
  conversion. */
  SystemConstraintIndex AddExternalConstraint(
      ExternalSystemConstraint constraint);
  //@}

  //----------------------------------------------------------------------------
  /** @name                        Calculations
  A Drake %System defines a set of common computations that are understood
  by the framework. Most of these are embodied in a `Calc` method that
  unconditionally performs the calculation into an output argument of the
  appropriate type, using only values from the given Context. These are
  paired with an `Eval` method that returns a reference to an
  already-calculated result residing in the cache; if needed that result is
  first obtained using the `Calc` method. See the "Evaluations" group for
  more information.

  This group also includes additional %System-specific operations that
  depend on both Context and additional input arguments. */
  //@{

  /** Calculates the time derivatives ẋ꜀ of the continuous state x꜀ into
  a given output argument. Prefer EvalTimeDerivatives() instead to avoid
  unnecessary recomputation.

  This method solves the %System equations in explicit form:

      ẋ꜀ = fₑ(𝓒)

  where `𝓒 = {a, p, t, x, u}` is the current value of the given Context from
  which accuracy a, parameters p, time t, state x (`={x꜀ xd xₐ}`) and
  input values u are obtained.

  @param[in] context The source for time, state, inputs, etc. defining the
      point at which the derivatives should be calculated.
  @param[out] derivatives The time derivatives ẋ꜀. Must be the same size as
      the continuous state vector in `context`.

  @see EvalTimeDerivatives() for more information.
  @see CalcImplicitTimeDerivativesResidual() for the implicit form of these
       equations.*/
  void CalcTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const;

  /** Evaluates the implicit form of the %System equations and returns the
  residual.

  The explicit and implicit forms of the %System equations are

      (1) ẋ꜀ = fₑ(𝓒)            explicit
      (2) 0 = fᵢ(𝓒; ẋ꜀)         implicit

  where `𝓒 = {a, p, t, x, u}` is the current value of the given Context from
  which accuracy a, parameters p, time t, state x (`={x꜀ xd xₐ}`) and input
  values u are obtained. Substituting (1) into (2) shows that the following
  condition must always hold:

      (3) fᵢ(𝓒; fₑ(𝓒)) = 0      always true

  When `fᵢ(𝓒; ẋ꜀ₚ)` is evaluated with a proposed time derivative ẋ꜀ₚ that
  differs from ẋ꜀ the result will be non-zero; we call that the _residual_ of
  the implicit equation. Given a Context and proposed time derivative ẋ꜀ₚ, this
  method returns the residual r such that

      (4) r = fᵢ(𝓒; ẋ꜀ₚ).

  The returned r will typically be the same length as x꜀ although that is not
  required. And even if r and x꜀ are the same size, there will not necessarily
  be any elementwise correspondence between them. (That is, you should not
  assume that r[i] is the "residual" of ẋ꜀ₚ[i].) For a Diagram, r is the
  concatenation of residuals from each of the subsystems, in order of subsystem
  index within the Diagram.

  A default implementation fᵢ⁽ᵈᵉᶠ⁾ for the implicit form is always provided and
  makes use of the explicit form as follows:

      (5) fᵢ⁽ᵈᵉᶠ⁾(𝓒; ẋ꜀ₚ) ≜ ẋ꜀ₚ − fₑ(𝓒)

  which satisfies condition (3) by construction. (Note that the default
  implementation requires the residual to have the same size as x꜀.) Substantial
  efficiency gains can often be obtained by replacing the default function with
  a customized implementation. Override DoCalcImplicitTimeDerivativesResidual()
  to replace the default implementation with a better one.

  @param[in] context The source for time, state, inputs, etc. to be used
      in calculating the residual.
  @param[in] proposed_derivatives The proposed value ẋ꜀ₚ for the time
      derivatives of x꜀.
  @param[out] residual The result r of evaluating the implicit function.
      Can be any mutable Eigen vector object of size
      implicit_time_derivatives_residual_size().

  @pre `proposed_derivatives` is compatible with this System.
  @pre `residual` is of size implicit_time_derivatives_residual_size().

  @see SystemBase::implicit_time_derivatives_residual_size()
  @see LeafSystem::DeclareImplicitTimeDerivativesResidualSize()
  @see DoCalcImplicitTimeDerivativesResidual()
  @see CalcTimeDerivatives() */
  void CalcImplicitTimeDerivativesResidual(
      const Context<T>& context, const ContinuousState<T>& proposed_derivatives,
      EigenPtr<VectorX<T>> residual) const;

  /** This method is the public entry point for dispatching all discrete
  variable update event handlers. Using all the discrete update handlers in
  @p events, the method calculates the update `xd(n+1)` to discrete
  variables `xd(n)` in @p context and outputs the results to @p
  discrete_state. See documentation for
  DispatchDiscreteVariableUpdateHandler() for more details. */
  void CalcDiscreteVariableUpdates(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state) const;

  /** Given the @p discrete_state results of a previous call to
  CalcDiscreteVariableUpdates() that dispatched the given collection of
  events, modifies the @p context to reflect the updated @p discrete_state.
  @param[in] events
      The Event collection that resulted in the given @p discrete_state.
  @param[in,out] discrete_state
      The updated discrete state from a CalcDiscreteVariableUpdates()
      call. This is mutable to permit its contents to be swapped with the
      corresponding @p context contents (rather than copied).
  @param[in,out] context
      The Context whose discrete state is modified to match
      @p discrete_state. Note that swapping contents with @p discrete_state
      may cause addresses of individual discrete state group vectors in
      @p context to be different on return than they were on entry.
  @pre @p discrete_state is the result of a previous
       CalcDiscreteVariableUpdates() call that dispatched this @p events
       collection. */
  void ApplyDiscreteVariableUpdate(
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state, Context<T>* context) const;

  /** This method forces a discrete update on the system given a @p context,
  and the updated discrete state is stored in @p discrete_state. The
  discrete update event will have a trigger type of kForced, with no
  attribute or custom callback. */
  void CalcDiscreteVariableUpdates(const Context<T>& context,
                                   DiscreteValues<T>* discrete_state) const;

  /** This method is the public entry point for dispatching all unrestricted
  update event handlers. Using all the unrestricted update handers in
  @p events, it updates *any* state variables in the @p context, and
  outputs the results to @p state. It does not allow the dimensionality
  of the state variables to change. See the documentation for
  DispatchUnrestrictedUpdateHandler() for more details.

  @throws std::exception if the dimensionality of the state variables
          changes in the callback. */
  void CalcUnrestrictedUpdate(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state) const;

  /** Given the @p state results of a previous call to CalcUnrestrictedUpdate()
  that dispatched the given collection of events, modifies the @p context to
  reflect the updated @p state.
  @param[in] events
      The Event collection that resulted in the given @p state.
  @param[in,out] state
      The updated State from a CalcUnrestrictedUpdate() call. This is
      mutable to permit its contents to be swapped with the corresponding
      @p context contents (rather than copied).
  @param[in,out] context
      The Context whose State is modified to match @p state. Note that
      swapping contents with the @p state may cause addresses of
      continuous, discrete, and abstract state containers in @p context
      to be different on return than they were on entry.
  @pre @p state is the result of a previous CalcUnrestrictedUpdate() call
       that dispatched this @p events collection. */
  void ApplyUnrestrictedUpdate(
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state, Context<T>* context) const;

  /** This method forces an unrestricted update on the system given a
  @p context, and the updated state is stored in @p state. The
  unrestricted update event will have a trigger type of kForced, with no
  additional data, attribute or custom callback.

  @sa CalcUnrestrictedUpdate(const Context<T>&, const
  EventCollection<UnrestrictedUpdateEvent<T>>*, State<T>* state)
      for more information. */
  void CalcUnrestrictedUpdate(const Context<T>& context,
                              State<T>* state) const;

  /** This method is called by a Simulator during its calculation of the size of
  the next continuous step to attempt. The System returns the next time at
  which some discrete action must be taken, and records what those actions
  ought to be in @p events. Upon reaching that time, the simulator will
  merge @p events with the other CompositeEventCollection instances
  triggered through other mechanisms (e.g. GetPerStepEvents()), and the
  merged CompositeEventCollection will be passed to all event handling
  mechanisms.

  If there is no timed event coming, the return value is Infinity. If
  a finite update time is returned, there will be at least one Event object
  in the returned event collection.

  @p events cannot be null. @p events will be cleared on entry. */
  T CalcNextUpdateTime(const Context<T>& context,
                       CompositeEventCollection<T>* events) const;

  /** This method is called by Simulator::Initialize() to gather all update
  and publish events that are to be handled in AdvanceTo() at the point
  before Simulator integrates continuous state. It is assumed that these
  events remain constant throughout the simulation. The "step" here refers
  to the major time step taken by the Simulator. During every simulation
  step, the simulator will merge @p events with the event collections
  populated by other types of event triggering mechanism (e.g.,
  CalcNextUpdateTime()), and the merged CompositeEventCollection objects
  will be passed to the appropriate handlers before Simulator integrates the
  continuous state.

  @p events cannot be null. @p events will be cleared on entry. */
  void GetPerStepEvents(const Context<T>& context,
                        CompositeEventCollection<T>* events) const;

  /** This method is called by Simulator::Initialize() to gather all
  update and publish events that need to be handled at initialization
  before the simulator starts integration.

  @p events cannot be null. @p events will be cleared on entry. */
  void GetInitializationEvents(const Context<T>& context,
                               CompositeEventCollection<T>* events) const;

  /** Gets whether there exists a unique periodic attribute that triggers
  one or more discrete update events (and, if so, returns that unique
  periodic attribute). Thus, this method can be used (1) as a test to
  determine whether a system's dynamics are at least partially governed by
  difference equations and (2) to obtain the difference equation update
  times.
  @returns optional<PeriodicEventData> Contains the periodic trigger
  attributes if the unique periodic attribute exists, otherwise `nullopt`. */
  std::optional<PeriodicEventData>
      GetUniquePeriodicDiscreteUpdateAttribute() const;

  /** Returns true iff the state dynamics of this system are governed
  exclusively by a difference equation on a single discrete state group
  and with a unique periodic update (having zero offset).  E.g., it is
  amenable to analysis of the form:
    x[n+1] = f(x[n], u[n])
  Note that we do NOT consider the number of input ports here, because
  in practice many systems of interest (e.g. MultibodyPlant) have input
  ports that are safely treated as constant during the analysis.
  Consider using get_input_port_selection() to choose one.

  @param[out] time_period if non-null, then iff the function
  returns `true`, then time_period is set to the period data
  returned from GetUniquePeriodicDiscreteUpdateAttribute().  If the
  function returns `false` (the system is not a difference equation
  system), then `time_period` does not receive a value. */
  bool IsDifferenceEquationSystem(double* time_period = nullptr) const;

  /** Gets all periodic triggered events for a system. Each periodic attribute
  (offset and period, in seconds) is mapped to one or more update events
  that are to be triggered at the proper times. */
  std::map<PeriodicEventData, std::vector<const Event<T>*>,
    PeriodicEventDataComparator> GetPeriodicEvents() const;

  /** Utility method that computes for _every_ output port i the value y(i) that
  should result from the current contents of the given Context. Note that
  individual output port values can be calculated using
  `get_output_port(i).Calc()`; this method invokes that for each output port
  in index order. The result may depend on time and the current values of
  input ports, parameters, and state variables. The result is written to
  `outputs` which must already have been allocated to have the right number
  of entries of the right types. */
  void CalcOutput(const Context<T>& context, SystemOutput<T>* outputs) const;

  /** Calculates and returns the potential energy represented by the current
  configuration provided in `context`. Prefer EvalPotentialEnergy() to
  avoid unnecessary recalculation.

  @see EvalPotentialEnergy() for more information. */
  T CalcPotentialEnergy(const Context<T>& context) const;

  /** Calculates and returns the kinetic energy represented by the current
  configuration and velocity provided in `context`. Prefer
  EvalKineticEnergy() to avoid unnecessary recalculation.

  @see EvalKineticEnergy() for more information. */
  T CalcKineticEnergy(const Context<T>& context) const;

  /** Calculates and returns the conservative power represented by the current
  contents of the given `context`. Prefer EvalConservativePower() to avoid
  unnecessary recalculation.

  @see EvalConservativePower() for more information. */
  T CalcConservativePower(const Context<T>& context) const;

  /** Calculates and returns the non-conservative power represented by the
  current contents of the given `context`. Prefer EvalNonConservativePower()
  to avoid unnecessary recalculation.

  @see EvalNonConservativePower() for more information. */
  T CalcNonConservativePower(const Context<T>& context) const;

  /** Transforms a given generalized velocity `v` to the time derivative `qdot`
  of the generalized configuration `q` taken from the supplied Context.
  `v` and `qdot` are related linearly by `qdot = N(q) * v`, where `N` is a
  block diagonal matrix. For example, in a multibody system there will be
  one block of `N` per tree joint. This computation requires only `O(nq)`
  time where `nq` is the size of `qdot`. Note that `v` is *not* taken from
  the Context; it is given as an argument here.

  See the alternate signature if you already have the generalized
  velocity in an Eigen VectorX object; this signature will copy the
  VectorBase into an Eigen object before performing the computation.
  @see MapQDotToVelocity() */
  void MapVelocityToQDot(const Context<T>& context,
                         const VectorBase<T>& generalized_velocity,
                         VectorBase<T>* qdot) const;

  /** Transforms the given generalized velocity to the time derivative of
  generalized configuration. See the other signature of MapVelocityToQDot()
  for more information. */
  void MapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* qdot) const;

  /** Transforms the time derivative `qdot` of the generalized configuration `q`
  to generalized velocities `v`. `v` and `qdot` are related linearly by
  `qdot = N(q) * v`, where `N` is a block diagonal matrix. For example, in a
  multibody system there will be one block of `N` per tree joint. Although
  `N` is not necessarily square, its left pseudo-inverse `N+` can be used to
  invert that relationship without residual error, provided that `qdot` is
  in the range space of `N` (that is, if it *could* have been produced as
  `qdot=N*v` for some `v`). Using the configuration `q` from the given
  Context this method calculates `v = N+ * qdot` (where `N+=N+(q)`) for
  a given `qdot`. This computation requires only `O(nq)` time where `nq` is
  the size of `qdot`. Note that this method does not take `qdot` from the
  Context.

  See the alternate signature if you already have `qdot` in an %Eigen
  VectorX object; this signature will copy the VectorBase into an %Eigen
  object before performing the computation.
  @see MapVelocityToQDot() */
  void MapQDotToVelocity(const Context<T>& context, const VectorBase<T>& qdot,
                         VectorBase<T>* generalized_velocity) const;

  /** Transforms the given time derivative `qdot` of generalized configuration
  `q` to generalized velocity `v`. This signature takes `qdot` as an %Eigen
  VectorX object for faster speed. See the other signature of
  MapQDotToVelocity() for additional information. */
  void MapQDotToVelocity(const Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         VectorBase<T>* generalized_velocity) const;
  //@}

  //----------------------------------------------------------------------------
  /** @name                    Subcontext access
  Methods in this section locate the Context belonging to a particular
  subsystem, from within the Context for a containing System (typically a
  Diagram). There are two common circumstances where this is needed:

  1. You are given a Diagram and its Context, and have a reference to a
     particular subsystem contained somewhere in that Diagram (that is,
     an immediate child or deeper descendent). You can ask the Diagram to
     find the subcontext of that subsystem, using GetSubsystemContext()
     or GetMutableSubsystemContext().
  2. You are given the root Context for a complete Diagram (typically by
     the Simulator as part of a generated trajectory). You don't have a
     reference to the Diagram, but you do have a reference to a subsystem
     of interest. You want to find its subcontext from within the root
     Context. Use GetMyContextFromRoot() or GetMyMutableContextFromRoot().

  The second case is particularly useful in monitor functions for the
  Drake Simulator. */
  //@{

  /** Returns a const reference to the subcontext that corresponds to the
  contained %System `subsystem`.
  @throws std::exception if `subsystem` not contained in `this` %System.
  @pre The given `context` is valid for use with `this` %System. */
  const Context<T>& GetSubsystemContext(const System<T>& subsystem,
                                        const Context<T>& context) const;

  /** Returns a mutable reference to the subcontext that corresponds to the
  contained %System `subsystem`.
  @throws std::exception if `subsystem` not contained in `this` %System.
  @pre The given `context` is valid for use with `this` %System. */
  Context<T>& GetMutableSubsystemContext(const System<T>& subsystem,
                                         Context<T>* context) const;

  /** Returns the const Context for `this` subsystem, given a root context. If
  `this` %System is already the top level (root) %System, just returns
  `root_context`. (A root Context is one that does not have a parent
  Context.)
  @throws std::exception if the given `root_context` is not actually
      a root context.
  @see GetSubsystemContext() */
  const Context<T>& GetMyContextFromRoot(const Context<T>& root_context) const;

  /** Returns the mutable subsystem context for `this` system, given a root
  context.
  @see GetMyContextFromRoot() */
  Context<T>& GetMyMutableContextFromRoot(Context<T>* root_context) const;
  //@}

  //----------------------------------------------------------------------------
  /** @cond */
  // Functions to avoid RTTI in Diagram. Conceptually, these should be protected
  // and should not be directly called, so they are hidden from doxygen.

  // TODO(siyuan): change all target_system to reference.

  // Returns @p context if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual const Context<T>* DoGetTargetSystemContext(
      const System<T>& target_system, const Context<T>* context) const;

  // Returns @p state if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual State<T>* DoGetMutableTargetSystemState(
      const System<T>& target_system, State<T>* state) const;

  // Returns @p state if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual const State<T>* DoGetTargetSystemState(const System<T>& target_system,
                                                 const State<T>* state) const;

  // Returns x꜀ if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual const ContinuousState<T>* DoGetTargetSystemContinuousState(
      const System<T>& target_system,
      const ContinuousState<T>* xc) const;

  // Returns @p events if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual CompositeEventCollection<T>*
  DoGetMutableTargetSystemCompositeEventCollection(
      const System<T>& target_system,
      CompositeEventCollection<T>* events) const;

  // Returns @p events if @p target_system equals `this`, nullptr otherwise.
  // Should not be directly called.
  virtual const CompositeEventCollection<T>*
  DoGetTargetSystemCompositeEventCollection(
      const System<T>& target_system,
      const CompositeEventCollection<T>* events) const;

  // The derived class implementation shall create the appropriate collection
  // for each of these three methods.
  //
  // Consumers of this class should never need to call the three methods below.
  // These three methods would ideally be designated as "protected", but
  // Diagram::AllocateForcedXEventCollection() needs to call these methods and,
  // perhaps surprisingly, is not able to access these methods when they are
  // protected. See:
  // https://stackoverflow.com/questions/16785069/why-cant-a-derived-class-call-protected-member-function-in-this-code.
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
  /** @endcond */

  //----------------------------------------------------------------------------
  /** @name                      Utility methods */
  //@{

  /** Returns a name for this %System based on a stringification of its type
  name and memory address.  This is intended for use in diagnostic output
  and should not be used for behavioral logic, because the stringification
  of the type name may produce differing results across platforms and
  because the address can vary from run to run. */
  std::string GetMemoryObjectName() const;

  // So we don't have to keep writing this->num_input_ports().
  using SystemBase::num_input_ports;
  using SystemBase::num_output_ports;

  // TODO(sherm1) Make this an InputPortIndex.
  /** Returns the typed input port at index @p port_index. */
  const InputPort<T>& get_input_port(int port_index) const {
    // Profiling revealed that it is too expensive to do a dynamic_cast here.
    // A static_cast is safe as long as GetInputPortBaseOrThrow always returns
    // a satisfactory type. As of this writing, it only ever returns values
    // supplied via SystemBase::AddInputPort, which atop its implementation
    // has a check that port.get_system_interface() matches `this` which is a
    // System<T>, so we are safe.
    return static_cast<const InputPort<T>&>(
        this->GetInputPortBaseOrThrow(__func__, port_index));
  }

  /** Convenience method for the case of exactly one input port. */
  const InputPort<T>& get_input_port() const {
    static constexpr char message[] =
        "Cannot use the get_input_port() convenience method unless there is"
        " exactly one input port. num_input_ports() = {}";
    if (num_input_ports() != 1) {
      throw std::logic_error(fmt::format(message, num_input_ports()));
    }
    return get_input_port(0);
  }

  /** Returns the typed input port specified by the InputPortSelection or by
  the InputPortIndex.  Returns nullptr if no port is selected.  This is
  provided as a convenience method since many algorithms provide the same
  common default or optional port semantics. */
  const InputPort<T>* get_input_port_selection(
      std::variant<InputPortSelection, InputPortIndex> port_index) const;

  /** Returns the typed input port with the unique name @p port_name.
  The current implementation performs a linear search over strings; prefer
  get_input_port() when performance is a concern.
  @throws std::exception if port_name is not found. */
  const InputPort<T>& GetInputPort(const std::string& port_name) const;

  /** Returns true iff the system has an InputPort of the given @p
   port_name. */
  bool HasInputPort(const std::string& port_name) const;

  // TODO(sherm1) Make this an OutputPortIndex.
  /** Returns the typed output port at index @p port_index. */
  const OutputPort<T>& get_output_port(int port_index) const {
    // Profiling revealed that it is too expensive to do a dynamic_cast here.
    // A static_cast is safe as long as GetInputPortBaseOrThrow always returns
    // a satisfactory type. As of this writing, it only ever returns values
    // supplied via SystemBase::AddInputPort, which atop its implementation
    // has a check that port.get_system_interface() matches `this` which is a
    // System<T>, so we are safe.
    return static_cast<const OutputPort<T>&>(
        this->GetOutputPortBaseOrThrow(__func__, port_index));
  }

  /** Convenience method for the case of exactly one output port. */
  const OutputPort<T>& get_output_port() const {
    static constexpr char message[] =
        "Cannot use the get_output_port() convenience method unless there is"
        " exactly one output port. num_output_ports() = {}";
    if (num_output_ports() != 1) {
      throw std::logic_error(fmt::format(message, num_output_ports()));
    }
    return get_output_port(0);
  }

  /** Returns the typed output port specified by the OutputPortSelection or by
  the OutputPortIndex.  Returns nullptr if no port is selected. This is
  provided as a convenience method since many algorithms provide the same
  common default or optional port semantics. */
  const OutputPort<T>* get_output_port_selection(
      std::variant<OutputPortSelection, OutputPortIndex> port_index) const;

  /** Returns the typed output port with the unique name @p port_name.
  The current implementation performs a linear search over strings; prefer
  get_output_port() when performance is a concern.
  @throws std::exception if port_name is not found. */
  const OutputPort<T>& GetOutputPort(const std::string& port_name) const;

  /** Returns true iff the system has an OutputPort of the given @p
   port_name. */
  bool HasOutputPort(const std::string& port_name) const;


  /** Returns the number of constraints specified for the system. */
  int num_constraints() const;

  /** Returns the constraint at index @p constraint_index.
  @throws std::exception for an invalid constraint_index. */
  const SystemConstraint<T>& get_constraint(
      SystemConstraintIndex constraint_index) const;

  /** Returns true if @p context satisfies all of the registered
  SystemConstraints with tolerance @p tol.  @see
  SystemConstraint::CheckSatisfied. */
  boolean<T> CheckSystemConstraintsSatisfied(
      const Context<T>& context, double tol) const;

  /** Returns a copy of the continuous state vector x꜀ into an Eigen
  vector. */
  VectorX<T> CopyContinuousStateVector(const Context<T>& context) const;
  //@}

  //----------------------------------------------------------------------------
  /** @name                      Graphviz methods */
  //@{

  /** Returns a Graphviz string describing this System.  To render the string,
  use the Graphviz tool, ``dot``. http://www.graphviz.org/

  @param max_depth Sets a limit to the depth of nested diagrams to
  visualize.  Set to zero to render a diagram as a single system block.

  @see GenerateHtml
  */
  std::string GetGraphvizString(
      int max_depth = std::numeric_limits<int>::max()) const;

  /** Appends a Graphviz fragment to the @p dot stream.  The fragment must be
  valid Graphviz when wrapped in a `digraph` or `subgraph` stanza.  Does
  nothing by default.

  @param max_depth Sets a limit to the depth of nested diagrams to
  visualize.  Set to zero to render a diagram as a single system block. */
  virtual void GetGraphvizFragment(int max_depth,
                                   std::stringstream* dot) const;

  /** Appends a fragment to the @p dot stream identifying the graphviz node
  representing @p port. Does nothing by default. */
  virtual void GetGraphvizInputPortToken(const InputPort<T>& port,
                                         int max_depth,
                                         std::stringstream* dot) const;

  /** Appends a fragment to the @p dot stream identifying the graphviz node
  representing @p port. Does nothing by default. */
  virtual void GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                          int max_depth,
                                          std::stringstream* dot) const;

  /** Returns an opaque integer that uniquely identifies this system in the
  Graphviz output. */
  int64_t GetGraphvizId() const;
  //@}

  //----------------------------------------------------------------------------
  /** @name                Automatic differentiation
  From a %System templatized by `double`, you can obtain an identical system
  templatized by an automatic differentiation scalar providing
  machine-precision computation of partial derivatives of any numerical
  result of the %System with respect to any of the numerical values that
  can be contained in a Context (time, inputs, parameters, and state). */

  // This group appears as a top-level heading in Doxygen because it contains
  // both static and non-static member functions.
  //@{

  /** Creates a deep copy of this System, transmogrified to use the autodiff
  scalar type, with a dynamic-sized vector of partial derivatives.  The
  result is never nullptr.
  @throws std::exception if this System does not support autodiff

  See @ref system_scalar_conversion for detailed background and examples
  related to scalar-type conversion support. */
  std::unique_ptr<System<AutoDiffXd>> ToAutoDiffXd() const;

  /** Creates a deep copy of `from`, transmogrified to use the autodiff scalar
  type, with a dynamic-sized vector of partial derivatives.  The result is
  never nullptr.
  @throws std::exception if `from` does not support autodiff

  Usage: @code
    MySystem<double> plant;
    std::unique_ptr<MySystem<AutoDiffXd>> ad_plant =
        systems::System<double>::ToAutoDiffXd(plant);
  @endcode

  @tparam S The specific System type to accept and return.

  See @ref system_scalar_conversion for detailed background and examples
  related to scalar-type conversion support. */
  template <template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<AutoDiffXd>> ToAutoDiffXd(const S<T>& from) {
    return System<T>::ToScalarType<AutoDiffXd>(from);
  }

  /** Creates a deep copy of this system exactly like ToAutoDiffXd(), but
  returns nullptr if this System does not support autodiff, instead of
  throwing an exception. */
  std::unique_ptr<System<AutoDiffXd>> ToAutoDiffXdMaybe() const;
  //@}

  //----------------------------------------------------------------------------
  /** @name                          Symbolics
  From a %System templatized by `double`, you can obtain an identical system
  templatized by a symbolic expression scalar. */

  // This group appears as a top-level heading in Doxygen because it contains
  // both static and non-static member functions.
  //@{

  /** Creates a deep copy of this System, transmogrified to use the symbolic
  scalar type. The result is never nullptr.
  @throws std::exception if this System does not support symbolic

  See @ref system_scalar_conversion for detailed background and examples
  related to scalar-type conversion support. */
  std::unique_ptr<System<symbolic::Expression>> ToSymbolic() const;

  /** Creates a deep copy of `from`, transmogrified to use the symbolic scalar
  type. The result is never nullptr.
  @throws std::exception if `from` does not support symbolic

  Usage: @code
    MySystem<double> plant;
    std::unique_ptr<MySystem<symbolic::Expression>> sym_plant =
        systems::System<double>::ToSymbolic(plant);
  @endcode

  @tparam S The specific System pointer type to return.

  See @ref system_scalar_conversion for detailed background and examples
  related to scalar-type conversion support. */
  template <template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<symbolic::Expression>> ToSymbolic(const S<T>& from) {
    return System<T>::ToScalarType<symbolic::Expression>(from);
  }

  /** Creates a deep copy of this system exactly like ToSymbolic(), but returns
  nullptr if this System does not support symbolic, instead of throwing an
  exception. */
  std::unique_ptr<System<symbolic::Expression>> ToSymbolicMaybe() const;
  //@}

  //----------------------------------------------------------------------------
  /** @name                Scalar type conversion utilities */
  //@{

  /** Fixes all of the input ports in @p target_context to their current values
  in @p other_context, as evaluated by @p other_system.
  @throws std::exception unless `other_context` and `target_context` both
  have the same shape as this System, and the `other_system`. Ignores
  disconnected inputs.
  @throws std::exception if `this` system's scalar type T != double and
  `other_system` has any abstract input ports whose contained type depends on
  scalar type. */
  void FixInputPortsFrom(const System<double>& other_system,
                         const Context<double>& other_context,
                         Context<T>* target_context) const;

  /** (Advanced) Returns the SystemScalarConverter for this object.  This is an
  expert-level API intended for framework authors.  Most users should
  prefer the convenience helpers such as System::ToAutoDiffXd. */
  const SystemScalarConverter& get_system_scalar_converter() const;
  //@}


  //----------------------------------------------------------------------------
  /** @name                Scalar type conversion by template parameter

   These routines allow arbitrary scalar type conversion to be attempted. Not
   all conversions will be supported, for various reasons.

   - "Self conversions" (T=U) are not supported because the definitions would
     be ambiguous with the (deleted) copy constructor.
   - Derived systems may decline to support some scalar types.
   */
  //@{
  /** Creates a deep copy of this System, transmogrified to use the
  scalar type selected by a template parameter. The result is never nullptr.
  @throws std::exception if this System does not support the destination type.

  @tparam U The destination scalar type. For a list of supported types, see the
  @ref default_scalars "default scalars".

  See @ref system_scalar_conversion for detailed background and examples
  related to scalar-type conversion support. */
  template <typename U>
  std::unique_ptr<System<U>> ToScalarType() const {
    return System<T>::ToScalarType<U>(*this);
  }

  /** Creates a deep copy of `from`, transmogrified to use the scalar
  type selected by a template parameter. The result is never nullptr.
  @throws std::exception if `from` does not support the destination type.

  Usage: @code
    MySystem<double> plant;
    auto sym_plant =
      systems::System<double>::ToScalarType<symbolic::Expression>(plant);
  @endcode

  @tparam U The destination scalar type. For a list of supported types, see the
  @ref default_scalars "default scalars".
  @tparam S The specific System pointer type to return.

  See @ref system_scalar_conversion for detailed background and examples
  related to scalar-type conversion support. */
  template <typename U, template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<U>> ToScalarType(const S<T>& from) {
    auto base_result = from.template ToScalarTypeMaybe<U>();
    if (!base_result) {
      const System<T>& upcast_from = from;
      throw std::logic_error(upcast_from.GetUnsupportedScalarConversionMessage(
          typeid(T), typeid(U)));
    }
    return dynamic_pointer_cast_or_throw<S<U>>(std::move(base_result));
  }

  /** Creates a deep copy of this system exactly like ToScalarType(), but
  returns nullptr if this System does not support the destination type, instead
  of throwing an exception.

  @tparam U The destination scalar type. For a list of supported types, see the
  @ref default_scalars "default scalars".
  */
  template <typename U>
  std::unique_ptr<System<U>> ToScalarTypeMaybe() const {
    auto result = system_scalar_converter_.Convert<U, T>(*this);
    if (result) { result->AddExternalConstraints(external_constraints_); }
    return result;
  }
  //@}

  /** Gets the witness functions active for the given state.
  DoGetWitnessFunctions() does the actual work. The vector of active witness
  functions are expected to change only upon an unrestricted update.
  @param context a valid context for the System (aborts if not true).
  @param[out] w a valid pointer to an empty vector that will store
              pointers to the witness functions active for the current
              state. The method aborts if witnesses is null or non-empty. */
  void GetWitnessFunctions(const Context<T>& context,
                           std::vector<const WitnessFunction<T>*>* w) const;

  /** Evaluates a witness function at the given context. */
  T CalcWitnessValue(const Context<T>& context,
                     const WitnessFunction<T>& witness_func) const;

  /** Add `event` to `events` due to a witness function triggering. `events`
  should be allocated with this system's AllocateCompositeEventCollection.
  Neither `event` nor `events` can be nullptr. Additionally, `event` must
  contain event data (event->get_event_data() must not be nullptr) and
  the type of that data must be WitnessTriggeredEventData. */
  virtual void AddTriggeredWitnessFunctionToCompositeEventCollection(
      Event<T>* event,
      CompositeEventCollection<T>* events) const = 0;

  // Promote these frequently-used methods so users (and tutorial examples)
  // don't need "this->" everywhere when in templated derived classes.
  // All pre-defined ticket methods should be listed here. They are ordered as
  // they appear in SystemBase to make it easy to check that none are missing.
  using SystemBase::nothing_ticket;
  using SystemBase::time_ticket;
  using SystemBase::accuracy_ticket;
  using SystemBase::q_ticket;
  using SystemBase::v_ticket;
  using SystemBase::z_ticket;
  using SystemBase::xc_ticket;
  using SystemBase::discrete_state_ticket;
  using SystemBase::xd_ticket;
  using SystemBase::abstract_state_ticket;
  using SystemBase::xa_ticket;
  using SystemBase::all_state_ticket;
  using SystemBase::numeric_parameter_ticket;
  using SystemBase::pn_ticket;
  using SystemBase::abstract_parameter_ticket;
  using SystemBase::pa_ticket;
  using SystemBase::all_parameters_ticket;
  using SystemBase::input_port_ticket;
  using SystemBase::all_input_ports_ticket;
  using SystemBase::all_sources_ticket;
  using SystemBase::cache_entry_ticket;
  using SystemBase::configuration_ticket;
  using SystemBase::kinematics_ticket;
  using SystemBase::xcdot_ticket;
  using SystemBase::pe_ticket;
  using SystemBase::ke_ticket;
  using SystemBase::pc_ticket;
  using SystemBase::pnc_ticket;

  // Don't promote output_port_ticket() since it is for internal use only.

 protected:
  // Promote these frequently-used methods so users (and tutorial examples)
  // don't need "this->" everywhere when in templated derived classes.
  using SystemBase::DeclareCacheEntry;

  /** Derived classes will implement this method to evaluate a witness function
  at the given context. */
  virtual T DoCalcWitnessValue(
      const Context<T>& context,
      const WitnessFunction<T>& witness_func) const = 0;

  /** Derived classes can override this method to provide witness functions
  active for the given state. The default implementation does nothing. On
  entry to this function, the context will have already been validated and
  the vector of witness functions will have been validated to be both empty
  and non-null. */
  virtual void DoGetWitnessFunctions(const Context<T>&,
      std::vector<const WitnessFunction<T>*>*) const;

  //----------------------------------------------------------------------------
  /** @name                 Event handler dispatch mechanism
  For a LeafSystem (or user implemented equivalent classes), these functions
  need to call the appropriate LeafSystem::DoX event handler. E.g.
  LeafSystem::DispatchPublishHandler() calls LeafSystem::DoPublish(). User
  supplied custom event callbacks embedded in each individual event need to
  be further dispatched in the LeafSystem::DoX handlers if desired. For a
  LeafSystem, the pseudo code of the complete default publish event handler
  dispatching is roughly:
  <pre>
    leaf_sys.Publish(context, event_collection)
    -> leaf_sys.DispatchPublishHandler(context, event_collection)
       -> leaf_sys.DoPublish(context, event_collection.get_events())
          -> for (event : event_collection_events):
               if (event.has_handler)
                 event.handler(context)
  </pre>
  Discrete update events and unrestricted update events are dispatched
  similarly for a LeafSystem.

  For a Diagram (or user implemented equivalent classes), these functions
  must iterate through all subsystems, extract their corresponding
  subcontext and subevent collections from @p context and @p events,
  and pass those to the subsystems' public non-virtual event handlers if
  the subevent collection is nonempty (e.g. System::Publish() for publish
  events).

  All of these functions are only called from their corresponding public
  non-virtual event dispatchers, where @p context is error checked. The
  derived implementations can assume that @p context is valid. See, e.g.,
  LeafSystem::DispatchPublishHandler() and Diagram::DispatchPublishHandler()
  for more details. */
  //@{

  /** This function dispatches all publish events to the appropriate
  handlers. */
  virtual void DispatchPublishHandler(
      const Context<T>& context,
      const EventCollection<PublishEvent<T>>& events) const = 0;

  /** This function dispatches all discrete update events to the appropriate
  handlers. @p discrete_state cannot be null. */
  virtual void DispatchDiscreteVariableUpdateHandler(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state) const = 0;

  virtual void DoApplyDiscreteVariableUpdate(
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state, Context<T>* context) const = 0;

  /** This function dispatches all unrestricted update events to the appropriate
  handlers. @p state cannot be null. */
  virtual void DispatchUnrestrictedUpdateHandler(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state) const = 0;

  virtual void DoApplyUnrestrictedUpdate(
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state, Context<T>* context) const = 0;
  //@}

  //----------------------------------------------------------------------------
  /** @name                    System construction
  Authors of derived %Systems can use these methods in the constructor
  for those %Systems. */
  //@{

  /** Constructs an empty %System base class object and allocates base class
  resources, possibly supporting scalar-type conversion support (AutoDiff,
  etc.) using @p converter.

  See @ref system_scalar_conversion for detailed background and examples
  related to scalar-type conversion support. */
  explicit System(SystemScalarConverter converter);

  /** Adds a port with the specified @p type and @p size to the input topology.

  Input port names must be unique for this system (passing in a duplicate
  @p name will throw std::exception). If @p name is given as
  kUseDefaultName, then a default value of e.g. "u2", where 2
  is the input number will be provided. An empty @p name is not permitted.

  If the port is intended to model a random noise or disturbance input,
  @p random_type can (optionally) be used to label it as such; doing so
  enables algorithms for design and analysis (e.g. state estimation) to
  reason explicitly about randomness at the system level.  All random input
  ports are assumed to be statistically independent.
  @pre @p name must not be empty.
  @throws std::exception for a duplicate port name.
  @returns the declared port. */
  InputPort<T>& DeclareInputPort(
      std::variant<std::string, UseDefaultName> name, PortDataType type,
      int size, std::optional<RandomDistribution> random_type = std::nullopt);

  //@}

  /** Adds an already-created constraint to the list of constraints for this
  System.  Ownership of the SystemConstraint is transferred to this system. */
  SystemConstraintIndex AddConstraint(
      std::unique_ptr<SystemConstraint<T>> constraint);

  //----------------------------------------------------------------------------
  /** @name               Virtual methods for calculations
  These virtuals allow concrete systems to implement the calculations
  defined by the `Calc` methods in the public interface. Most have default
  implementations that are usable for simple systems, but you are likely
  to need to override some or all of these in your concrete system to
  produce meaningful calculations.

  These methods are invoked by the corresponding method in the public
  interface that has the same name with `Do` removed. The public method
  performs error checking on the arguments so you do not need to do so in
  your implementation. Users cannot invoke these directly since they are
  protected. You should place your overrides in the protected or private
  sections of your concrete class. */
  //@{

  /** Override this if you have any continuous state variables x꜀ in your
  concrete %System to calculate their time derivatives. The `derivatives`
  vector will correspond elementwise with the state vector
  `Context.state.continuous_state.get_state()`. Thus, if the state in the
  Context has second-order structure `x꜀=[q v z]`, that same structure
  applies to the derivatives.

  This method is called only from the public non-virtual
  CalcTimeDerivatives() which will already have error-checked the parameters
  so you don't have to. In particular, implementations may assume that the
  given Context is valid for this %System; that the `derivatives` pointer is
  non-null, and that the referenced object has the same constituent
  structure as was produced by AllocateTimeDerivatives().

  The default implementation does nothing if the `derivatives` vector is
  size zero and aborts otherwise. */
  virtual void DoCalcTimeDerivatives(const Context<T>& context,
                                     ContinuousState<T>* derivatives) const;

  /** Override this if you have an efficient way to evaluate the implicit
  time derivatives residual for this System. Otherwise the default
  implementation is
  `residual = proposed_derivatives − EvalTimeDerivatives(context)`. Note that
  you cannot use the default implementation if you have changed the declared
  residual size.

  @note The public method has already verified that `proposed_derivatives`
  is compatible with this System and that `residual` is non-null and of the
  declared size (as reported by
  SystemBase::implicit_time_derivatives_residual_size()). You do not have to
  check those two conditions in your implementation, but if you have additional
  restrictions you should validate that they are also met. */
  virtual void DoCalcImplicitTimeDerivativesResidual(
      const Context<T>& context, const ContinuousState<T>& proposed_derivatives,
      EigenPtr<VectorX<T>> residual) const;

  /** Computes the next time at which this System must perform a discrete
  action.

  Override this method if your System has any discrete actions which must
  interrupt the continuous simulation. This method is called only from the
  public non-virtual CalcNextUpdateTime() which will already have
  error-checked the parameters so you don't have to. You may assume that
  @p context has already been validated and @p events pointer is not
  null.

  If you override this method, you _must_ set the returned @p time. Set it to
  Infinity if there are no upcoming timed events. If you return a finite update
  time, you _must_ put at least one Event object in the @p events collection.
  These requirements are enforced by the public CalcNextUpdateTime() method.

  The default implementation returns with the next sample time being
  Infinity and no events added to @p events. */
  virtual void DoCalcNextUpdateTime(const Context<T>& context,
                                    CompositeEventCollection<T>* events,
                                    T* time) const;

  /** Implement this method to return all periodic triggered events.
  @see GetPeriodicEvents() for a detailed description of the returned
       variable.
  @note The default implementation returns an empty map. */
  virtual std::map<PeriodicEventData,
      std::vector<const Event<T>*>, PeriodicEventDataComparator>
    DoGetPeriodicEvents() const = 0;

  /** Implement this method to return any events to be handled before the
  simulator integrates the system's continuous state at each time step.
  @p events is cleared in the public non-virtual GetPerStepEvents()
  before that method calls this function. You may assume that @p context
  has already been validated and that @p events is not null. @p events
  can be changed freely by the overriding implementation.

  The default implementation returns without changing @p events.
  @sa GetPerStepEvents() */
  virtual void DoGetPerStepEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* events) const;

  /** Implement this method to return any events to be handled at the
  simulator's initialization step. @p events is cleared in the public
  non-virtual GetInitializationEvents(). You may assume that @p context has
  already been validated and that @p events is not null. @p events can be
  changed freely by the overriding implementation.

  The default implementation returns without changing @p events.
  @sa GetInitializationEvents() */
  virtual void DoGetInitializationEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* events) const;

  /** Override this method for physical systems to calculate the potential
  energy PE currently stored in the configuration provided in the given
  Context. The default implementation returns 0 which is correct for
  non-physical systems. You may assume that `context` has already
  been validated before it is passed to you here.

  See EvalPotentialEnergy() for details on what you must compute here. In
  particular, your potential energy method must _not_ depend explicitly on
  time, velocities, or any input port values. */
  virtual T DoCalcPotentialEnergy(const Context<T>& context) const;

  /** Override this method for physical systems to calculate the kinetic
  energy KE currently present in the motion provided in the given
  Context. The default implementation returns 0 which is correct for
  non-physical systems. You may assume that `context` has already
  been validated before it is passed to you here.

  See EvalKineticEnergy() for details on what you must compute here. In
  particular, your kinetic energy method must _not_ depend explicitly on
  time or any input port values. */
  virtual T DoCalcKineticEnergy(const Context<T>& context) const;

  /** Override this method to return the rate Pc at which mechanical energy is
  being converted _from_ potential energy _to_ kinetic energy by this system
  in the given Context. By default, returns zero. Physical systems should
  override. You may assume that `context` has already been validated before
  it is passed to you here.

  See EvalConservativePower() for details on what you must compute here. In
  particular, this quantity must be _positive_ when potential energy
  is _decreasing_, and your conservative power method must _not_ depend
  explicitly on time or any input port values. */
  virtual T DoCalcConservativePower(const Context<T>& context) const;

  /** Override this method to return the rate Pnc at which work W is done on the
  system by non-conservative forces. By default, returns zero. Physical
  systems should override. You may assume that `context` has already been
  validated before it is passed to you here.

  See EvalNonConservativePower() for details on what you must compute here.
  In particular, this quantity must be _negative_ if the non-conservative
  forces are _dissipative_, positive otherwise. Your non-conservative power
  method can depend on anything you find in the given Context, including
  time and input ports. */
  virtual T DoCalcNonConservativePower(const Context<T>& context) const;

  /** Provides the substantive implementation of MapQDotToVelocity().

  The default implementation uses the identity mapping, and correctly does
  nothing if the %System does not have second-order state variables. It
  throws std::exception if the `generalized_velocity` and
  `qdot` are not the same size, but that is not enough to guarantee that
  the default implementation is adequate. Child classes must
  override this function if qdot != v (even if they are the same size).
  This occurs, for example, if a joint uses roll-pitch-yaw rotation angles
  for orientation but angular velocity for rotational rate rather than
  rotation angle derivatives.

  If you implement this method you are required to use no more than `O(nq)`
  time where `nq` is the size of `qdot`, so that the %System can meet the
  performance guarantee made for the public interface, and you must also
  implement DoMapVelocityToQDot(). Implementations may assume that `qdot`
  has already been validated to be the same size as `q` in the given
  Context, and that `generalized_velocity` is non-null. */
  virtual void DoMapQDotToVelocity(const Context<T>& context,
                                   const Eigen::Ref<const VectorX<T>>& qdot,
                                   VectorBase<T>* generalized_velocity) const;

  /** Provides the substantive implementation of MapVelocityToQDot().

  The default implementation uses the identity mapping, and correctly does
  nothing if the %System does not have second-order state variables. It
  throws std::exception if the `generalized_velocity` (`v`) and
  `qdot` are not the same size, but that is not enough to guarantee that
  the default implementation is adequate. Child classes must
  override this function if `qdot != v` (even if they are the same size).
  This occurs, for example, if a joint uses roll-pitch-yaw rotation angles
  for orientation but angular velocity for rotational rate rather than
  rotation angle derivatives.

  If you implement this method you are required to use no more than `O(nq)`
  time where `nq` is the size of `qdot`, so that the %System can meet the
  performance guarantee made for the public interface, and you must also
  implement DoMapQDotToVelocity(). Implementations may assume that
  `generalized_velocity` has already been validated to be the same size as
  `v` in the given Context, and that `qdot` is non-null. */
  virtual void DoMapVelocityToQDot(
      const Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      VectorBase<T>* qdot) const;
  //@}

  //----------------------------------------------------------------------------
  /** @name                 Utility methods (protected) */
  //@{

  /** Returns a mutable Eigen expression for a vector valued output port with
  index @p port_index in this system. All input ports that directly depend
  on this output port will be notified that upstream data has changed, and
  may invalidate cache entries as a result. */
  Eigen::VectorBlock<VectorX<T>> GetMutableOutputVector(SystemOutput<T>* output,
                                                        int port_index) const;
  //@}

  bool forced_publish_events_exist() const {
    return forced_publish_events_ != nullptr;
  }

  bool forced_discrete_update_events_exist() const {
    return forced_discrete_update_events_ != nullptr;
  }

  bool forced_unrestricted_update_events_exist() const {
    return forced_unrestricted_update_events_ != nullptr;
  }

  EventCollection<PublishEvent<T>>& get_mutable_forced_publish_events() {
    DRAKE_DEMAND(forced_publish_events_ != nullptr);
    return *forced_publish_events_;
  }

  EventCollection<DiscreteUpdateEvent<T>>&
  get_mutable_forced_discrete_update_events() {
    DRAKE_DEMAND(forced_discrete_update_events_ != nullptr);
    return *forced_discrete_update_events_;
  }

  EventCollection<UnrestrictedUpdateEvent<T>>&
  get_mutable_forced_unrestricted_update_events() {
    DRAKE_DEMAND(forced_unrestricted_update_events_ != nullptr);
    return *forced_unrestricted_update_events_;
  }

  const EventCollection<PublishEvent<T>>&
  get_forced_publish_events() const {
    DRAKE_DEMAND(forced_publish_events_ != nullptr);
    return *forced_publish_events_;
  }

  const EventCollection<DiscreteUpdateEvent<T>>&
  get_forced_discrete_update_events() const {
    DRAKE_DEMAND(forced_discrete_update_events_ != nullptr);
    return *forced_discrete_update_events_;
  }

  const EventCollection<UnrestrictedUpdateEvent<T>>&
  get_forced_unrestricted_update_events() const {
    DRAKE_DEMAND(forced_unrestricted_update_events_ != nullptr);
    return *forced_unrestricted_update_events_;
  }

  void set_forced_publish_events(
  std::unique_ptr<EventCollection<PublishEvent<T>>> forced) {
    forced_publish_events_ = std::move(forced);
  }

  void set_forced_discrete_update_events(
  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>> forced) {
    forced_discrete_update_events_ = std::move(forced);
  }

  void set_forced_unrestricted_update_events(
  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>> forced) {
    forced_unrestricted_update_events_ = std::move(forced);
  }

  /** Returns the SystemScalarConverter for `this` system. */
  SystemScalarConverter& get_mutable_system_scalar_converter() {
    return system_scalar_converter_;
  }

 private:
  // For any T1 & T2, System<T1> considers System<T2> a friend, so that System
  // can safely and efficiently convert scalar types. See for example
  // System<T>::ToScalarTypeMaybe.
  template <typename> friend class System;

  // Allocates an input of the leaf type that the System requires on the port
  // specified by @p input_port.  This is final in LeafSystem and Diagram.
  virtual std::unique_ptr<AbstractValue> DoAllocateInput(
      const InputPort<T>& input_port) const = 0;

  // Allocates a composite event collection for use with this system.
  // Implementers should not set system_id; that is done by the wrapping
  // AllocateCompositeEventCollection method. This method is final in
  // LeafSystem and Diagram.
  virtual std::unique_ptr<CompositeEventCollection<T>>
  DoAllocateCompositeEventCollection() const = 0;

  std::function<void(const AbstractValue&)> MakeFixInputPortTypeChecker(
      InputPortIndex port_index) const final;

  // Shared code for updating a vector input port and returning a pointer to its
  // value as a BasicVector<T>, or nullptr if the port is not connected. Throws
  // a logic_error if the port_index is out of range or if the input port is not
  // declared to be a vector-valued port. `func` should be the user-visible API
  // function name obtained with __func__.
  const BasicVector<T>* EvalBasicVectorInputImpl(
      const char* func, const Context<T>& context,
      InputPortIndex port_index) const;

  // Adds "external" constraints to this System.  This is a helper function to
  // minimize inline bloat in scalar conversion; it is marked private since the
  // signature matches the private data field type for efficiency.
  void AddExternalConstraints(
      const std::vector<ExternalSystemConstraint>& constraints);

  // The constraints_ vector encompass all constraints on this system, whether
  // they were declared by a concrete subclass during construction (e.g., by
  // calling DeclareInequalityConstraint), or added after construction (e.g.,
  // by AddExternalConstraint).  The constraints are listed in the order they
  // were added, which means that the construction-time constraints will always
  // appear earlier in the vector than the post-construction constraints.
  std::vector<std::unique_ptr<SystemConstraint<T>>> constraints_;
  // The external_constraints_ vector only contains constraints added after
  // construction (e.g., by AddExternalConstraint), in the order they were
  // added.  The contents of this vector is only used during scalar conversion
  // (so that the external constraints are preserved); for runtime calculations,
  // only the constraints_ vector is used.
  std::vector<ExternalSystemConstraint> external_constraints_;

  // These are only used to dispatch forced event handling. For a LeafSystem,
  // these contain at least one kForced triggered event. For a Diagram, they
  // are DiagramEventCollection, whose leafs are LeafEventCollection with
  // one or more kForced triggered events.
  std::unique_ptr<EventCollection<PublishEvent<T>>>
      forced_publish_events_{nullptr};
  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
      forced_discrete_update_events_{nullptr};
  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
      forced_unrestricted_update_events_{nullptr};

  // Functions to convert this system to use alternative scalar types.
  SystemScalarConverter system_scalar_converter_;

  CacheIndex time_derivatives_cache_index_;
  CacheIndex potential_energy_cache_index_;
  CacheIndex kinetic_energy_cache_index_;
  CacheIndex conservative_power_cache_index_;
  CacheIndex nonconservative_power_cache_index_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::System)
