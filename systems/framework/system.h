#pragma once

#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/random.h"
#include "drake/common/symbolic.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/event_collection.h"
#include "drake/systems/framework/input_port.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/system_base.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_scalar_converter.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace systems {

#if !defined(DRAKE_DOXYGEN_CXX)
// Private helper class for System.
class SystemImpl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemImpl)
  SystemImpl() = delete;

  // The implementation of System<T>::GetMemoryObjectName.
  static std::string GetMemoryObjectName(const std::string&, int64_t);

 private:
  // Attorney-Client idiom to expose a subset of private elements of System.
  // We are the attorney.  These are the clients that can access our private
  // members, and thus access some subset of System's private members.
  template <typename> friend class Diagram;

  // Return a mutable reference to the System's SystemScalarConverter.  Diagram
  // needs this in order to withdraw support for certain scalar type conversion
  // operations once it learns about what its subsystems support.
  template <typename T>
  static SystemScalarConverter& get_mutable_system_scalar_converter(
      System<T>* system) {
    DRAKE_DEMAND(system != nullptr);
    return system->system_scalar_converter_;
  }
};
#endif  // DRAKE_DOXYGEN_CXX

/// Base class for all System functionality that is dependent on the templatized
/// scalar type T for input, state, parameters, and outputs.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class System : public SystemBase {
 public:
  // System objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(System)

  ~System() override;

  //----------------------------------------------------------------------------
  /// @name           Resource allocation and initialization
  /// These methods are used to allocate and initialize Context resources.
  //@{

  /// Returns a Context<T> suitable for use with this System<T>.
  // This is just an intentional shadowing of the base class method to return
  // a more convenient type.
  std::unique_ptr<Context<T>> AllocateContext() const {
    return dynamic_pointer_cast_or_throw<Context<T>>(
        SystemBase::AllocateContext());
  }

  /// Allocates a CompositeEventCollection for this system. The allocated
  /// instance is used for registering events; for example, Simulator passes
  /// this object to System::CalcNextUpdateTime() to allow the system to
  /// register upcoming events.
  virtual std::unique_ptr<CompositeEventCollection<T>>
      AllocateCompositeEventCollection() const = 0;

  /// Given an input port, allocates the vector storage.  The @p input_port
  /// must match a port declared via DeclareInputPort.
  std::unique_ptr<BasicVector<T>> AllocateInputVector(
      const InputPort<T>& input_port) const {
    DRAKE_THROW_UNLESS(input_port.get_data_type() == kVectorValued);
    const int index = input_port.get_index();
    DRAKE_ASSERT(index >= 0 && index < num_input_ports());
    DRAKE_ASSERT(get_input_port(index).get_data_type() == kVectorValued);
    std::unique_ptr<AbstractValue> value = DoAllocateInput(input_port);
    return value->get_value<BasicVector<T>>().Clone();
  }

  /// Given an input port, allocates the abstract storage.  The @p input_port
  /// must match a port declared via DeclareInputPort.
  std::unique_ptr<AbstractValue> AllocateInputAbstract(
      const InputPort<T>& input_port) const {
    const int index = input_port.get_index();
    DRAKE_ASSERT(index >= 0 && index < num_input_ports());
    return DoAllocateInput(input_port);
  }

  /// Returns a container that can hold the values of all of this System's
  /// output ports. It is sized with the number of output ports and uses each
  /// output port's allocation method to provide an object of the right type
  /// for that port.
  std::unique_ptr<SystemOutput<T>> AllocateOutput() const {
    // make_unique can't invoke this private constructor.
    auto output = std::unique_ptr<SystemOutput<T>>(new SystemOutput<T>());
    for (int i = 0; i < this->num_output_ports(); ++i) {
      const OutputPort<T>& port = this->get_output_port(i);
      output->add_port(port.Allocate());
    }
    return output;
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

  /// Returns a DiscreteState of the same dimensions as the discrete_state
  /// allocated in CreateDefaultContext. The simulator will provide this state
  /// as the output argument to Update.
  /// By default, allocates nothing. Systems with discrete state variables
  /// should override.
  virtual std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables() const {
    return nullptr;
  }

  /// This convenience method allocates a context using AllocateContext() and
  /// sets its default values using SetDefaultContext().
  std::unique_ptr<Context<T>> CreateDefaultContext() const {
    std::unique_ptr<Context<T>> context = AllocateContext();
    SetDefaultContext(context.get());
    return context;
  }

  /// Assigns default values to all elements of the state. Overrides must not
  /// change the number of state variables.
  virtual void SetDefaultState(const Context<T>& context,
                               State<T>* state) const = 0;

  /// Assigns default values to all parameters. Overrides must not
  /// change the number of parameters.
  virtual void SetDefaultParameters(const Context<T>& context,
                                    Parameters<T>* parameters) const = 0;

  // Sets Context fields to their default values.  User code should not
  // override.
  void SetDefaultContext(Context<T>* context) const {
    // Set the default state, checking that the number of state variables does
    // not change.
    const int n_xc = context->num_continuous_states();
    const int n_xd = context->num_discrete_state_groups();
    const int n_xa = context->num_abstract_states();

    SetDefaultState(*context, &context->get_mutable_state());

    DRAKE_DEMAND(n_xc == context->num_continuous_states());
    DRAKE_DEMAND(n_xd == context->num_discrete_state_groups());
    DRAKE_DEMAND(n_xa == context->num_abstract_states());

    // Set the default parameters, checking that the number of parameters does
    // not change.
    const int num_params = context->num_numeric_parameter_groups();
    SetDefaultParameters(*context, &context->get_mutable_parameters());
    DRAKE_DEMAND(num_params == context->num_numeric_parameter_groups());
  }

  /// Assigns random values to all elements of the state.
  /// This default implementation calls SetDefaultState; override this method to
  /// provide random initial conditions using the stdc++ random library, e.g.:
  /// @code
  ///   std::normal_distribution<T> gaussian();
  ///   state->get_mutable_continuous_state()->get_mutable_vector()
  ///        ->SetAtIndex(0, gaussian(*generator));
  /// @endcode
  /// Overrides must not change the number of state variables.
  ///
  /// @see @ref stochastic_systems
  virtual void SetRandomState(const Context<T>& context, State<T>* state,
                              RandomGenerator* generator) const {
    unused(generator);
    SetDefaultState(context, state);
  }

  /// Assigns random values to all parameters.
  /// This default implementation calls SetDefaultParameters; override this
  /// method to provide random parameters using the stdc++ random library, e.g.:
  /// @code
  ///   std::uniform_real_distribution<T> uniform();
  ///   parameters->get_mutable_numeric_parameter(0)
  ///             ->SetAtIndex(0, uniform(*generator));
  /// @endcode
  /// Overrides must not change the number of state variables.
  ///
  /// @see @ref stochastic_systems
  virtual void SetRandomParameters(const Context<T>& context,
                                   Parameters<T>* parameters,
                                   RandomGenerator* generator) const {
    unused(generator);
    SetDefaultParameters(context, parameters);
  }

  // Sets Context fields to random values.  User code should not
  // override.
  void SetRandomContext(Context<T>* context, RandomGenerator* generator) const {
    // Set the default state, checking that the number of state variables does
    // not change.
    const int n_xc = context->num_continuous_states();
    const int n_xd = context->num_discrete_state_groups();
    const int n_xa = context->num_abstract_states();

    SetRandomState(*context, &context->get_mutable_state(), generator);

    DRAKE_DEMAND(n_xc == context->num_continuous_states());
    DRAKE_DEMAND(n_xd == context->num_discrete_state_groups());
    DRAKE_DEMAND(n_xa == context->num_abstract_states());

    // Set the default parameters, checking that the number of parameters does
    // not change.
    const int num_params = context->num_numeric_parameter_groups();
    SetRandomParameters(*context, &context->get_mutable_parameters(),
                        generator);
    DRAKE_DEMAND(num_params == context->num_numeric_parameter_groups());
  }

  /// For each input port, allocates a fixed input of the concrete type
  /// that this System requires, and binds it to the port, disconnecting any
  /// prior input. Does not assign any values to the fixed inputs.
  void AllocateFixedInputs(Context<T>* context) const {
    for (InputPortIndex i(0); i < num_input_ports(); ++i) {
      const InputPort<T>& port = get_input_port(i);
      if (port.get_data_type() == kVectorValued) {
        context->FixInputPort(port.get_index(), AllocateInputVector(port));
      } else {
        DRAKE_DEMAND(port.get_data_type() == kAbstractValued);
        context->FixInputPort(port.get_index(), AllocateInputAbstract(port));
      }
    }
  }

  /// Reports all direct feedthroughs from input ports to output ports. For
  /// a system with m input ports: `I = i₀, i₁, ..., iₘ₋₁`, and n output ports,
  /// `O = o₀, o₁, ..., oₙ₋₁`, the return map will contain pairs (u, v) such
  /// that
  ///
  /// - 0 ≤ u < m,
  /// - 0 ≤ v < n,
  /// - and there _might_ be a direct feedthrough from input iᵤ to each
  ///   output oᵥ.
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
  ///
  /// Methods in this group that specify preconditions operate as follows:
  /// The preconditions will be checked in Debug builds but some or all might
  /// not be checked in Release builds for performance reasons. If we do check
  /// and a precondition is violated, an std::logic_error will be thrown with
  /// a helpful message.
  //@{

  /// Returns a reference to the cached value of the continuous state variable
  /// time derivatives, evaluating first if necessary using
  /// CalcTimeDerivatives().
  ///
  /// This method returns the time derivatives `xcdot` of the continuous state
  /// `xc`. The referenced return object will correspond elementwise with the
  /// continuous state in the given Context. Thus, if the state in the Context
  /// has second-order structure `xc=[q v z]`, that same structure applies to
  /// the derivatives so we will have `xcdot=[qdot vdot zdot]`.
  ///
  /// @param context The Context whose time, input port, parameter, state, and
  /// accuracy values may be used to evaluate the derivatives.
  ///
  /// @retval xcdot The time derivatives of `xc` returned as a reference to an
  ///               object of the same type and size as this %Context's
  ///               continuous state.
  /// @see CalcTimeDerivatives(), get_time_derivatives_cache_entry()
  const ContinuousState<T>& EvalTimeDerivatives(
      const Context<T>& context) const {
    const CacheEntry& entry = get_time_derivatives_cache_entry();
    return entry.Eval<ContinuousState<T>>(context);
  }

  /// (Advanced) Returns the CacheEntry used to cache time derivatives for
  /// EvalTimeDerivatives().
  const CacheEntry& get_time_derivatives_cache_entry() const {
    return this->get_cache_entry(time_derivatives_cache_index_);
  }

  /// Returns a reference to the cached value of the potential energy (PE),
  /// evaluating first if necessary using CalcPotentialEnergy().
  ///
  /// By definition here, potential energy depends only on "configuration"
  /// (e.g. orientation and position), which includes a subset of the state
  /// variables, and parameters that affect configuration or conservative
  /// forces (such as lengths and masses). The calculated value may also be
  /// affected by the accuracy value supplied in the Context. PE cannot depend
  /// explicitly on time (∂PE/∂t = 0), velocities (∂PE/∂v = 0), or input port
  /// values (∂PE/∂u = 0).
  ///
  /// Non-physical systems where PE is not meaningful will return PE = 0.
  ///
  /// @param context The Context whose configuration variables may be used to
  ///                evaluate potential energy.
  /// @retval PE The potential energy in joules (J) represented by the
  ///            configuration given in `context`.
  /// @see CalcPotentialEnergy()
  const T& EvalPotentialEnergy(const Context<T>& context) const {
    const CacheEntry& entry =
        this->get_cache_entry(potential_energy_cache_index_);
    return entry.Eval<T>(context);
  }

  /// Returns a reference to the cached value of the kinetic energy (KE),
  /// evaluating first if necessary using CalcKineticEnergy().
  ///
  /// By definition here, kinetic energy depends only on "configuration" and
  /// "velocity" (e.g. angular and translational velocity) of moving masses
  /// which includes a subset of the state variables, and parameters that affect
  /// configuration, velocities, or mass properties. The calculated value may
  /// also be affected by the accuracy value supplied in the Context. KE cannot
  /// depend explicitly on time (∂KE/∂t = 0) or input port values (∂KE/∂u = 0).
  ///
  /// Non-physical systems where KE is not meaningful will return KE = 0.
  ///
  /// @param context The Context whose configuration and velocity variables may
  ///                be used to evaluate kinetic energy.
  /// @retval KE The kinetic energy in joules (J) represented by the
  ///            configuration and velocity given in `context`.
  /// @see CalcKineticEnergy()
  const T& EvalKineticEnergy(const Context<T>& context) const {
    const CacheEntry& entry =
        this->get_cache_entry(kinetic_energy_cache_index_);
    return entry.Eval<T>(context);
  }

  /// Returns a reference to the cached value of the conservative power (Pc),
  /// evaluating first if necessary using CalcConservativePower().
  ///
  /// The returned Pc represents the rate at which mechanical energy is being
  /// converted _from_ potential energy (PE) _to_ kinetic energy (KE) by this
  /// system in the given Context. This quantity will be _positive_ when PE
  /// is _decreasing_. By definition here, conservative power may depend only
  /// on quantities that explicitly contribute to PE and KE. See
  /// EvalPotentialEnergy() and EvalKineticEnergy() for details.
  ///
  /// Power due to non-conservative forces (e.g. dampers) can contribute to the
  /// rate of change of KE. Therefore this method alone cannot be used to
  /// determine whether KE is increasing or decreasing, only whether the
  /// conservative power is adding or removing kinetic energy.
  /// EvalNonConservativePower() can be used in conjunction with this method to
  /// find the total rate of change of KE.
  ///
  /// Non-physical systems where Pc is not meaningful will return Pc = 0.
  ///
  /// @param context The Context whose contents may be used to evaluate
  ///                conservative power.
  /// @retval Pc The conservative power in watts (W or J/s) represented by the
  ///            contents of the given `context`.
  /// @see CalcConservativePower(), EvalNonConservativePower(),
  ///      EvalPotentialEnergy(), EvalKineticEnergy()
  const T& EvalConservativePower(const Context<T>& context) const {
    const CacheEntry& entry =
        this->get_cache_entry(conservative_power_cache_index_);
    return entry.Eval<T>(context);
  }

  /// Returns a reference to the cached value of the non-conservative power
  /// (Pnc), evaluating first if necessary using CalcNonConservativePower().
  ///
  /// The returned Pnc represents the rate at which work W is done on the system
  /// by non-conservative forces. Pnc is _negative_ if the non-conservative
  /// forces are _dissipative_, positive otherwise. Time integration of Pnc
  /// yields work W, and the total mechanical energy `E = PE + KE − W` should be
  /// conserved by any physically-correct model, to within integration accuracy
  /// of W. Power is in watts (J/s). (Watts are abbreviated W but not to be
  /// confused with work!) Any values in the supplied Context (including time
  /// and input ports) may contribute to the computation of non-conservative
  /// power.
  ///
  /// Non-physical systems where Pnc is not meaningful will return Pnc = 0.
  ///
  /// @param context The Context whose contents may be used to evaluate
  ///                non-conservative power.
  /// @retval Pnc The non-conservative power in watts (W or J/s) represented by
  ///             the contents of the given `context`.
  /// @see CalcNonConservativePower(), EvalConservativePower()
  const T& EvalNonConservativePower(const Context<T>& context) const {
    const CacheEntry& entry =
        this->get_cache_entry(nonconservative_power_cache_index_);
    return entry.Eval<T>(context);
  }

  // TODO(jwnimmer-tri) Deprecate me.
  /// Returns the value of the vector-valued input port with the given
  /// `port_index` as a BasicVector or a specific subclass `Vec` derived from
  /// BasicVector. Causes the value to become up to date first if necessary. See
  /// EvalAbstractInput() for more information.
  ///
  /// The result is returned as a pointer to the input port's value of type
  /// `Vec<T>` or nullptr if the port is not connected.
  ///
  /// @pre `port_index` selects an existing input port of this System.
  /// @pre the port must have been declared to be vector-valued.
  /// @pre the port's value must be of type Vec<T>.
  ///
  /// @tparam Vec The template type of the input vector, which must be a
  ///             subclass of BasicVector.
  template <template <typename> class Vec = BasicVector>
  const Vec<T>* EvalVectorInput(const Context<T>& context,
                                int port_index) const {
    static_assert(
        std::is_base_of<BasicVector<T>, Vec<T>>::value,
        "In EvalVectorInput<Vec>, Vec must be a subclass of BasicVector.");

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

  // TODO(jwnimmer-tri) Deprecate me.
  /// Returns the value of the vector-valued input port with the given
  /// `port_index` as an %Eigen vector. Causes the value to become up to date
  /// first if necessary. See EvalAbstractInput() for more information.
  ///
  /// @pre `port_index` selects an existing input port of this System.
  /// @pre the port must have been declared to be vector-valued.
  /// @pre the port must be evaluable (connected or fixed).
  ///
  /// @see EvalVectorInput()
  Eigen::VectorBlock<const VectorX<T>> EvalEigenVectorInput(
      const Context<T>& context, int port_index) const {
    if (port_index < 0)
      ThrowNegativePortIndex(__func__, port_index);
    const InputPortIndex port(port_index);

    const BasicVector<T>* const basic_value =
        EvalBasicVectorInputImpl(__func__, context, port);
    if (basic_value == nullptr)
      ThrowCantEvaluateInputPort(__func__, port);

    return basic_value->get_value();
  }
  //@}

  //----------------------------------------------------------------------------
  /// @name               Constraint-related functions
  //@{

  /// Gets the number of constraint equations for this system using the given
  /// context (useful in case the number of constraints is dependent upon the
  /// current state (as might be the case with a system modeled using piecewise
  /// differential algebraic equations).
  int num_constraint_equations(const Context<T>& context) const {
    return do_get_num_constraint_equations(context);
  }

  /// Evaluates the constraint equations for the system at the generalized
  /// coordinates and generalized velocity specified by the context. The context
  /// allows the set of constraints to be dependent upon the current system
  /// state (as might be the case with a system modeled using piecewise
  /// differential algebraic equations).
  /// @returns a vector of dimension num_constraint_equations(); the
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
  /// @returns a vector of dimension num_constraint_equations().
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
    DRAKE_ASSERT(lambda.size() == num_constraint_equations(context));
    DRAKE_ASSERT(J.rows() == num_constraint_equations(context));
    DRAKE_ASSERT(
        J.cols() ==
        context.get_continuous_state().get_generalized_velocity().size());
    return DoCalcVelocityChangeFromConstraintImpulses(context, J, lambda);
  }

  /// Computes the norm on constraint error (used as a metric for comparing
  /// errors between the outputs of algebraic equations applied to two
  /// different state variable instances). This norm need be neither continuous
  /// nor differentiable.
  /// @throws std::logic_error if the dimension of @p err is not equivalent to
  ///         the output of num_constraint_equations().
  double CalcConstraintErrorNorm(const Context<T>& context,
                                 const Eigen::VectorXd& error) const {
    if (error.size() != num_constraint_equations(context))
      throw std::logic_error("Error vector is mis-sized.");
    return DoCalcConstraintErrorNorm(context, error);
  }

  /// Adds an "external" constraint to this System.
  ///
  /// This method is intended for use by applications that are examining this
  /// System to add additional constraints based on their particular situation
  /// (e.g., that a velocity state element has an upper bound); it is not
  /// intended for declaring intrinsic constraints that some particular System
  /// subclass might always impose on itself (e.g., that a mass parameter is
  /// non-negative).  To that end, this method should not be called by
  /// subclasses of `this` during their constructor.
  ///
  /// The `constraint` will automatically persist across system scalar
  /// conversion.
  SystemConstraintIndex AddExternalConstraint(
      ExternalSystemConstraint constraint) {
    const auto& calc = constraint.get_calc<T>();
    if (calc) {
      constraints_.emplace_back(std::make_unique<SystemConstraint<T>>(
          this, calc, constraint.bounds(), constraint.description()));
    } else {
      constraints_.emplace_back(std::make_unique<SystemConstraint<T>>(
          this, fmt::format(
              "{} (disabled for this scalar type)",
              constraint.description())));
    }
    external_constraints_.emplace_back(std::move(constraint));
    return SystemConstraintIndex(constraints_.size() - 1);
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

  /// Calculates the time derivatives `xcdot` of the continuous state `xc` into
  /// a given output argument. Prefer EvalTimeDerivatives() instead to avoid
  /// unnecessary recomputation.
  /// @see EvalTimeDerivatives() for more information.
  ///
  /// @param context The Context whose contents will be used to evaluate the
  ///                derivatives.
  /// @param derivatives The time derivatives `xcdot`. Must be the same size as
  ///                    the continuous state vector in `context`.
  void CalcTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const {
    DRAKE_DEMAND(derivatives != nullptr);
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DoCalcTimeDerivatives(context, derivatives);
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

  /// Given the @p discrete_state results of a previous call to
  /// CalcDiscreteVariableUpdates() that processed the given collection of
  /// events, modifies the @p context to reflect the updated @p discrete_state.
  /// @param[in] events
  ///     The Event collection that resulted in the given @p discrete_state.
  /// @param[in,out] discrete_state
  ///     The updated discrete state from a CalcDiscreteVariableUpdates()
  ///     call. This is mutable to permit its contents to be swapped with the
  ///     corresponding @p context contents (rather than copied).
  /// @param[in,out] context
  ///     The Context whose discrete state is modified to match
  ///     @p discrete_state. Note that swapping contents with @p discrete_state
  ///     may cause addresses of individual discrete state group vectors in
  ///     @p context to be different on return than they were on entry.
  /// @pre @p discrete_state is the result of a previous
  ///      CalcDiscreteVariableUpdates() call that processed this @p events
  ///      collection.
  void ApplyDiscreteVariableUpdate(
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state, Context<T>* context) const {
    DoApplyDiscreteVariableUpdate(events, discrete_state, context);
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
    const int continuous_state_dim = state->get_continuous_state().size();
    const int discrete_state_dim = state->get_discrete_state().num_groups();
    const int abstract_state_dim = state->get_abstract_state().size();

    DispatchUnrestrictedUpdateHandler(context, events, state);

    if (continuous_state_dim != state->get_continuous_state().size() ||
        discrete_state_dim != state->get_discrete_state().num_groups() ||
        abstract_state_dim != state->get_abstract_state().size())
      throw std::logic_error(
          "State variable dimensions cannot be changed "
          "in CalcUnrestrictedUpdate().");
  }

  /// Given the @p state results of a previous call to CalcUnrestrictedUpdate()
  /// that processed the given collection of events, modifies the @p context to
  /// reflect the updated @p state.
  /// @param[in] events
  ///     The Event collection that resulted in the given @p state.
  /// @param[in,out] state
  ///     The updated State from a CalcUnrestrictedUpdate() call. This is
  ///     mutable to permit its contents to be swapped with the corresponding
  ///     @p context contents (rather than copied).
  /// @param[in,out] context
  ///     The Context whose State is modified to match @p state. Note that
  ///     swapping contents with the @p state may cause addresses of
  ///     continuous, discrete, and abstract state containers in @p context
  ///     to be different on return than they were on entry.
  /// @pre @p state is the result of a previous CalcUnrestrictedUpdate() call
  ///      that processed this @p events collection.
  void ApplyUnrestrictedUpdate(
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state, Context<T>* context) const {
    DoApplyUnrestrictedUpdate(events, state, context);
  }

  /// This method forces an unrestricted update on the system given a
  /// @p context, and the updated state is stored in @p state. The
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
    T time{NAN};
    DoCalcNextUpdateTime(context, events, &time);
    using std::isnan;
    DRAKE_ASSERT(!isnan(time));
    return time;
  }

  /// This method is called by Simulator::Initialize() to gather all update
  /// and publish events that are to be handled in AdvanceTo() at the point
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

  /// This method is called by Simulator::Initialize() to gather all
  /// update and publish events that need to be handled at initialization
  /// before the simulator starts integration.
  ///
  /// @p events cannot be null. @p events will be cleared on entry.
  void GetInitializationEvents(const Context<T>& context,
                               CompositeEventCollection<T>* events) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DRAKE_DEMAND(events != nullptr);
    events->Clear();
    DoGetInitializationEvents(context, events);
  }

  /// Gets whether there exists a unique periodic attribute that triggers
  /// one or more discrete update events (and, if so, returns that unique
  /// periodic attribute). Thus, this method can be used (1) as a test to
  /// determine whether a system's dynamics are at least partially governed by
  /// difference equations and (2) to obtain the difference equation update
  /// times.
  /// @returns optional<PeriodicEventData> Contains the periodic trigger
  /// attributes if the unique periodic attribute exists, otherwise `nullopt`.
  optional<PeriodicEventData>
      GetUniquePeriodicDiscreteUpdateAttribute() const {
    optional<PeriodicEventData> saved_attr;
    auto periodic_events = GetPeriodicEvents();
    for (const auto& saved_attr_and_vector : periodic_events) {
      for (const auto& event : saved_attr_and_vector.second) {
        if (event->is_discrete_update()) {
          if (saved_attr)
            return nullopt;
          saved_attr = saved_attr_and_vector.first;
          break;
        }
      }
    }

    return saved_attr;
  }

  /// Gets all periodic triggered events for a system. Each periodic attribute
  /// (offset and period, in seconds) is mapped to one or more update events
  /// that are to be triggered at the proper times.
  std::map<PeriodicEventData, std::vector<const Event<T>*>,
    PeriodicEventDataComparator> GetPeriodicEvents() const {
    return DoGetPeriodicEvents();
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
    for (OutputPortIndex i(0); i < num_output_ports(); ++i) {
      // TODO(sherm1) Would be better to use Eval() here but we don't have
      // a generic abstract assignment capability that would allow us to
      // copy into existing memory in `outputs` (rather than clone). User
      // code depends on memory stability in SystemOutput.
      get_output_port(i).Calc(context, outputs->GetMutableData(i));
    }
  }

  /// Calculates and returns the potential energy represented by the current
  /// configuration provided in `context`. Prefer EvalPotentialEnergy() to
  /// avoid unnecessary recalculation.
  ///
  /// @see EvalPotentialEnergy() for more information.
  T CalcPotentialEnergy(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcPotentialEnergy(context);
  }

  /// Calculates and returns the kinetic energy represented by the current
  /// configuration and velocity provided in `context`. Prefer
  /// EvalKineticEnergy() to avoid unnecessary recalculation.
  ///
  /// @see EvalKineticEnergy() for more information.
  T CalcKineticEnergy(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcKineticEnergy(context);
  }

  /// Calculates and returns the conservative power represented by the current
  /// contents of the given `context`. Prefer EvalConservativePower() to avoid
  /// unnecessary recalculation.
  ///
  /// @see EvalConservativePower() for more information.
  T CalcConservativePower(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcConservativePower(context);
  }

  /// Calculates and returns the non-conservative power represented by the
  /// current contents of the given `context`. Prefer EvalNonConservativePower()
  /// to avoid unnecessary recalculation.
  ///
  /// @see EvalNonConservativePower() for more information.
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

  /// Returns @p xc if @p target_system equals `this`, nullptr otherwise.
  /// Should not be directly called.
  virtual const ContinuousState<T>* DoGetTargetSystemContinuousState(
      const System<T>& target_system,
      const ContinuousState<T>* xc) const {
    if (&target_system == this) return xc;
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
  /// @endcond

  //----------------------------------------------------------------------------
  /// @name                      Utility methods
  //@{

  /// Returns a name for this %System based on a stringification of its type
  /// name and memory address.  This is intended for use in diagnostic output
  /// and should not be used for behavioral logic, because the stringification
  /// of the type name may produce differing results across platforms and
  /// because the address can vary from run to run.
  std::string GetMemoryObjectName() const {
    return SystemImpl::GetMemoryObjectName(NiceTypeName::Get(*this),
                                           GetGraphvizId());
  }

  // So we don't have to keep writing this->num_input_ports().
  using SystemBase::num_input_ports;
  using SystemBase::num_output_ports;

  /// Returns the typed input port at index @p port_index.
  // TODO(sherm1) Make this an InputPortIndex.
  const InputPort<T>& get_input_port(int port_index) const {
    return dynamic_cast<const InputPort<T>&>(
        this->GetInputPortBaseOrThrow(__func__, port_index));
  }

  /// Returns the typed input port specified by the InputPortSelection or by
  /// the InputPortIndex.  Returns nullptr if no port is selected.  This is
  /// provided as a convenience method since many algorithms provide the same
  /// common default or optional port semantics.
  const InputPort<T>* get_input_port_selection(
      variant<InputPortSelection, InputPortIndex> port_index) const {
    if (holds_alternative<InputPortIndex>(port_index)) {
      return &get_input_port(get<InputPortIndex>(port_index));
    }

    switch (get<InputPortSelection>(port_index)) {
      case InputPortSelection::kUseFirstInputIfItExists:
        if (num_input_ports() > 0) {
          return &get_input_port(0);
        }
        return nullptr;
      case InputPortSelection::kNoInput:
        return nullptr;
    }
    return nullptr;
  }

  /// Returns the typed input port with the unique name @p port_name.
  /// The current implementation performs a linear search over strings; prefer
  /// get_input_port() when performance is a concern.
  /// @throws std::logic_error if port_name is not found.
  const InputPort<T>& GetInputPort(const std::string& port_name) const {
    for (InputPortIndex i{0}; i < num_input_ports(); i++) {
      if (port_name == get_input_port_base(i).get_name()) {
        return get_input_port(i);
      }
    }
    throw std::logic_error("System " + GetSystemName() +
                           " does not have an input port named " +
                           port_name);
  }

  /// Returns the typed output port at index @p port_index.
  // TODO(sherm1) Make this an OutputPortIndex.
  const OutputPort<T>& get_output_port(int port_index) const {
    return dynamic_cast<const OutputPort<T>&>(
        this->GetOutputPortBaseOrThrow(__func__, port_index));
  }

  /// Returns the typed output port specified by the OutputPortSelection or by
  /// the OutputPortIndex.  Returns nullptr if no port is selected. This is
  /// provided as a convenience method since many algorithms provide the same
  /// common default or optional port semantics.
  const OutputPort<T>* get_output_port_selection(
      variant<OutputPortSelection, OutputPortIndex> port_index) const {
    if (holds_alternative<OutputPortIndex>(port_index)) {
      return &get_output_port(get<OutputPortIndex>(port_index));
    }
    switch (get<OutputPortSelection>(port_index)) {
      case OutputPortSelection::kUseFirstOutputIfItExists:
        if (num_output_ports() > 0) {
          return &get_output_port(0);
        }
        return nullptr;
      case OutputPortSelection::kNoOutput:
        return nullptr;
    }
    return nullptr;
  }

  /// Returns the typed output port with the unique name @p port_name.
  /// The current implementation performs a linear search over strings; prefer
  /// get_output_port() when performance is a concern.
  /// @throws std::logic_error if port_name is not found.
  const OutputPort<T>& GetOutputPort(const std::string& port_name) const {
    for (OutputPortIndex i{0}; i < num_output_ports(); i++) {
      if (port_name == get_output_port_base(i).get_name()) {
        return get_output_port(i);
      }
    }
    throw std::logic_error("System " + GetSystemName() +
                           " does not have an output port named " +
                           port_name);
  }

  /// Returns the number of constraints specified for the system.
  int num_constraints() const {
    return static_cast<int>(constraints_.size());
  }

  /// Returns the dimension of the continuous state vector that has been
  /// declared until now.
  int num_continuous_states() const {
    return do_get_num_continuous_states();
  }

  /// Returns the constraint at index @p constraint_index.
  /// @throws std::out_of_range for an invalid constraint_index.
  const SystemConstraint<T>& get_constraint(
      SystemConstraintIndex constraint_index) const {
    if (constraint_index < 0 || constraint_index >= num_constraints()) {
      throw std::out_of_range("System " + get_name() + ": Constraint index " +
                              std::to_string(constraint_index) +
                              " is out of range. There are only " +
                              std::to_string(num_constraints()) +
                              " constraints.");
    }
    return *constraints_[constraint_index];
  }

  /// Returns true if @p context satisfies all of the registered
  /// SystemConstraints with tolerance @p tol.  @see
  /// SystemConstraint::CheckSatisfied.
  boolean<T> CheckSystemConstraintsSatisfied(
      const Context<T>& context, double tol) const {
    DRAKE_DEMAND(tol >= 0.0);
    boolean<T> result{true};
    for (const auto& constraint : constraints_) {
      result = result && constraint->CheckSatisfied(context, tol);
      // If T is a real number (not a symbolic expression), we can bail out
      // early with a diagnostic when the first constraint fails.
      if (scalar_predicate<T>::is_bool && !result) {
        SPDLOG_DEBUG(drake::log(),
                     "Context fails to satisfy SystemConstraint {}",
                     constraint->description());
        return result;
      }
    }
    return result;
  }

  /// Checks that @p output is consistent with the number and size of output
  /// ports declared by the system.
  /// @throws std::exception unless `output` is non-null and valid for this
  /// system.
  void CheckValidOutput(const SystemOutput<T>* output) const {
    DRAKE_THROW_UNLESS(output != nullptr);

    // Checks that the number of output ports in the system output is consistent
    // with the number of output ports declared by the System.
    DRAKE_THROW_UNLESS(output->num_ports() == num_output_ports());

    // Checks the validity of each output port.
    for (int i = 0; i < num_output_ports(); ++i) {
      // TODO(sherm1): consider adding (very expensive) validation of the
      // abstract ports also.
      if (get_output_port(i).get_data_type() == kVectorValued) {
        const BasicVector<T>* output_vector = output->get_vector_data(i);
        DRAKE_THROW_UNLESS(output_vector != nullptr);
        DRAKE_THROW_UNLESS(output_vector->size() == get_output_port(i).size());
      }
    }
  }

  /// Checks that @p context is consistent for this System template. Supports
  /// any scalar type, but expects T by default.
  ///
  /// @throws std::exception unless `context` is valid for this system.
  /// @tparam T1 the scalar type of the Context to check.
  // TODO(sherm1) This method needs to be unit tested.
  template <typename T1 = T>
  void CheckValidContextT(const Context<T1>& context) const {
    // Checks that the number of input ports in the context is consistent with
    // the number of ports declared by the System.
    DRAKE_THROW_UNLESS(context.num_input_ports() ==
                       this->num_input_ports());

    DRAKE_THROW_UNLESS(context.num_output_ports() ==
                       this->num_output_ports());

    // Checks that the size of the fixed vector input ports in the context
    // matches the declarations made by the system.
    for (InputPortIndex i(0); i < this->num_input_ports(); ++i) {
      const FixedInputPortValue* port_value =
          context.MaybeGetFixedInputPortValue(i);

      // If the port isn't fixed, we don't have anything else to check.
      if (port_value == nullptr) continue;
      const auto& input_port = get_input_port_base(i);
      // In the vector-valued case, check the size.
      if (input_port.get_data_type() == kVectorValued) {
        const BasicVector<T1>& input_vector =
            port_value->template get_vector_value<T1>();
        DRAKE_THROW_UNLESS(input_vector.size() == input_port.size());
      }
      // In the abstract-valued case, there is nothing else to check.
    }
  }

  /// Returns a copy of the continuous state vector `xc` into an Eigen vector.
  VectorX<T> CopyContinuousStateVector(const Context<T>& context) const {
    return context.get_continuous_state().CopyToVector();
  }
  //@}

  //----------------------------------------------------------------------------
  /// @name                      Graphviz methods
  //@{

  /// Returns a Graphviz string describing this System.  To render the string,
  /// use the Graphviz tool, ``dot``.
  /// http://www.graphviz.org/Documentation/dotguide.pdf
  ///
  /// @param max_depth Sets a limit to the depth of nested diagrams to
  // visualize.  Set to zero to render a diagram as a single system block.
  std::string GetGraphvizString(
      int max_depth = std::numeric_limits<int>::max()) const {
    DRAKE_DEMAND(max_depth >= 0);
    std::stringstream dot;
    dot << "digraph _" << this->GetGraphvizId() << " {" << std::endl;
    dot << "rankdir=LR" << std::endl;
    GetGraphvizFragment(max_depth, &dot);
    dot << "}" << std::endl;
    return dot.str();
  }

  /// Appends a Graphviz fragment to the @p dot stream.  The fragment must be
  /// valid Graphviz when wrapped in a `digraph` or `subgraph` stanza.  Does
  /// nothing by default.
  ///
  /// @param max_depth Sets a limit to the depth of nested diagrams to
  // visualize.  Set to zero to render a diagram as a single system block.
  virtual void GetGraphvizFragment(int max_depth,
                                   std::stringstream* dot) const {
    unused(dot, max_depth);
  }

  /// Appends a fragment to the @p dot stream identifying the graphviz node
  /// representing @p port. Does nothing by default.
  virtual void GetGraphvizInputPortToken(const InputPort<T>& port,
                                         int max_depth,
                                         std::stringstream* dot) const {
    unused(port, max_depth, dot);
  }

  /// Appends a fragment to the @p dot stream identifying the graphviz node
  /// representing @p port. Does nothing by default.
  virtual void GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                          int max_depth,
                                          std::stringstream* dot) const {
    unused(port, max_depth, dot);
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
  /// @throws std::exception if this System does not support autodiff
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  std::unique_ptr<System<AutoDiffXd>> ToAutoDiffXd() const {
    return System<T>::ToAutoDiffXd(*this);
  }

  /// Creates a deep copy of `from`, transmogrified to use the autodiff scalar
  /// type, with a dynamic-sized vector of partial derivatives.  The result is
  /// never nullptr.
  /// @throws std::exception if `from` does not support autodiff
  ///
  /// Usage: @code
  ///   MySystem<double> plant;
  ///   std::unique_ptr<MySystem<AutoDiffXd>> ad_plant =
  ///       systems::System<double>::ToAutoDiffXd(plant);
  /// @endcode
  ///
  /// @tparam S The specific System type to accept and return.
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  template <template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<AutoDiffXd>> ToAutoDiffXd(const S<T>& from) {
    using U = AutoDiffXd;
    std::unique_ptr<System<U>> base_result = from.ToAutoDiffXdMaybe();
    if (!base_result) {
      std::stringstream ss;
      ss << "The object named [" << from.get_name() << "] of type "
         << NiceTypeName::Get(from) << " does not support ToAutoDiffXd.";
      throw std::logic_error(ss.str().c_str());
    }

    return dynamic_pointer_cast_or_throw<S<U>>(std::move(base_result));
  }

  /// Creates a deep copy of this system exactly like ToAutoDiffXd(), but
  /// returns nullptr if this System does not support autodiff, instead of
  /// throwing an exception.
  std::unique_ptr<System<AutoDiffXd>> ToAutoDiffXdMaybe() const {
    using U = AutoDiffXd;
    auto result = system_scalar_converter_.Convert<U, T>(*this);
    if (result) {
      for (const auto& item : external_constraints_) {
        result->AddExternalConstraint(item);
      }
    }
    return result;
  }
  //@}

  //----------------------------------------------------------------------------
  /// @name                          Symbolics
  /// From a %System templatized by `double`, you can obtain an identical system
  /// templatized by a symbolic expression scalar.

  // This group appears as a top-level heading in Doxygen because it contains
  // both static and non-static member functions.
  //@{

  /// Creates a deep copy of this System, transmogrified to use the symbolic
  /// scalar type. The result is never nullptr.
  /// @throws std::exception if this System does not support symbolic
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  std::unique_ptr<System<symbolic::Expression>> ToSymbolic() const {
    return System<T>::ToSymbolic(*this);
  }

  /// Creates a deep copy of `from`, transmogrified to use the symbolic scalar
  /// type. The result is never nullptr.
  /// @throws std::exception if this System does not support symbolic
  ///
  /// Usage: @code
  ///   MySystem<double> plant;
  ///   std::unique_ptr<MySystem<symbolic::Expression>> sym_plant =
  ///       systems::System<double>::ToSymbolic(plant);
  /// @endcode
  ///
  /// @tparam S The specific System pointer type to return.
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  template <template <typename> class S = ::drake::systems::System>
  static std::unique_ptr<S<symbolic::Expression>> ToSymbolic(const S<T>& from) {
    using U = symbolic::Expression;
    std::unique_ptr<System<U>> base_result = from.ToSymbolicMaybe();
    if (!base_result) {
      std::stringstream ss;
      ss << "The object named [" << from.get_name() << "] of type "
         << NiceTypeName::Get(from) << " does not support ToSymbolic.";
      throw std::logic_error(ss.str().c_str());
    }

    return dynamic_pointer_cast_or_throw<S<U>>(std::move(base_result));
  }

  /// Creates a deep copy of this system exactly like ToSymbolic(), but returns
  /// nullptr if this System does not support symbolic, instead of throwing an
  /// exception.
  std::unique_ptr<System<symbolic::Expression>> ToSymbolicMaybe() const {
    using U = symbolic::Expression;
    auto result = system_scalar_converter_.Convert<U, T>(*this);
    if (result) {
      for (const auto& item : external_constraints_) {
        result->AddExternalConstraint(item);
      }
    }
    return result;
  }
  //@}

  //----------------------------------------------------------------------------
  /// @name                Scalar type conversion utilities
  //@{

  /// Fixes all of the input ports in @p target_context to their current values
  /// in @p other_context, as evaluated by @p other_system.
  /// @throws std::exception unless `other_context` and `target_context` both
  /// have the same shape as this System, and the `other_system`. Ignores
  /// disconnected inputs.
  void FixInputPortsFrom(const System<double>& other_system,
                         const Context<double>& other_context,
                         Context<T>* target_context) const {
    DRAKE_ASSERT_VOID(CheckValidContextT(other_context));
    DRAKE_ASSERT_VOID(CheckValidContext(*target_context));
    DRAKE_ASSERT_VOID(other_system.CheckValidContext(other_context));
    DRAKE_ASSERT_VOID(other_system.CheckValidContextT(*target_context));

    for (int i = 0; i < num_input_ports(); ++i) {
      const auto& input_port = get_input_port(i);
      const auto& other_port = other_system.get_input_port(i);
      if (!other_port.HasValue(other_context)) {
        continue;
      }

      switch (input_port.get_data_type()) {
        case kVectorValued: {
          // For vector-valued input ports, we placewise initialize a fixed
          // input vector using the explicit conversion from double to T.
          const Eigen::VectorBlock<const VectorX<double>> other_vec =
              other_port.Eval(other_context);
          auto our_vec = this->AllocateInputVector(input_port);
          for (int j = 0; j < our_vec->size(); ++j) {
            (*our_vec)[j] = T(other_vec[j]);
          }
          target_context->FixInputPort(i, *our_vec);
          continue;
        }
        case kAbstractValued: {
          // For abstract-valued input ports, we just clone the value and fix
          // it to the port.
          const auto& other_value =
              other_port.Eval<AbstractValue>(other_context);
          target_context->FixInputPort(i, other_value);
          continue;
        }
      }
      DRAKE_UNREACHABLE();
    }
  }

  /// (Advanced) Returns the SystemScalarConverter for this object.  This is an
  /// expert-level API intended for framework authors.  Most users should
  /// prefer the convenience helpers such as System::ToAutoDiffXd.
  const SystemScalarConverter& get_system_scalar_converter() const {
    return system_scalar_converter_;
  }
  //@}

  /// Gets the witness functions active for the given state.
  /// DoGetWitnessFunctions() does the actual work. The vector of active witness
  /// functions are expected to change only upon an unrestricted update.
  /// @param context a valid context for the System (aborts if not true).
  /// @param[out] w a valid pointer to an empty vector that will store
  ///             pointers to the witness functions active for the current
  ///             state. The method aborts if witnesses is null or non-empty.
  void GetWitnessFunctions(const Context<T>& context,
                           std::vector<const WitnessFunction<T>*>* w) const {
    DRAKE_DEMAND(w);
    DRAKE_DEMAND(w->empty());
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    DoGetWitnessFunctions(context, w);
  }

  /// Evaluates a witness function at the given context.
  T CalcWitnessValue(const Context<T>& context,
                     const WitnessFunction<T>& witness_func) const {
    DRAKE_ASSERT_VOID(CheckValidContext(context));
    return DoCalcWitnessValue(context, witness_func);
  }

  /// Add `event` to `events` due to a witness function triggering. `events`
  /// should be allocated with this system's AllocateCompositeEventCollection.
  /// Neither `event` nor `events` can be nullptr. Additionally, `event` must
  /// contain event data (event->get_event_data() must not be nullptr) and
  /// the type of that data must be WitnessTriggeredEventData.
  virtual void AddTriggeredWitnessFunctionToCompositeEventCollection(
      Event<T>* event,
      CompositeEventCollection<T>* events) const = 0;

  // Promote these frequently-used methods so users (and tutorial examples)
  // don't need "this->" everywhere when in templated derived classes.
  using SystemBase::DeclareCacheEntry;

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

#ifndef DRAKE_DOXYGEN_CXX
  DRAKE_DEPRECATED("2019-07-01", "Use num_continuous_states() instead.")
  int get_num_continuous_states() const { return num_continuous_states(); }
  DRAKE_DEPRECATED("2019-07-01", "Use num_constraints() instead.")
  int get_num_constraints() const { return num_constraints(); }
  DRAKE_DEPRECATED("2019-07-01", "Use num_constraint_equations() instead.")
  int get_num_constraint_equations(const Context<T>& context) const {
    return num_constraint_equations(context);
  }
#endif

 protected:
  /// Derived classes will implement this method to evaluate a witness function
  /// at the given context.
  virtual T DoCalcWitnessValue(
      const Context<T>& context,
      const WitnessFunction<T>& witness_func) const = 0;

  /// Derived classes can override this method to provide witness functions
  /// active for the given state. The default implementation does nothing. On
  /// entry to this function, the context will have already been validated and
  /// the vector of witness functions will have been validated to be both empty
  /// and non-null.
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

  virtual void DoApplyDiscreteVariableUpdate(
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state, Context<T>* context) const = 0;

  /// This function dispatches all unrestricted update events to the appropriate
  /// handlers. @p state cannot be null.
  virtual void DispatchUnrestrictedUpdateHandler(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state) const = 0;

  virtual void DoApplyUnrestrictedUpdate(
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state, Context<T>* context) const = 0;
  //@}

  //----------------------------------------------------------------------------
  /// @name                    System construction
  /// Authors of derived %Systems can use these methods in the constructor
  /// for those %Systems.
  //@{

  /// Constructs an empty %System base class object and allocates base class
  /// resources, possibly supporting scalar-type conversion support (AutoDiff,
  /// etc.) using @p converter.
  ///
  /// See @ref system_scalar_conversion for detailed background and examples
  /// related to scalar-type conversion support.
  explicit System(SystemScalarConverter converter)
      : system_scalar_converter_(std::move(converter)) {
    // Note that configuration and kinematics tickets also include dependence
    // on parameters and accuracy, but not time or input ports.

    // Potential and kinetic energy, and conservative power that measures
    // the transfer between them, must _not_ be (explicitly) time dependent.
    // See API documentation above for Eval{Potential|Kinetic}Energy() and
    // EvalConservativePower() to see why.

    // TODO(sherm1) Due to issue #9171 we cannot always recognize which
    // variables contribute to configuration so we'll invalidate on all changes.
    // Use configuration, kinematics, and mass tickets when #9171 is resolved.
    potential_energy_cache_index_ =
        DeclareCacheEntry("potential energy",
            &System<T>::CalcPotentialEnergy,
            {all_sources_ticket()})  // After #9171: configuration + mass.
            .cache_index();

    kinetic_energy_cache_index_ =
        DeclareCacheEntry("kinetic energy",
            &System<T>::CalcKineticEnergy,
            {all_sources_ticket()})  // After #9171: kinematics + mass.
            .cache_index();

    conservative_power_cache_index_ =
        DeclareCacheEntry("conservative power",
            &System<T>::CalcConservativePower,
            {all_sources_ticket()})  // After #9171: kinematics + mass.
            .cache_index();

    // Only non-conservative power can have an explicit time or input
    // port dependence. See API documentation above for
    // EvalNonConservativePower() to see why.
    nonconservative_power_cache_index_ =
        DeclareCacheEntry("non-conservative power",
                          &System<T>::CalcNonConservativePower,
                          {all_sources_ticket()})  // This is correct.
            .cache_index();

    // For the time derivative cache we need to use the general form for
    // cache creation because we're dealing with pre-defined allocator and
    // calculator method signatures.
    CacheEntry::AllocCallback alloc_derivatives = [this]() {
      return std::make_unique<Value<ContinuousState<T>>>(
          this->AllocateTimeDerivatives());
    };
    CacheEntry::CalcCallback calc_derivatives = [this](
        const ContextBase& context_base, AbstractValue* result) {
      DRAKE_DEMAND(result != nullptr);
      ContinuousState<T>& state =
          result->get_mutable_value<ContinuousState<T>>();
      const Context<T>& context = dynamic_cast<const Context<T>&>(context_base);
      CalcTimeDerivatives(context, &state);
    };

    // We must assume that time derivatives can depend on *any* context source.
    time_derivatives_cache_index_ =
        this->DeclareCacheEntryWithKnownTicket(
                xcdot_ticket(), "time derivatives",
                std::move(alloc_derivatives), std::move(calc_derivatives),
                {all_sources_ticket()})
            .cache_index();

    // TODO(sherm1) Allocate and use discrete update cache.
  }

  /// Adds a port with the specified @p type and @p size to the input topology.
  ///
  /// Input port names must be unique for this system (passing in a duplicate
  /// @p name will throw std::logic_error). If @p name is given as
  /// kUseDefaultName, then a default value of e.g. "u2", where 2
  /// is the input number will be provided. An empty @p name is not permitted.
  ///
  /// If the port is intended to model a random noise or disturbance input,
  /// @p random_type can (optionally) be used to label it as such; doing so
  /// enables algorithms for design and analysis (e.g. state estimation) to
  /// reason explicitly about randomness at the system level.  All random input
  /// ports are assumed to be statistically independent.
  /// @pre @p name must not be empty.
  /// @throws std::logic_error for a duplicate port name.
  /// @returns the declared port.
  const InputPort<T>& DeclareInputPort(
      variant<std::string, UseDefaultName> name, PortDataType type, int size,
      optional<RandomDistribution> random_type = nullopt) {
    const InputPortIndex port_index(num_input_ports());

    const DependencyTicket port_ticket(this->assign_next_dependency_ticket());
    auto eval = [this, port_index](const ContextBase& context_base) {
      return this->EvalAbstractInput(context_base, port_index);
    };
    this->AddInputPort(internal::FrameworkFactory::Make<InputPort<T>>(
        this, this, NextInputPortName(std::move(name)), port_index, port_ticket,
        type, size, random_type, std::move(eval)));
    return get_input_port(port_index);
  }

  //@}

  // =========================================================================
  /// @name             To-be-deprecated declarations
  /// Methods in this section leave out the port name parameter and are the same
  /// as invoking the corresponding method with `kUseDefaultName` as the name.
  /// We intend to make specifying the name required and will deprecate these
  /// soon. Don't use them.
  //@{

  /// See the nearly identical signature with an additional (first) argument
  /// specifying the port name.  This version will be deprecated as discussed
  /// in #9447.
  const InputPort<T>& DeclareInputPort(
      PortDataType type, int size,
      optional<RandomDistribution> random_type = nullopt) {
    return DeclareInputPort(kUseDefaultName, type, size, random_type);
  }
  //@}

  /// Adds an already-created constraint to the list of constraints for this
  /// System.  Ownership of the SystemConstraint is transferred to this system.
  SystemConstraintIndex AddConstraint(
      std::unique_ptr<SystemConstraint<T>> constraint) {
    DRAKE_DEMAND(constraint != nullptr);
    DRAKE_DEMAND(&constraint->get_system() == this);
    if (!external_constraints_.empty()) {
      throw std::logic_error(fmt::format(
          "System {} cannot add an internal constraint (named {}) "
          "after an external constraint (named {}) has already been added",
          GetSystemName(), constraint->description(),
          external_constraints_.front().description()));
    }
    constraints_.push_back(std::move(constraint));
    return SystemConstraintIndex(constraints_.size() - 1);
  }

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
  /// concrete %System to calculate their time derivatives. The `derivatives`
  /// vector will correspond elementwise with the state vector
  /// `Context.state.continuous_state.get_state()`. Thus, if the state in the
  /// Context has second-order structure `xc=[q,v,z]`, that same structure
  /// applies to the derivatives.
  ///
  /// This method is called only from the public non-virtual
  /// CalcTimeDerivatives() which will already have error-checked the parameters
  /// so you don't have to. In particular, implementations may assume that the
  /// given Context is valid for this %System; that the `derivatives` pointer is
  /// non-null, and that the referenced object has the same constituent
  /// structure as was produced by AllocateTimeDerivatives().
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
    *time = std::numeric_limits<double>::infinity();
  }

  /// Implement this method to return all periodic triggered events.
  /// @see GetPeriodicEvents() for a detailed description of the returned
  ///      variable.
  /// @note The default implementation returns an empty map.
  virtual std::map<PeriodicEventData,
      std::vector<const Event<T>*>, PeriodicEventDataComparator>
    DoGetPeriodicEvents() const = 0;

  /// Implement this method to return any events to be handled before the
  /// simulator integrates the system's continuous state at each time step.
  /// @p events is cleared in the public non-virtual GetPerStepEvents()
  /// before that method calls this function. You may assume that @p context
  /// has already been validated and that @p events is not null. @p events
  /// can be changed freely by the overriding implementation.
  ///
  /// The default implementation returns without changing @p events.
  /// @sa GetPerStepEvents()
  virtual void DoGetPerStepEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* events) const {
    unused(context, events);
  }

  /// Implement this method to return any events to be handled at the
  /// simulator's initialization step. @p events is cleared in the public
  /// non-virtual GetInitializationEvents(). You may assume that @p context has
  /// already been validated and that @p events is not null. @p events can be
  /// changed freely by the overriding implementation.
  ///
  /// The default implementation returns without changing @p events.
  /// @sa GetInitializationEvents()
  virtual void DoGetInitializationEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* events) const {
    unused(context, events);
  }

  /// Override this method for physical systems to calculate the potential
  /// energy PE currently stored in the configuration provided in the given
  /// Context. The default implementation returns 0 which is correct for
  /// non-physical systems. You may assume that `context` has already
  /// been validated before it is passed to you here.
  ///
  /// See EvalPotentialEnergy() for details on what you must compute here. In
  /// particular, your potential energy method must _not_ depend explicitly on
  /// time, velocities, or any input port values.
  virtual T DoCalcPotentialEnergy(const Context<T>& context) const {
    unused(context);
    return T(0);
  }

  /// Override this method for physical systems to calculate the kinetic
  /// energy KE currently present in the motion provided in the given
  /// Context. The default implementation returns 0 which is correct for
  /// non-physical systems. You may assume that `context` has already
  /// been validated before it is passed to you here.
  ///
  /// See EvalKineticEnergy() for details on what you must compute here. In
  /// particular, your kinetic energy method must _not_ depend explicitly on
  /// time or any input port values.
  virtual T DoCalcKineticEnergy(const Context<T>& context) const {
    unused(context);
    return T(0);
  }

  /// Override this method to return the rate Pc at which mechanical energy is
  /// being converted _from_ potential energy _to_ kinetic energy by this system
  /// in the given Context. By default, returns zero. Physical systems should
  /// override. You may assume that `context` has already been validated before
  /// it is passed to you here.
  ///
  /// See EvalConservativePower() for details on what you must compute here. In
  /// particular, this quantity must be _positive_ when potential energy
  /// is _decreasing_, and your conservative power method must _not_ depend
  /// explicitly on time or any input port values.
  virtual T DoCalcConservativePower(const Context<T>& context) const {
    unused(context);
    return T(0);
  }

  /// Override this method to return the rate Pnc at which work W is done on the
  /// system by non-conservative forces. By default, returns zero. Physical
  /// systems should override. You may assume that `context` has already been
  /// validated before it is passed to you here.
  ///
  /// See EvalNonConservativePower() for details on what you must compute here.
  /// In particular, this quantity must be _negative_ if the non-conservative
  /// forces are _dissipative_, positive otherwise. Your non-conservative power
  /// method can depend on anything you find in the given Context, including
  /// time and input ports.
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
  //@}

  //----------------------------------------------------------------------------
  /// @name             Constraint-related functions (protected).
  //@{

  /// Totals the number of continuous state variables in this System or Diagram.
  virtual int do_get_num_continuous_states() const = 0;

  /// Gets the number of constraint equations for this system from the given
  /// context. The context is supplied in case the number of constraints is
  /// dependent upon the current state (as might be the case with a piecewise
  /// differential algebraic equation). Derived classes can override this
  /// function, which is called by num_constraint_equations().
  /// @sa num_constraint_equations() for parameter documentation.
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
  /// @returns a vector of dimension num_constraint_equations(); the
  ///          zero vector indicates that the algebraic constraints are all
  ///          satisfied.
  virtual Eigen::VectorXd DoEvalConstraintEquations(
      const Context<T>& context) const {
    DRAKE_DEMAND(num_constraint_equations(context) == 0);
    return Eigen::VectorXd();
  }

  /// Computes the time derivative of each constraint equation, evaluated at
  /// the generalized coordinates and generalized velocity specified by the
  /// context.  The context allows the set of constraints to be dependent upon
  /// the current system state (as might be the case with a piecewise
  /// differential algebraic equation). The default implementation of this
  /// function returns a zero-dimensional vector. Derived classes can override
  /// this function, which is called by EvalConstraintEquationsDot().
  /// @returns a vector of dimension num_constraint_equations().
  /// @sa EvalConstraintEquationsDot() for parameter documentation.
  virtual Eigen::VectorXd DoEvalConstraintEquationsDot(
      const Context<T>& context) const {
    DRAKE_DEMAND(num_constraint_equations(context) == 0);
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
    DRAKE_DEMAND(num_constraint_equations(context) == 0);
    const auto& gv = context.get_continuous_state().get_generalized_velocity();
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
  //@}

  //----------------------------------------------------------------------------
  /// @name                 Utility methods (protected)
  //@{

  /// Returns a mutable Eigen expression for a vector valued output port with
  /// index @p port_index in this system. All input ports that directly depend
  /// on this output port will be notified that upstream data has changed, and
  /// may invalidate cache entries as a result.
  Eigen::VectorBlock<VectorX<T>> GetMutableOutputVector(SystemOutput<T>* output,
                                                        int port_index) const {
    DRAKE_ASSERT(0 <= port_index && port_index < num_output_ports());

    BasicVector<T>* output_vector = output->GetMutableVectorData(port_index);
    DRAKE_ASSERT(output_vector != nullptr);
    DRAKE_ASSERT(output_vector->size() == get_output_port(port_index).size());

    return output_vector->get_mutable_value();
  }
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
    return *forced_publish_events_;
  }

  EventCollection<DiscreteUpdateEvent<T>>&
  get_mutable_forced_discrete_update_events() {
    return *forced_discrete_update_events_;
  }

  const EventCollection<PublishEvent<T>>&
  get_forced_publish_events() const {
    DRAKE_DEMAND(forced_publish_events_.get());
    return *forced_publish_events_;
  }

  const EventCollection<DiscreteUpdateEvent<T>>&
  get_forced_discrete_update_events() const {
    DRAKE_DEMAND(forced_discrete_update_events_.get());
    return *forced_discrete_update_events_;
  }

  const EventCollection<UnrestrictedUpdateEvent<T>>&
  get_forced_unrestricted_update_events() const {
    DRAKE_DEMAND(forced_unrestricted_update_events_.get());
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

 private:
  // Attorney-Client idiom to expose a subset of private elements of System.
  // Refer to SystemImpl comments for details.
  friend class SystemImpl;

  // Allocates an input of the leaf type that the System requires on the port
  // specified by @p input_port.  This is final in LeafSystem and Diagram.
  virtual std::unique_ptr<AbstractValue> DoAllocateInput(
      const InputPort<T>& input_port) const = 0;

  // SystemBase override checks a Context of same type T.
  void DoCheckValidContext(const ContextBase& context_base) const final {
    const Context<T>* context = dynamic_cast<const Context<T>*>(&context_base);
    DRAKE_THROW_UNLESS(context != nullptr);
    CheckValidContextT(*context);
  }

  std::function<void(const AbstractValue&)> MakeFixInputPortTypeChecker(
      InputPortIndex port_index) const final {
    const InputPort<T>& port = this->get_input_port(port_index);
    const std::string& port_name = port.get_name();
    const std::string path_name = this->GetSystemPathname();

    // Note that our lambdas below will capture all necessary items by-value,
    // so that they do not rely on this System still being alive.  (We do not
    // allow a Context and System to have pointers to each other.)
    switch (port.get_data_type()) {
      case kAbstractValued: {
        // For abstract inputs, we only need to ensure that both runtime values
        // share the same base T in the Value<T>. Even if the System declared a
        // model_value that was a subtype of T, there is no EvalInputValue
        // sugar that allows the System to evaluate the input by downcasting to
        // that subtype, so here we should not insist that some dynamic_cast
        // would succeed. If the user writes the downcast on their own, it's
        // fine to let them also handle detailed error reporting on their own.
        const std::type_info& expected_type =
            this->AllocateInputAbstract(port)->static_type_info();
        return [&expected_type, port_index, path_name, port_name](
            const AbstractValue& actual) {
          if (actual.static_type_info() != expected_type) {
            SystemBase::ThrowInputPortHasWrongType(
                "FixInputPortTypeCheck", path_name, port_index, port_name,
                NiceTypeName::Get(expected_type),
                NiceTypeName::Get(actual.type_info()));
          }
        };
      }
      case kVectorValued: {
        // For vector inputs, check that the size is the same.
        // TODO(jwnimmer-tri) We should type-check the vector, eventually.
        const std::unique_ptr<BasicVector<T>> model_vector =
            this->AllocateInputVector(port);
        const int expected_size = model_vector->size();
        return [expected_size, port_index, path_name, port_name](
            const AbstractValue& actual) {
          const BasicVector<T>* const actual_vector =
              actual.maybe_get_value<BasicVector<T>>();
          if (actual_vector == nullptr) {
            SystemBase::ThrowInputPortHasWrongType(
                "FixInputPortTypeCheck", path_name, port_index, port_name,
                NiceTypeName::Get<Value<BasicVector<T>>>(),
                NiceTypeName::Get(actual));
          }
          // Check that vector sizes match.
          if (actual_vector->size() != expected_size) {
            SystemBase::ThrowInputPortHasWrongType(
                "FixInputPortTypeCheck", path_name, port_index, port_name,
                fmt::format("{} with size={}",
                            NiceTypeName::Get<BasicVector<T>>(),
                            expected_size),
                fmt::format("{} with size={}",
                            NiceTypeName::Get(*actual_vector),
                            actual_vector->size()));
          }
        };
      }
    }
    DRAKE_UNREACHABLE();
  }

  // Shared code for updating a vector input port and returning a pointer to its
  // value as a BasicVector<T>, or nullptr if the port is not connected. Throws
  // a logic_error if the port_index is out of range or if the input port is not
  // declared to be a vector-valued port. `func` should be the user-visible API
  // function name obtained with __func__.
  const BasicVector<T>* EvalBasicVectorInputImpl(
      const char* func, const Context<T>& context,
      InputPortIndex port_index) const {
    // Make sure this is the right kind of port before worrying about whether
    // it is connected up properly.
    const InputPortBase& port = GetInputPortBaseOrThrow(func, port_index);
    if (port.get_data_type() != kVectorValued)
      ThrowNotAVectorInputPort(func, port_index);

    // If there is no value at all, the port is not connected which is not
    // a problem here.
    const AbstractValue* const abstract_value =
        EvalAbstractInputImpl(func, context, port_index);
    if (abstract_value == nullptr) {
      return nullptr;
    }

    // We have a vector port with a value, it better be a BasicVector!
    const auto* basic_vector = &abstract_value->get_value<BasicVector<T>>();

    // Shouldn't have been possible to create this vector-valued port with
    // the wrong size.
    DRAKE_DEMAND(basic_vector->size() == port.size());

    return basic_vector;
  }

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

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T>
System<T>::~System() = default;

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::System)
