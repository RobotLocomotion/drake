#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/common/value.h"
#include "drake/systems/framework/abstract_value_cloner.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_output_port.h"
#include "drake/systems/framework/model_values.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_constraint.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/system_scalar_converter.h"
#include "drake/systems/framework/value_producer.h"

namespace drake {
namespace systems {

/** A superclass template that extends System with some convenience utilities
that are not applicable to Diagrams.

@tparam_default_scalar */
template <typename T>
class LeafSystem : public System<T> {
 public:
  // LeafSystem objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafSystem)

  ~LeafSystem() override;

  /** Shadows System<T>::AllocateContext to provide a more concrete return
  type LeafContext<T>. */
  std::unique_ptr<LeafContext<T>> AllocateContext() const;

  // =========================================================================
  // Implementations of System<T> methods.

#ifndef DRAKE_DOXYGEN_CXX
  // The three methods below are hidden from doxygen, as described in
  // documentation for their corresponding methods in System.
  std::unique_ptr<EventCollection<PublishEvent<T>>>
  AllocateForcedPublishEventCollection() const override;

  std::unique_ptr<EventCollection<DiscreteUpdateEvent<T>>>
  AllocateForcedDiscreteUpdateEventCollection() const override;

  std::unique_ptr<EventCollection<UnrestrictedUpdateEvent<T>>>
  AllocateForcedUnrestrictedUpdateEventCollection() const override;
#endif

  std::unique_ptr<ContextBase> DoAllocateContext() const final;

  /** Default implementation: sets all continuous state to the model vector
  given in DeclareContinuousState (or zero if no model vector was given) and
  discrete states to zero. Overrides must not change the number of state
  variables. */
  void SetDefaultState(const Context<T>& context,
                       State<T>* state) const override;

  /** Default implementation: sets all numeric parameters to the model vector
  given to DeclareNumericParameter, or else if no model was provided sets
  the numeric parameter to one.  It sets all abstract parameters to the
  model value given to DeclareAbstractParameter.  Overrides must not change
  the number of parameters. */
  void SetDefaultParameters(const Context<T>& context,
                            Parameters<T>* parameters) const override;

  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const final;

  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteVariables() const final;

  std::multimap<int, int> GetDirectFeedthroughs() const final;

 protected:
  // Promote so we don't need "this->" in defaults which show up in Doxygen.
  using SystemBase::all_sources_ticket;

  /** Default constructor that declares no inputs, outputs, state, parameters,
  events, nor scalar-type conversion support (AutoDiff, etc.).  To enable
  AutoDiff support, use the SystemScalarConverter-based constructor. */
  LeafSystem();

  /** Constructor that declares no inputs, outputs, state, parameters, or
  events, but allows subclasses to declare scalar-type conversion support
  (AutoDiff, etc.).

  The scalar-type conversion support will use @p converter.
  To enable scalar-type conversion support, pass a `SystemTypeTag<S>{}`
  where `S` must be the exact class of `this` being constructed.

  See @ref system_scalar_conversion for detailed background and examples
  related to scalar-type conversion support. */
  explicit LeafSystem(SystemScalarConverter converter);

  /** Provides a new instance of the leaf context for this system. Derived
  leaf systems with custom derived leaf system contexts should override this
  to provide a context of the appropriate type. The returned context should
  be "empty"; invoked by AllocateContext(), the caller will take the
  responsibility to initialize the core LeafContext data. The default
  implementation provides a default-constructed `LeafContext<T>`. */
  virtual std::unique_ptr<LeafContext<T>> DoMakeLeafContext() const;

  /** Derived classes that impose restrictions on what resources are permitted
  should check those restrictions by implementing this. For example, a
  derived class might require a single input and single output. Note that
  the supplied Context will be complete except that input and output
  dependencies on peer and parent subcontexts will not yet have been set up,
  so you may not consider them for validation.
  The default implementation does nothing. */
  virtual void DoValidateAllocatedLeafContext(
      const LeafContext<T>& context) const {
    unused(context);
  }

  // =========================================================================
  // Implementations of System<T> methods.

  T DoCalcWitnessValue(const Context<T>& context,
                       const WitnessFunction<T>& witness_func) const final;

  void AddTriggeredWitnessFunctionToCompositeEventCollection(
      Event<T>* event,
      CompositeEventCollection<T>* events) const final;

  /** Computes the next update time based on the configured periodic events, for
  scalar types that are arithmetic, or aborts for scalar types that are not
  arithmetic. Subclasses that require aperiodic events should override, but
  be sure to invoke the parent class implementation at the start of the
  override if you want periodic events to continue to be handled.

  @post `time` is set to a value greater than or equal to
        `context.get_time()` on return.
  @warning If you override this method, think carefully before setting
           `time` to `context.get_time()` on return, which can inadvertently
           cause simulations of systems derived from %LeafSystem to loop
           interminably. Such a loop will occur if, for example, the
           event(s) does not modify the state. */
  void DoCalcNextUpdateTime(const Context<T>& context,
                            CompositeEventCollection<T>* events,
                            T* time) const override;

  /** Emits a graphviz fragment for this System. Leaf systems are visualized as
  records. For instance, a leaf system with 2 inputs and 1 output is:

  @verbatim
  123456 [shape= record, label="name | {<u0> 0 |<y0> 0} | {<u1> 1 | }"];
  @endverbatim

  which looks like:

  @verbatim
  +------------+----+
  | name  | u0 | u1 |
  |       | y0 |    |
  +-------+----+----+
  @endverbatim */
  void GetGraphvizFragment(int max_depth,
                           std::stringstream* dot) const override;

  void GetGraphvizInputPortToken(const InputPort<T>& port,
                                 int max_depth,
                                 std::stringstream *dot) const final;

  void GetGraphvizOutputPortToken(const OutputPort<T>& port,
                                  int max_depth,
                                  std::stringstream *dot) const final;

  // =========================================================================
  // Allocation helper utilities.

  /** Returns a copy of the state declared in the most recent
  DeclareContinuousState() call, or else a zero-sized state if that method
  has never been called. */
  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const;

  /** Returns a copy of the states declared in DeclareDiscreteState() calls. */
  std::unique_ptr<DiscreteValues<T>> AllocateDiscreteState() const;

  /** Returns a copy of the states declared in DeclareAbstractState() calls. */
  std::unique_ptr<AbstractValues> AllocateAbstractState() const;

  /** Returns a copy of the parameters declared in DeclareNumericParameter()
  and DeclareAbstractParameter() calls. */
  std::unique_ptr<Parameters<T>> AllocateParameters() const;

  // =========================================================================
  // New methods for subclasses to use

  /** Declares a numeric parameter using the given @p model_vector.
  LeafSystem's default implementation of SetDefaultParameters() will reset
  parameters to their model vectors.  If the @p model_vector declares any
  VectorBase::GetElementBounds() constraints, they will be re-declared as
  inequality constraints on this system (see
  DeclareInequalityConstraint()).  Returns the index of the new parameter. */
  int DeclareNumericParameter(const BasicVector<T>& model_vector);

  /** Extracts the numeric parameters of type U from the @p context at @p index.
  Asserts if the context is not a LeafContext, or if it does not have a
  vector-valued parameter of type U at @p index. */
  template <template <typename> class U = BasicVector>
  const U<T>& GetNumericParameter(const Context<T>& context, int index) const {
    this->ValidateContext(context);
    static_assert(std::is_base_of_v<BasicVector<T>, U<T>>,
                  "U must be a subclass of BasicVector.");
    const auto& leaf_context =
        dynamic_cast<const systems::LeafContext<T>&>(context);
    const auto* const params =
        dynamic_cast<const U<T>*>(&leaf_context.get_numeric_parameter(index));
    DRAKE_ASSERT(params != nullptr);
    return *params;
  }

  /** Extracts the numeric parameters of type U from the @p context at @p index.
  Asserts if the context is not a LeafContext, or if it does not have a
  vector-valued parameter of type U at @p index. */
  template <template <typename> class U = BasicVector>
  U<T>& GetMutableNumericParameter(Context<T>* context, int index) const {
    this->ValidateContext(context);
    static_assert(std::is_base_of_v<BasicVector<T>, U<T>>,
                  "U must be a subclass of BasicVector.");
    auto* leaf_context = dynamic_cast<systems::LeafContext<T>*>(context);
    DRAKE_ASSERT(leaf_context != nullptr);
    auto* const params = dynamic_cast<U<T>*>(
        &leaf_context->get_mutable_numeric_parameter(index));
    DRAKE_ASSERT(params != nullptr);
    return *params;
  }

  /** Declares an abstract parameter using the given @p model_value.
  LeafSystem's default implementation of SetDefaultParameters() will reset
  parameters to their model values.  Returns the index of the new
  parameter. */
  int DeclareAbstractParameter(const AbstractValue& model_value);

  // =========================================================================
  /** @anchor declare_periodic_events
  @name                  Declare periodic events
  Methods in this group declare that this System has an event that
  is triggered periodically. The first periodic trigger will occur at
  t = `offset_sec`, and it will recur at every `period_sec` thereafter.
  Several signatures are provided to allow for a general Event object to be
  triggered or for simpler class member functions to be invoked instead.

  Reaching a designated time causes a periodic event to be dispatched
  to one of the three available types of event dispatcher: publish (read
  only), discrete update, and unrestricted update.

  @note If you want to handle timed events that are _not_ periodic
  (timers, alarms, etc.), overload DoCalcNextUpdateTime() rather than using
  the methods in this section.

  Template arguments to these methods are inferred from the argument lists
  and need not be specified explicitly.
  @pre `period_sec` > 0 and `offset_sec` ≥ 0. */
  //@{

  /** Declares that a Publish event should occur periodically and that it should
  invoke the given event handler method. The handler should be a class
  member function (method) with this signature:
  @code
    EventStatus MySystem::MyPublish(const Context<T>&) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_periodic_events "Declare periodic events" for more
  information.

  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `publish` must not be null.

  @see DeclarePeriodicDiscreteUpdateEvent()
  @see DeclarePeriodicUnrestrictedUpdateEvent()
  @see DeclarePeriodicEvent() */
  template <class MySystem>
  void DeclarePeriodicPublishEvent(
      double period_sec, double offset_sec,
      EventStatus (MySystem::*publish)(const Context<T>&) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    DRAKE_DEMAND(publish != nullptr);

    DeclarePeriodicEvent(
        period_sec, offset_sec,
        PublishEvent<T>(TriggerType::kPeriodic, [publish](
                            const System<T>& system,
                            const Context<T>& context,
                            const PublishEvent<T>&) {
          const auto& sys = dynamic_cast<const MySystem&>(system);
          // TODO(sherm1) Forward the return status.
          (sys.*publish)(context);  // Ignore return status for now.
        }));
  }

  /** This variant accepts a handler that is assumed to succeed rather than
  one that returns an EventStatus result. The handler signature is:
  @code
    void MySystem::MyPublish(const Context<T>&) const;
  @endcode
  See the other signature for more information.
  @exclude_from_pydrake_mkdoc{This overload is not bound.} */
  template <class MySystem>
  void DeclarePeriodicPublishEvent(double period_sec, double offset_sec,
                                   void (MySystem::*publish)(const Context<T>&)
                                       const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    DRAKE_DEMAND(publish != nullptr);

    DeclarePeriodicEvent(
        period_sec, offset_sec,
        PublishEvent<T>(
            TriggerType::kPeriodic,
            [publish](const System<T>& system,
                      const Context<T>& context,
                      const PublishEvent<T>&) {
              const auto& sys = dynamic_cast<const MySystem&>(system);
              (sys.*publish)(context);
              // TODO(sherm1) return EventStatus::Succeeded()
            }));
  }

  /** Declares that a DiscreteUpdate event should occur periodically and that it
  should invoke the given event handler method. The handler should be a
  class member function (method) with this signature:
  @code
    EventStatus MySystem::MyUpdate(const Context<T>&,
                                   DiscreteValues<T>*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_periodic_events "Declare periodic events" for more
  information.

  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `update` must not be null.

  @see DeclarePeriodicPublishEvent()
  @see DeclarePeriodicUnrestrictedUpdateEvent()
  @see DeclarePeriodicEvent() */
  template <class MySystem>
  void DeclarePeriodicDiscreteUpdateEvent(
      double period_sec, double offset_sec,
      EventStatus (MySystem::*update)(const Context<T>&, DiscreteValues<T>*)
          const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    DRAKE_DEMAND(update != nullptr);

    DeclarePeriodicEvent(
        period_sec, offset_sec,
        DiscreteUpdateEvent<T>(
            TriggerType::kPeriodic,
            [update](const System<T>& system,
                     const Context<T>& context,
                     const DiscreteUpdateEvent<T>&,
                     DiscreteValues<T>* xd) {
              const auto& sys = dynamic_cast<const MySystem&>(system);
              // TODO(sherm1) Forward the return status.
              (sys.*update)(context, &*xd);  // Ignore return status for now.
            }));
  }

  /** This variant accepts a handler that is assumed to succeed rather than
  one that returns an EventStatus result. The handler signature is:
  @code
    void MySystem::MyUpdate(const Context<T>&,
                            DiscreteValues<T>*) const;
  @endcode
  See the other signature for more information.
  @exclude_from_pydrake_mkdoc{This overload is not bound.} */
  template <class MySystem>
  void DeclarePeriodicDiscreteUpdateEvent(
      double period_sec, double offset_sec,
      void (MySystem::*update)(const Context<T>&, DiscreteValues<T>*) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    DRAKE_DEMAND(update != nullptr);

    DeclarePeriodicEvent(
        period_sec, offset_sec,
        DiscreteUpdateEvent<T>(
            TriggerType::kPeriodic,
            [update](const System<T>& system,
                     const Context<T>& context,
                     const DiscreteUpdateEvent<T>&,
                     DiscreteValues<T>* xd) {
              const auto& sys = dynamic_cast<const MySystem&>(system);
              (sys.*update)(context, &*xd);
              // TODO(sherm1) return EventStatus::Succeeded()
            }));
  }

  /** Declares that an UnrestrictedUpdate event should occur periodically and
  that it should invoke the given event handler method. The handler should
  be a class member function (method) with this signature:
  @code
    EventStatus MySystem::MyUpdate(const Context<T>&, State<T>*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_periodic_events "Declare periodic events" for more
  information.

  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `update` must not be null.

  @see DeclarePeriodicPublishEvent()
  @see DeclarePeriodicDiscreteUpdateEvent()
  @see DeclarePeriodicEvent() */
  template <class MySystem>
  void DeclarePeriodicUnrestrictedUpdateEvent(
      double period_sec, double offset_sec,
      EventStatus (MySystem::*update)(const Context<T>&, State<T>*) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    DRAKE_DEMAND(update != nullptr);

    DeclarePeriodicEvent(
        period_sec, offset_sec,
        UnrestrictedUpdateEvent<T>(
            TriggerType::kPeriodic,
            [update](const System<T>& system,
                     const Context<T>& context,
                     const UnrestrictedUpdateEvent<T>&, State<T>* x) {
              const auto& sys = dynamic_cast<const MySystem&>(system);
              // TODO(sherm1) Forward the return status.
              (sys.*update)(context, &*x);  // Ignore return status for now.
            }));
  }

  /** This variant accepts a handler that is assumed to succeed rather than
  one that returns an EventStatus result. The handler signature is:
  @code
    void MySystem::MyUpdate(const Context<T>&, State<T>*) const;
  @endcode
  See the other signature for more information.
  @exclude_from_pydrake_mkdoc{This overload is not bound.} */
  template <class MySystem>
  void DeclarePeriodicUnrestrictedUpdateEvent(
      double period_sec, double offset_sec,
      void (MySystem::*update)(const Context<T>&, State<T>*) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    DRAKE_DEMAND(update != nullptr);

    DeclarePeriodicEvent(
        period_sec, offset_sec,
        UnrestrictedUpdateEvent<T>(
            TriggerType::kPeriodic,
            [update](const System<T>& system, const Context<T>& context,
                     const UnrestrictedUpdateEvent<T>&, State<T>* x) {
              const auto& sys = dynamic_cast<const MySystem&>(system);
              (sys.*update)(context, &*x);
              // TODO(sherm1) return EventStatus::Succeeded()
            }));
  }

  /** (Advanced) Declares that a particular Event object should be dispatched
  periodically. This is the most general form for declaring periodic events
  and most users should use one of the other methods in this group instead.

  @see DeclarePeriodicPublishEvent()
  @see DeclarePeriodicDiscreteUpdateEvent()
  @see DeclarePeriodicUnrestrictedUpdateEvent()

  See @ref declare_periodic_events "Declare periodic events" for more
  information.

  Depending on the type of `event`, when triggered it will be passed to
  the Publish, DiscreteUpdate, or UnrestrictedUpdate event dispatcher. If
  the `event` object contains a handler function, Drake's default
  dispatchers will invoke that handler. If not, then no further action is
  taken. Thus an `event` with no handler has no effect unless its dispatcher
  has been overridden. We strongly recommend that you _do not_ override the
  dispatcher and instead _do_ supply a handler.

  The given `event` object is deep-copied (cloned), and the copy is stored
  internally so you do not need to keep the object around after this call.

  @pre `event`'s associated trigger type must be TriggerType::kUnknown or
  already set to TriggerType::kPeriodic. */
  template <typename EventType>
  void DeclarePeriodicEvent(double period_sec, double offset_sec,
                            const EventType& event) {
    DRAKE_DEMAND(event.get_trigger_type() == TriggerType::kUnknown ||
                 event.get_trigger_type() == TriggerType::kPeriodic);
    PeriodicEventData periodic_data;
    periodic_data.set_period_sec(period_sec);
    periodic_data.set_offset_sec(offset_sec);
    auto event_copy = event.Clone();
    event_copy->set_trigger_type(TriggerType::kPeriodic);
    event_copy->set_event_data(periodic_data);
    event_copy->AddToComposite(TriggerType::kPeriodic, &periodic_events_);
  }

  /** (Advanced) Declares a periodic publish event with no handler function.
  When triggered, the event will invoke the DoPublish() dispatcher, but no
  other processing will occur unless you have overridden the dispatcher (not
  recommended). Otherwise the only visible effect will be that a Simulator step
  will end exactly at the publish time.

  Prefer DeclarePeriodicPublishEvent() where you can supply a handler. */
  void DeclarePeriodicPublishNoHandler(double period_sec,
                                       double offset_sec = 0);

  /** (Advanced) Declares a periodic discrete update event with no handler
  function. When triggered, the event will invoke the
  DoCalcDiscreteVariableUpdates() dispatcher, but no other processing will occur
  unless you have overridden the dispatcher (not recommended). Otherwise the
  only visible effect will be that a Simulator step will end exactly at the
  publish time.

  Prefer DeclarePeriodicDiscreteUpdateEvent() where you can supply a handler. */
  void DeclarePeriodicDiscreteUpdateNoHandler(double period_sec,
                                              double offset_sec = 0);

  /** (Advanced) Declares a periodic unrestricted update event with no handler
  function. When triggered, the event will invoke the
  DoCalcUnrestrictedUpdate() dispatcher, but no other processing will occur
  unless you have overridden the dispatcher (not recommended). Otherwise the
  only visible effect will be that a Simulator step will end exactly at the
  publish time.

  Prefer DeclarePeriodicUnrestrictedUpdateEvent() where you can supply a
  handler. */
  void DeclarePeriodicUnrestrictedUpdateNoHandler(double period_sec,
                                                  double offset_sec = 0);

  DRAKE_DEPRECATED("2023-03-01",
                   "Use DeclarePeriodicPublishNoHandler() instead")
  void DeclarePeriodicPublish(double period_sec, double offset_sec = 0) {
    DeclarePeriodicPublishNoHandler(period_sec, offset_sec);
  }

  DRAKE_DEPRECATED("2023-03-01",
                   "Use DeclarePeriodicDiscreteUpdateNoHandler() instead")
  void DeclarePeriodicDiscreteUpdate(double period_sec, double offset_sec = 0) {
    DeclarePeriodicDiscreteUpdateNoHandler(period_sec, offset_sec);
  }

  DRAKE_DEPRECATED("2023-03-01",
                   "Use DeclarePeriodicUnrestrictedUpdateNoHandler() instead")
  void DeclarePeriodicUnrestrictedUpdate(double period_sec,
                                         double offset_sec = 0) {
    DeclarePeriodicUnrestrictedUpdateNoHandler(period_sec, offset_sec);
  }

  //@}

  // =========================================================================
  /** @anchor declare_per-step_events
  @name                 Declare per-step events
  These methods are used to declare events that are triggered whenever the
  Drake Simulator advances the simulated trajectory. Note that each call to
  Simulator::AdvanceTo() typically generates many trajectory-advancing
  steps of varying time intervals; per-step events are triggered for each
  of those steps.

  Per-step events are useful for taking discrete action at every point of a
  simulated trajectory (generally spaced irregularly in time) without
  missing anything. For example, per-step events can be used to implement
  a high-accuracy signal delay by maintaining a buffer of past signal
  values, updated at each step. Because the steps are smaller in regions
  of rapid change, the interpolated signal retains the accuracy provided
  by the denser sampling. A periodic sampling would produce less-accurate
  interpolations.

  As with any Drake event trigger type, a per-step event is dispatched to
  one of the three available types of event dispatcher: publish (read only),
  discrete state update, and unrestricted state update. Several signatures
  are provided below to allow for a general Event object to be triggered, or
  simpler class member functions to be invoked instead.

  Per-step events are issued as follows: First, the Simulator::Initialize()
  method queries and records the set of declared per-step events. That set
  does not change during a simulation. Any per-step publish events are
  dispatched at the end of Initialize() to publish the initial value of the
  trajectory. Then every AdvanceTo() internal step dispatches unrestricted
  and discrete update events at the start of the step, and dispatches
  publish events at the end of the step (that is, after time advances).
  This means that a per-step event at fixed step size h behaves
  identically to a periodic event of period h, offset 0.

  Template arguments to these methods are inferred from the argument lists
  and need not be specified explicitly. */
  //@{

  /** Declares that a Publish event should occur at initialization and at the
  end of every trajectory-advancing step and that it should invoke the
  given event handler method. The handler should be a class member function
  (method) with this signature:
  @code
    EventStatus MySystem::MyPublish(const Context<T>&) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  @warning These per-step publish events are independent of the Simulator's
  optional "publish every time step" and "publish at initialization"
  features. Generally if you are declaring per-step publish events yourself
  you should turn off those Simulation options.

  See @ref declare_per-step_events "Declare per-step events" for more
  information.

  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `publish` must not be null.

  @see DeclarePerStepDiscreteUpdateEvent()
  @see DeclarePerStepUnrestrictedUpdateEvent()
  @see DeclarePerStepEvent()
  @see Simulator::set_publish_at_initialization()
  @see Simulator::set_publish_every_time_step() */
  template <class MySystem>
  void DeclarePerStepPublishEvent(
      EventStatus (MySystem::*publish)(const Context<T>&) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    DRAKE_DEMAND(publish != nullptr);

    DeclarePerStepEvent<PublishEvent<T>>(PublishEvent<T>(
        TriggerType::kPerStep,
        [publish](const System<T>& system, const Context<T>& context,
                  const PublishEvent<T>&) {
          const auto& sys = dynamic_cast<const MySystem&>(system);
          // TODO(sherm1) Forward the return status.
          (sys.*publish)(context);  // Ignore return status for now.
        }));
  }

  /** Declares that a DiscreteUpdate event should occur at the start of every
  trajectory-advancing step and that it should invoke the given event
  handler method. The handler should be a class member function (method)
  with this signature:
  @code
    EventStatus MySystem::MyUpdate(const Context<T>&,
                                   DiscreteValues<T>*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_per-step_events "Declare per-step events" for more
  information.

  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `update` must not be null.

  @see DeclarePerStepPublishEvent()
  @see DeclarePerStepUnrestrictedUpdateEvent()
  @see DeclarePerStepEvent() */
  template <class MySystem>
  void DeclarePerStepDiscreteUpdateEvent(EventStatus (MySystem::*update)(
      const Context<T>&, DiscreteValues<T>*) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    DRAKE_DEMAND(update != nullptr);

    DeclarePerStepEvent(
        DiscreteUpdateEvent<T>(
            TriggerType::kPerStep,
            [update](const System<T>& system, const Context<T>& context,
                     const DiscreteUpdateEvent<T>&, DiscreteValues<T>* xd) {
              const auto& sys = dynamic_cast<const MySystem&>(system);
              // TODO(sherm1) Forward the return status.
              (sys.*update)(context, &*xd);  // Ignore return status for now.
            }));
  }

  /** Declares that an UnrestrictedUpdate event should occur at the start of
  every trajectory-advancing step and that it should invoke the given
  event handler method. The handler should be a class member function
  (method) with this signature:
  @code
    EventStatus MySystem::MyUpdate(const Context<T>&,
                                   State<T>*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_per-step_events "Declare per-step events" for more
  information.

  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `update` must not be null.

  @see DeclarePerStepPublishEvent()
  @see DeclarePerStepDiscreteUpdateEvent()
  @see DeclarePerStepEvent() */
  template <class MySystem>
  void DeclarePerStepUnrestrictedUpdateEvent(
      EventStatus (MySystem::*update)(const Context<T>&, State<T>*) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    DRAKE_DEMAND(update != nullptr);

    DeclarePerStepEvent(UnrestrictedUpdateEvent<T>(
        TriggerType::kPerStep,
        [update](const System<T>& system, const Context<T>& context,
                 const UnrestrictedUpdateEvent<T>&, State<T>* x) {
          const auto& sys = dynamic_cast<const MySystem&>(system);
          // TODO(sherm1) Forward the return status.
          (sys.*update)(context, &*x);  // Ignore return status for now.
        }));
  }

  /** (Advanced) Declares that a particular Event object should be dispatched at
  every trajectory-advancing step. Publish events are dispatched at
  the end of initialization and at the end of each step. Discrete- and
  unrestricted update events are dispatched at the start of each step.
  This is the most general form for declaring per-step events and most users
  should use one of the other methods in this group instead.

  @see DeclarePerStepPublishEvent()
  @see DeclarePerStepDiscreteUpdateEvent()
  @see DeclarePerStepUnrestrictedUpdateEvent()

  See @ref declare_per-step_events "Declare per-step events" for more
  information.

  Depending on the type of `event`, at each step it will be passed to
  the Publish, DiscreteUpdate, or UnrestrictedUpdate event dispatcher. If
  the `event` object contains a handler function, Drake's default
  dispatchers will invoke that handler. If not, then no further action is
  taken. Thus an `event` with no handler has no effect unless its dispatcher
  has been overridden. We strongly recommend that you _do not_ override the
  dispatcher and instead _do_ supply a handler.

  The given `event` object is deep-copied (cloned), and the copy is stored
  internally so you do not need to keep the object around after this call.

  @pre `event`'s associated trigger type must be TriggerType::kUnknown or
  already set to TriggerType::kPerStep. */
  template <typename EventType>
  void DeclarePerStepEvent(const EventType& event) {
    DRAKE_DEMAND(event.get_trigger_type() == TriggerType::kUnknown ||
        event.get_trigger_type() == TriggerType::kPerStep);
    event.AddToComposite(TriggerType::kPerStep, &per_step_events_);
  }
  //@}

  // =========================================================================
  /** @anchor declare_initialization_events
  @name                 Declare initialization events
  These methods are used to declare events that occur when the Drake
  Simulator::Initialize() method is invoked.

  During Initialize(), initialization-triggered unrestricted update events
  are dispatched first for the whole Diagram, then initialization-triggered
  discrete update events are dispatched for the whole Diagram. No other
  _update_ events occur during initialization. On the other hand, any
  _publish_ events, including initialization-triggered, per-step,
  and time-triggered publish events that trigger at the initial time, are
  dispatched together during initialization.

  Template arguments to these methods are inferred from the argument lists
  and need not be specified explicitly. */
  //@{

  /** Declares that a Publish event should occur at initialization and that it
  should invoke the given event handler method. The handler should be a
  class member function (method) with this signature:
  @code
    EventStatus MySystem::MyPublish(const Context<T>&) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_initialization_events "Declare initialization events" for
  more information.

  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `publish` must not be null.

  @see DeclareInitializationDiscreteUpdateEvent()
  @see DeclareInitializationUnrestrictedUpdateEvent()
  @see DeclareInitializationEvent() */
  template <class MySystem>
  void DeclareInitializationPublishEvent(
      EventStatus(MySystem::*publish)(const Context<T>&) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(publish != nullptr);

    DeclareInitializationEvent<PublishEvent<T>>(PublishEvent<T>(
        TriggerType::kInitialization,
        [this_ptr, publish](const Context<T>& context,
                            const PublishEvent<T>&) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*publish)(context);  // Ignore return status for now.
        }));
  }

  /** Declares that a DiscreteUpdate event should occur at initialization
  and that it should invoke the given event handler method. The handler
  should be a class member function (method) with this signature:
  @code
    EventStatus MySystem::MyUpdate(const Context<T>&,
                                   DiscreteValues<T>*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_initialization_events "Declare initialization events" for
  more information.

  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `update` must not be null.

  @see DeclareInitializationPublishEvent()
  @see DeclareInitializationUnrestrictedUpdateEvent()
  @see DeclareInitializationEvent() */
  template <class MySystem>
  void DeclareInitializationDiscreteUpdateEvent(
      EventStatus(MySystem::*update)
          (const Context<T>&, DiscreteValues<T>*) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    DeclareInitializationEvent(DiscreteUpdateEvent<T>(
        TriggerType::kInitialization,
        [this_ptr, update](const Context<T>& context,
                           const DiscreteUpdateEvent<T>&,
                           DiscreteValues<T>* xd) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*update)(context,
                              &*xd);  // Ignore return status for now.
        }));
  }

  /** Declares that an UnrestrictedUpdate event should occur at initialization
  and that it should invoke the given event handler method. The handler
  should be a class member function (method) with this signature:
  @code
    EventStatus MySystem::MyUpdate(const Context<T>&,
                                   State<T>*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_initialization_events "Declare initialization events" for
  more information.

  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `update` must not be null.

  @see DeclareInitializationPublishEvent()
  @see DeclareInitializationDiscreteUpdateEvent()
  @see DeclareInitializationEvent() */
  template <class MySystem>
  void DeclareInitializationUnrestrictedUpdateEvent(
      EventStatus(MySystem::*update)
          (const Context<T>&, State<T>*) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    DeclareInitializationEvent(UnrestrictedUpdateEvent<T>(
        TriggerType::kInitialization,
        [this_ptr, update](const Context<T>& context,
                           const UnrestrictedUpdateEvent<T>&, State<T>* x) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*update)(context,
                              &*x);  // Ignore return status for now.
        }));
  }

  /** (Advanced) Declares that a particular Event object should be dispatched at
  initialization. This is the most general form for declaring initialization
  events and most users should use one of the other methods in this group
  instead.

  @see DeclareInitializationPublishEvent()
  @see DeclareInitializationDiscreteUpdateEvent()
  @see DeclareInitializationUnrestrictedUpdateEvent()

  See @ref declare_initialization_events "Declare initialization events" for
  more information.

  Depending on the type of `event`, on initialization it will be passed to
  the Publish, DiscreteUpdate, or UnrestrictedUpdate event dispatcher. If
  the `event` object contains a handler function, Drake's default
  dispatchers will invoke that handler. If not, then no further action is
  taken. Thus an `event` with no handler has no effect unless its dispatcher
  has been overridden. We strongly recommend that you _do not_ override the
  dispatcher and instead _do_ supply a handler.

  The given `event` object is deep-copied (cloned), and the copy is stored
  internally so you do not need to keep the object around after this call.

  @pre `event`'s associated trigger type must be TriggerType::kUnknown or
  already set to TriggerType::kInitialization. */
  template <typename EventType>
  void DeclareInitializationEvent(const EventType& event) {
    DRAKE_DEMAND(event.get_trigger_type() == TriggerType::kUnknown ||
        event.get_trigger_type() == TriggerType::kInitialization);
    event.AddToComposite(TriggerType::kInitialization, &initialization_events_);
  }
  //@}

  // =========================================================================
  /** @anchor declare_forced_events
  @name                  Declare forced events
  Forced events are those that are triggered through invocation of
  System::ForcedPublish(const Context&),
  System::CalcForcedDiscreteVariableUpdate(const Context&, DiscreteValues<T>*),
  or System::CalcForcedUnrestrictedUpdate(const Context&, State<T>*),
  rather than as a response to some computation-related event (e.g.,
  the beginning of a period of time was reached, a trajectory-advancing
  step was performed, etc.) One useful application of a forced publish:
  a process receives a network message and wants to trigger message
  emissions in various systems embedded within a Diagram in response.

  Template arguments to these methods are inferred from the argument lists.
  and need not be specified explicitly.

  @note It's rare that an event needs to be triggered by force. Please
  consider per-step and periodic triggered events first.

  @warning Simulator handles forced publish events at initialization
  and on a per-step basis when its "publish at initialization" and
  "publish every time step" options are set (not recommended).
  @see Simulator::set_publish_at_initialization()
  @see Simulator::set_publish_every_time_step() */
  //@{

  /** Declares a function that is called whenever a user directly calls
  ForcedPublish(const Context&). Multiple calls to
  DeclareForcedPublishEvent() will cause multiple handlers to be called
  upon a call to ForcedPublish(); these handlers which will be called with the
  same const Context in arbitrary order. The handler should be a class
  member function (method) with this signature:
  @code
    EventStatus MySystem::MyPublish(const Context<T>&) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_forced_events "Declare forced events" for more
  information.
  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `publish` must not be null. */
  template <class MySystem>
  void DeclareForcedPublishEvent(
    EventStatus (MySystem::*publish)(const Context<T>&) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(publish != nullptr);

    // Instantiate the event.
    PublishEvent<T> forced(
        TriggerType::kForced,
        [this_ptr, publish](const Context<T>& context, const PublishEvent<T>&) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*publish)(context);  // Ignore return status for now.
        });

    // Add the event to the collection of forced publish events.
    this->get_mutable_forced_publish_events().AddEvent(std::move(forced));
  }

  /** Declares a function that is called whenever a user directly calls
  CalcForcedDiscreteVariableUpdate(const Context&, DiscreteValues<T>*). Multiple
  calls to DeclareForcedDiscreteUpdateEvent() will cause multiple handlers
  to be called upon a call to CalcForcedDiscreteVariableUpdate(); these handlers
  will be called with the same const Context in arbitrary order. The
  handler should be a class member function (method) with this signature:
  @code
    EventStatus MySystem::MyDiscreteVariableUpdates(const Context<T>&,
    DiscreteValues<T>*);
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_forced_events "Declare forced events" for more
  information.
  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `update` must not be null. */
  template <class MySystem>
  void DeclareForcedDiscreteUpdateEvent(EventStatus
      (MySystem::*update)(const Context<T>&, DiscreteValues<T>*) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    // Instantiate the event.
    DiscreteUpdateEvent<T> forced(
        TriggerType::kForced,
        [this_ptr, update](const Context<T>& context,
                           const DiscreteUpdateEvent<T>&,
                           DiscreteValues<T>* discrete_state) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*update)(
              context, discrete_state);  // Ignore return status for now.
        });

    // Add the event to the collection of forced discrete update events.
    this->get_mutable_forced_discrete_update_events().AddEvent(
        std::move(forced));
  }

  /** Declares a function that is called whenever a user directly calls
  CalcForcedUnrestrictedUpdate(const Context&, State<T>*). Multiple calls to
  DeclareForcedUnrestrictedUpdateEvent() will cause multiple handlers to be
  called upon a call to CalcForcedUnrestrictedUpdate(); these handlers which
  will be called with the same const Context in arbitrary order.The handler
  should be a class member function (method) with this signature:
  @code
    EventStatus MySystem::MyUnrestrictedUpdates(const Context<T>&,
    State<T>*);
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and the method
  name is arbitrary.

  See @ref declare_forced_events "Declare forced events" for more
  information.
  @pre `this` must be dynamic_cast-able to MySystem.
  @pre `update` must not be null. */
  template <class MySystem>
  void DeclareForcedUnrestrictedUpdateEvent(
      EventStatus (MySystem::*update)(const Context<T>&, State<T>*) const) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    DRAKE_DEMAND(update != nullptr);

    // Instantiate the event.
    UnrestrictedUpdateEvent<T> forced(
        TriggerType::kForced,
        [this_ptr, update](const Context<T>& context,
                           const UnrestrictedUpdateEvent<T>&, State<T>* state) {
          // TODO(sherm1) Forward the return status.
          (this_ptr->*update)(context, state);  // Ignore return status for now.
        });

    // Add the event to the collection of forced unrestricted update events.
    this->get_mutable_forced_unrestricted_update_events().AddEvent(
        std::move(forced));
  }
  //@}

  /** @name          Declare continuous state variables
  Continuous state consists of up to three kinds of variables: generalized
  coordinates q, generalized velocities v, and miscellaneous continuous
  variables z. Methods in this section provide different ways to declare
  these, and offer the ability to provide a `model_vector` to specify the
  initial values for these variables. Model values are useful when you want
  the values of these variables in a default Context to be something other
  than zero.

  If multiple calls are made to DeclareContinuousState() methods, only the
  last call has any effect. */
  //@{

  /** Declares that this System should reserve continuous state with
  @p num_state_variables state variables, which have no second-order
  structure.
  @return index of the declared state (currently always zero). */
  ContinuousStateIndex DeclareContinuousState(int num_state_variables);

  /** Declares that this System should reserve continuous state with @p num_q
  generalized positions, @p num_v generalized velocities, and @p num_z
  miscellaneous state variables.
  @return index of the declared state (currently always zero). */
  ContinuousStateIndex DeclareContinuousState(int num_q, int num_v, int num_z);

  /** Declares that this System should reserve continuous state with
  @p model_vector.size() miscellaneous state variables, stored in a
  vector cloned from @p model_vector.
  @return index of the declared state (currently always zero). */
  ContinuousStateIndex DeclareContinuousState(
      const BasicVector<T>& model_vector);

  /** Declares that this System should reserve continuous state with @p num_q
  generalized positions, @p num_v generalized velocities, and @p num_z
  miscellaneous state variables, stored in a vector cloned from
  @p model_vector. Aborts if @p model_vector has the wrong size. If the
  @p model_vector declares any VectorBase::GetElementBounds()
  constraints, they will be re-declared as inequality constraints on this
  system (see DeclareInequalityConstraint()).
  @return index of the declared state (currently always zero). */
  ContinuousStateIndex DeclareContinuousState(
      const BasicVector<T>& model_vector, int num_q, int num_v, int num_z);
  //@}

  /** @name            Declare discrete state variables
  Discrete state consists of any number of discrete state "groups", each
  of which is a vector of discrete state variables. Methods in this section
  provide different ways to declare these, and offer the ability to provide
  a `model_vector` to specify the initial values for each group of
  variables. Model values are useful when you want the values of
  these variables in a default Context to be something other than zero.

  Each call to a DeclareDiscreteState() method produces another
  discrete state group, and the group index is returned. */
  //@{

  /** Declares a discrete state group with @p model_vector.size() state
  variables, stored in a vector cloned from @p model_vector (preserving the
  concrete type and value). */
  DiscreteStateIndex DeclareDiscreteState(const BasicVector<T>& model_vector);

  /** Declares a discrete state group with @p vector.size() state variables,
  stored in a BasicVector initialized with the contents of @p vector. */
  DiscreteStateIndex DeclareDiscreteState(
      const Eigen::Ref<const VectorX<T>>& vector);

  /** Declares a discrete state group with @p num_state_variables state
  variables, stored in a BasicVector initialized to be all-zero. If you want
  non-zero initial values, use an alternate DeclareDiscreteState() signature
  that accepts a `model_vector` parameter.
  @pre `num_state_variables` must be non-negative. */
  DiscreteStateIndex DeclareDiscreteState(int num_state_variables);
  //@}

  /** @name            Declare abstract state variables
  Abstract state consists of any number of arbitrarily-typed variables, each
  represented by an AbstractValue. Each call to the DeclareAbstractState()
  method produces another abstract state variable, and the abstract state
  variable index is returned. */
  //@{

  /** Declares an abstract state variable and provides a model value
  for it. A Context obtained with CreateDefaultContext() will contain this
  abstract state variable initially set to a clone of the `model_value`
  given here. The actual concrete type is always preserved.

  @param model_value The abstract state model value to be cloned as needed.
  @returns index of the declared abstract state variable. */
  AbstractStateIndex DeclareAbstractState(
      const AbstractValue& model_value);
  //@}


  /** @name    (Advanced) Declare size of implicit time derivatives residual
  for use with System::CalcImplicitTimeDerivativeResidual(). Most commonly
  the default value, same as num_continuous_states(), will be the correct
  size for the residual. */
  //@{

  /** (Advanced) Overrides the default size for the implicit time
  derivatives residual. If no value is set, the default size is
  n=num_continuous_states().

  @param[in] n The size of the residual vector output argument of
               System::CalcImplicitTimeDerivativesResidual(). If n <= 0
               restore to the default, num_continuous_states().

  @see implicit_time_derivatives_residual_size()
  @see System::CalcImplicitTimeDerivativesResidual() */
  void DeclareImplicitTimeDerivativesResidualSize(int n) {
    this->set_implicit_time_derivatives_residual_size(n);
  }
  //@}

  // =========================================================================
  /** @name                    Declare input ports
  Methods in this section are used by derived classes to declare their
  input ports, which may be vector valued or abstract valued.

  You should normally provide a meaningful name for any input port you
  create. Names must be unique for this system (passing in a duplicate
  name will throw std::exception). However, if you specify
  kUseDefaultName as the name, then a default name of e.g. "u2", where 2
  is the input port number will be provided. An empty name is not
  permitted. */
  //@{

  /** Declares a vector-valued input port using the given @p model_vector.
  This is the best way to declare LeafSystem input ports that require
  subclasses of BasicVector.  The port's size and type will be the same as
  model_vector. If the port is intended to model a random noise or
  disturbance input, @p random_type can (optionally) be used to label it
  as such.  If the @p model_vector declares any
  VectorBase::GetElementBounds() constraints, they will be
  re-declared as inequality constraints on this system (see
  DeclareInequalityConstraint()).

  @see System::DeclareInputPort() for more information.
  @pydrake_mkdoc_identifier{3args_model_vector} */
  InputPort<T>& DeclareVectorInputPort(
      std::variant<std::string, UseDefaultName> name,
      const BasicVector<T>& model_vector,
      std::optional<RandomDistribution> random_type = std::nullopt);

  /** Declares a vector-valued input port with type BasicVector and size @p
  size.  If the port is intended to model a random noise or disturbance input,
  @p random_type can (optionally) be used to label it as such.

  @see System::DeclareInputPort() for more information.
  @pydrake_mkdoc_identifier{3args_size} */
  InputPort<T>& DeclareVectorInputPort(
      std::variant<std::string, UseDefaultName> name,
      int size,
      std::optional<RandomDistribution> random_type = std::nullopt);

  /** Declares an abstract-valued input port using the given @p model_value.
  This is the best way to declare LeafSystem abstract input ports.

  Any port connected to this input, and any call to FixValue for this
  input, must provide for values whose type matches this @p model_value.

  @see System::DeclareInputPort() for more information. */
  InputPort<T>& DeclareAbstractInputPort(
      std::variant<std::string, UseDefaultName> name,
      const AbstractValue& model_value);

  /** Flags an already-declared input port as deprecated. The first attempt to
  use the port in a program will log a warning message. This function may be
  called at most once for any given port. */
  void DeprecateInputPort(const InputPort<T>& port, std::string message);
  //@}

  // =========================================================================
  /** @name                    Declare output ports
  @anchor DeclareLeafOutputPort_documentation

  Methods in this section are used by derived classes to declare their
  output ports, which may be vector valued or abstract valued. Every output
  port must have an _allocator_ function and
  a _calculator_ function. The allocator returns an object suitable for
  holding a value of the output port. The calculator uses the contents of
  a given Context to produce the output port's value, which is placed in
  an object of the type returned by the allocator.

  Although the allocator and calculator functions ultimately satisfy generic
  function signatures defined in LeafOutputPort, we provide a variety
  of `DeclareVectorOutputPort()` and `DeclareAbstractOutputPort()`
  signatures here for convenient specification, with mapping to the generic
  form handled invisibly. In particular, allocators are most easily defined
  by providing a model value that can be used to construct an
  allocator that copies the model when a new value object is needed.
  Alternatively a method can be provided that constructs a value object when
  invoked (those methods are conventionally, but not necessarily, named
  `MakeSomething()` where `Something` is replaced by the output port value
  type).

  Because output port values are ultimately stored in AbstractValue objects,
  the underlying types must be suitable. For vector ports, that means the
  type must be BasicVector or a class derived from BasicVector. For abstract
  ports, the type must be copy constructible or cloneable. For
  methods below that are not given an explicit model value or construction
  ("make") method, the underlying type must be default constructible.
  @see drake::Value for more about abstract values.

  A list of prerequisites may be provided for the calculator function to
  avoid unnecessary recomputation. If no prerequisites are provided, the
  default is to assume the output port value is dependent on all possible
  sources. See @ref DeclareCacheEntry_documentation "DeclareCacheEntry"
  for more information about prerequisites.

  Output ports must have a name that is unique within the owning subsystem.
  Users can provide meaningful names or specify the name as
  `kUseDefaultName` in which case a name like "y3" is
  automatically provided, where the number is the output port index. An
  empty name is not permitted.

  @anchor DeclareLeafOutputPort_feedthrough
  <em><u>Direct feedthrough</u></em>

  By default, %LeafSystem assumes there is direct feedthrough of values
  from every input to every output. This is a conservative assumption that
  ensures we detect and can prevent the formation of algebraic loops
  (implicit computations) in system Diagrams. Systems which do not have
  direct feedthrough may override that assumption in either of two ways:

  (1) When declaring an output port (e.g., DeclareVectorOutputPort()),
  provide a non-default value for the `prerequisites_of_calc` argument.
  In that case the dependency path from each input port to that output port is
  probed via the fast cache invalidation mechanism to see if it has a direct or
  indirect dependence that input port. For example:
  @code
  PendulumPlant<T>::PendulumPlant() {
    // No feedthrough because the output port depends only on state,
    // and state has no dependencies.
    this->DeclareVectorOutputPort(
        "state", &PendulumPlant::CopyStateOut,
        {this->all_state_ticket()});

    // Has feedthrough from input port 0 but not from any others.
    this->DeclareVectorOutputPort(
        "tau", &PendulumPlant::CopyTauOut,
        {this->input_port_ticket(InputPortIndex(0))});

    // Doesn't specify prerequisites. We'll assume feedthrough from all
    // inputs unless we can apply symbolic analysis (see below).
    this->DeclareVectorOutputPort(
        "result", &PendulumPlant::CalcResult);
  }
  @endcode

  See @ref DependencyTicket_documentation "Dependency tickets" for more
  information about tickets, including a list of possible ticket options.

  (2) Add support for the symbolic::Expression scalar type, per
  @ref system_scalar_conversion_how_to_write_a_system
  "How to write a System that supports scalar conversion".
  This allows the %LeafSystem to infer the sparsity from the symbolic
  equations for any of the output ports that don't specify an explicit
  list of prerequisites.

  Option 2 is a convenient default for simple systems that already support
  symbolic::Expression, but option 1 should be preferred as the most direct
  mechanism to control feedthrough reporting.

  Normally the direct-feedthrough relations are checked automatically to
  detect algebraic loops. If you want to examine the computed feedthrough
  status for all ports or a particular port, see
  System::GetDirectFeedthroughs(), System::HasDirectFeedthrough(), and
  related methods. */
  //@{

  /** Declares a vector-valued output port by specifying (1) a model vector of
  type BasicVectorSubtype derived from BasicVector and initialized to the
  correct size and desired initial value, and (2) a calculator function that
  is a class member function (method) with signature:
  @code
  void MySystem::CalcOutputVector(const Context<T>&,
                                  BasicVectorSubtype*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>`. Template
  arguments will be deduced and do not need to be specified.
  @exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
  template <class MySystem, typename BasicVectorSubtype>
  LeafOutputPort<T>& DeclareVectorOutputPort(
      std::variant<std::string, UseDefaultName> name,
      const BasicVectorSubtype& model_vector,
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived System.");
    static_assert(std::is_base_of_v<BasicVector<T>, BasicVectorSubtype>,
                  "Expected vector type derived from BasicVector.");
    // We need to obtain a `this` pointer of the right derived type to capture
    // in the calculator functor, so that it will be able to invoke the given
    // mmember function `calc()`.
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    // Currently all vector ports in Drake require a fixed size that is known
    // at the time the port is declared.
    auto& port = CreateVectorLeafOutputPort(
        NextOutputPortName(std::move(name)), model_vector.size(),
        // Allocator function just clones the given model vector.
        MakeAllocateCallback<BasicVector<T>>(model_vector),
        // Calculator function downcasts to specific vector type and invokes
        // the given member function.
        [this_ptr, calc](const Context<T>& context, BasicVector<T>* result) {
          auto typed_result = dynamic_cast<BasicVectorSubtype*>(result);
          DRAKE_DEMAND(typed_result != nullptr);
          (this_ptr->*calc)(context, typed_result);
        },
        std::move(prerequisites_of_calc));
    // Caution: "name" is empty now.
    MaybeDeclareVectorBaseInequalityConstraint(
        "output " + std::to_string(int{port.get_index()}), model_vector,
        [&port](const Context<T>& context) -> const VectorBase<T>& {
          return port.template Eval<BasicVector<T>>(context);
        });
    return port;
  }

  /** Declares a vector-valued output port with type BasicVector and size @p
  size, using the drake::dummy_value<T>, which is NaN when T = double. @p calc
  is a calculator function that is a class member function (method) with
  signature:
  @code
  void MySystem::CalcOutputVector(const Context<T>&,
                                  BasicVector<T>*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>`. Template
  arguments will be deduced and do not need to be specified.
  @exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
  template <class MySystem>
  LeafOutputPort<T>& DeclareVectorOutputPort(
      std::variant<std::string, UseDefaultName> name, int size,
      void (MySystem::*calc)(const Context<T>&, BasicVector<T>*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareVectorOutputPort(std::move(name), BasicVector<T>(size),
                                   std::move(calc),
                                   std::move(prerequisites_of_calc));
  }

  /** Declares a vector-valued output port by specifying _only_ a calculator
  function that is a class member function (method) with signature:
  @code
  void MySystem::CalcOutputVector(const Context<T>&,
                                  BasicVectorSubtype*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>` and
  `BasicVectorSubtype` is derived from `BasicVector<T>` and has a suitable
  default constructor that allocates a vector of the expected size. This
  will use `BasicVectorSubtype{}` (that is, the default constructor) to
  produce a model vector for the output port's value.
  Template arguments will be deduced and do not need to be specified.

  @note The default constructor will be called once immediately, and
  subsequent allocations will just copy the model value without invoking the
  constructor again. If you want the constructor invoked again at each
  allocation (not common), use one of the other signatures to explicitly
  provide a method for the allocator to call; that method can then invoke
  the `BasicVectorSubtype` default constructor.
  @exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
  template <class MySystem, typename BasicVectorSubtype>
  LeafOutputPort<T>& DeclareVectorOutputPort(
      std::variant<std::string, UseDefaultName> name,
      void (MySystem::*calc)(const Context<T>&, BasicVectorSubtype*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    static_assert(
        std::is_default_constructible_v<BasicVectorSubtype>,
        "LeafSystem::DeclareVectorOutputPort(calc): the one-argument form of "
        "this method requires that the output type has a default constructor");
    // Invokes the previous method.
    return DeclareVectorOutputPort(NextOutputPortName(std::move(name)),
                                   BasicVectorSubtype{}, calc,
                                   std::move(prerequisites_of_calc));
  }

  /** (Advanced) Declares a vector-valued output port using the given
  `model_vector` and a function for calculating the port's value at runtime.
  The port's size will be model_vector.size(), and the default allocator for
  the port will be model_vector.Clone(). Note that this takes the calculator
  function in its most generic form; if you have a member function available
  use one of the other signatures.
  @see LeafOutputPort::CalcVectorCallback
  @pydrake_mkdoc_identifier{4args_model_vector} */
  LeafOutputPort<T>& DeclareVectorOutputPort(
      std::variant<std::string, UseDefaultName> name,
      const BasicVector<T>& model_vector,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()});

  /** (Advanced) Declares a vector-valued output port with type BasicVector<T>
  and size @p size, using the drake::dummy_value<T>, which is NaN when T =
  double.  @p vector_calc_function is a function for calculating the port's
  value at runtime. Note that this takes the calculator function in its most
  generic form; if you have a member function available use one of the other
  signatures.
  @see LeafOutputPort::CalcVectorCallback
  @pydrake_mkdoc_identifier{4args_size} */
  LeafOutputPort<T>& DeclareVectorOutputPort(
      std::variant<std::string, UseDefaultName> name, int size,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    return DeclareVectorOutputPort(std::move(name), BasicVector<T>(size),
                                   std::move(vector_calc_function),
                                   std::move(prerequisites_of_calc));
  }

  /** Declares an abstract-valued output port by specifying a model value of
  concrete type `OutputType` and a calculator function that is a class
  member function (method) with signature:
  @code
  void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  @endcode
  where `MySystem` must be a class derived from `LeafSystem<T>`.
  `OutputType` must be such that `Value<OutputType>` is permitted.
  Template arguments will be deduced and do not need to be specified.
  @see drake::Value */
  template <class MySystem, typename OutputType>
  LeafOutputPort<T>& DeclareAbstractOutputPort(
      std::variant<std::string, UseDefaultName> name,
      const OutputType& model_value,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    auto& port = CreateAbstractLeafOutputPort(
        NextOutputPortName(std::move(name)),
        ValueProducer(this, model_value, calc),
        std::move(prerequisites_of_calc));
    return port;
  }

  /** Declares an abstract-valued output port by specifying only a calculator
  function that is a class member function (method) with signature:
  @code
  void MySystem::CalcOutputValue(const Context<T>&, OutputType*) const;
  @endcode
  where `MySystem` is a class derived from `LeafSystem<T>`. `OutputType`
  is a concrete type such that `Value<OutputType>` is permitted, and
  must be default constructible, so that we can create a model value using
  `Value<OutputType>{}` (value initialized so numerical types will be
  zeroed in the model).
  Template arguments will be deduced and do not need to be specified.

  @note The default constructor will be called once immediately, and
  subsequent allocations will just copy the model value without invoking the
  constructor again. If you want the constructor invoked again at each
  allocation (not common), use one of the other signatures to explicitly
  provide a method for the allocator to call; that method can then invoke
  the `OutputType` default constructor.
  @see drake::Value */
  template <class MySystem, typename OutputType>
  LeafOutputPort<T>& DeclareAbstractOutputPort(
      std::variant<std::string, UseDefaultName> name,
      void (MySystem::*calc)(const Context<T>&, OutputType*) const,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()}) {
    static_assert(
        std::is_default_constructible_v<OutputType>,
        "LeafSystem::DeclareAbstractOutputPort(calc): the one-argument form of "
        "this method requires that the output type has a default constructor");
    // Note that value initialization {} is required here.
    return DeclareAbstractOutputPort(NextOutputPortName(std::move(name)),
                                     OutputType{}, calc,
                                     std::move(prerequisites_of_calc));
  }

  /** (Advanced) Declares an abstract-valued output port using the given
  allocator and calculator functions provided in their most generic forms.
  If you have a member function available use one of the other signatures.
  @see LeafOutputPort::AllocCallback, LeafOutputPort::CalcCallback */
  LeafOutputPort<T>& DeclareAbstractOutputPort(
      std::variant<std::string, UseDefaultName> name,
      typename LeafOutputPort<T>::AllocCallback alloc_function,
      typename LeafOutputPort<T>::CalcCallback calc_function,
      std::set<DependencyTicket> prerequisites_of_calc = {
          all_sources_ticket()});

  /** Declares a vector-valued output port whose value is the continuous state
  of this system.
  @param state_index must be ContinuousStateIndex(0) for now, since LeafSystem
  only supports a single continuous state group at the moment.
  @pydrake_mkdoc_identifier{continuous} */
  LeafOutputPort<T>& DeclareStateOutputPort(
      std::variant<std::string, UseDefaultName> name,
      ContinuousStateIndex state_index);

  /** Declares a vector-valued output port whose value is the given discrete
  state group of this system.
  @pydrake_mkdoc_identifier{discrete} */
  LeafOutputPort<T>& DeclareStateOutputPort(
      std::variant<std::string, UseDefaultName> name,
      DiscreteStateIndex state_index);

  /** Declares an abstract-valued output port whose value is the given abstract
  state of this system.
  @pydrake_mkdoc_identifier{abstract} */
  LeafOutputPort<T>& DeclareStateOutputPort(
      std::variant<std::string, UseDefaultName> name,
      AbstractStateIndex state_index);

  /** Flags an already-declared output port as deprecated. The first attempt to
  use the port in a program will log a warning message. This function may be
  called at most once for any given port. */
  void DeprecateOutputPort(const OutputPort<T>& port, std::string message);
  //@}

  // =========================================================================
  /** @name                    Make witness functions
  Methods in this section are used by derived classes to make any
  witness functions useful for ensuring that integration ends a step upon
  entering particular times or states.

  In contrast to other declaration methods (e.g., DeclareVectorOutputPort(),
  for which the System class creates and stores the objects and returns
  references to them, the witness function declaration functions return
  heap-allocated objects that the subclass of leaf system owns. This
  facilitates returning pointers to these objects in
  System::DoGetWitnessFunctions(). */
  //@{

  /** Constructs the witness function with the given description (used primarily
  for debugging and logging), direction type, and calculator function; and
  with no event object.
  @note Constructing a witness function with no corresponding event forces
        Simulator's integration of an ODE to end a step at the witness
        isolation time. For example, isolating a function's minimum or
        maximum values can be realized with a witness that triggers on a
        sign change of the function's time derivative, ensuring that the
        actual extreme value is present in the discretized trajectory.

  @note In order for the witness function to be used, you MUST
  overload System::DoGetWitnessFunctions().
  @exclude_from_pydrake_mkdoc{Only the std::function versions are bound.} */
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> MakeWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const) const {
    return std::make_unique<WitnessFunction<T>>(
        this, this, description, direction_type, calc);
  }

  /** Constructs the witness function with the given description (used primarily
  for debugging and logging), direction type, and calculator function; and
  with no event object.

  @note In order for the witness function to be used, you MUST
  overload System::DoGetWitnessFunctions(). */
  std::unique_ptr<WitnessFunction<T>> MakeWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      std::function<T(const Context<T>&)> calc) const;

  /** Constructs the witness function with the given description (used primarily
  for debugging and logging), direction type, calculator function, and
  publish event callback function for when this triggers.

  @note In order for the witness function to be used, you MUST
  overload System::DoGetWitnessFunctions().
  @exclude_from_pydrake_mkdoc{Only the std::function versions are bound.} */
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> MakeWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      void (MySystem::*publish_callback)(
          const Context<T>&, const PublishEvent<T>&) const) const {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived system.");
    auto fn = [this, publish_callback](
        const Context<T>& context, const PublishEvent<T>& publish_event) {
      auto system_ptr = dynamic_cast<const MySystem*>(this);
      DRAKE_DEMAND(system_ptr != nullptr);
      return (system_ptr->*publish_callback)(context, publish_event);
    };
    PublishEvent<T> publish_event(fn);
    publish_event.set_trigger_type(TriggerType::kWitness);
    return std::make_unique<WitnessFunction<T>>(
        this, this, description, direction_type, calc, publish_event.Clone());
  }

  /** Constructs the witness function with the given description (used primarily
  for debugging and logging), direction type, calculator function, and
  discrete update event callback function for when this triggers.

  @note In order for the witness function to be used, you MUST
  overload System::DoGetWitnessFunctions().
  @exclude_from_pydrake_mkdoc{Only the std::function versions are bound.} */
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> MakeWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      void (MySystem::*du_callback)(const Context<T>&,
          const DiscreteUpdateEvent<T>&, DiscreteValues<T>*) const) const {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived system.");
    auto fn = [this, du_callback](const Context<T>& context,
        const DiscreteUpdateEvent<T>& du_event, DiscreteValues<T>* values) {
      auto system_ptr = dynamic_cast<const MySystem*>(this);
      DRAKE_DEMAND(system_ptr != nullptr);
      return (system_ptr->*du_callback)(context, du_event, values);
    };
    DiscreteUpdateEvent<T> du_event(fn);
    du_event.set_trigger_type(TriggerType::kWitness);
    return std::make_unique<WitnessFunction<T>>(
        this, this, description, direction_type, calc, du_event.Clone());
  }

  /** Constructs the witness function with the given description (used primarily
  for debugging and logging), direction type, calculator function, and
  unrestricted update event callback function for when this triggers.

  @note In order for the witness function to be used, you MUST
  overload System::DoGetWitnessFunctions().
  @exclude_from_pydrake_mkdoc{Only the std::function versions are bound.} */
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> MakeWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      void (MySystem::*uu_callback)(const Context<T>&,
          const UnrestrictedUpdateEvent<T>&, State<T>*) const) const {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived system.");
    auto fn = [this, uu_callback](const Context<T>& context,
        const UnrestrictedUpdateEvent<T>& uu_event, State<T>* state) {
      auto system_ptr = dynamic_cast<const MySystem*>(this);
      DRAKE_DEMAND(system_ptr != nullptr);
      return (system_ptr->*uu_callback)(context, uu_event, state);
    };
    UnrestrictedUpdateEvent<T> uu_event(fn);
    uu_event.set_trigger_type(TriggerType::kWitness);
    return std::make_unique<WitnessFunction<T>>(
        this, this, description, direction_type, calc, uu_event.Clone());
  }

  /** Constructs the witness function with the given description (used primarily
  for debugging and logging), direction type, and calculator
  function, and with an object corresponding to the event that is to be
  dispatched when this witness function triggers. Example types of event
  objects are publish, discrete variable update, unrestricted update events.
  A clone of the event will be owned by the newly constructed
  WitnessFunction.

  @note In order for the witness function to be used, you MUST
  overload System::DoGetWitnessFunctions().
  @exclude_from_pydrake_mkdoc{Only the std::function versions are bound.} */
  template <class MySystem>
  std::unique_ptr<WitnessFunction<T>> MakeWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      T (MySystem::*calc)(const Context<T>&) const,
      const Event<T>& e) const {
    static_assert(std::is_base_of_v<LeafSystem<T>, MySystem>,
                  "Expected to be invoked from a LeafSystem-derived system.");
    return std::make_unique<WitnessFunction<T>>(
        this, this, description, direction_type, calc, e.Clone());
  }

  /** Constructs the witness function with the given description (used primarily
  for debugging and logging), direction type, and calculator
  function, and with an object corresponding to the event that is to be
  dispatched when this witness function triggers. Example types of event
  objects are publish, discrete variable update, unrestricted update events.
  A clone of the event will be owned by the newly constructed
  WitnessFunction.

  @note In order for the witness function to be used, you MUST
  overload System::DoGetWitnessFunctions(). */
  std::unique_ptr<WitnessFunction<T>> MakeWitnessFunction(
      const std::string& description,
      const WitnessFunctionDirection& direction_type,
      std::function<T(const Context<T>&)> calc,
      const Event<T>& e) const;
  //@}

  /** Declares a system constraint of the form
    f(context) = 0
  by specifying a member function to use to calculate the (VectorX)
  constraint value with a signature:
  @code
  void MySystem::CalcConstraint(const Context<T>&, VectorX<T>*) const;
  @endcode

  @param count is the dimension of the VectorX output.
  @param description should be a human-readable phrase.
  @returns The index of the constraint.
  Template arguments will be deduced and do not need to be specified.

  @see SystemConstraint<T> for more information about the meaning of
  these constraints. */
  template <class MySystem>
  SystemConstraintIndex DeclareEqualityConstraint(
      void (MySystem::*calc)(const Context<T>&, VectorX<T>*) const,
      int count, std::string description) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    return DeclareEqualityConstraint(
        [this_ptr, calc](const Context<T>& context, VectorX<T>* value) {
          DRAKE_DEMAND(value != nullptr);
          (this_ptr->*calc)(context, value);
        },
        count, std::move(description));
  }

  /** Declares a system constraint of the form
    f(context) = 0
  by specifying a std::function to use to calculate the (Vector) constraint
  value with a signature:
  @code
  void CalcConstraint(const Context<T>&, VectorX<T>*);
  @endcode

  @param count is the dimension of the VectorX output.
  @param description should be a human-readable phrase.
  @returns The index of the constraint.

  @see SystemConstraint<T> for more information about the meaning of
  these constraints. */
  SystemConstraintIndex DeclareEqualityConstraint(
      ContextConstraintCalc<T> calc, int count,
      std::string description);

  /** Declares a system constraint of the form
    bounds.lower() <= calc(context) <= bounds.upper()
  by specifying a member function to use to calculate the (VectorX)
  constraint value with a signature:
  @code
  void MySystem::CalcConstraint(const Context<T>&, VectorX<T>*) const;
  @endcode

  @param description should be a human-readable phrase.
  @returns The index of the constraint.
  Template arguments will be deduced and do not need to be specified.

  @see SystemConstraint<T> for more information about the meaning of
  these constraints. */
  template <class MySystem>
  SystemConstraintIndex DeclareInequalityConstraint(
      void (MySystem::*calc)(const Context<T>&, VectorX<T>*) const,
      SystemConstraintBounds bounds,
      std::string description) {
    auto this_ptr = dynamic_cast<const MySystem*>(this);
    DRAKE_DEMAND(this_ptr != nullptr);
    return DeclareInequalityConstraint(
        [this_ptr, calc](const Context<T>& context, VectorX<T>* value) {
          DRAKE_DEMAND(value != nullptr);
          (this_ptr->*calc)(context, value);
        },
        std::move(bounds), std::move(description));
  }

  /** Declares a system constraint of the form
    bounds.lower() <= calc(context) <= bounds.upper()
  by specifying a std::function to use to calculate the (Vector) constraint
  value with a signature:
  @code
  void CalcConstraint(const Context<T>&, VectorX<T>*);
  @endcode

  @param description should be a human-readable phrase.
  @returns The index of the constraint.

  @see SystemConstraint<T> for more information about the meaning of
  these constraints. */
  SystemConstraintIndex DeclareInequalityConstraint(
      ContextConstraintCalc<T> calc,
      SystemConstraintBounds bounds,
      std::string description);

  /** Derived-class event dispatcher for all simultaneous publish events
  in @p events. Override this in your derived LeafSystem only if you require
  behavior other than the default dispatch behavior (not common).
  The default behavior is to traverse events in the arbitrary order they
  appear in @p events, and for each event that has a callback function,
  to invoke the callback with @p context and that event.

  Do not override this just to handle an event -- instead declare the event
  and a handler callback for it using one of the `Declare...PublishEvent()`
  methods.

  This method is called only from the virtual DispatchPublishHandler, which
  is only called from the public non-virtual Publish(), which will have
  already error-checked @p context so you may assume that it is valid.

  @param[in] context Const current context.
  @param[in] events All the publish events that need handling. */
  virtual void DoPublish(
      const Context<T>& context,
      const std::vector<const PublishEvent<T>*>& events) const;

  // TODO(sherm1) This virtual implementation of CalcDiscreteVariableUpdate()
  //  uses the plural "Updates" instead for unfortunate historical reasons.
  //  Consider whether it is worth changing.

  /** Derived-class event dispatcher for all simultaneous discrete update
  events. Override this in your derived LeafSystem only if you require
  behavior other than the default dispatch behavior (not common).
  The default behavior is to traverse events in the arbitrary order they
  appear in @p events, and for each event that has a callback function,
  to invoke the callback with @p context, that event, and @p discrete_state.
  Note that the same (possibly modified) @p discrete_state is passed to
  subsequent callbacks.

  Do not override this just to handle an event -- instead declare the event
  and a handler callback for it using one of the
  `Declare...DiscreteUpdateEvent()` methods.

  This method is called only from the virtual
  DispatchDiscreteVariableUpdateHandler(), which is only called from
  the public non-virtual CalcDiscreteVariableUpdate(), which will already
  have error-checked the parameters so you don't have to. In particular,
  implementations may assume that @p context is valid; that
  @p discrete_state is non-null, and that the referenced object has the
  same constituent structure as was produced by AllocateDiscreteVariables().

  @param[in] context The "before" state.
  @param[in] events All the discrete update events that need handling.
  @param[in,out] discrete_state The current state of the system on input;
  the desired state of the system on return. */
  virtual void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>& events,
      DiscreteValues<T>* discrete_state) const;

  // TODO(sherm1) Shouldn't require preloading of the output state; better to
  //              note just the changes since usually only a small subset will
  //              be changed by this method.

  /** Derived-class event dispatcher for all simultaneous unrestricted update
  events. Override this in your derived LeafSystem only if you require
  behavior other than the default dispatch behavior (not common).
  The default behavior is to traverse events in the arbitrary order they
  appear in @p events, and for each event that has a callback function,
  to invoke the callback with @p context, that event, and @p state.
  Note that the same (possibly modified) @p state is passed to subsequent
  callbacks.

  Do not override this just to handle an event -- instead declare the event
  and a handler callback for it using one of the
  `Declare...UnrestrictedUpdateEvent()` methods.

  This method is called only from the virtual
  DispatchUnrestrictedUpdateHandler(), which is only called from the
  non-virtual public CalcUnrestrictedUpdate(), which will already have
  error-checked the parameters so you don't have to. In particular,
  implementations may assume that the @p context is valid; that @p state
  is non-null, and that the referenced object has the same constituent
  structure as the state in @p context.

  @param[in]     context The "before" state that is to be used to calculate
                         the returned state update.
  @param[in]     events All the unrestricted update events that need
                        handling.
  @param[in,out] state   The current state of the system on input; the
                         desired state of the system on return. */
  virtual void DoCalcUnrestrictedUpdate(
      const Context<T>& context,
      const std::vector<const UnrestrictedUpdateEvent<T>*>& events,
      State<T>* state) const;

 private:
  using SystemBase::NextInputPortName;
  using SystemBase::NextOutputPortName;

  // Either clones the model_value, or else for vector ports allocates a
  // BasicVector, or else for abstract ports throws an exception.
  std::unique_ptr<AbstractValue> DoAllocateInput(
      const InputPort<T>& input_port) const final;

  std::unique_ptr<CompositeEventCollection<T>>
      DoAllocateCompositeEventCollection() const final;

  std::map<PeriodicEventData, std::vector<const Event<T>*>,
           PeriodicEventDataComparator>
  DoMapPeriodicEventsByTiming(const Context<T>& context) const final;

  void DoFindUniquePeriodicDiscreteUpdatesOrThrow(
      const char* api_name, const Context<T>& context,
      std::optional<PeriodicEventData>* timing,
      EventCollection<DiscreteUpdateEvent<T>>* events) const final;

  // Calls DoPublish.
  // Assumes @param events is an instance of LeafEventCollection, throws
  // std::bad_cast otherwise.
  // Assumes @param events is not empty. Aborts otherwise.
  void DispatchPublishHandler(
      const Context<T>& context,
      const EventCollection<PublishEvent<T>>& events) const final;

  // Calls DoCalcDiscreteVariableUpdates.
  // Assumes @p events is an instance of LeafEventCollection, throws
  // std::bad_cast otherwise.
  // Assumes @p events is not empty. Aborts otherwise.
  void DispatchDiscreteVariableUpdateHandler(
      const Context<T>& context,
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state) const final;

  // To get here:
  // - The EventCollection must be a LeafEventCollection, and
  // - There must be at least one Event belonging to this leaf subsystem.
  void DoApplyDiscreteVariableUpdate(
      const EventCollection<DiscreteUpdateEvent<T>>& events,
      DiscreteValues<T>* discrete_state, Context<T>* context) const final;

  // Calls DoCalcUnrestrictedUpdate.
  // Assumes @p events is an instance of LeafEventCollection, throws
  // std::bad_cast otherwise.
  // Assumes @p events is not empty. Aborts otherwise.
  void DispatchUnrestrictedUpdateHandler(
      const Context<T>& context,
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state) const final;

  // To get here:
  // - The EventCollection must be a LeafEventCollection, and
  // - There must be at least one Event belonging to this leaf subsystem.
  void DoApplyUnrestrictedUpdate(
      const EventCollection<UnrestrictedUpdateEvent<T>>& events,
      State<T>* state, Context<T>* context) const final;

  void DoGetPeriodicEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* events) const final;

  void DoGetPerStepEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* events) const final;

  void DoGetInitializationEvents(
      const Context<T>& context,
      CompositeEventCollection<T>* events) const final;

  // Creates a new cached, vector-valued LeafOutputPort in this LeafSystem and
  // returns a reference to it.
  LeafOutputPort<T>& CreateVectorLeafOutputPort(
      std::string name,
      int fixed_size,
      typename LeafOutputPort<T>::AllocCallback vector_allocator,
      typename LeafOutputPort<T>::CalcVectorCallback vector_calculator,
      std::set<DependencyTicket> calc_prerequisites);

  // Creates a new cached, abstract-valued LeafOutputPort in this LeafSystem and
  // returns a reference to it.
  LeafOutputPort<T>& CreateAbstractLeafOutputPort(
      std::string name,
      ValueProducer producer,
      std::set<DependencyTicket> calc_prerequisites);

  // Creates a new cached LeafOutputPort in this LeafSystem and returns a
  // reference to it. Pass fixed_size == nullopt for abstract ports, or the
  // port size for vector ports. Prerequisites list must not be empty.
  LeafOutputPort<T>& CreateCachedLeafOutputPort(
      std::string name, const std::optional<int>& fixed_size,
      ValueProducer value_producer,
      std::set<DependencyTicket> calc_prerequisites);

  // Creates an abstract output port allocator function from an arbitrary type
  // model value.
  template <typename OutputType>
  static ValueProducer::AllocateCallback MakeAllocateCallback(
      const OutputType& model_value) {
    return internal::AbstractValueCloner(model_value);
  }

  // If @p model_vector's GetElementBounds provides any constraints,
  // then declares inequality constraints on `this` using a calc function that
  // obtains a VectorBase from a Context using @p get_vector_from_context and
  // then compares the runtime value against the declaration-time
  // VectorBase::GetElementBounds. The inequality constraint is imposed as
  // lower_bounds <= get_vector_from_context(context) <= upper_bounds
  // where lower_bounds and upper_bounds are obtained from
  // model_vector.GetElementBounds.
  void MaybeDeclareVectorBaseInequalityConstraint(
      const std::string& kind, const VectorBase<T>& model_vector,
      const std::function<const VectorBase<T>&(const Context<T>&)>&
          get_vector_from_context);

  // Periodic Update or Publish events declared by this system.
  LeafCompositeEventCollection<T> periodic_events_;

  // Update or Publish events declared by this system for every simulator
  // major time step.
  LeafCompositeEventCollection<T> per_step_events_;

  // Update or Publish events that need to be handled at system initialization.
  LeafCompositeEventCollection<T> initialization_events_;

  // A model continuous state to be used during Context allocation.
  std::unique_ptr<BasicVector<T>> model_continuous_state_vector_{
      std::make_unique<BasicVector<T>>(0)};

  // A model discrete state to be used during Context allocation.
  DiscreteValues<T> model_discrete_state_;

  // A model abstract state to be used during Context allocation.
  internal::ModelValues model_abstract_states_;

  // Model inputs to be used in AllocateInput{Vector,Abstract}.
  internal::ModelValues model_input_values_;

  // Model numeric parameters to be used during Context allocation.
  internal::ModelValues model_numeric_parameters_;

  // Model abstract parameters to be used during Context allocation.
  internal::ModelValues model_abstract_parameters_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafSystem)
