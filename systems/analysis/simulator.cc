#include "drake/systems/analysis/simulator.h"

#include <thread>

#include "drake/common/extract_double.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"

namespace drake {
namespace systems {

template <typename T>
Simulator<T>::Simulator(const System<T>& system,
                        std::unique_ptr<Context<T>> context)
    : Simulator(&system, nullptr, std::move(context)) {}

template <typename T>
Simulator<T>::Simulator(std::unique_ptr<const System<T>> owned_system,
                        std::unique_ptr<Context<T>> context)
    : Simulator(nullptr, std::move(owned_system), std::move(context)) {}

template <typename T>
Simulator<T>::Simulator(const System<T>* system,
                        std::unique_ptr<const System<T>> owned_system,
                        std::unique_ptr<Context<T>> context)
    : owned_system_(std::move(owned_system)),
      system_(owned_system_ ? *owned_system_ : *system),
      context_(std::move(context)) {
  // TODO(dale.mcconachie) move this default to SimulatorConfig
  constexpr double kDefaultInitialStepSizeTarget = 1e-4;

  // Create a context if necessary.
  if (!context_) context_ = system_.CreateDefaultContext();

  // Create a default integrator and initialize it.
  DRAKE_DEMAND(SimulatorConfig{}.integration_scheme == "runge_kutta3");
  integrator_ = std::unique_ptr<IntegratorBase<T>>(
      new RungeKutta3Integrator<T>(system_, context_.get()));
  integrator_->request_initial_step_size_target(
      kDefaultInitialStepSizeTarget);
  integrator_->set_maximum_step_size(SimulatorConfig{}.max_step_size);
  integrator_->set_target_accuracy(SimulatorConfig{}.accuracy);
  integrator_->Initialize();

  // Allocate the necessary temporaries for storing state in update calls
  // (which will then be transferred back to system state).
  discrete_updates_ = system_.AllocateDiscreteVariables();
  unrestricted_updates_ = context_->CloneState();

  // Allocate the vector of active witness functions.
  witness_functions_ = std::make_unique<
      std::vector<const WitnessFunction<T>*>>();

  // Allocate the necessary temporary for witness-based event handling.
  event_handler_xc_ = system_.AllocateTimeDerivatives();
}

template <typename T>
SimulatorStatus Simulator<T>::Initialize(const InitializeParams& params) {
  // TODO(sherm1) Modify Context to satisfy constraints.
  // TODO(sherm1) Invoke System's initial conditions computation.
  if (!context_)
    throw std::logic_error("Initialize(): Context has not been set.");

  // Record the current time so we can restore it later (see below).
  // *Don't* use a reference here!
  const T current_time = context_->get_time();

  // Assumes success.
  SimulatorStatus status(ExtractDoubleOrThrow(current_time));

  // Initialize the integrator.
  integrator_->Initialize();

  // Restore default values.
  ResetStatistics();

  // Process all the initialization events.
  merged_events_ = system_.AllocateCompositeEventCollection();
  if (!params.suppress_initialization_events) {
    system_.GetInitializationEvents(*context_, merged_events_.get());
  }

  // Do unrestricted updates first.
  HandleUnrestrictedUpdate(merged_events_->get_unrestricted_update_events());
  // Do restricted (discrete variable) updates next.
  HandleDiscreteUpdate(merged_events_->get_discrete_update_events());

  // Gets all per-step events to be handled.
  per_step_events_ = system_.AllocateCompositeEventCollection();
  DRAKE_DEMAND(per_step_events_ != nullptr);
  system_.GetPerStepEvents(*context_, per_step_events_.get());

  // Allocate timed events collection.
  timed_events_ = system_.AllocateCompositeEventCollection();
  DRAKE_DEMAND(timed_events_ != nullptr);

  // Ensure that CalcNextUpdateTime() can return the current time by perturbing
  // current time as slightly toward negative infinity as we can allow.
  const T slightly_before_current_time =
      internal::GetPreviousNormalizedValue(current_time);
  context_->PerturbTime(slightly_before_current_time, current_time);

  // Get the next timed event.
  const T time_of_next_timed_event =
      system_.CalcNextUpdateTime(*context_, timed_events_.get());

  // Reset the context time.
  context_->SetTime(current_time);

  // Indicate a timed event is to be handled, if appropriate.
  if (time_of_next_timed_event == current_time) {
    time_or_witness_triggered_ = kTimeTriggered;
  } else {
    time_or_witness_triggered_ = kNothingTriggered;
  }

  // Allocate the witness function collection.
  witnessed_events_ = system_.AllocateCompositeEventCollection();

  // Do any publishes last. Merge the initialization events with per-step
  // events and current_time timed events (if any). We expect all initialization
  // events to precede any per-step or timed events in the merged collection.
  // Note that per-step and timed discrete/unrestricted update events are *not*
  // processed here; just publish events.
  merged_events_->AddToEnd(*per_step_events_);
  if (time_or_witness_triggered_ & kTimeTriggered)
    merged_events_->AddToEnd(*timed_events_);
  HandlePublish(merged_events_->get_publish_events());

  // TODO(siyuan): transfer publish entirely to individual systems.
  // Do a force-publish before the simulation starts.
  if (publish_at_initialization_) {
    system_.Publish(*context_);
    ++num_publishes_;
  }

  CallMonitorUpdateStatusAndMaybeThrow(&status);

  // Initialize runtime variables.
  initialization_done_ = true;
  last_known_simtime_ = ExtractDoubleOrThrow(context_->get_time());

  return status;
}

// Processes UnrestrictedUpdateEvent events.
template <typename T>
void Simulator<T>::HandleUnrestrictedUpdate(
    const EventCollection<UnrestrictedUpdateEvent<T>>& events) {
  if (events.HasEvents()) {
    // First, compute the unrestricted updates into a temporary buffer.
    system_.CalcUnrestrictedUpdate(*context_, events,
        unrestricted_updates_.get());
    // Now write the update back into the context.
    system_.ApplyUnrestrictedUpdate(events, unrestricted_updates_.get(),
        context_.get());
    ++num_unrestricted_updates_;

    // Mark the witness function vector as needing to be redetermined.
    redetermine_active_witnesses_ = true;
  }
}

// Processes DiscreteEvent events.
template <typename T>
void Simulator<T>::HandleDiscreteUpdate(
    const EventCollection<DiscreteUpdateEvent<T>>& events) {
  if (events.HasEvents()) {
    // First, compute the discrete updates into a temporary buffer.
    system_.CalcDiscreteVariableUpdates(*context_, events,
        discrete_updates_.get());
    // Then, write them back into the context.
    system_.ApplyDiscreteVariableUpdate(events, discrete_updates_.get(),
        context_.get());
    ++num_discrete_updates_;
  }
}

// Processes Publish events.
template <typename T>
void Simulator<T>::HandlePublish(
    const EventCollection<PublishEvent<T>>& events) {
  if (events.HasEvents()) {
    system_.Publish(*context_, events);
    ++num_publishes_;
  }
}

template <typename T>
SimulatorStatus Simulator<T>::AdvanceTo(const T& boundary_time) {
  if (!initialization_done_) {
    const SimulatorStatus initialize_status = Initialize();
    if (!initialize_status.succeeded())
      return initialize_status;
  }

  DRAKE_DEMAND(!std::isnan(last_known_simtime_));
  if (last_known_simtime_ != context_->get_time()) {
    throw std::logic_error(
        "Simulation time has changed since last Initialize() or AdvanceTo()."
        " Resetting simulation time requires a call to Initialize().");
  }

  DRAKE_THROW_UNLESS(boundary_time >= context_->get_time());

  // Assume success.
  SimulatorStatus status(ExtractDoubleOrThrow(boundary_time));

  // Integrate until desired interval has completed.
  DRAKE_DEMAND(timed_events_ != nullptr);
  DRAKE_DEMAND(witnessed_events_ != nullptr);
  DRAKE_DEMAND(merged_events_ != nullptr);

  // Clear events for the loop iteration.
  merged_events_->Clear();
  merged_events_->AddToEnd(*per_step_events_);

  // Merge in timed and witnessed events, if necessary.
  if (time_or_witness_triggered_ & kTimeTriggered)
    merged_events_->AddToEnd(*timed_events_);
  if (time_or_witness_triggered_ & kWitnessTriggered)
    merged_events_->AddToEnd(*witnessed_events_);

  while (true) {
    // Starting a new step on the trajectory.
    const T step_start_time = context_->get_time();
    DRAKE_LOGGER_TRACE("Starting a simulation step at {}", step_start_time);

    // Delay to match target realtime rate if requested and possible.
    PauseIfTooFast();

    // The general policy here is to do actions in decreasing order of
    // "violence" to the state, i.e. unrestricted -> discrete -> continuous ->
    // publish. The "timed" actions happen before the "per step" ones.

    // Do unrestricted updates first.
    HandleUnrestrictedUpdate(merged_events_->get_unrestricted_update_events());
    // Do restricted (discrete variable) updates next.
    HandleDiscreteUpdate(merged_events_->get_discrete_update_events());

    // How far can we go before we have to handle timed events? This can return
    // infinity, meaning we don't see any timed events coming. When an earlier
    // event trigger time is returned, at least one Event object must be
    // returned. Note that if the returned time is the current time, we handle
    // the Events and then restart at the same time, possibly discovering more
    // events.
    const T time_of_next_timed_event =
        system_.CalcNextUpdateTime(*context_, timed_events_.get());
    DRAKE_DEMAND(time_of_next_timed_event >= step_start_time);

    using std::isfinite;
    DRAKE_DEMAND(!isfinite(time_of_next_timed_event) ||
                 timed_events_->HasEvents());

    // Determine whether the set of events requested by the System at
    // time_of_next_timed_event includes an Update action, a Publish action, or
    // both.
    T next_update_time = std::numeric_limits<double>::infinity();
    T next_publish_time = std::numeric_limits<double>::infinity();
    if (timed_events_->HasDiscreteUpdateEvents() ||
        timed_events_->HasUnrestrictedUpdateEvents()) {
      next_update_time = time_of_next_timed_event;
    }
    if (timed_events_->HasPublishEvents()) {
      next_publish_time = time_of_next_timed_event;
    }

    // Integrate the continuous state forward in time. Note that if
    // time_of_next_timed_event is the current time, this will return
    // immediately without time having advanced. That still counts as a step.
    time_or_witness_triggered_ = IntegrateContinuousState(
        next_publish_time,
        next_update_time,
        boundary_time,
        witnessed_events_.get());

    // Update the number of simulation steps taken.
    ++num_steps_taken_;

    // TODO(sherm1) Constraint projection goes here.

    // Clear events for the next loop iteration.
    merged_events_->Clear();

    // Merge in per-step events.
    merged_events_->AddToEnd(*per_step_events_);

    // Only merge timed / witnessed events in if an event was triggered.
    if (time_or_witness_triggered_ & kTimeTriggered)
      merged_events_->AddToEnd(*timed_events_);
    if (time_or_witness_triggered_ & kWitnessTriggered)
      merged_events_->AddToEnd(*witnessed_events_);

    // Handle any publish events at the end of the loop.
    HandlePublish(merged_events_->get_publish_events());

    // TODO(siyuan): transfer per step publish entirely to individual systems.
    // Allow System a chance to produce some output.
    if (get_publish_every_time_step()) {
      system_.Publish(*context_);
      ++num_publishes_;
    }

    CallMonitorUpdateStatusAndMaybeThrow(&status);
    if (!status.succeeded())
      break;  // Done.

    // Break out of the loop after timed and witnessed events are merged in
    // to the event collection and after any publishes.
    if (context_->get_time() >= boundary_time)
      break;
  }

  // TODO(edrumwri): Add test coverage to complete #8490.
  redetermine_active_witnesses_ = true;

  // Record the time to detect unexpected jumps.
  last_known_simtime_ = ExtractDoubleOrThrow(context_->get_time());

  return status;
}

template <class T>
std::optional<T> Simulator<T>::GetCurrentWitnessTimeIsolation() const {
  using std::max;

  // TODO(edrumwri): Add ability to disable witness time isolation through
  // a Simulator setting.

  // The scale factor for witness isolation accuracy, which can allow witness
  // function zeros to be isolated more or less tightly, for positive values
  // less than one and greater than one, respectively. This number should be a
  // reasonable default that allows witness isolation accuracy to be
  // commensurate with integrator accuracy for most systems.
  const double iso_scale_factor = 0.01;

  // TODO(edrumwri): Acquire characteristic time properly from the system
  //                 (i.e., modify the System to provide this value).
  const double characteristic_time = 1.0;

  // Get the accuracy setting.
  const std::optional<double>& accuracy = get_context().get_accuracy();

  // Determine the length of the isolation interval.
  if (integrator_->get_fixed_step_mode()) {
    // Look for accuracy information.
    if (accuracy) {
      return max(integrator_->get_working_minimum_step_size(),
                 T(iso_scale_factor * accuracy.value() *
                     integrator_->get_maximum_step_size()));
    } else {
      return std::optional<T>();
    }
  }

  // Integration with error control isolation window determination.
  if (!accuracy) {
    throw std::logic_error("Integrator is not operating in fixed step mode "
                               "and accuracy is not set in the context.");
  }

  // Note: the max computation is used (here and above) because it is
  // ineffectual to attempt to isolate intervals smaller than the current time
  // in the context can allow.
  return max(integrator_->get_working_minimum_step_size(),
             iso_scale_factor * accuracy.value() * characteristic_time);
}

// Determines whether any witnesses trigger over the interval [t0, tw],
// where tw - t0 < ε and ε is the "witness isolation length". If one or more
// witnesses does trigger over this interval, the time (and corresponding state)
// will be advanced to tw and those witnesses will be stored in
// `triggered_witnesses` on return. On the other hand (i.e., if no witnesses)
// trigger over [t0, t0 + ε], time (and corresponding state) will be advanced
// to some tc in the open interval (t0, tf) such that no witnesses trigger
// over [t0, tc]; in other words, we deem it "safe" to integrate to tc.
// @param[in,out] triggered_witnesses on entry, the set of witness functions
//                that triggered over [t0, tf]; on exit, the set of witness
//                functions that triggered over [t0, tw], where tw is some time
//                such that tw - t0 < ε. If no functions trigger over
//                [t0, t0 + ε], `triggered_witnesses` will be empty on exit.
// @pre The time and state are at tf and x(tf), respectively, and at least
//      one witness function has triggered over [t0, tf].
// @post If `triggered_witnesses` is empty, the time and state will be
//       set to some tc and x(tc), respectively, such that no witnesses trigger
//       over [t0, tc]. Otherwise, the time and state will be set to tw and
//       x(tw), respectively.
// @note The underlying assumption is that a witness function triggers over a
//       interval [a, d] for d ≤ the maximum integrator step size if that
//       witness also triggers over interval [a, b] for some b < d. Per
//       WitnessFunction documentation, we assume that a witness function
//       crosses zero at most once over an interval of size [t0, tf]).
template <class T>
void Simulator<T>::IsolateWitnessTriggers(
    const std::vector<const WitnessFunction<T>*>& witnesses,
    const VectorX<T>& w0,
    const T& t0, const VectorX<T>& x0, const T& tf,
    std::vector<const WitnessFunction<T>*>* triggered_witnesses) {

  // Verify that the vector of triggered witnesses is non-null.
  DRAKE_DEMAND(triggered_witnesses != nullptr);

  // TODO(edrumwri): Speed this process using interpolation between states,
  // more powerful root finding methods, and/or introducing the concept of
  // a dead band.

  // Will need to alter the context repeatedly.
  Context<T>& context = get_mutable_context();

  // Get the witness isolation interval length.
  const std::optional<T> witness_iso_len = GetCurrentWitnessTimeIsolation();

  // Check whether witness functions *are* to be isolated. If not, the witnesses
  // that were triggered on entry will be the set that is returned.
  if (!witness_iso_len)
    return;

  // Mini function for integrating the system forward in time from t0.
  std::function<void(const T&)> integrate_forward =
      [&t0, &x0, &context, this](const T& t_des) {
    const T inf = std::numeric_limits<double>::infinity();
    context.SetTime(t0);
    context.SetContinuousState(x0);
    while (context.get_time() < t_des)
      integrator_->IntegrateNoFurtherThanTime(inf, inf, t_des);
  };

  // Starting from c = (t0 + tf)/2, look for a witness function triggering
  // over the interval [t0, tc]. Assuming a witness does trigger, c will
  // continue moving leftward as a witness function triggers until the length of
  // the time interval is small. If a witness fails to trigger as c moves
  // leftward, we return, indicating that no witnesses triggered over [t0, c].
  DRAKE_LOGGER_DEBUG(
      "Isolating witness functions using isolation window of {} over [{}, {}]",
      witness_iso_len.value(), t0, tf);
  VectorX<T> wc(witnesses.size());
  T a = t0;
  T b = tf;
  do {
    // Compute the midpoint and evaluate the witness functions at it.
    T c = (a + b) / 2;
    DRAKE_LOGGER_DEBUG("Integrating forward to time {}", c);
    integrate_forward(c);

    // See whether any witness functions trigger.
    bool trigger = false;
    for (size_t i = 0; i < witnesses.size(); ++i) {
      wc[i] = get_system().CalcWitnessValue(context, *witnesses[i]);
      if (witnesses[i]->should_trigger(w0[i], wc[i]))
        trigger = true;
    }

    // If no witness function triggered, we can continue integrating forward.
    if (!trigger) {
      // NOTE: Since we're always checking that the sign changes over [t0,c],
      // it's also feasible to replace the two lines below with "a = c" without
      // violating Simulator's contract to only integrate once over the interval
      // [a, c], for some c <= b before per-step events are handled (i.e., it's
      // unacceptable to take two steps of (c - a)/2 without processing per-step
      // events first). That change would avoid handling unnecessary per-step
      // events- we know no other events are to be handled between t0 and tf-
      // but the current logic appears easier to follow.
      DRAKE_LOGGER_DEBUG("No witness functions triggered up to {}", c);
      triggered_witnesses->clear();
      return;  // Time is c.
    } else {
      b = c;
    }
  } while (b - a > witness_iso_len.value());

  // Determine the set of triggered witnesses.
  triggered_witnesses->clear();
  for (size_t i = 0; i < witnesses.size(); ++i) {
    if (witnesses[i]->should_trigger(w0[i], wc[i]))
      triggered_witnesses->push_back(witnesses[i]);
  }
}

// Evaluates the given vector of witness functions.
template <class T>
VectorX<T> Simulator<T>::EvaluateWitnessFunctions(
    const std::vector<const WitnessFunction<T>*>& witness_functions,
    const Context<T>& context) const {
  const System<T>& system = get_system();
  VectorX<T> weval(witness_functions.size());
  for (size_t i = 0; i < witness_functions.size(); ++i)
    weval[i] = system.CalcWitnessValue(context, *witness_functions[i]);
  return weval;
}

// Determines whether at least one of a collection of witness functions
// triggered over a time interval [t0, tf] using the values of those functions
// evaluated at the left and right hand sides of that interval.
// @param witness_functions a vector of all witness functions active over
//        [t0, tf].
// @param w0 the values of the witnesses evaluated at t0.
// @param wf the values of the witnesses evaluated at tf.
// @param [out] triggered_witnesses Returns one of the witnesses that triggered,
//              if any.
// @returns `true` if a witness triggered or `false` otherwise.
template <class T>
bool Simulator<T>::DidWitnessTrigger(
    const std::vector<const WitnessFunction<T>*>& witness_functions,
    const VectorX<T>& w0,
    const VectorX<T>& wf,
    std::vector<const WitnessFunction<T>*>* triggered_witnesses) {
  // See whether a witness function triggered.
  triggered_witnesses->clear();
  bool witness_triggered = false;
  for (size_t i = 0; i < witness_functions.size() && !witness_triggered; ++i) {
      if (witness_functions[i]->should_trigger(w0[i], wf[i])) {
        witness_triggered = true;
        triggered_witnesses->push_back(witness_functions[i]);
      }
  }

  return witness_triggered;
}

// Populates event data for `event` triggered by a witness function (`witness`)
// that was evaluated over the time interval [`t0`, `tf`] and adds it to the
// given event collection (`events`).
template <class T>
void Simulator<T>::PopulateEventDataForTriggeredWitness(
    const T& t0, const T& tf, const WitnessFunction<T>* witness,
    Event<T>* event, CompositeEventCollection<T>* events) const {
  // Populate the event data.
  auto event_data = static_cast<WitnessTriggeredEventData<T>*>(
      event->get_mutable_event_data());
  event_data->set_triggered_witness(witness);
  event_data->set_t0(t0);
  event_data->set_tf(tf);
  event_data->set_xc0(event_handler_xc_.get());
  event_data->set_xcf(&context_->get_continuous_state());
  get_system().AddTriggeredWitnessFunctionToCompositeEventCollection(
      event, events);
}

// (Re)determines the set of witness functions active over this interval,
// if necessary.
template <class T>
void Simulator<T>::RedetermineActiveWitnessFunctionsIfNecessary() {
  const System<T>& system = get_system();
  if (redetermine_active_witnesses_) {
    witness_functions_->clear();
    system.GetWitnessFunctions(get_context(), witness_functions_.get());
    redetermine_active_witnesses_ = false;
  }
}

// Integrates the continuous state forward in time while also locating
// the first zero of any triggered witness functions. Any of these times may
// be set to infinity to indicate that nothing is scheduled.
//
// @param next_publish_time the time at which the next publish event occurs.
// @param next_update_time the time at which the next update event occurs.
// @param boundary_time the maximum time to advance to.
// @param witnessed_events a non-null collection of events, which the method
//     will clear on entry.
// @returns the kind of event triggers that terminated integration.
template <class T>
typename Simulator<T>::TimeOrWitnessTriggered
Simulator<T>::IntegrateContinuousState(
    const T& next_publish_time, const T& next_update_time,
    const T& boundary_time, CompositeEventCollection<T>* witnessed_events) {
  using std::abs;

  // Clear the composite event collection.
  DRAKE_ASSERT(witnessed_events != nullptr);
  witnessed_events->Clear();

  // Save the time and current state.
  const Context<T>& context = get_context();
  const T t0 = context.get_time();
  const VectorX<T> x0 = context.get_continuous_state().CopyToVector();

  // Get the set of witness functions active at the current state.
  RedetermineActiveWitnessFunctionsIfNecessary();
  const auto& witness_functions = *witness_functions_;

  // Evaluate the witness functions.
  w0_ = EvaluateWitnessFunctions(witness_functions, context);

  // Attempt to integrate. Updates and boundary times are consciously
  // distinguished between. See internal documentation for
  // IntegratorBase::IntegrateNoFurtherThanTime() for more information.
  typename IntegratorBase<T>::StepResult result =
      integrator_->IntegrateNoFurtherThanTime(
          next_publish_time, next_update_time, boundary_time);
  const T tf = context.get_time();

  // Evaluate the witness functions again.
  wf_ = EvaluateWitnessFunctions(witness_functions, context);

  // Triggering requires isolating the witness function time.
  if (DidWitnessTrigger(witness_functions, w0_, wf_, &triggered_witnesses_)) {
    // Isolate the time that the witness function triggered. If witness triggers
    // are detected in the interval [t0, tf], any additional time-triggered
    // events are only relevant iff at least one witness function is
    // successfully isolated (see IsolateWitnessTriggers() for details).
    IsolateWitnessTriggers(
        witness_functions, w0_, t0, x0, tf, &triggered_witnesses_);

    // Store the state at x0 in the temporary continuous state. We only do this
    // if there are triggered witnesses (even though `witness_triggered` is
    // `true`, the witness might not have actually triggered after isolation).
    if (!triggered_witnesses_.empty())
      event_handler_xc_->SetFromVector(x0);

    // Store witness function(s) that triggered.
    for (const WitnessFunction<T>* fn : triggered_witnesses_) {
      DRAKE_LOGGER_DEBUG("Witness function {} crossed zero at time {}",
          fn->description(), context.get_time());

      // Skip witness functions that have no associated event (i.e., skip
      // witness functions whose sole purpose is to insert a break in the
      // integration of continuous state).
      if (!fn->get_event())
        continue;

      // Get the event object that corresponds to this witness function. If
      // Simulator has yet to create this object, go ahead and create it.
      auto& event = witness_function_events_[fn];
      if (!event) {
        event = fn->get_event()->Clone();
        event->set_trigger_type(TriggerType::kWitness);
        event->set_event_data(std::make_unique<WitnessTriggeredEventData<T>>());
      }
      PopulateEventDataForTriggeredWitness(t0, tf, fn, event.get(),
                                           witnessed_events);
    }

    // When successful, the isolation process produces a vector of witnesses
    // that trigger over every interval [t0, ti], ∀ti in (t0, tf]. If this
    // vector (triggered_witnesses_) is empty, then time advanced to the first
    // ti such that no witnesses triggered over [t0, ti].
    const T& ti = context_->get_time();
    if (!triggered_witnesses_.empty()) {
      // We now know that integration terminated at a witness function crossing.
      // Now we need to look for the unusual case in which a timed event should
      // also trigger simultaneously.
      // IntegratorBase::IntegrateNoFurtherThanTime(.) pledges to step no
      // further than min(next_publish_time, next_update_time, boundary_time),
      // so we'll verify that assertion.
      DRAKE_DEMAND(ti <= next_update_time && tf <= next_publish_time);
      if (ti == next_update_time || ti == next_publish_time) {
        return kBothTriggered;
      } else {
        return kWitnessTriggered;
      }
    } else {
      // Integration didn't succeed on the larger interval [t0, tf]; instead,
      // the continuous state was integrated to the intermediate time ti, where
      // t0 < ti < tf. Since any publishes/updates must occur at tf, there
      // should be no triggers.
      DRAKE_DEMAND(t0 < ti && ti < tf);

      // The contract for IntegratorBase::IntegrateNoFurtherThanTime() specifies
      // that tf must be less than or equal to next_update_time and
      // next_publish_time. Since ti must be strictly less than tf, it follows
      // that ti must be strictly less than next_update_time and
      // next_publish_time.
      DRAKE_DEMAND(next_update_time > ti && next_publish_time > ti);
      return kNothingTriggered;
    }
  }

  // No witness function triggered; handle integration as usual.
  // Updates and boundary times are consciously distinguished between. See
  // internal documentation for IntegratorBase::IntegrateNoFurtherThanTime() for
  // more information.
  switch (result) {
    case IntegratorBase<T>::kReachedUpdateTime:
    case IntegratorBase<T>::kReachedPublishTime:
      return kTimeTriggered;

    // We do nothing for these two cases.
    case IntegratorBase<T>::kTimeHasAdvanced:
    case IntegratorBase<T>::kReachedBoundaryTime:
      return kNothingTriggered;

    case IntegratorBase<T>::kReachedZeroCrossing:
    case IntegratorBase<T>::kReachedStepLimit:
      throw std::logic_error("Unexpected integrator result");
  }

  DRAKE_UNREACHABLE();
}

template <typename T>
void Simulator<T>::PauseIfTooFast() const {
  if (target_realtime_rate_ <= 0) return;  // Run at full speed.
  const double simtime_now = ExtractDoubleOrThrow(get_context().get_time());
  const double simtime_passed = simtime_now - initial_simtime_;
  const TimePoint desired_realtime =
      initial_realtime_ + Duration(simtime_passed / target_realtime_rate_);
  // TODO(sherm1): Could add some slop to now() and not sleep if
  // we are already close enough. But what is a reasonable value?
  if (desired_realtime > Clock::now())
    std::this_thread::sleep_until(desired_realtime);
}

template <typename T>
double Simulator<T>::get_actual_realtime_rate() const {
  const double simtime_now = ExtractDoubleOrThrow(get_context().get_time());
  const double simtime_passed = simtime_now - initial_simtime_;
  const Duration realtime_passed = Clock::now() - initial_realtime_;
  const double rate = (simtime_passed / realtime_passed.count());
  return rate;
}

template <typename T>
void Simulator<T>::ResetStatistics() {
  integrator_->ResetStatistics();
  num_steps_taken_ = 0;
  num_discrete_updates_ = 0;
  num_unrestricted_updates_ = 0;
  num_publishes_ = 0;

  initial_simtime_ = ExtractDoubleOrThrow(get_context().get_time());
  initial_realtime_ = Clock::now();
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::Simulator)
