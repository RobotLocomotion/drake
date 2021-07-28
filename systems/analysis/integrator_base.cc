#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

template <class T>
bool IntegratorBase<T>::StepOnceErrorControlledAtMost(const T& h_max) {
  using std::isnan;
  using std::min;

  // Verify that the integrator supports error estimates.
  if (!supports_error_estimation()) {
    throw std::logic_error("StepOnceErrorControlledAtMost() requires error "
                               "estimation.");
  }

  // Save time, continuous variables, and time derivative because we'll possibly
  // revert time and state.
  const Context<T>& context = get_context();
  const T current_time = context.get_time();
  VectorBase<T>& xc =
      get_mutable_context()->get_mutable_continuous_state_vector();
  xc0_save_ = xc.CopyToVector();

  // Set the step size to attempt.
  T step_size_to_attempt = get_ideal_next_step_size();
  if (isnan(step_size_to_attempt)) {
    // Integrator has not taken a step. Set the current step size to the
    // initial step size.
    step_size_to_attempt = get_initial_step_size_target();
    DRAKE_DEMAND(!isnan(step_size_to_attempt));
  }

  // This variable indicates when the integrator has been pushed to its minimum
  // step limit. It can only be "true" if minimum step exceptions have been
  // suppressed by the user via set_throw_on_minimum_step_size_violation(false),
  // and the error control mechanism determines that the step is as low as it
  // can go.
  bool at_minimum_step_size = false;

  bool step_succeeded = false;
  do {
    // Constants used to determine whether modifications to the step size are
    // close enough to the attempted step size to use the unadjusted originals,
    // or (1) whether the step size to be attempted is so small that we should
    // consider it to be artificially limited or (2) whether the step size to
    // be attempted is sufficiently close to that requested such that the step
    // size should be stretched slightly.
    const double near_enough_smaller = 0.95;
    const double near_enough_larger = 1.001;

    // If we lose more than a small fraction of the step size we wanted
    // to take due to a need to stop at h_max, make a note of that so the
    // step size adjuster won't try to grow from the current step.
    bool h_was_artificially_limited = false;
    if (h_max < near_enough_smaller * step_size_to_attempt) {
      // h_max much smaller than current step size.
      h_was_artificially_limited = true;
      step_size_to_attempt = h_max;
    } else {
      if (h_max < near_enough_larger * step_size_to_attempt) {
        // h_max is roughly current step. Make it the step size to prevent
        // creating a small sliver (the remaining step).
        step_size_to_attempt = h_max;
      }
    }

    // Limit the current step size.
    step_size_to_attempt = min(step_size_to_attempt, get_maximum_step_size());

    // Keep adjusting the integration step size until any integrator
    // convergence failures disappear. Note: this loop's correctness is
    // predicated on the assumption that an integrator will always converge for
    // a sufficiently small, yet nonzero step size.
    T adjusted_step_size = step_size_to_attempt;
    while (!Step(adjusted_step_size)) {
      DRAKE_LOGGER_DEBUG("Sub-step failed at {}", adjusted_step_size);
      adjusted_step_size *= subdivision_factor_;

      // Note: we could give the user more rope to hang themselves by looking
      // for zero rather than machine epsilon, which might be advantageous if
      // the user were modeling systems over extremely small time scales.
      // However, that issue could be addressed instead by scaling units, and
      // using machine epsilon allows failure to be detected much more rapidly.
      if (adjusted_step_size < std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Integrator has been directed to a near zero-"
                                 "length step in order to obtain convergence.");
      }
      ValidateSmallerStepSize(step_size_to_attempt, adjusted_step_size);
      ++num_shrinkages_from_substep_failures_;
      ++num_substep_failures_;
      if (get_dense_output()) {
        // Take dense output one step back to undo
        // the last integration step.
        dense_output_->RemoveFinalSegment();
      }
    }
    step_size_to_attempt = adjusted_step_size;

    //--------------------------------------------------------------------
    T err_norm = CalcStateChangeNorm(*get_error_estimate());
    T next_step_size;
    std::tie(step_succeeded, next_step_size) = CalcAdjustedStepSize(
        err_norm, step_size_to_attempt, &at_minimum_step_size);
    DRAKE_LOGGER_DEBUG("Succeeded? {}, Next step size: {}",
        step_succeeded, next_step_size);

    if (step_succeeded) {
      // Only update the next step size (retain the previous one) if the
      // step size was not artificially limited.
      if (!h_was_artificially_limited)
        ideal_next_step_size_ = next_step_size;

      if (isnan(get_actual_initial_step_size_taken()))
        set_actual_initial_step_size_taken(step_size_to_attempt);

      // Record the adapted step size taken.
      if (isnan(get_smallest_adapted_step_size_taken()) ||
          (step_size_to_attempt < get_smallest_adapted_step_size_taken() &&
                step_size_to_attempt < h_max))
          set_smallest_adapted_step_size_taken(step_size_to_attempt);
    } else {
      ++num_shrinkages_from_error_control_;

      // Set the next step size to attempt.
      step_size_to_attempt = next_step_size;

      // Reset the time, state, and time derivative at t0.
      get_mutable_context()->SetTime(current_time);
      xc.SetFromVector(xc0_save_);
      if (get_dense_output()) {
        // Take dense output one step back to undo
        // the last integration step.
        dense_output_->RemoveFinalSegment();
      }
    }
  } while (!step_succeeded);
  return static_cast<bool>(step_size_to_attempt == h_max);
}

template <class T>
T IntegratorBase<T>::CalcStateChangeNorm(
    const ContinuousState<T>& dx_state) const {
  using std::max;
  const Context<T>& context = get_context();
  const System<T>& system = get_system();

  // Get weighting matrices.
  const auto& qbar_v_weight = this->get_generalized_state_weight_vector();
  const auto& z_weight = this->get_misc_state_weight_vector();

  // Get the differences in the generalized position, velocity, and
  // miscellaneous continuous state vectors.
  const VectorBase<T>& dgq = dx_state.get_generalized_position();
  const VectorBase<T>& dgv = dx_state.get_generalized_velocity();
  const VectorBase<T>& dgz = dx_state.get_misc_continuous_state();

  // (re-)Initialize pinvN_dq_change_ and weighted_q_change_, if necessary.
  // Reinitialization might be required if the system state variables can
  // change during the course of the simulation.
  if (pinvN_dq_change_ == nullptr) {
    pinvN_dq_change_ = std::make_unique<BasicVector<T>>(dgv.size());
    weighted_q_change_ = std::make_unique<BasicVector<T>>(dgq.size());
  }
  DRAKE_DEMAND(pinvN_dq_change_->size() == dgv.size());
  DRAKE_DEMAND(weighted_q_change_->size() == dgq.size());

  // TODO(edrumwri): Acquire characteristic time properly from the system
  //                 (i.e., modify the System to provide this value).
  const double characteristic_time = 1.0;

  // Computes the infinity norm of the weighted velocity variables.
  unweighted_substate_change_ = dgv.CopyToVector();
  T v_nrm = qbar_v_weight.cwiseProduct(unweighted_substate_change_).
      template lpNorm<Eigen::Infinity>() * characteristic_time;

  // Compute the infinity norm of the weighted auxiliary variables.
  unweighted_substate_change_ = dgz.CopyToVector();
  T z_nrm = (z_weight.cwiseProduct(unweighted_substate_change_))
                .template lpNorm<Eigen::Infinity>();

  // Compute N * Wq * dq = N * Wꝗ * N+ * dq.
  unweighted_substate_change_ = dgq.CopyToVector();
  system.MapQDotToVelocity(context, unweighted_substate_change_,
                           pinvN_dq_change_.get());
  system.MapVelocityToQDot(
      context, qbar_v_weight.cwiseProduct(pinvN_dq_change_->CopyToVector()),
      weighted_q_change_.get());
  T q_nrm = weighted_q_change_->CopyToVector().
      template lpNorm<Eigen::Infinity>();
  DRAKE_LOGGER_DEBUG("dq norm: {}, dv norm: {}, dz norm: {}",
      q_nrm, v_nrm, z_nrm);

  // Return NaN if one of the values is NaN (whether std::max does this is
  // dependent upon ordering!)
  using std::isnan;
  if (isnan(q_nrm) || isnan(v_nrm) || isnan(z_nrm))
    return std::numeric_limits<T>::quiet_NaN();

  // TODO(edrumwri): Record the worst offender (which of the norms resulted
  // in the largest value).
  // Infinity norm of the concatenation of multiple vectors is equal to the
  // maximum of the infinity norms of the individual vectors.
  return max(z_nrm, max(q_nrm, v_nrm));
}

template <class T>
std::pair<bool, T> IntegratorBase<T>::CalcAdjustedStepSize(
    const T& err,
    const T& step_taken,
    bool* at_minimum_step_size) const {
  using std::pow;
  using std::min;
  using std::max;
  using std::isnan;
  using std::isinf;

  // Magic numbers come from Simbody.
  const double kSafety = 0.9;
  const double kMinShrink = 0.1;
  const double kMaxGrow = 5.0;
  const double kHysteresisLow = 0.9;
  const double kHysteresisHigh = 1.2;

  // Get the order for the integrator's error estimate.
  const int err_order = get_error_estimate_order();

  // Set value for new step size to invalid value initially.
  T new_step_size(-1);

  // First, make a guess at the next step size to use based on
  // the supplied error norm. Watch out for NaN. Further adjustments will be
  // made in blocks of code that follow.
  if (isnan(err) || isinf(err)) {  // e.g., integrand returned NaN.
    new_step_size = kMinShrink * step_taken;
    return std::make_pair(false, new_step_size);
  } else {
    if (err == 0) {  // A "perfect" step; can happen if no dofs for example.
      new_step_size = kMaxGrow * step_taken;
    } else {  // Choose best step for skating just below the desired accuracy.
      new_step_size = kSafety * step_taken *
                      pow(get_accuracy_in_use() / err, 1.0 / err_order);
    }
  }

  // Error indicates that the step size can be increased.
  if (new_step_size > step_taken) {
    // If the integrator has been directed down to the minimum step size, but
    // now error indicates that the step size can be increased, de-activate
    // at_minimum_step_size.
    *at_minimum_step_size = false;

    // If the new step is bigger than the old, don't make the change if the
    // old one was small for some unimportant reason (like reached a publishing
    // interval). Also, don't grow the step size if the change would be very
    // small; better to keep the step size stable in that case (maybe just
    // for aesthetic reasons).
    if (new_step_size < kHysteresisHigh * step_taken)
      new_step_size = step_taken;
  }

  // If error indicates that we should shrink the step size but are not allowed
  // to, quit and indicate that the step was successful.
  if (new_step_size < step_taken && *at_minimum_step_size) {
    return std::make_pair(true, step_taken);
  }

  // If we're supposed to shrink the step size but the one we have actually
  // achieved the desired accuracy last time, we won't change the step now.
  // Otherwise, if we are going to shrink the step, let's not be shy -- we'll
  // shrink it by at least a factor of kHysteresisLow.
  if (new_step_size < step_taken) {
    if (err <= get_accuracy_in_use()) {
      new_step_size = step_taken;  // not this time
    } else {
      T test_value = kHysteresisLow * step_taken;
      new_step_size = min(new_step_size, test_value);
    }
  }

  // Keep the size change within the allowable bounds.
  T max_grow_step = kMaxGrow * step_taken;
  T min_shrink_step = kMinShrink * step_taken;
  new_step_size = min(new_step_size, max_grow_step);
  new_step_size = max(new_step_size, min_shrink_step);

  // Apply user-requested limits on min and max step size.
  // TODO(edrumwri): Introduce some feedback to the user when integrator wants
  // to take a smaller step than user has selected as the minimum. Options for
  // this feedback could include throwing a special exception, logging, setting
  // a flag in the integrator that allows throwing an exception, or returning
  // a special status from IntegrateNoFurtherThanTime().
  if (!isnan(get_maximum_step_size()))
    new_step_size = min(new_step_size, get_maximum_step_size());
  ValidateSmallerStepSize(step_taken, new_step_size);

  // Increase the next step size, as necessary.
  new_step_size = max(new_step_size, get_working_minimum_step_size());
  if (new_step_size == get_working_minimum_step_size()) {
    // Indicate that the step is integrator is now trying the minimum step
    // size.
    *at_minimum_step_size = true;

    // If the integrator wants to shrink the step size below the
    // minimum allowed and exceptions are suppressed, indicate that status.
    if (new_step_size < step_taken)
      return std::make_pair(false, new_step_size);
  }

  return std::make_pair(
      static_cast<bool>(new_step_size >= step_taken),
      new_step_size);
}

template <class T>
typename IntegratorBase<T>::StepResult
    IntegratorBase<T>::IntegrateNoFurtherThanTime(
        const T& publish_time, const T& update_time, const T& boundary_time) {
  if (!IntegratorBase<T>::is_initialized())
    throw std::logic_error("Integrator not initialized.");

  // Now that integrator has been checked for initialization, get the current
  // time.
  const T t0 = context_->get_time();

  // Verify that h's are non-negative.
  const T publish_dt = publish_time - t0;
  const T update_dt = update_time - t0;
  const T boundary_dt = boundary_time - t0;
  if (publish_dt < 0.0)
    throw std::logic_error("Publish h is negative.");
  if (update_dt < 0.0)
    throw std::logic_error("Update h is negative.");
  if (boundary_dt < 0.0)
    throw std::logic_error("Boundary h is negative.");

  // The size of the integration step is the minimum of the time until the next
  // update event, the time until the next publish event, the boundary time
  // (i.e., the maximum time that the user wished to step to), and the maximum
  // step size (which may stretch slightly to hit a discrete event).

  // We report to the caller which event ultimately constrained the step size.
  // If multiple events constrained it equally, we prefer to report update
  // events over publish events, publish events over boundary step limits,
  // and boundary limits over maximum step size limits. The caller must
  // determine event simultaneity by inspecting the time.

  // The maintainer of this code is advised to consider that, while updates
  // and boundary times, may both conceptually be deemed events, the distinction
  // is made for a reason. If both an update and a boundary time occur
  // simultaneously, the following behavior should result:
  // (1) kReachedUpdateTime is returned, (2) Simulator::AdvanceTo() performs the
  // necessary update, (3) IntegrateNoFurtherThanTime() is called with
  // boundary_time equal to the current time in the context and returns
  // kReachedBoundaryTime, and (4) the simulation terminates. This sequence of
  // operations will ensure that the simulation state is valid if
  // Simulator::AdvanceTo() is called again to advance time further.

  // We now analyze the following simultaneous cases with respect to Simulator:
  //
  // { publish, update }
  // kReachedUpdateTime will be returned, an update will be followed by a
  // publish.
  //
  // { publish, update, max step }
  // kReachedUpdateTime will be returned, an update will be followed by a
  // publish.
  //
  // { publish, boundary time, max step }
  // kReachedPublishTime will be returned, a publish will be performed followed
  // by another call to this function, which should return kReachedBoundaryTime
  // (followed in rapid succession by AdvanceTo(.) return).
  //
  // { publish, boundary time, max step }
  // kReachedPublishTime will be returned, a publish will be performed followed
  // by another call to this function, which should return kReachedBoundaryTime
  // (followed in rapid succession by AdvanceTo(.) return).
  //
  // { publish, update, boundary time, maximum step size }
  // kUpdateTimeReached will be returned, an update followed by a publish
  // will then be performed followed by another call to this function, which
  // should return kReachedBoundaryTime (followed in rapid succession by
  // AdvanceTo(.) return).

  // By default, the target time is that of the next discrete update event.
  StepResult candidate_result = IntegratorBase<T>::kReachedUpdateTime;
  T target_time = update_time;

  // If the next discrete publish event is sooner than the next discrete update
  // event, the time of the publish event becomes the target time.
  if (publish_time < update_time) {
    candidate_result = IntegratorBase<T>::kReachedPublishTime;
    target_time = publish_time;
  }

  // If the stop time (boundary time) is sooner than the candidate, use it
  // instead.
  if (boundary_time < target_time) {
    candidate_result = IntegratorBase<T>::kReachedBoundaryTime;
    target_time = boundary_time;
  }

  // If there is no continuous state, there will be no need to limit the
  // integration step size.
  if (get_context().num_continuous_states() == 0) {
    Context<T>* context = get_mutable_context();
    context->SetTime(target_time);
    return candidate_result;
  }

  // If all events are further into the future than the maximum step
  // size times a stretch factor of 1.01, the maximum time becomes the
  // target time. Put another way, if the maximum step occurs right before
  // an update or a publish, the update or publish is done instead. In contrast,
  // we never step past boundary_time, even if doing so would allow hitting a
  // publish or an update.
  const bool reached_boundary =
      (candidate_result == IntegratorBase<T>::kReachedBoundaryTime);
  const T& max_h = this->get_maximum_step_size();
  const T max_integrator_time = t0 + max_h;
  if ((reached_boundary && max_integrator_time < target_time) ||
      (!reached_boundary && t0 + max_h * get_stretch_factor() < target_time)) {
    candidate_result = IntegratorBase<T>::kTimeHasAdvanced;
    target_time = max_integrator_time;
  }

  T h = target_time - t0;
  if (h < 0.0) throw std::logic_error("Negative h.");

  // If error control is disabled, call the generic stepper. Otherwise, use
  // the error controlled method.
  bool full_step = true;
  if (this->get_fixed_step_mode()) {
    T adjusted_h = h;
    while (!Step(adjusted_h)) {
      ++num_shrinkages_from_substep_failures_;
      ++num_substep_failures_;
      adjusted_h *= subdivision_factor_;
      ValidateSmallerStepSize(h, adjusted_h);
      full_step = false;
    }
  } else {
    full_step = StepOnceErrorControlledAtMost(h);
  }

  // Update generic statistics.
  const T actual_h = context_->get_time() - t0;
  UpdateStepStatistics(actual_h);

  if (full_step || context_->get_time() >= target_time) {
    // Correct any rounding error that may have caused the time to overrun
    // the target time.
    context_->SetTime(target_time);

    // If the integrator took the entire maximum step size we allowed above,
    // we report to the caller that a step constraint was hit, which may
    // indicate a discrete event has arrived.
    return candidate_result;
  } else {
    // Otherwise, we expect that time has advanced, but no event has arrived.
    return IntegratorBase<T>::kTimeHasAdvanced;
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::IntegratorBase)
