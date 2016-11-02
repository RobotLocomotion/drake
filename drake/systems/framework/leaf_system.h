#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/number_traits.h"
#include "drake/math/autodiff_overloads.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/difference_state.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/modal_state.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// A token describing an event that recurs on a fixed period.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
struct PeriodicEvent {
  /// The period with which this event should recur.
  T period_sec{0.0};
  /// The time after zero when this event should first occur.
  T offset_sec{0.0};
  /// The action that should be taken when this event occurs.
  DiscreteEvent<T> event;
};

/// A superclass template that extends System with some convenience utilities
/// that are not applicable to Diagrams.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class LeafSystem : public System<T> {
 public:
  ~LeafSystem() override {}

  // =========================================================================
  // Implementations of System<T> methods.

  std::unique_ptr<Context<T>> CreateDefaultContext() const override {
    std::unique_ptr<LeafContext<T>> context(new LeafContext<T>);
    // Reserve inputs that have already been declared.
    context->SetNumInputPorts(this->get_num_input_ports());
    // Reserve continuous state via delegation to subclass.
    context->set_continuous_state(this->AllocateContinuousState());
    // Reserve discrete state via delegation to subclass.
    context->set_difference_state(this->AllocateDifferenceState());
    context->set_modal_state(this->AllocateModalState());
    return std::unique_ptr<Context<T>>(context.release());
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
    for (const auto& descriptor : this->get_output_ports()) {
      // TODO(liang.fok) Generalize this method to support ports of type
      // kAbstractValued.
      DRAKE_DEMAND(descriptor.get_data_type() == kVectorValued);
      output->get_mutable_ports()->emplace_back(
          new OutputPort(AllocateOutputVector(descriptor)));
    }
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  /// Returns the AllocateContinuousState value, which must not be nullptr.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    return AllocateContinuousState();
  }

  /// Returns the AllocateDifferenceState value, which must not be nullptr.
  std::unique_ptr<DifferenceState<T>> AllocateDifferenceVariables()
      const override {
    return AllocateDifferenceState();
  }

  // LeafSystem objects are neither copyable nor moveable.
  explicit LeafSystem(const System<T>& other) = delete;
  LeafSystem& operator=(const System<T>& other) = delete;
  explicit LeafSystem(System<T>&& other) = delete;
  LeafSystem& operator=(System<T>&& other) = delete;

 protected:
  LeafSystem() {}

  // =========================================================================
  // Implementations of System<T> methods.

  /// Computes the next update time based on the configured periodic events, for
  /// scalar types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic. Subclasses that require aperiodic events should override.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            UpdateActions<T>* events) const override {
    DoCalcNextUpdateTimeImpl(context, events);
  }

  // =========================================================================
  // New methods for subclasses to override

  /// Returns a ContinuousState used to implement both CreateDefaultContext and
  /// AllocateTimeDerivatives. Allocates the state configured with
  /// DeclareContinuousState, or none by default. Systems with continuous state
  /// variables may override.
  virtual std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const {
    if (model_continuous_state_vector_ != nullptr) {
      return std::make_unique<ContinuousState<T>>(
          model_continuous_state_vector_->Clone(), num_generalized_positions_,
          num_generalized_velocities_, num_misc_continuous_state_);
    }
    return std::make_unique<ContinuousState<T>>();
  }

  /// Reserves the difference state as required by CreateDefaultContext. By
  /// default, reserves no state. Systems with difference state should override.
  virtual std::unique_ptr<DifferenceState<T>> AllocateDifferenceState() const {
    return std::make_unique<DifferenceState<T>>();
  }

  /// Reserves the modal state as required by CreateDefaultContext. By
  /// default, reserves no state. Systems with modal state should override.
  virtual std::unique_ptr<ModalState> AllocateModalState() const {
    return std::make_unique<ModalState>();
  }

  /// Given a port descriptor, allocates the vector storage.  The default
  /// implementation in this class allocates a BasicVector.  Subclasses can
  /// override to use output vector types other than BasicVector.  The
  /// descriptor must match a port declared via DeclareOutputPort.
  virtual std::unique_ptr<BasicVector<T>> AllocateOutputVector(
      const SystemPortDescriptor<T>& descriptor) const {
    return std::make_unique<BasicVector<T>>(descriptor.get_size());
  }

  /// Declares that this System has a simple, fixed-period discrete update.
  /// The first tick will be at t = period_sec, and it will recur at every
  /// period_sec thereafter. On the discrete tick, the system may update
  /// the discrete state. Clobbers any other periodic behaviors previously
  /// declared.
  /// TODO(david-german-tri): Add more sophisticated mutators for more complex
  /// periodic behaviors.
  void DeclareUpdatePeriodSec(const T& period_sec) {
    DeclarePeriodicUpdate(period_sec, 0.0);
  }

  /// Declares that this System has a simple, fixed-period discrete update.
  /// The first tick will be at t= offset_sec, and it will recur at every
  /// period_sec thereafter. On the discrete tick, the system may update the
  /// discrete state. Clobbers any other periodc behaviors previously declared.
  void DeclarePeriodicUpdate(const T& period_sec, const T& offset_sec) {
    PeriodicEvent<T> event;
    event.period_sec = period_sec;
    event.offset_sec = offset_sec;
    event.event.recipient = this;
    event.event.action = DiscreteEvent<T>::kUpdateAction;
    periodic_events_ = {event};
  }

  /// Declares that this System should reserve continuous state with
  /// @p num_state_variables state variables, which have no second-order
  /// structure. Has no effect if AllocateContinuousState is overridden.
  void DeclareContinuousState(int num_state_variables) {
    const int num_q = 0, num_v = 0;
    DeclareContinuousState(num_q, num_v, num_state_variables);
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables.  Has no effect if AllocateContinuousState
  /// is overridden.
  void DeclareContinuousState(int num_q, int num_v, int num_z) {
    const int n = num_q + num_v + num_z;
    DeclareContinuousState(std::make_unique<BasicVector<T>>(n),
                           num_q, num_v, num_z);
  }

  /// Declares that this System should reserve continuous state with @p num_q
  /// generalized positions, @p num_v generalized velocities, and @p num_z
  /// miscellaneous state variables, stored in the a vector Cloned from
  /// @p model_vector. Aborts if @p model_vector is nullptr or has the wrong
  /// size. Has no effect if AllocateContinuousState is overridden.
  void DeclareContinuousState(std::unique_ptr<BasicVector<T>> model_vector,
                              int num_q, int num_v, int num_z) {
    DRAKE_DEMAND(model_vector != nullptr);
    DRAKE_DEMAND(model_vector->size() == num_q + num_v + num_z);
    model_continuous_state_vector_ = std::move(model_vector);
    num_generalized_positions_ = num_q;
    num_generalized_velocities_ = num_v;
    num_misc_continuous_state_ = num_z;
  }

 private:
  // Aborts for scalar types that are not numeric, since there is no reasonable
  // definition of "next update time" outside of the real line.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<!is_numeric<T1>::value>::type
  DoCalcNextUpdateTimeImpl(const Context<T1>& context,
                           UpdateActions<T1>* events) const {
    DRAKE_ABORT_MSG(
        "The default implementation of LeafSystem<T>::DoCalcNextUpdateTime "
        "only works with types that are drake::is_numeric.");
  }

  // Computes the next update time across all the scheduled events, for
  // scalar types that are numeric.
  //
  // @tparam T1 SFINAE boilerplate for the scalar type. Do not set.
  template <typename T1 = T>
  typename std::enable_if<is_numeric<T1>::value>::type DoCalcNextUpdateTimeImpl(
      const Context<T1>& context, UpdateActions<T1>* actions) const {
    T1 min_time =
        std::numeric_limits<typename Eigen::NumTraits<T1>::Real>::infinity();
    if (periodic_events_.empty()) {
      // No discrete update.
      actions->time = min_time;
      return;
    }

    // Find the minimum next sample time across all registered events, and
    // the set of registered events that will occur at that time.
    std::vector<const PeriodicEvent<T>*> next_events;
    for (const PeriodicEvent<T>& event : periodic_events_) {
      T1 t = GetNextSampleTime(event, context.get_time());
      if (t < min_time) {
        min_time = t;
        next_events = {&event};
      } else if (t == min_time) {
        next_events.push_back(&event);
      }
    }

    // Write out the events that fire at min_time.
    actions->time = min_time;
    for (const PeriodicEvent<T>* event : next_events) {
      actions->events.push_back(event->event);
    }
  }

  // Returns the next sample time for the given @p event.
  static T GetNextSampleTime(const PeriodicEvent<T>& event,
                             const T& current_time_sec) {
    const T& period = event.period_sec;
    DRAKE_ASSERT(period > 0);
    const T& offset = event.offset_sec;
    DRAKE_ASSERT(offset >= 0);

    // If the first sample time hasn't arrived yet, then that is the next
    // sample time.
    if (current_time_sec < offset) {
      return offset;
    }

    // NOLINTNEXTLINE(build/namespaces): Needed for ADL of floor and ceil.
    using namespace std;

    // Compute the index in the sequence of samples for the next time to sample.
    // If the current time is exactly a sample time, use the next index.
    const T offset_time = current_time_sec - offset;
    const int64_t prev_k = static_cast<int64_t>(floor(offset_time / period));
    const int64_t next_k = static_cast<int64_t>(ceil(offset_time / period));
    const int64_t k = (prev_k == next_k) ? next_k + 1 : next_k;
    return offset + (k * period);
  }

  // Periodic Update or Publish events registered on this system.
  std::vector<PeriodicEvent<T>> periodic_events_;

  // A model continuous state to be used in AllocateDefaultContext.
  std::unique_ptr<BasicVector<T>> model_continuous_state_vector_;
  int num_generalized_positions_{0};
  int num_generalized_velocities_{0};
  int num_misc_continuous_state_{0};
};

}  // namespace systems
}  // namespace drake
