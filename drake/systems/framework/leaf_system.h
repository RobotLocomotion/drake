#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/number_traits.h"
#include "drake/math/autodiff.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

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
    context->set_modal_state(this->AllocateModalState());
    context->set_difference_state(this->AllocateDifferenceState());
    return std::unique_ptr<Context<T>>(context.release());
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput(
      const Context<T>& context) const override {
    std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
    for (const auto& descriptor : this->get_output_ports()) {
      output->get_mutable_ports()->emplace_back(
          new OutputPort(AllocateOutputVector(descriptor)));
    }
    return std::unique_ptr<SystemOutput<T>>(output.release());
  }

  /// Returns the AllocateContinuousState value, which may be nullptr.
  std::unique_ptr<ContinuousState<T>> AllocateTimeDerivatives() const override {
    return AllocateContinuousState();
  }

 protected:
  LeafSystem() {}

  // =========================================================================
  // Implementations of System<T> methods.

  /// Computes the next update time based on the configured period_sec_, for
  /// scalar types that are arithmetic, or aborts for scalar types that are not
  /// arithmetic.
  void DoCalcNextUpdateTime(const Context<T>& context,
                            UpdateActions<T>* events) const override {
    DoCalcNextUpdateTimeImpl(context, events);
  }

  // =========================================================================
  // New methods for subclasses to override

  /// Returns a ContinuousState used to implement both CreateDefaultContext and
  /// AllocateTimeDerivatives. By default, allocates no state. Systems with
  /// continuous state variables should override.
  virtual std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const {
    return nullptr;
  }

  /// Reserves the modal state as required by CreateDefaultContext.  By
  /// default, reserves no state. Systems with modal state should override.
  virtual std::unique_ptr<AbstractValue> AllocateModalState() const {
    return nullptr;
  }

  /// Reserves the difference state as required by CreateDefaultContext.  By
  /// default, reserves no state. Systems with difference state should override.
  virtual std::unique_ptr<VectorBase<T>> AllocateDifferenceState() const {
    return nullptr;
  }

  /// Given a port descriptor, allocates the vector storage.  The default
  /// implementation in this class allocates a BasicVector.  Subclasses can
  /// override to use output vector types other than BasicVector.  The
  /// descriptor must match a port declared via DeclareOutputPort.
  virtual std::unique_ptr<BasicVector<T>> AllocateOutputVector(
      const SystemPortDescriptor<T>& descriptor) const {
    return std::make_unique<BasicVector<T>>(descriptor.get_size());
  }

  // Declares that this System has a simple, fixed-period discrete update.
  // On the discrete tick, the state is updated, and then the outputs are
  // recomputed. Systems that require more elaborate sampling should override
  // DoCalcNextUpdateTime instead of calling this method.
  void DeclarePeriodSec(const T& period_sec) {
    period_sec_ = period_sec;
  }

 private:
  /// Aborts for scalar types that are not numeric, since there is no reasonable
  /// definition of "next update time" outside of the real line.
  template<typename T1 = T>
  typename std::enable_if<!is_numeric<T1>::value>::type
  DoCalcNextUpdateTimeImpl(const Context<T1>& context,
                                UpdateActions<T1>* events) const {
    DRAKE_ABORT_MSG(
        "The default implementation of LeafSystem<T>::DoCalcNextUpdateTime "
        "only works with types that are both drake::is_arithmetic and "
        "drake::is_roundable.");
  }

  /// Computes the next update time based on the configured period_sec_, for
  /// scalar types that are numeric.
  template<typename T1 = T>
  typename std::enable_if<is_numeric<T1>::value>::type
  DoCalcNextUpdateTimeImpl(const Context<T1>& context,
                           UpdateActions<T1>* actions) const {
    DRAKE_ASSERT(period_sec_ >= 0);
    if (period_sec_ == 0) {
      // No discrete update.
      actions->time = std::numeric_limits<T1>::infinity();
      return;
    }

    // Compute the next sample time and write it into the event.
    // NOLINTNEXTLINE(build/namespaces): Needed for ADL of ceil.
    using namespace std;
    const T1& current_time_sec = context.get_time();
    const T1 prev_period = floor(current_time_sec / period_sec_);
    const T1 next_period = ceil(current_time_sec / period_sec_);
    if (prev_period == next_period) {
      actions->time = (next_period + 1) * period_sec_;
    } else {
      actions->time = next_period * period_sec_;
    }

    // Create an action specifying that the discrete event is a state update.
    actions->events.emplace_back();
    DiscreteEvent<T1>& event = actions->events.back();
    event.recipient = this;
    event.action = DiscreteEvent<T1>::kUpdateAction;
  }

  // SystemInterface objects are neither copyable nor moveable.
  explicit LeafSystem(const System<T>& other) = delete;
  LeafSystem& operator=(const System<T>& other) = delete;
  explicit LeafSystem(System<T>&& other) = delete;
  LeafSystem& operator=(System<T>&& other) = delete;

  // If this system has difference state with a constant update rate,
  // period_sec_ is the update period in seconds. Otherwise, zero.
  // Note that while the time in the System framework may have any scalar type,
  // the period, for LeafSystem's purposes, is always a double.
  double period_sec_{0};
};



}  // namespace systems
}  // namespace drake
