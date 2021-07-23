#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "drake/systems/primitives/signal_logger.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
SignalLogger<T>::SignalLogger(int input_size, int batch_allocation_size)
    : LeafSystem<T>(SystemTypeTag<SignalLogger>{}),
      log_(input_size, batch_allocation_size) {
  this->DeclareInputPort("data", kVectorValued, input_size);

  // Use a per-step event by default; disabled by set_publish_period() or
  // set_forced_publish_only().
  this->DeclarePerStepPublishEvent(&SignalLogger<T>::PerStepWriteToLog);
  logging_mode_ = kPerStep;
}

// The scalar-converting copy constructor should return a result whose logging
// mode matches whatever set_publish_period / set_forced_publish_only calls the
// user has made on `other`.
template <typename T>
template <typename U>
SignalLogger<T>::SignalLogger(const SignalLogger<U>& other)
    : SignalLogger<T>(other.get_input_port().size()) {
  switch (static_cast<LoggingMode>(other.logging_mode_)) {
    case kPeriodic: {
      const auto& events = other.GetPeriodicEvents();
      DRAKE_DEMAND(events.size() == 1);
      const PeriodicEventData& timing = events.begin()->first;
      DRAKE_DEMAND(timing.offset_sec() == 0.0);
      this->set_publish_period(timing.period_sec());
      DRAKE_DEMAND(logging_mode_ == kPeriodic);
      return;
    }
    case kForced: {
      this->set_forced_publish_only();
      DRAKE_DEMAND(logging_mode_ == kForced);
      return;
    }
    case kPerStep: {
      DRAKE_DEMAND(logging_mode_ == kPerStep);
      return;
    }
  }
  DRAKE_UNREACHABLE();
}

template <typename T>
void SignalLogger<T>::set_publish_period(double period) {
  switch (logging_mode_) {
    case kPeriodic:
      throw std::logic_error(
          "SignalLogger::set_publish_period(): can only be called once.");
    case kForced:
      throw std::logic_error(
          "SignalLogger::set_publish_period(): cannot be called if "
          "set_forced_publish_only() has been called.");
    case kPerStep:
      this->DeclarePeriodicPublishEvent(period, 0.,  // period, offset
                                        &SignalLogger<T>::WriteToLog);
      logging_mode_ = kPeriodic;
      return;
  }
}

template <typename T>
void SignalLogger<T>::set_forced_publish_only() {
  switch (logging_mode_) {
    case kForced:
      return;  // Already forced-publishing.
    case kPeriodic:
      throw std::logic_error(
          "SignalLogger::set_forced_publish_only(): "
          "cannot be called if set_publish_period() has been called.");
    case kPerStep:
      this->DeclareForcedPublishEvent(&SignalLogger<T>::WriteToLog);
      logging_mode_ = kForced;
      return;
  }
}

template <typename T>
EventStatus SignalLogger<T>::WriteToLog(const Context<T>& context) const {
  log_.AddData(context.get_time(), this->get_input_port().Eval(context));
  return EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SignalLogger)

#pragma GCC diagnostic pop
