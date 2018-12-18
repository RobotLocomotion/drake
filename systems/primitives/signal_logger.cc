#include "drake/systems/primitives/signal_logger.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
SignalLogger<T>::SignalLogger(int input_size, int batch_allocation_size)
    : log_(input_size, batch_allocation_size) {
  this->DeclareInputPort("data", kVectorValued, input_size);

  // Use a per-step event by default; disabled by set_publish_period() or
  // set_forced_publish_only().
  this->DeclarePerStepPublishEvent(&SignalLogger<T>::PerStepWriteToLog);
  logging_mode_ = kPerStep;
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
  log_.AddData(context.get_time(),
               this->EvalVectorInput(context, 0)->get_value());
  return EventStatus::Succeeded();
}

template <typename T>
const InputPort<T>& SignalLogger<T>::get_input_port() const {
  return System<T>::get_input_port(0);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SignalLogger)
