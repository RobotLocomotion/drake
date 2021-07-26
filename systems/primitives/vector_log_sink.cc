#include "drake/systems/primitives/vector_log_sink.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
VectorLogSink<T>::VectorLogSink(int input_size)
    : LeafSystem<T>(SystemTypeTag<VectorLogSink>{}) {
  // This cache entry just maintains log storage. It is only ever updated
  // by WriteToLog(). This declaration of the cache entry invokes no
  // invalidation support from the cache system.
  log_cache_index_ =
      this->DeclareCacheEntry(
          "log", ValueProducer(
              // TODO(jwnimmer-tri) Improve ValueProducer constructor sugar.
              internal::AbstractValueCloner(VectorLog<T>(input_size)),
              &ValueProducer::NoopCalc),
          {this->nothing_ticket()}).cache_index();

  this->DeclareInputPort("data", kVectorValued, input_size);

  // Use a per-step event by default; disabled by SetPublishPeriod() or
  // SetForcedPublishOnly().
  this->DeclarePerStepPublishEvent(&VectorLogSink<T>::PerStepWriteToLog);
  logging_mode_ = kPerStep;
}

// The scalar-converting copy constructor should return a result whose logging
// mode matches whatever SetPublishPeriod / SetForcedPublishOnly calls the
// user has made on `other`.
template <typename T>
template <typename U>
VectorLogSink<T>::VectorLogSink(const VectorLogSink<U>& other)
    : VectorLogSink<T>(other.get_input_port().size()) {
  switch (static_cast<LoggingMode>(other.logging_mode_)) {
    case kPeriodic: {
      const auto& events = other.GetPeriodicEvents();
      DRAKE_DEMAND(events.size() == 1);
      const PeriodicEventData& timing = events.begin()->first;
      DRAKE_DEMAND(timing.offset_sec() == 0.0);
      this->SetPublishPeriod(timing.period_sec());
      DRAKE_DEMAND(logging_mode_ == kPeriodic);
      return;
    }
    case kForced: {
      this->SetForcedPublishOnly();
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
void VectorLogSink<T>::SetPublishPeriod(double period) {
  switch (logging_mode_) {
    case kPeriodic:
      throw std::logic_error(
          "VectorLogSink::SetPublishPeriod(): can only be called once.");
    case kForced:
      throw std::logic_error(
          "VectorLogSink::SetPublishPeriod(): cannot be called if "
          "SetForcedPublishOnly() has been called.");
    case kPerStep:
      this->DeclarePeriodicPublishEvent(period, 0.,  // period, offset
                                        &VectorLogSink<T>::WriteToLog);
      logging_mode_ = kPeriodic;
      return;
  }
}

template <typename T>
void VectorLogSink<T>::SetForcedPublishOnly() {
  switch (logging_mode_) {
    case kForced:
      return;  // Already forced-publishing.
    case kPeriodic:
      throw std::logic_error(
          "VectorLogSink::SetForcedPublishOnly(): "
          "cannot be called if SetPublishPeriod() has been called.");
    case kPerStep:
      this->DeclareForcedPublishEvent(&VectorLogSink<T>::WriteToLog);
      logging_mode_ = kForced;
      return;
  }
}

template <typename T>
const VectorLog<T>&
VectorLogSink<T>::GetLog(const Context<T>& context) const {
  // Relying on the mutable implementation here avoids pointless out-of-date
  // checks.
  return GetMutableLog(context);
}

template <typename T>
VectorLog<T>&
VectorLogSink<T>::GetMutableLog(const Context<T>& context) const {
  CacheEntryValue& value =
      this->get_cache_entry(log_cache_index_)
      .get_mutable_cache_entry_value(context);
  return value.GetMutableValueOrThrow<VectorLog<T>>();
}

template <typename T>
const VectorLog<T>&
VectorLogSink<T>::FindLog(const Context<T>& context) const {
  return FindMutableLog(context);
}

template <typename T>
VectorLog<T>&
VectorLogSink<T>::FindMutableLog(const Context<T>& context) const {
  return GetMutableLog(this->GetMyContextFromRoot(context));
}

template <typename T>
EventStatus VectorLogSink<T>::WriteToLog(const Context<T>& context) const {
  GetMutableLog(context).AddData(
      context.get_time(), this->get_input_port().Eval(context));
  return EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorLogSink)
