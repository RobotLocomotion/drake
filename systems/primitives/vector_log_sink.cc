#include "drake/systems/primitives/vector_log_sink.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

// TODO(rpoyner-tri) All of the multi-trigger API and implementation here
// (TriggerTypeSet usage, logic for trigger types and publish periods) is
// duplicated from LcmPublisherSystem. It should all be factored to LeafSystem,
// especially if a third use of this pattern turns up.

template <typename T>
VectorLogSink<T>::VectorLogSink(int input_size, double publish_period)
    : VectorLogSink<T>(
        input_size,
        (publish_period > 0.0) ?
        TriggerTypeSet({TriggerType::kForced, TriggerType::kPeriodic}) :
        TriggerTypeSet({TriggerType::kForced, TriggerType::kPerStep}),
        publish_period) {}

template <typename T>
VectorLogSink<T>::VectorLogSink(
    int input_size,
    const TriggerTypeSet& publish_triggers,
    double publish_period)
    : LeafSystem<T>(SystemTypeTag<VectorLogSink>{}),
    publish_triggers_(publish_triggers),
    publish_period_(publish_period) {
  DRAKE_DEMAND(publish_period >= 0.0);
  DRAKE_DEMAND(!publish_triggers.empty());

  // This cache entry just maintains log storage. It is only ever updated
  // by WriteToLog(). This declaration of the cache entry invokes no
  // invalidation support from the cache system.
  log_cache_index_ =
      this->DeclareCacheEntry(
          "log",
          ValueProducer(VectorLog<T>(input_size), &ValueProducer::NoopCalc),
          {this->nothing_ticket()}).cache_index();

  this->DeclareInputPort("data", kVectorValued, input_size);

  // Check that publish_triggers does not contain an unsupported trigger.
  for (const auto& trigger : publish_triggers) {
      DRAKE_THROW_UNLESS((trigger == TriggerType::kForced) ||
        (trigger == TriggerType::kPeriodic) ||
        (trigger == TriggerType::kPerStep));
  }

  // Declare a forced publish so that any time Publish(.) is called on this
  // system (or a Diagram containing it), a message is emitted.
  if (publish_triggers.find(TriggerType::kForced) != publish_triggers.end()) {
    this->DeclareForcedPublishEvent(&VectorLogSink<T>::WriteToLog);
  }

  if (publish_triggers.find(TriggerType::kPeriodic) != publish_triggers.end()) {
    DRAKE_THROW_UNLESS(publish_period > 0.0);
    const double offset = 0.0;
    this->DeclarePeriodicPublishEvent(
        publish_period, offset, &VectorLogSink<T>::WriteToLog);
  } else {
    // publish_period > 0 without TriggerType::kPeriodic has no meaning and is
    // likely a mistake.
    DRAKE_THROW_UNLESS(publish_period == 0.0);
  }

  if (publish_triggers.find(TriggerType::kPerStep) != publish_triggers.end()) {
    this->DeclarePerStepEvent(
    systems::PublishEvent<T>([this](
        const systems::Context<T>& context,
        const systems::PublishEvent<T>&) {
      this->WriteToLog(context);
    }));
  }
}

template <typename T>
template <typename U>
VectorLogSink<T>::VectorLogSink(const VectorLogSink<U>& other)
    : VectorLogSink<T>(other.get_input_port().size(),
                       other.publish_triggers_,
                       other.publish_period_) {}

template <typename T>
const VectorLog<T>&
VectorLogSink<T>::GetLog(const Context<T>& context) const {
  // Relying on the mutable implementation here avoids pointless out-of-date
  // checks.
  return GetLogFromCache(context);
}

template <typename T>
VectorLog<T>&
VectorLogSink<T>::GetMutableLog(Context<T>* context) const {
  return GetLogFromCache(*context);
}

template <typename T>
const VectorLog<T>&
VectorLogSink<T>::FindLog(const Context<T>& root_context) const {
  return GetLogFromCache(this->GetMyContextFromRoot(root_context));
}

template <typename T>
VectorLog<T>&
VectorLogSink<T>::FindMutableLog(Context<T>* root_context) const {
  return GetLogFromCache(this->GetMyMutableContextFromRoot(root_context));
}

template <typename T>
VectorLog<T>&
VectorLogSink<T>::GetLogFromCache(const Context<T>& context) const {
  this->ValidateContext(context);
  CacheEntryValue& value =
      this->get_cache_entry(log_cache_index_)
      .get_mutable_cache_entry_value(context);
  return value.GetMutableValueOrThrow<VectorLog<T>>();
}

template <typename T>
EventStatus VectorLogSink<T>::WriteToLog(const Context<T>& context) const {
  GetLogFromCache(context).AddData(
      context.get_time(), this->get_input_port().Eval(context));
  return EventStatus::Succeeded();
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorLogSink)
