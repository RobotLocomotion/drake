#include "drake/systems/framework/event_collection.h"

namespace drake {
namespace systems {

template <typename EventType>
EventCollection<EventType>::~EventCollection() = default;

template <typename T>
CompositeEventCollection<T>::~CompositeEventCollection() = default;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::CompositeEventCollection);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafCompositeEventCollection);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramCompositeEventCollection);

// Due to a circular dependency (the various event types depend on the
// collection types and vice versa) it's not possible to instantiate the event
// types without the collections available.  For that reason, we create the
// instances for both in this file.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::WitnessTriggeredEventData);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PublishEvent);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteUpdateEvent);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::UnrestrictedUpdateEvent);

template class ::drake::systems::EventCollection<
    ::drake::systems::DiscreteUpdateEvent<double>>;
template class ::drake::systems::EventCollection<
    ::drake::systems::DiscreteUpdateEvent<::drake::AutoDiffXd>>;
template class ::drake::systems::EventCollection<
    ::drake::systems::DiscreteUpdateEvent<::drake::symbolic::Expression>>;

template class ::drake::systems::EventCollection<
    ::drake::systems::UnrestrictedUpdateEvent<double>>;
template class ::drake::systems::EventCollection<
    ::drake::systems::UnrestrictedUpdateEvent<::drake::AutoDiffXd>>;
template class ::drake::systems::EventCollection<
    ::drake::systems::UnrestrictedUpdateEvent<::drake::symbolic::Expression>>;

template class ::drake::systems::EventCollection<
    ::drake::systems::PublishEvent<double>>;
template class ::drake::systems::EventCollection<
    ::drake::systems::PublishEvent<::drake::AutoDiffXd>>;
template class ::drake::systems::EventCollection<
    ::drake::systems::PublishEvent<::drake::symbolic::Expression>>;
