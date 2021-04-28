#include "drake/systems/framework/event_collection.h"

namespace drake {
namespace systems {

template <typename EventType>
void LeafEventCollection<EventType>::CheckInvariant() const {
  // Always retain at least one block.
  DRAKE_ASSERT(!events_storage_.empty());
  // Last block indexes within allocated storage.
  const int block_count = static_cast<int>(events_storage_.size());
  DRAKE_ASSERT(last_block_ < block_count);

  // Check blocks and sizes. Also collect count of stored events.
  int block_index = 0;
  int event_count = 0;
  for (const auto& block : events_storage_) {
    // Block size is never negative.
    DRAKE_ASSERT(block.size >= 0);
    if (block_index < last_block_) {
      // Blocks before last_block_ are full.
      DRAKE_ASSERT(block.size == kStride);
    } else if (block_index == last_block_) {
      // Last block might be full.
      DRAKE_ASSERT(block.size <= kStride);
    } else {  // block_index > last_block_
      // Blocks beyond last_block_ are empty.
      DRAKE_ASSERT(block.size == 0);
    }

    event_count += block.size;
    block_index++;
  }

  // Check event pointer vector.
  int event_ptr_count = static_cast<int>(events_.size());
  DRAKE_ASSERT(event_count == event_ptr_count);
  auto block_iterator = events_storage_.cbegin();
  int event_index = 0;
  for (auto ptr : events_) {
    DRAKE_ASSERT(ptr == &block_iterator->entries[event_index]);
    event_index++;
    if (event_index >= kStride) {
      ++block_iterator;
      event_index = 0;
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::CompositeEventCollection)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafCompositeEventCollection)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramCompositeEventCollection)

// Due to a circular dependency (the various event types depend on the
// collection types and vice versa) it's not possible to instantiate the event
// types without the collections available.  For that reason, we create the
// instances for both in this file.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::WitnessTriggeredEventData)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Event)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PublishEvent)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteUpdateEvent)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::UnrestrictedUpdateEvent)
