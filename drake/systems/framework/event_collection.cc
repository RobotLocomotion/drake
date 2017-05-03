#include "drake/systems/framework/event_collection.h"

#include <utility>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class PublishEvent<double>;
template class PublishEvent<AutoDiffXd>;

template class DiscreteUpdateEvent<double>;
template class DiscreteUpdateEvent<AutoDiffXd>;

template class UnrestrictedUpdateEvent<double>;
template class UnrestrictedUpdateEvent<AutoDiffXd>;

template class DiagramEventCollection<PublishEvent<double>>;
template class DiagramEventCollection<DiscreteUpdateEvent<double>>;
template class DiagramEventCollection<UnrestrictedUpdateEvent<double>>;

template class LeafEventCollection<PublishEvent<double>>;
template class LeafEventCollection<DiscreteUpdateEvent<double>>;
template class LeafEventCollection<UnrestrictedUpdateEvent<double>>;

template class LeafCombinedEventCollection<double>;

}  // namespace systems
}  // namespace drake
