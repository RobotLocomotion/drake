#include "drake/systems/framework/event_collection.h"

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

template class LeafCompositeEventCollection<double>;
template class LeafCompositeEventCollection<AutoDiffXd>;

template class DiagramCompositeEventCollection<double>;
template class DiagramCompositeEventCollection<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
