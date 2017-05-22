#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template struct PeriodicEvent<double>;

template class LeafOutputPort<double>;
template class LeafOutputPort<AutoDiffXd>;
template class LeafOutputPort<symbolic::Expression>;

template class LeafSystem<double>;

}  // namespace systems
}  // namespace drake
