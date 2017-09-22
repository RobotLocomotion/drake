#include "drake/systems/framework/leaf_system.h"

#include "drake/common/symbolic.h"

namespace drake {
namespace systems {

template class LeafSystem<double>;
template class LeafSystem<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
