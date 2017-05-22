#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template struct PeriodicEvent<double>;

template class LeafSystem<double>;

}  // namespace systems
}  // namespace drake
