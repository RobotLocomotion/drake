#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

template struct DiscreteEvent<double>;

template struct UpdateActions<double>;

template class System<double>;

}  // namespace systems
}  // namespace drake
