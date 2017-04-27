#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template struct StepInfo<double>;

template class Context<double>;

}  // namespace systems
}  // namespace drake
