#include "drake/systems/analysis/simulator.h"

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

template class Simulator<double>;

}  // namespace systems
}  // namespace drake