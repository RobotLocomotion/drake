#include "drake/systems/analysis/simulator.h"

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

template class Simulator<double>;

// TODO(sherm1) Just need some code so the library will get built.
namespace {
DRAKESYSTEMANALYSIS_EXPORT int ignore_this_thing() { return 0; }
}

}  // namespace systems
}  // namespace drake