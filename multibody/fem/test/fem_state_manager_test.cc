#include "drake/multibody/fem/fem_state_manager.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

// Verifies that FemStateManager can declare discrete state and cache entry.
GTEST_TEST(FemStateManagerTest, DeclareDiscreteStateAndCache) {
  FemStateManager<double> fem_state_manager;
  fem_state_manager.DeclareDiscreteState(1);
  const double model_data = 0.5;
  fem_state_manager.DeclareCacheEntry(
      "dummy_data",
      systems::ValueProducer(model_data, &systems::ValueProducer::NoopCalc));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
