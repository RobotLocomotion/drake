#include "drake/systems/analysis/simulator_status.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

GTEST_TEST(SimulatorStatusTest, ConstructionAndRetrieval) {
  const SystemBase* dummy = reinterpret_cast<const SystemBase*>(0x1234);
  SimulatorStatus status(1.25);  // Always constructed with a boundary_time.

  // Default is that the status reached the boundary time.
  EXPECT_EQ(status.boundary_time(), 1.25);
  EXPECT_EQ(status.return_time(), status.boundary_time());
  EXPECT_EQ(status.reason(), SimulatorStatus::kReachedBoundaryTime);
  EXPECT_EQ(status.system(), nullptr);
  EXPECT_EQ(status.message(), std::string());

  status.SetReachedTermination(1., dummy, "Hello there");
  EXPECT_EQ(status.boundary_time(), 1.25);
  EXPECT_EQ(status.return_time(), 1.);
  EXPECT_EQ(status.reason(), SimulatorStatus::kReachedTerminationCondition);
  EXPECT_EQ(status.system(), dummy);
  EXPECT_EQ(status.message(), std::string("Hello there"));
}

GTEST_TEST(SimulatorStatusTest, CopyConstruction) {
  const SystemBase* dummy = reinterpret_cast<const SystemBase*>(0x1234);
  SimulatorStatus status(1.0);
  status.SetEventHandlerFailed(0.5, dummy, "test message");

  SimulatorStatus copy_status(status);
  EXPECT_TRUE(copy_status.IsIdenticalStatus(status));
}

}  // namespace
}  // namespace systems
}  // namespace drake
