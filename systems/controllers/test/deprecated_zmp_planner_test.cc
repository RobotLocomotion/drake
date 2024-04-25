#include <gtest/gtest.h>

#include "drake/systems/controllers/zmp_planner.h"

namespace drake {
namespace systems {
namespace controllers {
namespace {

GTEST_TEST(DeprecatedZmpPlannerTest, Simple) {
  ZmpPlanner dut;
  EXPECT_FALSE(dut.has_planned());
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
