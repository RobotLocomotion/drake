#include "planning/path_planning_errors.h"

#include <gtest/gtest.h>

namespace anzu {
namespace planning {
namespace {

GTEST_TEST(PathPlanningErrorsTest, Empty) {
  PathPlanningErrors dut;
  EXPECT_TRUE(dut.empty());
  EXPECT_FALSE(dut.contains(PathPlanningError::kTimeExceeded));
  EXPECT_EQ(fmt::to_string(dut), "{}");

  PathPlanningErrors copy(dut);
  EXPECT_TRUE(copy.empty());
}

GTEST_TEST(PathPlanningErrorsTest, Single) {
  PathPlanningErrors dut(PathPlanningError::kTimeExceeded);
  EXPECT_FALSE(dut.empty());
  EXPECT_TRUE(dut.contains(PathPlanningError::kTimeExceeded));
  EXPECT_FALSE(dut.contains(PathPlanningError::kCannotFindPath));
  EXPECT_EQ(fmt::to_string(dut), "{kTimeExceeded}");

  PathPlanningErrors copy(dut);
  EXPECT_FALSE(copy.empty());
}

GTEST_TEST(PathPlanningErrorsTest, Insert) {
  PathPlanningErrors dut;
  EXPECT_TRUE(dut.empty());
  PathPlanningErrors copy(dut);
  EXPECT_TRUE(copy.empty());

  dut.insert(PathPlanningError::kTimeExceeded);
  EXPECT_FALSE(dut.empty());
  EXPECT_TRUE(dut.contains(PathPlanningError::kTimeExceeded));
  EXPECT_FALSE(dut.contains(PathPlanningError::kCannotFindPath));
  EXPECT_FALSE(dut.contains(PathPlanningError::kNoValidGoal));
  EXPECT_TRUE(copy.empty());

  dut.insert(PathPlanningError::kCannotFindPath);
  EXPECT_FALSE(dut.empty());
  EXPECT_TRUE(dut.contains(PathPlanningError::kTimeExceeded));
  EXPECT_TRUE(dut.contains(PathPlanningError::kCannotFindPath));
  EXPECT_FALSE(dut.contains(PathPlanningError::kNoValidGoal));
  EXPECT_TRUE(copy.empty());
  EXPECT_EQ(fmt::to_string(dut), "{kCannotFindPath, kTimeExceeded}");

  copy = dut;
  EXPECT_TRUE(copy.contains(PathPlanningError::kTimeExceeded));
  EXPECT_TRUE(copy.contains(PathPlanningError::kCannotFindPath));
  EXPECT_FALSE(copy.contains(PathPlanningError::kNoValidGoal));
}

}  // namespace
}  // namespace planning
}  // namespace anzu
