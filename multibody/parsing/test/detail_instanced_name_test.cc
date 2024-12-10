#include "drake/multibody/parsing/detail_instanced_name.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

GTEST_TEST(InstanceNameTest, ToString) {
  InstancedName empty;
  EXPECT_EQ(empty.to_string(), "[nullopt, ]");
  InstancedName global_a{{}, "a"};
  EXPECT_EQ(global_a.to_string(), "[nullopt, a]");
  InstancedName one_a{ModelInstanceIndex(1), "a"};
  EXPECT_EQ(one_a.to_string(), "[1, a]");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
