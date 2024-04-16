#include "drake/multibody/parsing/detail_instanced_name.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

// TODO(rpoyner-tri): replace explicit spaceship implementation with compiler
// default once macOS Ventura support ends. Once it is gone, this test (and
// perhaps this whole file) can go too.
GTEST_TEST(InstancedNameTest, ExplicitSpaceship) {
  InstancedName empty;
  InstancedName global_a{{}, "a"};
  InstancedName global_b{{}, "b"};
  InstancedName one_a{ModelInstanceIndex(1), "a"};
  InstancedName one_b{ModelInstanceIndex(1), "b"};
  InstancedName two_a{ModelInstanceIndex(2), "a"};
  InstancedName two_b{ModelInstanceIndex(2), "b"};

  std::vector ordered{
    empty, global_a, global_b, one_a, one_b, two_a, two_b};

  for (int ii = 0; ii < ssize(ordered); ++ii) {
    for (int jj = 0; jj < ssize(ordered); ++jj) {
      SCOPED_TRACE(fmt::format("compare at ({}, {})", ii, jj));
      EXPECT_EQ(ordered[ii] <=> ordered[jj], ii <=> jj);
    }
  }
}

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
