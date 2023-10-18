// This file serves to check that the shims for the PackageMap in the
// drake::multibody::parsing namespace build as expected. When the stub is
// removed, the whole test can be removed.

#include <gtest/gtest.h>

#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace {
GTEST_TEST(PackageMapStub, Smoke) {
  auto map = drake::multibody::PackageMap::MakeEmpty();
  ASSERT_EQ(map.size(), 0);
}
}  // namespace
}  // namespace multibody
}  // namespace drake
