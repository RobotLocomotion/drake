#include "drake/multibody/parsing/package_map.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Simply confirm that the deprecated class can be instantiated.
GTEST_TEST(PackageMapTest, Deprecated) {
  drake::multibody::PackageMap map;
  // The drake and drake_models packages.
  EXPECT_EQ(map.size(), 2);
}
#pragma GCC diagnostic pop
}  // namespace
}  // namespace multibody
}  // namespace drake
