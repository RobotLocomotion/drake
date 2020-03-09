#include "drake/examples/hsr/hsr_world.h"

#include <gtest/gtest.h>

namespace drake {
namespace examples {
namespace hsr {
namespace {

GTEST_TEST(HsrWorldTest, ConstructionTest) {
  const std::string kConfigFile = "test";
  HsrWorld<double> hsr_world(kConfigFile);
  (void)hsr_world;
}

}  // namespace
}  // namespace hsr
}  // namespace examples
}  // namespace drake
