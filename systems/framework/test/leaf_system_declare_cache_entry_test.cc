#include <gtest/gtest.h>

#include "drake/systems/framework/test/public_leaf_system.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(LeafSystemDeclareCacheEntryTest, AcceptanceTest) {
  test::PublicLeafSystem<double> dut;
  dut.DeclareCacheEntry<std::string>(
      "foo_port", std::string(),
      [](const Context<double>&, std::string* value) {
        *value = "foo";
      });
}

}  // namespace
}  // namespace systems
}  // namespace drake
