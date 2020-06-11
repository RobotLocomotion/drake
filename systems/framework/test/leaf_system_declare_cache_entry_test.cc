#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

class DummySystem : public LeafSystem<double> {};

GTEST_TEST(LeafSystemDeclareCacheEntryTest, AcceptanceTest) {
  DummySystem dut;
  dut.DeclareCacheEntry<std::string>(
      "foo_port", std::string(),
      [](const Context<double>&, std::string* value) {
        *value = "foo";
      });
}

}  // namespace
}  // namespace systems
}  // namespace drake
