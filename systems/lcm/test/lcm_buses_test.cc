#include "drake/systems/lcm/lcm_buses.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm.h"

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;

namespace drake {
namespace systems {
namespace lcm {
namespace {

// A basic acceptance test.
GTEST_TEST(LcmBuses, Basic) {
  // Start with an empty map.
  LcmBuses dut;
  EXPECT_EQ(dut.size(), 0);
  EXPECT_THROW(dut.Find("foo", "foo"), std::exception);
  EXPECT_EQ(dut.GetAllBusNames().size(), 0);

  // Add a single bus.
  DrakeLcm lcm;
  dut.Add("default", &lcm);
  EXPECT_EQ(dut.size(), 1);
  EXPECT_NO_THROW(dut.Find("Basic test", "default"));
  EXPECT_EQ(dut.GetAllBusNames(), std::vector<std::string_view>{"default"});

  // Re-adding the same name will throw.
  EXPECT_THROW(dut.Add("default", &lcm), std::exception);

  // Adding nullptr will throw.
  EXPECT_THROW(dut.Add("nullptr", nullptr), std::exception);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
