#include "drake/systems/lcm/lcm_config_functions.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm_params.h"

using drake::lcm::DrakeLcmParams;
using drake::systems::DiagramBuilder;

namespace drake {
namespace systems {
namespace lcm {
namespace {

// A basic acceptance test.
GTEST_TEST(LcmConfigFunctionsTest, Basic) {
  // We'll test using two buses.
  const std::map<std::string, DrakeLcmParams> lcm_buses{
    {"foo", {"memq://1"}},
    {"bar", {"memq://2"}},
  };

  // Invoke the device under test.
  DiagramBuilder<double> builder;
  auto name_to_interface = ApplyLcmBusConfig(lcm_buses, &builder);
  ASSERT_EQ(name_to_interface.size(), 2);
  auto diagram = builder.Build();

  // Check its results.
  auto* interface1 = name_to_interface.Find("Basic test", "foo");
  auto* interface2 = name_to_interface.Find("Basic test", "bar");
  ASSERT_NE(interface1, nullptr);
  ASSERT_NE(interface2, nullptr);

  // Sanity check the names.
  const std::string drawing = diagram->GetGraphvizString();
  EXPECT_THAT(drawing, ::testing::HasSubstr("bus_name=foo"));
  EXPECT_THAT(drawing, ::testing::HasSubstr("bus_name=bar"));
  EXPECT_THAT(drawing, ::testing::HasSubstr("lcm_url=memq://1"));
  EXPECT_THAT(drawing, ::testing::HasSubstr("lcm_url=memq://2"));

  // TODO(jwnimmer-tri) It might be worth trying to pub/sub some messages
  // here to confirm the wiring? But the risk of a bug seems low, at least
  // for now. We can revisit later, if we ever have doubts.
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
