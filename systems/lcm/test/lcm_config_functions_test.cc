#include "drake/systems/lcm/lcm_config_functions.h"

#include <map>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_params.h"
#include "drake/lcmt_drake_signal.hpp"

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeLcmParams;
using drake::lcm::Subscriber;
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

// Check the sunny-day case of looking up a bus_name.
GTEST_TEST(LcmConfigFunctionsFindOrCreateTest, FindBus) {
  DrakeLcm* const forced_result = nullptr;
  LcmBuses lcm_buses;
  DiagramBuilder<double> builder;
  const std::string description = "Find Bus";
  std::string bus_name = "special";
  DrakeLcm special;
  lcm_buses.Add(bus_name, &special);

  // We successfully look up the bus_name.
  DrakeLcmInterface* result = FindOrCreateLcmBus(
      forced_result, &lcm_buses, &builder, description, bus_name);
  EXPECT_EQ(result, &special);

  // It's an error to use a non-existent bus_name.
  bus_name = "missing";
  DRAKE_EXPECT_THROWS_MESSAGE(
      FindOrCreateLcmBus(forced_result, &lcm_buses, &builder, description,
                         bus_name),
      ".*missing.*does not exist.*");
}

// When the forced_result pointer is directly provided, then the lcm_buses and
// bus_name are ignored (i.e., nothing crashes)
GTEST_TEST(LcmConfigFunctionsFindOrCreateTest, IgnoredLcmBuses) {
  DrakeLcm forced_result;
  DiagramBuilder<double> builder;
  const std::string description = "Ignored Lcm Buses";
  const std::string bus_name = "will_be_ignored";

  DrakeLcmInterface* result = FindOrCreateLcmBus(
      &forced_result, nullptr /* lcm_buses */, &builder, description, bus_name);
  EXPECT_EQ(result, &forced_result);
  EXPECT_EQ(builder.GetSystems().size(), 0);

  // The same holds true even when an lcm_buses has been provided.
  const LcmBuses empty_lcm_buses;
  result = FindOrCreateLcmBus(&forced_result, &empty_lcm_buses, &builder,
                              description, bus_name);
  EXPECT_EQ(result, &forced_result);
  EXPECT_EQ(builder.GetSystems().size(), 0);
}

// When the forced_result and lcm_buses are both null, then the bus_name must
// be "default" and a new object will be created.
GTEST_TEST(LcmConfigFunctionsFindOrCreateTest, CreateNew) {
  DrakeLcm* const forced_result = nullptr;
  const LcmBuses* const lcm_buses = nullptr;
  DiagramBuilder<double> builder;
  const std::string description = "Create New";
  std::string bus_name = "default";

  // A new bus gets created.
  DrakeLcmInterface* result = FindOrCreateLcmBus(
      forced_result, lcm_buses, &builder, description, bus_name);
  EXPECT_NE(result, nullptr);
  EXPECT_EQ(builder.GetSystems().size(), 1);

  // It's an error to use a different bus_name.
  bus_name = "special";
  DRAKE_EXPECT_THROWS_MESSAGE(
      FindOrCreateLcmBus(forced_result, lcm_buses, &builder, description,
                         bus_name),
      ".*non-default.*special.*");
}

// Special handling for opting-out of LCM.
GTEST_TEST(LcmConfigFunctionsTest, Nulls) {
  const std::map<std::string, std::optional<DrakeLcmParams>> lcm_buses{
      {"foo", std::nullopt},
      {"bar", DrakeLcmParams{.lcm_url = LcmBuses::kLcmUrlMemqNull}},
  };

  // Invoke the device under test.
  //
  // Note that this test case only ever calls the overload of ApplyLcmBusConfig
  // that uses std::optional<>, but the other ApplyLcmBusConfig overload also
  // allows for passing kLcmUrlMemqNull to accomplish the same thing. We rely on
  // "glass-box" reasoning to know that both overloads share a common underlying
  // implementation, so we don't need to test them both.
  DiagramBuilder<double> builder;
  auto name_to_interface = ApplyLcmBusConfig(lcm_buses, &builder);
  ASSERT_EQ(name_to_interface.size(), 2);
  auto diagram = builder.Build();

  // Check its results.
  DrakeLcmInterface* interface1 = name_to_interface.Find("Nulls test", "foo");
  DrakeLcmInterface* interface2 = name_to_interface.Find("Nulls test", "bar");
  ASSERT_NE(interface1, nullptr);
  ASSERT_NE(interface2, nullptr);
  EXPECT_EQ(interface1->get_lcm_url(), "memq://null");
  EXPECT_EQ(interface2->get_lcm_url(), "memq://null");

  // Check that no unwanted systems were added.
  std::vector<std::string> names;
  for (const System<double>* system : diagram->GetSystems()) {
    names.push_back(system->get_name());
  }
  const std::vector<std::string> expected{
      // The two `SharedPtrSystem`s are the only things that get added.
      // Note that there are no `LcmInterfaceSystem`s anywhere here.
      "DrakeLcm(bus_name=foo)",
      "DrakeLcm(bus_name=bar)",
  };
  EXPECT_THAT(names, testing::UnorderedElementsAreArray(expected));

  // The interface is inert -- it does not pass messages.
  const std::string channel{"lcm_config_functions_test"};
  Subscriber<lcmt_drake_signal> subscriber1(interface1, channel);
  Subscriber<lcmt_drake_signal> subscriber2(interface2, channel);
  Publish(interface1, channel, lcmt_drake_signal{});
  Publish(interface2, channel, lcmt_drake_signal{});
  interface1->HandleSubscriptions(10);
  interface2->HandleSubscriptions(10);
  EXPECT_EQ(subscriber1.count(), 0);
  EXPECT_EQ(subscriber2.count(), 0);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
