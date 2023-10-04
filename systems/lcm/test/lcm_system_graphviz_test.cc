#include "drake/systems/lcm/lcm_system_graphviz.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace systems {
namespace lcm {
namespace internal {
namespace {

using drake::lcm::DrakeLcm;

using Params = systems::SystemBase::GraphvizFragmentParams;
using Result = systems::SystemBase::GraphvizFragment;

GTEST_TEST(LcmSystemGraphvizTest, Publish) {
  const DrakeLcm lcm;
  const std::string channel = "SIGNAL";
  const std::type_info& message_type = typeid(lcmt_drake_signal);
  const bool publish = true;
  const bool subscribe = false;
  LcmSystemGraphviz dut(lcm, channel, &message_type, publish, subscribe);

  const Params params_orig{.node_id = "foo",
                           .header_lines = {"existing header"}};
  const Params params = dut.DecorateParams(params_orig);
  EXPECT_EQ(params.node_id, "foo");
  EXPECT_THAT(params.header_lines,
              testing::ElementsAre(
                  // Prior item.
                  "existing header",
                  // Newly-added items.
                  "channel=SIGNAL", "type=lcmt_drake_signal"));

  const Result result_orig{.input_ports = {"foo:u0"},
                           .output_ports = {"foo:y0"},
                           .fragments = {"existing fragment"}};
  const Result result = dut.DecorateResult(Result{result_orig});
  EXPECT_EQ(result.input_ports, result_orig.input_ports);
  EXPECT_EQ(result.output_ports, result_orig.output_ports);
  ASSERT_GE(result.fragments.size(), 2);
  EXPECT_EQ(result.fragments.front(), "existing fragment");
  EXPECT_THAT(result.fragments.back(), testing::HasSubstr("-> drakelcm"));
  EXPECT_THAT(result.fragments.back(),
              testing::Not(testing::HasSubstr("out ->")));
}

GTEST_TEST(LcmSystemGraphvizTest, Subscribe) {
  const DrakeLcm lcm;
  const std::string channel = "SIGNAL";
  const std::type_info& message_type = typeid(lcmt_drake_signal);
  const bool publish = false;
  const bool subscribe = true;
  LcmSystemGraphviz dut(lcm, channel, &message_type, publish, subscribe);

  const Params params_orig{.node_id = "foo",
                           .header_lines = {"existing header"}};
  const Params params = dut.DecorateParams(params_orig);
  EXPECT_EQ(params.node_id, "foo");
  EXPECT_THAT(params.header_lines,
              testing::ElementsAre(
                  // Prior item.
                  "existing header",
                  // Newly-added items.
                  "channel=SIGNAL", "type=lcmt_drake_signal"));

  const Result result_orig{.input_ports = {"foo:u0"},
                           .output_ports = {"foo:y0"},
                           .fragments = {"existing fragment"}};
  const Result result = dut.DecorateResult(Result{result_orig});
  EXPECT_EQ(result.input_ports, result_orig.input_ports);
  EXPECT_EQ(result.output_ports, result_orig.output_ports);
  ASSERT_GE(result.fragments.size(), 2);
  EXPECT_EQ(result.fragments.front(), "existing fragment");
  EXPECT_THAT(result.fragments.back(), testing::HasSubstr("out ->"));
  EXPECT_THAT(result.fragments.back(),
              testing::Not(testing::HasSubstr("-> drakelcm")));
}

GTEST_TEST(LcmSystemGraphvizTest, Neither) {
  const DrakeLcm lcm;
  const std::string channel;
  const std::type_info* message_type = nullptr;
  const bool publish = false;
  const bool subscribe = false;
  LcmSystemGraphviz dut(lcm, channel, message_type, publish, subscribe);

  const Params params_orig{.node_id = "foo",
                           .header_lines = {"existing header"}};
  const Params params = dut.DecorateParams(params_orig);
  EXPECT_EQ(params.node_id, "foo");
  EXPECT_EQ(params.header_lines, params_orig.header_lines);

  const Result result_orig{.input_ports = {"foo:u0"},
                           .output_ports = {"foo:y0"},
                           .fragments = {"existing fragment"}};
  const Result result = dut.DecorateResult(Result{result_orig});
  EXPECT_EQ(result.input_ports, result_orig.input_ports);
  EXPECT_EQ(result.output_ports, result_orig.output_ports);
  EXPECT_EQ(result.fragments, result_orig.fragments);
}

}  // namespace
}  // namespace internal
}  // namespace lcm
}  // namespace systems
}  // namespace drake
