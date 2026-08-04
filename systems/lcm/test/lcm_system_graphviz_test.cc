#include "drake/systems/lcm/lcm_system_graphviz.h"

#include <string>
#include <vector>

#include "absl/strings/str_split.h"
#include <fmt/ranges.h>
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

GTEST_TEST(LcmSystemGraphvizTest, TwoDifferentPublish) {
  const DrakeLcm alpha;
  const DrakeLcm bravo;
  std::vector<std::string> alpha_graphviz;
  std::vector<std::string> bravo_graphviz;
  {
    LcmSystemGraphviz dut(alpha, "CHANNEL", nullptr, true, false);
    dut.DecorateParams(Params{.node_id = "alpha"});
    const Result result = dut.DecorateResult(Result{});
    alpha_graphviz =
        absl::StrSplit(fmt::format("{}", fmt::join(result.fragments, "")), " ");
  }
  {
    LcmSystemGraphviz dut(bravo, "CHANNEL", nullptr, true, false);
    dut.DecorateParams(Params{.node_id = "bravo"});
    const Result result = dut.DecorateResult(Result{});
    bravo_graphviz =
        absl::StrSplit(fmt::format("{}", fmt::join(result.fragments, "")), " ");
  }
  // Confirm that the destination node (after the "->") differs for the two
  // edges, since each one should be pointing to a different interface.
  EXPECT_EQ(alpha_graphviz.at(0), "alpha:e");
  EXPECT_EQ(bravo_graphviz.at(0), "bravo:e");
  EXPECT_EQ(alpha_graphviz.at(1), "->");
  EXPECT_EQ(bravo_graphviz.at(1), "->");
  EXPECT_THAT(alpha_graphviz.at(2), testing::StartsWith("drakelcm"));
  EXPECT_THAT(bravo_graphviz.at(2), testing::StartsWith("drakelcm"));
  EXPECT_NE(alpha_graphviz.at(2), bravo_graphviz.at(2));
}

}  // namespace
}  // namespace internal
}  // namespace lcm
}  // namespace systems
}  // namespace drake
