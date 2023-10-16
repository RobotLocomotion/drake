#include "drake/geometry/meshcat_graphviz.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Params = systems::SystemBase::GraphvizFragmentParams;
using Result = systems::SystemBase::GraphvizFragment;

GTEST_TEST(MeshcatGraphvizTest, Publish) {
  MeshcatGraphviz dut("path_foo", /* subscribe = */ false);

  const Params params_orig{.node_id = "node_bar",
                           .header_lines = {"existing header"}};
  const Params params = dut.DecorateParams(params_orig);
  EXPECT_EQ(params.node_id, "node_bar");
  EXPECT_THAT(params.header_lines,
              testing::ElementsAre("existing header", "path=/drake/path_foo"));

  const Result result_orig{.input_ports = {"node_bar:u0"},
                           .output_ports = {"node_bar:y0"},
                           .fragments = {"existing fragment"}};
  const Result result = dut.DecorateResult(Result{result_orig});
  EXPECT_EQ(result.input_ports, result_orig.input_ports);
  EXPECT_EQ(result.output_ports, result_orig.output_ports);
  ASSERT_GE(result.fragments.size(), 2);
  EXPECT_EQ(result.fragments.front(), "existing fragment");
  EXPECT_THAT(result.fragments.back(), testing::HasSubstr("meshcat_in"));
  EXPECT_THAT(result.fragments.back(),
              testing::Not(testing::HasSubstr("meshcat_out")));
}

GTEST_TEST(MeshcatGraphvizTest, PublishAbsolutePath) {
  MeshcatGraphviz dut("/absolute", /* subscribe = */ false);

  const Params params = dut.DecorateParams(Params{});
  EXPECT_THAT(params.header_lines, testing::ElementsAre("path=/absolute"));
}

GTEST_TEST(MeshcatGraphvizTest, Subscribe) {
  MeshcatGraphviz dut(/* path = */ std::nullopt, /* subscribe = */ true);

  const Params params_orig{.node_id = "node_bar",
                           .header_lines = {"existing header"}};
  const Params params = dut.DecorateParams(params_orig);
  EXPECT_EQ(params.node_id, "node_bar");
  EXPECT_THAT(params.header_lines, testing::ElementsAre("existing header"));

  const Result result_orig{.input_ports = {"node_bar:u0"},
                           .output_ports = {"node_bar:y0"},
                           .fragments = {"existing fragment"}};
  const Result result = dut.DecorateResult(Result{result_orig});
  EXPECT_EQ(result.input_ports, result_orig.input_ports);
  EXPECT_EQ(result.output_ports, result_orig.output_ports);
  ASSERT_GE(result.fragments.size(), 2);
  EXPECT_EQ(result.fragments.front(), "existing fragment");
  EXPECT_THAT(result.fragments.back(), testing::HasSubstr("meshcat_out"));
  EXPECT_THAT(result.fragments.back(),
              testing::Not(testing::HasSubstr("meshcat_in")));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
