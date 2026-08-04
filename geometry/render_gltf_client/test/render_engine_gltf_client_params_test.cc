#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(RenderEngineGltfClientParams, GetUrl) {
  struct TestData {
    std::string base_url;
    std::string render_endpoint;
    std::string expected_full_url;
  };
  // clang-format off
  std::vector<TestData> all_test_cases{{
      // Check that sandwiched slashes are added or removed correctly.
      {"127.0.0.1:8000",    "render",    "127.0.0.1:8000/render"},
      {"127.0.0.1:8000/",   "render",    "127.0.0.1:8000/render"},
      {"127.0.0.1:8000//",  "render",    "127.0.0.1:8000/render"},
      {"127.0.0.1:8000",    "/render",   "127.0.0.1:8000/render"},
      {"127.0.0.1:8000",    "//render",  "127.0.0.1:8000/render"},
      {"127.0.0.1:8000/",   "/render",   "127.0.0.1:8000/render"},
      {"127.0.0.1:8000//",  "//render",  "127.0.0.1:8000/render"},

      // Check that an empty (or vacuous) endpoint is allowed.
      {"127.0.0.1:8000",    "",          "127.0.0.1:8000/"},
      {"127.0.0.1:8000",    "/",         "127.0.0.1:8000/"},
      {"127.0.0.1:8000",    "//",        "127.0.0.1:8000/"},
      {"127.0.0.1:8000//",  "//",        "127.0.0.1:8000/"},

      // Check that non-sandwiched slashes in are kept as-is.
      {"127.0.0.1:8000",    "render/",   "127.0.0.1:8000/render/"},
      {"/127.0.0.1:8000",   "render",    "/127.0.0.1:8000/render"},
      {"http://host:8000",  "render",    "http://host:8000/render"},

      // A questionable base_url still produces the specified results.
      {"",             "",          "/"},
      {"///",          "///",       "/"},
      {"///",          "render",    "/render"},
  }};
  // clang-format on
  for (const auto& one_case : all_test_cases) {
    RenderEngineGltfClientParams dut;
    dut.base_url = one_case.base_url;
    dut.render_endpoint = one_case.render_endpoint;
    EXPECT_EQ(dut.GetUrl(), one_case.expected_full_url);
  }
}

GTEST_TEST(RenderEngineGltfClientParams, Serialization) {
  using Params = RenderEngineGltfClientParams;
  const Params original{
      .base_url = "http://hello",
      .render_endpoint = "world",
      .verbose = true,
      .cleanup = false,
  };
  const std::string yaml = yaml::SaveYamlString<Params>(original);
  const Params dut = yaml::LoadYamlString<Params>(yaml);
  EXPECT_EQ(dut.base_url, original.base_url);
  EXPECT_EQ(dut.render_endpoint, original.render_endpoint);
  EXPECT_EQ(dut.verbose, original.verbose);
  EXPECT_EQ(dut.cleanup, original.cleanup);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
