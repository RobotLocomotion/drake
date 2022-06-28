#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(RenderEngineGltfClientParams, GetUrl) {
  const std::string base_url{"127.0.0.1"};
  const int port{8000};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = false;

  {
    /* Test if the trailing slashes and the leading slashes are pruned
     correctly. */
    std::vector<std::pair<std::string, std::string>> valid_configs{
        {{base_url, render_endpoint},
         {base_url + "/", render_endpoint},
         {base_url + "/", render_endpoint},
         {base_url + "/", "/" + render_endpoint},
         {base_url + "///", "///" + render_endpoint}}};
    for (const auto& iter : valid_configs) {
      RenderEngineGltfClientParams params{std::nullopt, iter.first, port,
                                          iter.second,  verbose,    no_cleanup};
      EXPECT_EQ(params.GetUrl(), base_url + "/" + render_endpoint);
    }
  }

  {
    /* Test if the leading slashes in `base_url` and the trailing slashes in
     `render_endpoint` are kept as-is. Also, empty endpoint is allowed. */
    std::vector<std::pair<std::string, std::string>> still_valid_configs{
        {{base_url, ""},
         {"/" + base_url, render_endpoint},
         {base_url, render_endpoint + "/"}}};
    for (const auto& iter : still_valid_configs) {
      RenderEngineGltfClientParams params{std::nullopt, iter.first, port,
                                          iter.second,  verbose,    no_cleanup};
      EXPECT_EQ(params.GetUrl(), iter.first + "/" + iter.second);
    }
  }

  {
    std::vector<std::pair<std::string, std::string>> invalid_configs{
        {{base_url, "/"},
         {"/", render_endpoint},
         {base_url, "///"},
         {"///", render_endpoint},
         {"///", "///"}}};
    // Provided base_url may not end with slash.
    for (const auto& iter : invalid_configs) {
      RenderEngineGltfClientParams params{std::nullopt, iter.first, port,
                                          iter.second,  verbose,    no_cleanup};
      DRAKE_EXPECT_THROWS_MESSAGE(
          params.GetUrl(),
          "RenderEngineGltfClientParams: invalid base_url or render_endpoint "
          "is provided that contains only `/`.");
    }
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
