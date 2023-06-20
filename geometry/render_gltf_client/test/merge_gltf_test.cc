#include <drake_vendor/nlohmann/json.hpp>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {
namespace {

using nlohmann::json;

// Temporary smoke test to show that json has been properly included. This will
// be replaced when the gltf merging code has been implemented.
GTEST_TEST(JsonTest, Smoke) {
  json source;
  EXPECT_FALSE(source.contains("data"));
  json& data = source["data"];
  data.push_back(10);
  EXPECT_TRUE(data.is_array());
  json& value = data.back();
  EXPECT_TRUE(value.is_number());
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
