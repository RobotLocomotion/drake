#include "drake/geometry/meshcat_internal.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(MeshcatInternalTest, GetMeshcatStaticResource) {
  // This matches the list of URLs in the API doc.
  const std::vector<const char*> urls{
      "/",           "/favicon.ico",  "/index.html", "/meshcat.html",
      "/meshcat.js", "/stats.min.js",
  };
  for (const auto& url : urls) {
    SCOPED_TRACE(fmt::format("url = {}", url));
    const std::optional<std::string_view> result =
        GetMeshcatStaticResource(url);
    ASSERT_TRUE(result);
    EXPECT_FALSE(result->empty());
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
