#include "drake/geometry/meshcat_internal.h"

#include <gmock/gmock.h>
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

GTEST_TEST(MeshcatInternalTest, UuidGenerator) {
  UuidGenerator dut;
  std::string foo = dut.GenerateRandom();
  std::string bar = dut.GenerateRandom();
  EXPECT_NE(foo, bar);

  const std::string_view pattern =
      "[[:xdigit:]]{8,8}-"
      "[[:xdigit:]]{4,4}-"
      "[[:xdigit:]]{4,4}-"
      "[[:xdigit:]]{4,4}-"
      "[[:xdigit:]]{12,12}";
  EXPECT_THAT(foo, testing::MatchesRegex(pattern));
  EXPECT_THAT(bar, testing::MatchesRegex(pattern));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
