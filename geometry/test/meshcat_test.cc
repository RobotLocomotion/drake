#include "drake/geometry/meshcat.h"

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

using ::testing::HasSubstr;

GTEST_TEST(MeshcatTest, TestHttp) {
  drake::geometry::Meshcat meshcat;
  // Note: The server doesn't respect all requests; unfortunately we can't use
  // curl --head and wget --spider nor curl --range to avoid downloading the
  // full file.
  EXPECT_EQ(system(fmt::format("curl -o /dev/null --silent {}/index.html",
                               meshcat.web_url())
                       .c_str()),
            0);
  EXPECT_EQ(system(fmt::format("curl -o /dev/null --silent {}/main.min.js",
                               meshcat.web_url())
                       .c_str()),
            0);
  EXPECT_EQ(system(fmt::format("curl -o /dev/null --silent {}/favicon.ico",
                               meshcat.web_url())
                       .c_str()),
            0);
}

GTEST_TEST(MeshcatTest, ConstructMultiple) {
  drake::geometry::Meshcat meshcat;
  drake::geometry::Meshcat meshcat2;

  EXPECT_THAT(meshcat.web_url(), HasSubstr("http://localhost:"));
  EXPECT_THAT(meshcat.ws_url(), HasSubstr("ws://localhost:"));
  EXPECT_THAT(meshcat2.web_url(), HasSubstr("http://localhost:"));
  EXPECT_THAT(meshcat2.ws_url(), HasSubstr("ws://localhost:"));
  EXPECT_NE(meshcat.web_url(), meshcat2.web_url());
}

void CheckCommand(const drake::geometry::Meshcat& meshcat, int message_num,
                  const std::string& desired_command_json) {
  EXPECT_EQ(
      system(fmt::format(
          "python3 geometry/test/meshcat_websocket_client.py '{}' {} '{}'",
          meshcat.ws_url(), message_num, desired_command_json).c_str()),
      0);
}

GTEST_TEST(MeshcatTest, SetProperty) {
  drake::geometry::Meshcat meshcat;
  meshcat.SetProperty("/Background", "visible", false);
  CheckCommand(meshcat, 1, "{"
              "\"property\":\"visible\", "
              "\"value\":false, "
              "\"path\":\"/Background\", "
              "\"type\":\"set_property\" "
              "}");
  meshcat.SetProperty("/Grid", "visible", false);
  // Note: The order of the messages is due to "/Background" < "/Grid" in the
  // std::map, not due to the order that SetProperty was called.
  CheckCommand(meshcat, 1, "{"
              "\"property\":\"visible\", "
              "\"value\":false, "
              "\"path\":\"/Background\", "
              "\"type\":\"set_property\" "
              "}");
  CheckCommand(meshcat, 2, "{"
              "\"property\":\"visible\", "
              "\"value\":false, "
              "\"path\":\"/Grid\", "
              "\"type\":\"set_property\" "
              "}");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
