#include "drake/geometry/meshcat.h"

#include <cstdlib>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace geometry {
namespace {

using ::testing::HasSubstr;

// A small wrapper around std::system to ensure correct argument passing.
int SystemCall(const std::vector<std::string>& argv) {
  std::string command;
  for (const std::string& arg : argv) {
    // Note: Can't use ASSERT_THAT inside this subroutine.
    EXPECT_THAT(arg, ::testing::Not(::testing::HasSubstr("'")));
    command = std::move(command) + "'" + arg + "' ";
  }
  return std::system(command.c_str());
}

GTEST_TEST(MeshcatTest, TestHttp) {
  drake::geometry::Meshcat meshcat;
  // Note: The server doesn't respect all requests; unfortunately we can't use
  // curl --head and wget --spider nor curl --range to avoid downloading the
  // full file.
  EXPECT_EQ(SystemCall({"/usr/bin/curl", "-o", "/dev/null", "--silent",
                        meshcat.web_url() + "/index.html"}),
            0);
  EXPECT_EQ(SystemCall({"/usr/bin/curl", "-o", "/dev/null", "--silent",
                        meshcat.web_url() + "/main.min.js"}),
            0);
  EXPECT_EQ(SystemCall({"/usr/bin/curl", "-o", "/dev/null", "--silent",
                        meshcat.web_url() + "/favicon.ico"}),
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
  EXPECT_EQ(SystemCall(
                {FindResourceOrThrow("drake/geometry/meshcat_websocket_client"),
                 meshcat.ws_url(), std::to_string(message_num),
                 desired_command_json}),
            0);
}

GTEST_TEST(MeshcatTest, SetProperty) {
  drake::geometry::Meshcat meshcat;
  meshcat.SetProperty("/Background", "visible", false);
  CheckCommand(meshcat, 1, R"""({
              "property": "visible",
              "value": false,
              "path": "/Background",
              "type": "set_property"
              })""");
  meshcat.SetProperty("/Grid", "visible", false);
  // Note: The order of the messages is due to "/Background" < "/Grid" in the
  // std::map, not due to the order that SetProperty was called.
  CheckCommand(meshcat, 1, R"""({
              "property": "visible",
              "value": false,
              "path": "/Background",
              "type": "set_property"
              })""");
  CheckCommand(meshcat, 2, R"""({
              "property": "visible",
              "value": false,
              "path": "/Grid",
              "type": "set_property"
              })""");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
