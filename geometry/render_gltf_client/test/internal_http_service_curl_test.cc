#include "drake/geometry/render_gltf_client/internal_http_service_curl.h"

#include <filesystem>
#include <fstream>
#include <string>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {
namespace {

namespace fs = std::filesystem;

// NOTE: we do not have a server, can only test failure scenarios.
GTEST_TEST(HttpServiceCurlTest, PostForm) {
  /* NOTE: do not use a URL / port that may be active...
   Use verbose=true (last parameter) to increase coverage via curl callbacks. */
  const std::string temp_dir = drake::temp_directory();
  const std::string url{"notawebsite:1234/render"};
  const bool verbose{true};
  HttpServiceCurl service;

  {
    /* Case 1: tries to setup temporary file for server writeback, but it
     already exists.  This test requires knowledge of
     http_service_curl.cc:NextTempFile to engineer. */
    auto temp_file_path = fs::path(temp_dir) / fmt::format("{:0>19}.curl", 1);
    std::ofstream temp_file{temp_file_path.string()};
    temp_file << "this file exists\n";
    temp_file.close();
    DRAKE_EXPECT_THROWS_MESSAGE(
        service.PostForm(temp_dir, url, {}, {}, verbose),
        fmt::format(".*refusing to overwrite temporary file '{}' that "
                    "already exists, please cleanup temporary directory '{}'.",
                    temp_file_path.string(), temp_dir));
    fs::remove(temp_file_path);
  }

  {
    // Case 2: cannot create temporary file without write permission.
    const auto orig_perms = fs::status(temp_dir).permissions();
    const auto all_write = fs::perms::owner_write | fs::perms::group_write |
                           fs::perms::others_write;
    fs::permissions(temp_dir, all_write, fs::perm_options::remove);
    DRAKE_EXPECT_THROWS_MESSAGE(
        service.PostForm(temp_dir, url, {}, {}, verbose),
        fmt::format(".*unable to open temporary file '{}.*\\.curl'.",
                    temp_dir));
    fs::permissions(temp_dir, orig_perms, fs::perm_options::replace);
  }

  {
    // Validate that the response indicates failure in absence of server.
    const auto res_1 = service.PostForm(temp_dir, url, {}, {}, verbose);
    EXPECT_FALSE(res_1.Good());
    EXPECT_FALSE(res_1.data_path.has_value());
    EXPECT_TRUE(res_1.service_error_message.has_value());
    EXPECT_THAT(res_1.service_error_message.value(),
                testing::MatchesRegex("Could.?n.t resolve host.?name"));

    /* We can also validate that form entries will be added, but the same error
     (Couldn't resolve host name) is expected. */
    // Two cases for files: with and without mime type.
    const auto test_json_path = (fs::path(temp_dir) / "test.json").string();
    const auto test_json_mime = "application/json";
    std::ofstream test_json{test_json_path};
    test_json << "{\"hello\": \"world\", \"alive\": true}\n";
    test_json.close();

    const auto test_binary_path = (fs::path(temp_dir) / "file.bin").string();
    const auto test_binary_mime = std::nullopt;
    std::ofstream test_binary{test_binary_path, std::ios::binary};
    test_binary << 111.111 << 222.222 << 333.333;
    test_binary.close();

    const auto res_2 =
        service.PostForm(temp_dir, url, {{"width", "640"}, {"height", "480"}},
                         {{"json", {test_json_path, test_json_mime}},
                          {"binary", {test_binary_path, test_binary_mime}}},
                         verbose);
    EXPECT_FALSE(res_2.Good());
    EXPECT_FALSE(res_2.data_path.has_value());
    EXPECT_TRUE(res_2.service_error_message.has_value());
    EXPECT_THAT(res_2.service_error_message.value(),
                testing::MatchesRegex("Could.?n.t resolve host.?name"));

    fs::remove(test_json_path);
    fs::remove(test_binary_path);
  }

  fs::remove(temp_dir);
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
