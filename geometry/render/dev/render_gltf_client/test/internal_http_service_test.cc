#include "drake/geometry/render/dev/render_gltf_client/internal_http_service.h"

#include <fstream>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {
namespace {

namespace fs = drake::filesystem;

// A concrete implementation of HttpService that does nothing.
class EmptyService : public HttpService {
 public:
  EmptyService() : HttpService() {}

  HttpResponse PostForm(
      const std::string& /* temp_directory */, const std::string& url,
      int /* port */, const std::string& endpoint,
      const std::map<std::string, std::string>& /* data_fields */,
      const std::map<std::string,
                     std::pair<std::string, std::optional<std::string>>>&
          file_fields,
      bool /* verbose */ = false) override {
    ThrowIfUrlInvalid(url);
    ThrowIfEndpointInvalid(endpoint);
    ThrowIfFilesMissing(file_fields);
    HttpResponse ret;
    ret.http_code = 200;
    return ret;
  }
};

GTEST_TEST(HttpService, ThrowIfUrlInvalid) {
  const std::string localhost{"127.0.0.1"};
  const EmptyService es;

  // A valid url should not raise.
  DRAKE_EXPECT_NO_THROW(es.ThrowIfUrlInvalid(localhost));

  // An empty url should raise.
  DRAKE_EXPECT_THROWS_MESSAGE(es.ThrowIfUrlInvalid(""),
                              "HttpService: url parameter may not be empty.");

  // A url with a '/' at the end should raise.
  DRAKE_EXPECT_THROWS_MESSAGE(es.ThrowIfUrlInvalid(localhost + "/"),
                              "HttpService: url may not end with '/'.");
}

GTEST_TEST(HttpService, ThrowIfEndpointInvalid) {
  const EmptyService es;

  {
    // Valid endpoints should not raise.
    DRAKE_EXPECT_NO_THROW(es.ThrowIfEndpointInvalid("render"));
    // Empty string implies route to /.
    DRAKE_EXPECT_NO_THROW(es.ThrowIfEndpointInvalid(""));
    // Interior / is allowed.
    DRAKE_EXPECT_NO_THROW(es.ThrowIfEndpointInvalid("render/scene"));
  }

  {
    // Invalid endpoints should raise.
    const std::string exc_message =
        "Provided endpoint='{}' is not valid, it may not start or end with a "
        "'/'.";
    const std::vector<std::string> bad_endpoints{"/",
                                                 "//",
                                                 "/render",
                                                 "render/",
                                                 "/render/",
                                                 "/render/scene",
                                                 "/render/scene/",
                                                 "render/scene/"};
    for (const auto& endpoint : bad_endpoints) {
      DRAKE_EXPECT_THROWS_MESSAGE(es.ThrowIfEndpointInvalid(endpoint),
                                  fmt::format(exc_message, endpoint));
    }
  }
}

GTEST_TEST(HttpService, ThrowIfFilesMissing) {
  // Create an EmptyService and some files to test with.
  const auto temp_dir = drake::temp_directory();
  const EmptyService es;

  const auto test_txt_path = (fs::path(temp_dir) / "test.txt").string();
  std::ofstream test_txt{test_txt_path};
  test_txt << "test!\n";
  test_txt.close();

  const auto fake_jpg_path = (fs::path(temp_dir) / "fake.jpg").string();
  std::ofstream fake_jpg{fake_jpg_path};
  fake_jpg << "not really a jpg!\n";
  fake_jpg.close();

  {
    // No exception should be thrown if all files are present.
    DRAKE_EXPECT_NO_THROW(es.ThrowIfFilesMissing({
        {"test", {test_txt_path, std::nullopt}},
        {"image", {fake_jpg_path, "image/jpeg"}},
    }));
  }

  // Some file paths that do not exist for testing.
  const std::string missing_1_key = "missing_1";
  const std::pair<std::string, std::optional<std::string>> missing_1_value = {
      "/unlikely/to/exist.file_extension", std::nullopt};
  const std::string missing_1_desc{
      "missing_1='/unlikely/to/exist.file_extension'"};
  ASSERT_FALSE(fs::is_regular_file(missing_1_value.first));

  const std::string missing_2_key = "missing_2";
  const std::pair<std::string, std::optional<std::string>> missing_2_value = {
      "/this/is/not/a.real_file", std::nullopt};
  const std::string missing_2_desc{"missing_2='/this/is/not/a.real_file'"};
  ASSERT_FALSE(fs::is_regular_file(missing_2_value.first));

  // The exception message prefix.
  const std::string prefix = "Provided file fields had missing file\\(s\\): ";
  // Since it is a map, order can change -- only support building regex for 2.
  auto make_regex = [&prefix](const std::string& p1, const std::string& p2) {
    return fmt::format("{0}(({1}, {2})|({2}, {1}))\\.", prefix, p1, p2);
  };

  {
    // Exception thrown: one file, does not exist.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.ThrowIfFilesMissing({{missing_1_key, missing_1_value}}),
        prefix + missing_1_desc + "\\.");
  }

  {
    // Exception thrown: multiple files, none exist.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.ThrowIfFilesMissing({{missing_1_key, missing_1_value},
                                {missing_2_key, missing_2_value}}),
        make_regex(missing_1_desc, missing_2_desc));
  }

  {
    // Exception thrown: one file exists, the other does not.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.ThrowIfFilesMissing({{"test", {test_txt_path, std::nullopt}},
                                {missing_1_key, missing_1_value}}),
        prefix + missing_1_desc + "\\.");
  }

  {
    // Exception thrown: multiple files exist, multiple do not.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.ThrowIfFilesMissing({{"test", {test_txt_path, std::nullopt}},
                                {missing_1_key, missing_1_value},
                                {"image", {fake_jpg_path, "image/jpeg"}},
                                {missing_2_key, missing_2_value}}),
        make_regex(missing_1_desc, missing_2_desc));
  }

  // 3 deletions: 2 files + 1 folder.
  EXPECT_EQ(fs::remove_all(temp_dir), 3);
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
