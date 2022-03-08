#include "drake/geometry/render/dev/http_service.h"

#include <fstream>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

namespace fs = drake::filesystem;

// A concrete implementation of HttpService that does nothing.
class EmptyService : public HttpService {
 public:
  EmptyService(const std::string& temp_directory, const std::string& url,
               int32_t port, bool verbose)
      : HttpService(temp_directory, url, port, verbose) {}

  EmptyService(const EmptyService& other) : HttpService(other) {}

  HttpResponse PostForm(
      const std::string& endpoint,
      const std::map<std::string, std::string>& /* data_fields */,
      const std::map<std::string,
                     std::pair<std::string, std::optional<std::string>>>&
          file_fields) override {
    ThrowIfEndpointInvalid(endpoint);
    ThrowIfFilesMissing(file_fields);
    HttpResponse ret;
    ret.http_code = 200;
    return ret;
  }

  std::unique_ptr<HttpService> DoClone() const override {
    return std::unique_ptr<EmptyService>(new EmptyService(*this));
  }
};

// A concrete derived-derived class that properly implements cloning.
class EmptyServiceGoodClone : public EmptyService {
 public:
  EmptyServiceGoodClone(const std::string& temp_directory,
                        const std::string& url, int32_t port, bool verbose)
      : EmptyService(temp_directory, url, port, verbose) {}

  EmptyServiceGoodClone(const EmptyServiceGoodClone& other)
      : EmptyService(other) {}

  std::unique_ptr<HttpService> DoClone() const override {
    return std::unique_ptr<EmptyServiceGoodClone>(
        new EmptyServiceGoodClone(*this));
  }
};

// A concrete derived-derived class that does *not* implement cloning.
class EmptyServiceBadClone : public EmptyService {
 public:
  EmptyServiceBadClone(const std::string& temp_directory,
                       const std::string& url, int32_t port, bool verbose)
      : EmptyService(temp_directory, url, port, verbose) {}

  EmptyServiceBadClone(const EmptyServiceBadClone& other)
      : EmptyService(other) {}
};

GTEST_TEST(HttpServiceTest, Constructor) {
  const std::string localhost{"127.0.0.1"};
  const int32_t port{8000};

  {
    // Validate normal construction works as expected.
    const auto temp_dir = drake::temp_directory();
    EmptyService es{temp_dir, localhost, port, false};
    EXPECT_EQ(temp_dir, es.temp_directory());
    EXPECT_EQ(localhost, es.url());
    EXPECT_EQ(port, es.port());
    EXPECT_EQ(false, es.verbose());
    fs::remove(temp_dir);
  }

  {
    // An empty url should raise.
    const auto temp_dir = drake::temp_directory();
    DRAKE_EXPECT_THROWS_MESSAGE(EmptyService(temp_dir, "", port, false),
                                "HttpService: url parameter may not be empty.");

    // A url with a '/' at the end should raise.
    DRAKE_EXPECT_THROWS_MESSAGE(
        EmptyService(temp_dir, localhost + "/", port, false),
        "HttpService: url may not end with '/'.");
    fs::remove(temp_dir);
  }
}

GTEST_TEST(HttpService, Clone) {
  // Create some parameters to use for testing clones.
  const auto temp_dir = drake::temp_directory();
  const std::string url{"0.0.0.0"};
  const int32_t port{8192};
  const bool verbose{true};
  auto verify_attributes = [&](const HttpService& service) {
    EXPECT_EQ(service.temp_directory(), temp_dir);
    EXPECT_EQ(service.url(), url);
    EXPECT_EQ(service.port(), port);
    EXPECT_EQ(service.verbose(), verbose);
  };

  {
    // Verify construction and cloning of EmptyService works.
    EmptyService es{temp_dir, url, port, verbose};
    verify_attributes(es);
    auto clone = es.Clone();
    verify_attributes(*clone);
  }

  {
    // Verify construction and cloning of EmptyServiceGoodClone works.
    EmptyServiceGoodClone esgc{temp_dir, url, port, verbose};
    verify_attributes(esgc);
    auto clone = esgc.Clone();
    verify_attributes(*clone);
  }

  {
    /* Verify that a derived class of a derived class that does not implement
     DoClone() throws an exception. */
    EmptyServiceBadClone esbc{temp_dir, url, port, verbose};
    verify_attributes(esbc);
    DRAKE_EXPECT_THROWS_MESSAGE(
        esbc.Clone(),
        "Error in cloning HttpService class of type.*EmptyServiceBadClone; the "
        "clone returns type .*EmptyService\\..*EmptyServiceBadClone::DoClone"
        "\\(\\) was probably not implemented");
  }

  fs::remove(temp_dir);
}

GTEST_TEST(HttpService, ThrowIfEndpointInvalid) {
  auto temp_dir = drake::temp_directory();
  EmptyService es{temp_dir, "127.0.0.1", 8000, false};

  {
    // Valid endpoint: no error.
    auto response = es.PostForm("render", {}, {});
    EXPECT_EQ(response.http_code, 200);

    // Valid endpoint: no error, empty implies route to /.
    response = es.PostForm("", {}, {});
    EXPECT_EQ(response.http_code, 200);

    // Valid endpoint: no error, interior / is allowed.
    response = es.PostForm("render/scene", {}, {});
    EXPECT_EQ(response.http_code, 200);
  }

  const std::string exc_message =
      "Provided endpoint='{}' is not valid, it may not start or end with a "
      "'/'.";
  {
    const std::vector<std::string> bad_endpoints{"/",
                                                 "//",
                                                 "/render",
                                                 "render/",
                                                 "/render/",
                                                 "/render/scene",
                                                 "/render/scene/",
                                                 "render/scene/"};
    for (const auto& endpoint : bad_endpoints) {
      DRAKE_EXPECT_THROWS_MESSAGE(es.PostForm(endpoint, {}, {}),
                                  fmt::format(exc_message, endpoint));
    }
  }
}

GTEST_TEST(HttpService, ThrowIfFilesMissing) {
  // Create an EmptyService and some files to test with.
  auto temp_dir = drake::temp_directory();
  EmptyService es{temp_dir, "127.0.0.1", 8000, true};

  auto test_txt_path = (fs::path(temp_dir) / "test.txt").string();
  std::ofstream test_txt{test_txt_path};
  test_txt << "test!\n";
  test_txt.close();

  auto fake_jpg_path = (fs::path(temp_dir) / "fake.jpg").string();
  std::ofstream fake_jpg{fake_jpg_path};
  fake_jpg << "not really a jpg!\n";
  fake_jpg.close();

  {
    // No exception should be thrown if all files are present.
    auto response = es.PostForm("upload", {},
                                {
                                    {"test", {test_txt_path, std::nullopt}},
                                    {"image", {fake_jpg_path, "image/jpeg"}},
                                });
    EXPECT_EQ(response.http_code, 200);
  }

  // Some file paths that do not exist for testing.
  const std::string missing_1_key = "missing_1";
  const std::pair<std::string, std::optional<std::string>> missing_1_value = {
      "/unlikely/to/exist.file_extension", std::nullopt};
  const std::string missing_1_desc{
      "missing_1='/unlikely/to/exist.file_extension'"};
  EXPECT_FALSE(fs::is_regular_file(missing_1_value.first));

  const std::string missing_2_key = "missing_2";
  const std::pair<std::string, std::optional<std::string>> missing_2_value = {
      "/this/is/not/a.real_file", std::nullopt};
  const std::string missing_2_desc{"missing_2='/this/is/not/a.real_file'"};
  EXPECT_FALSE(fs::is_regular_file(missing_2_value.first));

  // The exception message prefix.
  const std::string prefix = "Provided file fields had missing file\\(s\\): ";
  // Since it is a map, order can change -- only support building regex for 2.
  auto make_regex = [&prefix](const std::string& p1, const std::string& p2) {
    return fmt::format("{0}(({1}, {2})|({2}, {1}))\\.", prefix, p1, p2);
  };

  {
    // Exception thrown: one file, does not exist.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.PostForm("upload", {}, {{missing_1_key, missing_1_value}}),
        prefix + missing_1_desc + "\\.");
  }

  {
    // Exception thrown: multiple files, none exist.
    DRAKE_EXPECT_THROWS_MESSAGE(es.PostForm("upload", {},
                                            {{missing_1_key, missing_1_value},
                                             {missing_2_key, missing_2_value}}),
                                make_regex(missing_1_desc, missing_2_desc));
  }

  {
    // Exception thrown: one file exists, the other does not.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.PostForm("upload", {},
                    {{"test", {test_txt_path, std::nullopt}},
                     {missing_1_key, missing_1_value}}),
        prefix + missing_1_desc + "\\.");
  }

  {
    // Exception thrown: multiple files exist, multiple do not.
    DRAKE_EXPECT_THROWS_MESSAGE(
        es.PostForm("upload", {},
                    {{"test", {test_txt_path, std::nullopt}},
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
}  // namespace render
}  // namespace geometry
}  // namespace drake
