#include "drake/geometry/render_gltf_client/internal_http_service.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {
namespace {

namespace fs = std::filesystem;

// A concrete implementation of HttpService that does nothing.
class EmptyService : public HttpService {
 public:
  EmptyService() = default;

 protected:
  HttpResponse DoPostForm(const std::string& /* temp_directory */,
                          const std::string& /* url */,
                          const DataFieldsMap& /* data_fields */,
                          const FileFieldsMap& /* file_fields */,
                          bool /* verbose */ = false) override {
    HttpResponse ret;
    ret.http_code = 200;
    return ret;
  }
};

class HttpServicePostFormTest : public ::testing::Test {
 public:
  HttpServicePostFormTest() {
    // Create temporary files for this test case.
    std::ofstream txt_file{txt_path_};
    txt_file << "test!\n";
    txt_file.close();

    std::ofstream jpg_file{jpg_path_};
    jpg_file << "not really a jpg!\n";
    jpg_file.close();
  }

  // Exercise HttpService::PostForm using the given file_fields mapping from
  // field name to file path.  (The mime_type is always nullopt.)
  void PostForm(const DataFieldsMap& file_fields) {
    FileFieldsMap actual_fields;
    for (const auto& [field_name, file_path] : file_fields) {
      FileFieldPayload payload;
      payload.first = file_path;
      actual_fields.emplace(field_name, std::move(payload));
    }
    EmptyService empty_service;
    const std::string url = "http://127.0.0.1:8000/render";
    const DataFieldsMap string_fields;
    empty_service.PostForm(temp_dir_, url, string_fields, actual_fields);
  }

 protected:
  const std::string temp_dir_{drake::temp_directory()};
  const std::string txt_path_{temp_dir_ + "/test.txt"};
  const std::string jpg_path_{temp_dir_ + "/fake.jpg"};
};

// When no files are posted, no exceptions appear.
TEST_F(HttpServicePostFormTest, NoFiles) {
  DRAKE_EXPECT_NO_THROW(PostForm({}));
}

// When valid filenames are provided, no exceptions appear.
TEST_F(HttpServicePostFormTest, ValidFiles) {
  DRAKE_EXPECT_NO_THROW(PostForm({{"text", txt_path_}}));
  DRAKE_EXPECT_NO_THROW(PostForm({{"image", jpg_path_}}));
  DRAKE_EXPECT_NO_THROW(PostForm({{"text", txt_path_}, {"image", jpg_path_}}));
}

// A request to upload a missing file will throw.
TEST_F(HttpServicePostFormTest, NoSuchFile) {
  DRAKE_EXPECT_THROWS_MESSAGE(PostForm({{"foo", "/no/such/file"}}),
                              ".*missing.*foo.*/no/such/file.*");
}

// A request to upload a multiple missing files reports all of the errors.
TEST_F(HttpServicePostFormTest, NoSuchFileMultiple) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      PostForm({{"bar", "/no/such/other"}, {"foo", "/no/such/file"}}),
      ".*missing.*bar.*/no/such/other.*foo.*/no/such/file.*");
}

// When one files exists and one does not, only the missing file is reported.
TEST_F(HttpServicePostFormTest, MixedFiles) {
  EXPECT_THAT(
      [&]() {
        PostForm({{"image", jpg_path_}, {"foo", "/no/such/file"}});
      },
      testing::ThrowsMessage<std::exception>(
          testing::AllOf(testing::Not(testing::HasSubstr("image")),
                         testing::HasSubstr("/no/such/file"))));
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
