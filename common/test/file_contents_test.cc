#include "drake/common/file_contents.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace common {
namespace {

GTEST_TEST(FileContentsTest, DefaultConstruct) {
  const FileContents contents;
  EXPECT_TRUE(contents.contents().empty());
  EXPECT_TRUE(contents.filename_hint().empty());
  EXPECT_EQ(contents.sha256(), Sha256());
}

GTEST_TEST(FileContentsTest, BasicApi) {
  const std::string content_str = R"""(Some arbitrary
                contents with newlines and whatnot.)""";
  const std::string filename = "Just some file";
  const FileContents contents(content_str, filename);

  EXPECT_EQ(contents.contents(), content_str);
  EXPECT_EQ(contents.filename_hint(), filename);
  EXPECT_EQ(contents.sha256(), Sha256::Checksum(content_str));
}

GTEST_TEST(FileContentsTest, StaticMethods) {
  const std::string path =
      FindResourceOrThrow("drake/common/test/find_resource_test_data.txt");
  const std::string contents = FileContents::Read(path);
  const FileContents file_contents = FileContents::Make(path);
  const FileContents ref(contents, path);

  EXPECT_EQ(contents, file_contents.contents());
  EXPECT_EQ(file_contents.sha256(), ref.sha256());
  EXPECT_EQ(path, file_contents.filename_hint());
}

}  // namespace
}  // namespace common
}  // namespace drake
