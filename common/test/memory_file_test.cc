#include "drake/common/memory_file.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace {

GTEST_TEST(MemoryFileTest, DefaultConstruct) {
  const MemoryFile contents;
  EXPECT_TRUE(contents.contents().empty());
  EXPECT_TRUE(contents.filename_hint().empty());
  EXPECT_EQ(contents.sha256(), Sha256());
}

GTEST_TEST(MemoryFileTest, BasicApi) {
  const std::string content_str = R"""(Some arbitrary
                contents with newlines and whatnot.)""";
  const std::string filename = "Just some file";
  const MemoryFile contents(content_str, filename);

  EXPECT_EQ(contents.contents(), content_str);
  EXPECT_EQ(contents.filename_hint(), filename);
  EXPECT_EQ(contents.sha256(), Sha256::Checksum(content_str));
}

GTEST_TEST(MemoryFileTest, StaticMethods) {
  const std::string path =
      FindResourceOrThrow("drake/common/test/find_resource_test_data.txt");
  const std::string contents = ReadFileOrThrow(path);
  const MemoryFile memory_file = MemoryFile::Make(path);
  const MemoryFile ref(contents, path);

  EXPECT_EQ(contents, memory_file.contents());
  EXPECT_EQ(memory_file.sha256(), ref.sha256());
  EXPECT_EQ(path, memory_file.filename_hint());
}

}  // namespace
}  // namespace drake
