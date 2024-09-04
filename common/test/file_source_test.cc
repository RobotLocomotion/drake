#include "drake/common/file_source.h"

#include <gtest/gtest.h>

#include "drake/common/unused.h"

namespace drake {
namespace {

GTEST_TEST(FileSourceTest, DefaultConstructor) {
  const FileSource s;
  EXPECT_FALSE(s.is_path());
  EXPECT_FALSE(s.is_memory_file());
  EXPECT_TRUE(s.empty());
  EXPECT_EQ(s.description(), "");
}

GTEST_TEST(FileSourceTest, PathConstructor) {
  const FileSource s(std::filesystem::path("path"));
  EXPECT_TRUE(s.is_path());
  EXPECT_FALSE(s.is_memory_file());
  EXPECT_FALSE(s.empty());
  EXPECT_EQ(s.path(), "path");
  EXPECT_EQ(s.description(), "path");
}

GTEST_TEST(FileSourceTest, MemoryFileConstructor) {
  const FileSource s(MemoryFile("contents", ".ext", "hint"));
  EXPECT_FALSE(s.is_path());
  EXPECT_TRUE(s.is_memory_file());
  EXPECT_FALSE(s.empty());
  EXPECT_EQ(s.memory_file().contents(), "contents");
  EXPECT_EQ(s.description(), "hint");
}

GTEST_TEST(FileSourceTest, MoveSemantics) {
  FileSource memory_source(MemoryFile("contents", ".ext", "hint"));
  FileSource path_source(std::filesystem::path("path"));

  FileSource target(std::move(memory_source));
  ASSERT_TRUE(target.is_memory_file());
  EXPECT_EQ(target.description(), "hint");
  ASSERT_TRUE(memory_source.empty());

  target = std::move(path_source);
  ASSERT_TRUE(target.is_path());
  EXPECT_EQ(target.description(), "path");
  ASSERT_TRUE(path_source.empty());
}

GTEST_TEST(FileSourceTest, Clear) {
  FileSource s("path");
  EXPECT_FALSE(s.empty());

  s.clear();

  EXPECT_FALSE(s.is_path());
  EXPECT_FALSE(s.is_memory_file());
  EXPECT_TRUE(s.empty());
}

/* Dummy function for testing implicit conversion. */
void ConsumeFileSource(const FileSource&) {}

/* We allow implicit conversion from a file path and MemoryFile to a FileSource.
 Successful compilation and execution implies test success. */
GTEST_TEST(FileSourceTest, ImplicitConversion) {
  ConsumeFileSource(std::filesystem::path("path"));
  ConsumeFileSource(MemoryFile("contents", ".ext", "hint"));
}

}  // namespace
}  // namespace drake
