#include "drake/common/file_source.h"

#include <gtest/gtest.h>

#include "drake/common/unused.h"

namespace drake {
namespace {

GTEST_TEST(FileSourceTest, DefaultConstructor) {
  const FileSource s;
  EXPECT_FALSE(s.is_path());
  EXPECT_FALSE(s.is_in_memory());
  EXPECT_TRUE(s.empty());
  EXPECT_EQ(s.description(), "");
}

GTEST_TEST(FileSourceTest, PathConstructor) {
  const FileSource s(std::filesystem::path("path"));
  EXPECT_TRUE(s.is_path());
  EXPECT_FALSE(s.is_in_memory());
  EXPECT_FALSE(s.empty());
  EXPECT_EQ(s.path(), "path");
  EXPECT_EQ(s.description(), "path");
}

GTEST_TEST(FileSourceTest, MemoryFileConstructor) {
  const FileSource s(MemoryFile("contents", ".ext", "hint"));
  EXPECT_FALSE(s.is_path());
  EXPECT_TRUE(s.is_in_memory());
  EXPECT_FALSE(s.empty());
  EXPECT_EQ(s.memory_file().contents(), "contents");
  EXPECT_EQ(s.description(), "hint");
}

GTEST_TEST(FileSourceTest, StringConstructor) {
  const char c_ptr[] = "path1";
  const std::string_view view("path2");
  const std::string str("path3");

  /* C-style string. */
  {
    const FileSource s(c_ptr);
    EXPECT_TRUE(s.is_path());
    EXPECT_FALSE(s.is_in_memory());
    EXPECT_FALSE(s.empty());
  }

  /* std::string_view. */
  {
    const FileSource s(view);
    EXPECT_TRUE(s.is_path());
    EXPECT_FALSE(s.is_in_memory());
    EXPECT_FALSE(s.empty());
  }

  /* std::string_view. */
  {
    const FileSource s(str);
    EXPECT_TRUE(s.is_path());
    EXPECT_FALSE(s.is_in_memory());
    EXPECT_FALSE(s.empty());
  }
}

GTEST_TEST(FileSourceTest, Clear) {
  FileSource s("path");
  EXPECT_FALSE(s.empty());

  s.clear();

  EXPECT_FALSE(s.is_path());
  EXPECT_FALSE(s.is_in_memory());
  EXPECT_TRUE(s.empty());
}

/* Dummy function for testing implicit conversion. */
void ConsumeFileSource(const FileSource&) {}

/* We allow implicit conversion from a filename/path and MemoryFile to a
 FileSource. Successful compilation and execution implies test success. */
GTEST_TEST(FileSourceTest, ImplicitConversion) {
  ConsumeFileSource("char_pointer");
  ConsumeFileSource(std::string("string"));
  ConsumeFileSource(std::string_view("string_view"));
  ConsumeFileSource(std::filesystem::path("path"));
  ConsumeFileSource(MemoryFile("contents", ".ext", "hint"));
}

}  // namespace
}  // namespace drake
