#include "drake/common/file_source.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(FileSourceTest, ToString) {
  EXPECT_EQ(to_string(FileSource("a/b/c")), "a/b/c");
  const MemoryFile file("012345789", ".ext", "hint");
  EXPECT_EQ(to_string(FileSource(file)), file.to_string());
}
}  // namespace
}  // namespace drake
