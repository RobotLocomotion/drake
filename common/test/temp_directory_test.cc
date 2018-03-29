#include "drake/common/temp_directory.h"

#include <cstdlib>
#include <string>

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(TempDirectoryTest, TempDirectoryTest) {
  const char* test_tmpdir = std::getenv("TEST_TMPDIR");
  ASSERT_STRNE(nullptr, test_tmpdir);

  const std::string tmpdir = temp_directory();
  EXPECT_NE('/', tmpdir.back());
  EXPECT_EQ(tmpdir, std::string(test_tmpdir));
}

}  // namespace
}  // namespace drake
