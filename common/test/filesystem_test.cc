#include "drake/common/filesystem.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace {

// Sanity check a few operations to make sure that the namespace aliasing is
// working correctly.
GTEST_TEST(NamespaceAliasSmokeTest, ExistTest) {
  EXPECT_FALSE(filesystem::is_regular_file({"."}));
  EXPECT_FALSE(filesystem::is_regular_file({"common"}));
  EXPECT_TRUE(filesystem::is_regular_file({"common/filesystem_test"}));
  EXPECT_FALSE(filesystem::is_regular_file({"/no/such/path"}));

  EXPECT_TRUE(filesystem::is_directory({"."}));
  EXPECT_TRUE(filesystem::is_directory({"common"}));
  EXPECT_FALSE(filesystem::is_directory({"common/filesystem_test"}));
  EXPECT_FALSE(filesystem::is_directory({"/no/such/path"}));

  DRAKE_EXPECT_THROWS_MESSAGE(
      filesystem::read_symlink({"/no_such_readlink"}),
      ".*(No such file or directory|Invalid argument).+/no_such_readlink.+");
}

}  // namespace
}  // namespace drake
