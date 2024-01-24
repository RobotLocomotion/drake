#include "drake/geometry/meshcat_file_storage_internal.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(MeshcatFileStorageInternalTest, BasicWorkflow) {
  using HandlePtr = std::shared_ptr<const FileStorage::Handle>;
  FileStorage dut;

  // Start out empty.
  EXPECT_FALSE(dut.Find(Sha256{}));
  EXPECT_EQ(dut.DumpEverything().size(), 0);
  EXPECT_EQ(dut.size(), 0);

  // Add one item.
  std::string one_content{"content_1"};
  std::string one_hint{"hint_1"};
  HandlePtr one = dut.Insert(std::move(one_content), std::move(one_hint));
  ASSERT_TRUE(one);
  EXPECT_EQ(one.use_count(), 1);
  EXPECT_EQ(one->content, "content_1");
  EXPECT_NE(one->sha256, Sha256{});
  EXPECT_EQ(one->filename_hint, "hint_1");
  EXPECT_EQ(dut.size(), 1);
  EXPECT_EQ(one_content, "");
  EXPECT_EQ(one_hint, "");
  const Sha256 one_sha256 = one->sha256;

  // Add a second item.
  // Newlines in the filename are replaced with blanks.
  HandlePtr two = dut.Insert("content_2", "hint\r\n2");
  ASSERT_TRUE(two);
  EXPECT_EQ(two.use_count(), 1);
  EXPECT_EQ(two->content, "content_2");
  EXPECT_NE(two->sha256, one_sha256);
  EXPECT_EQ(two->filename_hint, "hint  2");
  EXPECT_EQ(dut.size(), 2);
  const Sha256 two_sha256 = two->sha256;

  // Add the first item again.
  std::string repeat_content{"content_1"};
  std::string repeat_hint{"will_be_ignored"};
  HandlePtr repeat =
      dut.Insert(std::move(repeat_content), std::move(repeat_hint));
  ASSERT_TRUE(repeat);
  EXPECT_EQ(repeat.use_count(), 2);
  EXPECT_EQ(one.use_count(), 2);
  EXPECT_EQ(repeat.get(), one.get());
  EXPECT_EQ(dut.size(), 2);
  EXPECT_EQ(repeat_content, "");
  EXPECT_EQ(repeat_hint, "");

  // Look up the first item.
  HandlePtr find_one = dut.Find(one_sha256);
  ASSERT_TRUE(find_one);
  EXPECT_EQ(find_one.use_count(), 3);
  EXPECT_EQ(find_one->content, "content_1");
  EXPECT_EQ(find_one->filename_hint, "hint_1");

  // Dump everything. We don't care what order the assets come out in,
  // as long as it's deterministic -- we enforce that by hard-coding
  // an assumed order here.
  std::vector<HandlePtr> everything = dut.DumpEverything();
  ASSERT_EQ(everything.size(), 2);
  EXPECT_EQ(everything.at(0)->filename_hint, "hint_1");
  EXPECT_EQ(everything.at(1)->filename_hint, "hint  2");
  everything = {};

  // Reset up all handles to the first item, so that it will be
  // removed from storage.
  one = {};
  repeat = {};
  find_one = {};
  EXPECT_FALSE(dut.Find(one_sha256));
  EXPECT_EQ(dut.size(), 1);

  // Look up the second item.
  HandlePtr find_two = dut.Find(two_sha256);
  ASSERT_TRUE(find_two);
  EXPECT_EQ(find_two.use_count(), 2);
  EXPECT_EQ(find_two->filename_hint, "hint  2");
  EXPECT_EQ(find_two->content, "content_2");

  // Reset the original handle, but keeping the Find() handle.
  two = {};
  EXPECT_EQ(find_two.use_count(), 1);
  EXPECT_EQ(find_two->content, "content_2");
  EXPECT_EQ(dut.size(), 1);

  // Reset the last remaining handle.
  find_two = {};
  EXPECT_FALSE(dut.Find(two_sha256));
  EXPECT_EQ(dut.size(), 0);
}

GTEST_TEST(MeshcatFileStorageInternalTest, ReinsertExpired) {
  using HandlePtr = std::shared_ptr<const FileStorage::Handle>;
  FileStorage dut;

  // Insert content and then immediately expire it.
  HandlePtr one = dut.Insert("content", "old_hint");
  one = {};
  EXPECT_EQ(dut.size(), 0);

  // Re-insert the same content again and make sure it's findable.
  one = dut.Insert("content", "new_hint");
  ASSERT_TRUE(one);
  EXPECT_EQ(one.use_count(), 1);
  EXPECT_EQ(one->content, "content");
  EXPECT_EQ(one->filename_hint, "new_hint");
  HandlePtr find_one = dut.Find(one->sha256);
  ASSERT_TRUE(find_one);
  EXPECT_EQ(find_one.use_count(), 2);
  EXPECT_EQ(find_one.get(), one.get());
  EXPECT_EQ(find_one->content, "content");
  EXPECT_EQ(find_one->filename_hint, "new_hint");
  EXPECT_EQ(dut.size(), 1);

  // Clear everything.
  one = {};
  find_one = {};
  EXPECT_EQ(dut.size(), 0);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
