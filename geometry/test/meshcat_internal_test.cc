#include "drake/geometry/meshcat_internal.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(MeshcatInternalTest, GetMeshcatStaticResource) {
  // This matches the list of URLs in the API doc.
  const std::vector<const char*> urls{
      "/",           "/favicon.ico",  "/index.html", "/meshcat.html",
      "/meshcat.js", "/stats.min.js",
  };
  for (const auto& url : urls) {
    SCOPED_TRACE(fmt::format("url = {}", url));
    const std::optional<std::string_view> result =
        GetMeshcatStaticResource(url);
    ASSERT_TRUE(result);
    EXPECT_FALSE(result->empty());
  }
}

GTEST_TEST(MeshcatInternalTest, UuidGenerator) {
  UuidGenerator dut;
  std::string foo = dut.GenerateRandom();
  std::string bar = dut.GenerateRandom();
  EXPECT_NE(foo, bar);

  const std::string_view pattern =
      "[[:xdigit:]]{8,8}-"
      "[[:xdigit:]]{4,4}-"
      "[[:xdigit:]]{4,4}-"
      "[[:xdigit:]]{4,4}-"
      "[[:xdigit:]]{12,12}";
  EXPECT_THAT(foo, testing::MatchesRegex(pattern));
  EXPECT_THAT(bar, testing::MatchesRegex(pattern));
}

GTEST_TEST(MeshcatInternalTest, FileStorage) {
  using Handle = FileStorage::Handle;
  FileStorage dut;

  // Start out empty.
  EXPECT_FALSE(dut.Find(Sha256{}).content);
  EXPECT_EQ(dut.DumpEverything().size(), 0);
  EXPECT_EQ(dut.ShrinkToFit(), 0);

  // Add one item.
  std::string one_content{"content_1"};
  std::string one_hint{"hint_1"};
  Handle one = dut.Insert(std::move(one_content), std::move(one_hint));
  EXPECT_EQ(one_content, "");
  EXPECT_EQ(one_hint, "");
  EXPECT_NE(one.sha256, Sha256{});
  EXPECT_EQ(one.filename_hint, "hint_1");
  EXPECT_THAT(one.content, testing::Pointee(testing::Eq("content_1")));
  EXPECT_EQ(one.content.use_count(), 1);
  EXPECT_EQ(dut.ShrinkToFit(), 0);

  // Add a second item.
  // Newlines in the filename are replaced with blanks.
  Handle two = dut.Insert("content_2", "hint\r\n2");
  EXPECT_NE(two.sha256, one.sha256);
  EXPECT_EQ(two.filename_hint, "hint  2");
  EXPECT_THAT(two.content, testing::Pointee(testing::Eq("content_2")));
  EXPECT_EQ(two.content.use_count(), 1);
  EXPECT_EQ(dut.ShrinkToFit(), 0);

  // Add the first item again.
  std::string repeat_content{"content_1"};
  std::string repeat_hint{"will_be_ignored"};
  Handle repeat = dut.Insert(std::move(repeat_content), std::move(repeat_hint));
  EXPECT_EQ(repeat_content, "");
  EXPECT_EQ(repeat_hint, "");
  EXPECT_EQ(repeat.sha256, one.sha256);
  EXPECT_EQ(repeat.filename_hint, "hint_1");
  EXPECT_EQ(repeat.content.get(), one.content.get());
  EXPECT_EQ(repeat.content.use_count(), 2);
  EXPECT_EQ(one.content.use_count(), 2);
  EXPECT_EQ(dut.ShrinkToFit(), 0);

  // Look up the first item.
  Handle find_one = dut.Find(one.sha256);
  EXPECT_EQ(find_one.filename_hint, "hint_1");
  EXPECT_THAT(find_one.content, testing::Pointee(testing::Eq("content_1")));
  EXPECT_EQ(find_one.content.use_count(), 3);
  EXPECT_EQ(dut.ShrinkToFit(), 0);

  // Dump everything. We don't care what order the assets come out in,
  // as long as it's deterministic -- we enforce that by hard-coding
  // an assumed order here.
  std::vector<Handle> everything = dut.DumpEverything();
  ASSERT_EQ(everything.size(), 2);
  EXPECT_EQ(everything.at(0).filename_hint, "hint_1");
  EXPECT_EQ(everything.at(1).filename_hint, "hint  2");
  everything = {};

  // Reset up all handles to the first item, so that it will be
  // removed from storage.
  one = {};
  repeat = {};
  find_one = {};
  EXPECT_FALSE(dut.Find(one.sha256).content);
  EXPECT_EQ(dut.ShrinkToFit(), 1);

  // Look up the second item.
  Handle find_two = dut.Find(two.sha256);
  EXPECT_EQ(find_two.filename_hint, "hint  2");
  EXPECT_THAT(find_two.content, testing::Pointee(testing::Eq("content_2")));
  EXPECT_EQ(find_two.content.use_count(), 2);
  EXPECT_EQ(dut.ShrinkToFit(), 0);

  // Reset the original handle, but keeping the Find() handle.
  two = {};
  EXPECT_THAT(find_two.content, testing::Pointee(testing::Eq("content_2")));
  EXPECT_EQ(find_two.content.use_count(), 1);
  EXPECT_EQ(dut.ShrinkToFit(), 0);

  // Reset the last remaining handle.
  find_two = {};
  EXPECT_FALSE(dut.Find(two.sha256).content);
  EXPECT_EQ(dut.ShrinkToFit(), 1);
}

GTEST_TEST(MeshcatInternalTest, FileStorageInsertExpired) {
  using Handle = FileStorage::Handle;
  FileStorage dut;

  // Insert content and then immediately expire it.
  Handle one = dut.Insert("content", "old_hint");
  one = {};

  // Re-insert the same content again and make sure it's findable.
  one = dut.Insert("content", "new_hint");
  EXPECT_EQ(one.filename_hint, "new_hint");
  EXPECT_THAT(one.content, testing::Pointee(testing::Eq("content")));
  EXPECT_EQ(one.content.use_count(), 1);
  Handle find_one = dut.Find(one.sha256);
  EXPECT_EQ(find_one.filename_hint, "new_hint");
  EXPECT_THAT(find_one.content, testing::Pointee(testing::Eq("content")));
  EXPECT_EQ(dut.ShrinkToFit(), 0);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
