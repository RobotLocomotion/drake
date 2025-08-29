#include "drake/geometry/meshcat_file_storage_internal.h"

#include <deque>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/random.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(MeshcatFileStorageInternalTest, BasicWorkflow) {
  using ContentsPtr = std::shared_ptr<const MemoryFile>;
  FileStorage dut;

  // Start out empty.
  EXPECT_FALSE(dut.Find(Sha256{}));
  EXPECT_EQ(dut.DumpEverything().size(), 0);
  EXPECT_EQ(dut.size(), 0);

  // Add one item.
  std::string one_content{"content_1"};
  std::string one_hint{"hint_1"};
  ContentsPtr one = dut.Insert(std::move(one_content), std::move(one_hint));
  ASSERT_TRUE(one);
  EXPECT_EQ(one.use_count(), 1);
  EXPECT_EQ(one->contents(), "content_1");
  EXPECT_EQ(one->extension(), "");
  EXPECT_NE(one->sha256(), Sha256{});
  EXPECT_EQ(one->filename_hint(), "hint_1");
  EXPECT_EQ(dut.size(), 1);
  EXPECT_EQ(one_content, "");
  EXPECT_EQ(one_hint, "");
  const Sha256 one_sha256 = one->sha256();

  // Add a second item.
  // Newlines in the filename are replaced with blanks.
  ContentsPtr two = dut.Insert("content_2", "hint\r\n2");
  ASSERT_TRUE(two);
  EXPECT_EQ(two.use_count(), 1);
  EXPECT_EQ(two->contents(), "content_2");
  EXPECT_EQ(two->extension(), "");
  EXPECT_NE(two->sha256(), one_sha256);
  EXPECT_EQ(two->filename_hint(), "hint  2");
  EXPECT_EQ(dut.size(), 2);
  const Sha256 two_sha256 = two->sha256();

  // Add the first item again.
  std::string repeat_content{"content_1"};
  std::string repeat_hint{"will_be_ignored"};
  ContentsPtr repeat =
      dut.Insert(std::move(repeat_content), std::move(repeat_hint));
  ASSERT_TRUE(repeat);
  EXPECT_EQ(repeat.use_count(), 2);
  EXPECT_EQ(one.use_count(), 2);
  EXPECT_EQ(repeat.get(), one.get());
  EXPECT_EQ(dut.size(), 2);
  EXPECT_EQ(repeat_content, "");
  EXPECT_EQ(repeat_hint, "");

  // The CAS URLs are as expected.
  EXPECT_EQ(FileStorage::GetCasUrl(*one), FileStorage::GetCasUrl(*repeat));
  EXPECT_NE(FileStorage::GetCasUrl(*one), FileStorage::GetCasUrl(*two));

  // Look up the first item.
  ContentsPtr find_one = dut.Find(one_sha256);
  ASSERT_TRUE(find_one);
  EXPECT_EQ(find_one.use_count(), 3);
  EXPECT_EQ(find_one->contents(), "content_1");
  EXPECT_EQ(find_one->filename_hint(), "hint_1");

  // Dump everything. We don't care what order the assets come out in,
  // as long as it's deterministic -- we enforce that by hard-coding
  // an assumed order here.
  std::vector<ContentsPtr> everything = dut.DumpEverything();
  ASSERT_EQ(everything.size(), 2);
  EXPECT_EQ(everything.at(0)->filename_hint(), "hint_1");
  EXPECT_EQ(everything.at(1)->filename_hint(), "hint  2");
  everything = {};

  // Reset up all handles to the first item, so that it will be
  // removed from storage.
  one = {};
  repeat = {};
  find_one = {};
  EXPECT_FALSE(dut.Find(one_sha256));
  EXPECT_EQ(dut.size(), 1);

  // Look up the second item.
  ContentsPtr find_two = dut.Find(two_sha256);
  ASSERT_TRUE(find_two);
  EXPECT_EQ(find_two.use_count(), 2);
  EXPECT_EQ(find_two->filename_hint(), "hint  2");
  EXPECT_EQ(find_two->contents(), "content_2");

  // Reset the original handle, but keeping the Find() handle.
  two = {};
  EXPECT_EQ(find_two.use_count(), 1);
  EXPECT_EQ(find_two->contents(), "content_2");
  EXPECT_EQ(dut.size(), 1);

  // Reset the last remaining handle.
  find_two = {};
  EXPECT_FALSE(dut.Find(two_sha256));
  EXPECT_EQ(dut.size(), 0);
}

GTEST_TEST(MeshcatFileStorageInternalTest, ReinsertExpired) {
  using ContentsPtr = std::shared_ptr<const MemoryFile>;
  FileStorage dut;

  // Insert content and then immediately expire it.
  ContentsPtr one = dut.Insert("content", "old_hint");
  one = {};
  EXPECT_EQ(dut.size(), 0);

  // Re-insert the same content again and make sure it's findable.
  one = dut.Insert("content", "new_hint");
  ASSERT_TRUE(one);
  EXPECT_EQ(one.use_count(), 1);
  EXPECT_EQ(one->contents(), "content");
  EXPECT_EQ(one->extension(), "");
  EXPECT_EQ(one->filename_hint(), "new_hint");
  ContentsPtr find_one = dut.Find(one->sha256());
  ASSERT_TRUE(find_one);
  EXPECT_EQ(find_one.use_count(), 2);
  EXPECT_EQ(find_one.get(), one.get());
  EXPECT_EQ(find_one->contents(), "content");
  EXPECT_EQ(one->extension(), "");
  EXPECT_EQ(find_one->filename_hint(), "new_hint");
  EXPECT_EQ(dut.size(), 1);

  // Clear everything.
  one = {};
  find_one = {};
  EXPECT_EQ(dut.size(), 0);
}

GTEST_TEST(MeshcatFileStorageInternalTest, ThreadingStress) {
  // A list of arbitrary content to use for testing.
  const std::vector<std::string> contents{
      "alpha", "bravo", "charlie", "delta", "echo", "fox",
  };

  using ContentsPtr = std::shared_ptr<const MemoryFile>;
  FileStorage dut;

  // This loop imitates the outer Meshcat class, by adding and removing content
  // from the database.
  auto producer_loop = [&contents, &dut]() {
    RandomGenerator random;
    std::deque<ContentsPtr> handles;
    for (int i = 0; i < 1000; ++i) {
      if (handles.size() > 0 && random() % 8 == 0) {
        // Occasionally, drop an item completely from the database.
        ContentsPtr victim = handles.front();
        handles.erase(std::remove(handles.begin(), handles.end(), victim),
                      handles.end());
      } else if (handles.size() > 0 && random() % 3 == 0) {
        // Sometimes, remove a handle.
        handles.pop_front();
      } else {
        // Typically, add a handle.
        std::string content = contents.at(random() % contents.size());
        handles.push_back(dut.Insert(std::move(content), "filename"));
        // Sanity check that it was actually inserted.
        DRAKE_DEMAND(dut.Find(handles.back()->sha256()) != nullptr);
      }
    }
  };

  // This loop imitates the inner (i.e., worker) Meshcat class, by accessing
  // content in the database.
  auto consumer_loop = [&contents, &dut]() {
    RandomGenerator random;
    for (int i = 0; i < 1000; ++i) {
      if (random() % 16 == 0) {
        // Occasionally do a full dump.
        unused(dut.DumpEverything());
      } else {
        // Typically, just look up an arbitrary file.
        // We don't care whether the file was found.
        const std::string& needle = contents.at(random() % contents.size());
        unused(dut.Find(Sha256::Checksum(needle)));
      }
    }
  };

  // Launch three threads, and then wait for them all to finish. In the current
  // implementation of the outer Meshcat class there can be at most one producer
  // (the "main thread"), but here we test with >1 producer in anticipation of
  // enhacing Meshcat to allow concurrent use at some point down the road.
  std::future<void> producer1 = std::async(std::launch::async, producer_loop);
  std::future<void> producer2 = std::async(std::launch::async, producer_loop);
  std::future<void> consumer = std::async(std::launch::async, consumer_loop);
  EXPECT_NO_THROW(producer1.get());
  EXPECT_NO_THROW(producer2.get());
  EXPECT_NO_THROW(consumer.get());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
