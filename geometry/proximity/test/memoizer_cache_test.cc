#include "drake/geometry/proximity/memoizer_cache.h"

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include <fmt/format.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {

using std::shared_ptr;
using std::string;
using std::unordered_map;

// Note: For now, we're not testing thread safety here. We're relying on visual
// inspection on the use of the mutex. If that becomes a problem, we can add
// the necessary scaffolding to test it.

// Use the Dump() method to confirm the cache's contents.
void DumpMatchesExpectation(const MemoizerCache<int, string>& cache,
                            const unordered_map<int, string>& expected_dump,
                            std::string_view context) {
  SCOPED_TRACE(context);
  unordered_map<int, shared_ptr<const string>> dump = cache.Dump();
  EXPECT_EQ(dump.size(), expected_dump.size());
  for (const auto& [key, value] : expected_dump) {
    ASSERT_TRUE(dump.contains(key));
    EXPECT_EQ(*dump.at(key), value);
  }
}

// Confirms:
//  1. Novel keys produce a new cache entry.
//  2. Reused keys produce reused entry.
//  3. It depends on key, and not value produced by the callback.
GTEST_TEST(MemoizerCacheTest, CacheInsertionAndReuse) {
  MemoizerCache<int, string> cache;

  std::unordered_map<int, string> expected_dump;

  DumpMatchesExpectation(cache, expected_dump, "Initially empty cache");

  // (1) - Novel key introduces novel entry.
  shared_ptr<const string> result1 = cache.FindOrInsert(42, []() {
    return "forty-two";
  });
  EXPECT_EQ(*result1, "forty-two");
  expected_dump[42] = "forty-two";
  DumpMatchesExpectation(cache, expected_dump, "After inserting 42");

  // (2), (3) Cached key gives cached value (not new callback value).
  shared_ptr<const string> result2 = cache.FindOrInsert(42, []() {
    return "should not be used";
  });
  EXPECT_EQ(result1, result2);
  EXPECT_EQ(*result2, "forty-two");
  DumpMatchesExpectation(cache, expected_dump, "Cache hit on 42");

  // (1) - Novel key on non-empty cache, introduces correct novel entry.
  shared_ptr<const string> result3 = cache.FindOrInsert(43, []() {
    return "forty-three";
  });
  EXPECT_EQ(*result3, "forty-three");
  expected_dump[43] = *result3;
  DumpMatchesExpectation(cache, expected_dump, "After inserting 43");

  // (3) Different key, but shared callback value, produces a new entry with a
  // unique copy fo the value.
  shared_ptr<const string> result5 = cache.FindOrInsert(142, []() {
    return "forty-two";
  });
  EXPECT_EQ(*result5, *result1);
  EXPECT_NE(result5, result1);
  expected_dump[142] = *result5;
  DumpMatchesExpectation(cache, expected_dump, "After inserting 142");
}

// When the last external shared_ptr goes away, the cache entry pops, but leaves
// other entries intact. We'll create two entries, one that gets two references
// and one that gets one.
GTEST_TEST(MemoizerCacheTest, AutoEviction) {
  MemoizerCache<int, string> cache;

  shared_ptr<const string> result1_a = cache.FindOrInsert(42, []() {
    return "forty-two";
  });
  shared_ptr<const string> result1_b = cache.FindOrInsert(42, []() {
    return "forty-two";
  });
  shared_ptr<const string> result2 = cache.FindOrInsert(43, []() {
    return "forty-three";
  });

  std::unordered_map<int, string> expected_dump{{42, "forty-two"},
                                                {43, "forty-three"}};
  DumpMatchesExpectation(cache, expected_dump, "Initial insertion");

  // Clear one of two references, cache remains unchanged.
  result1_a.reset();
  DumpMatchesExpectation(cache, expected_dump,
                         "After resetting one reference to 42");

  // Clear second of two references, one cache entry evicts, other remains.
  result1_b = nullptr;
  expected_dump.erase(42);
  DumpMatchesExpectation(cache, expected_dump,
                         "After resetting second reference to 42");

  result2 = nullptr;
  expected_dump.erase(43);
  DumpMatchesExpectation(cache, expected_dump,
                         "After resetting reference to 43");
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
