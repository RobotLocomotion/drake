#include "drake/multibody/fem/element_cache_entry.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

struct Data {
  double foo;
};

GTEST_TEST(ElementCacheEntryTest, Staleness) {
  ElementCacheEntry<Data> cache_entry;
  /* The cache entry is initialized to be stale. */
  EXPECT_TRUE(cache_entry.is_stale());
  /* Test setter and getter. */
  cache_entry.set_stale(false);
  EXPECT_FALSE(cache_entry.is_stale());
  cache_entry.set_stale(true);
  EXPECT_TRUE(cache_entry.is_stale());
}

GTEST_TEST(ElementCacheEntryTest, CachedData) {
  ElementCacheEntry<Data> cache_entry;
  EXPECT_TRUE(cache_entry.is_stale());
  /* Set the cache entry value. */
  Data& mutable_data = cache_entry.mutable_element_data();
  mutable_data.foo = 1.23;
  const Data& const_data = cache_entry.element_data();
  /* Verify the cache entry value has been modified. */
  EXPECT_EQ(const_data.foo, 1.23);
  /* Verify the staleness flag is untouched. */
  EXPECT_TRUE(cache_entry.is_stale());
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
