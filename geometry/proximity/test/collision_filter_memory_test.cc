#include <atomic>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/collision_filter.h"

// Track memory allocated and freed.
struct MemoryTracker {
  static std::atomic<size_t> current_usage;
};

std::atomic<size_t> MemoryTracker::current_usage{0};

void* operator new(size_t size) {
  MemoryTracker::current_usage += size;
  return malloc(size);
}

void operator delete(void* memory, size_t size) noexcept {
  MemoryTracker::current_usage -= size;
  free(memory);
}

void operator delete(void* memory) noexcept {
  // The compiler requires this old, unsized version. We assume it won't be
  // invoked in the contexts in which this test is built and run.
  free(memory);
}

namespace drake {
namespace geometry {
class GeometrySetTester {
 public:
  static std::unordered_set<GeometryId> geometries(const GeometrySet& s,
                                                   CollisionFilterScope) {
    return s.geometries();
  }
};

namespace internal {

/* CollisionFilter's memory footprint should grow linearly with the number of
 *pairs* added to the filter (it's specifically designed to be this way to
 prevent the others O(N^2) memory runaway. This is a regression test for that
 property.

 With the allocation overloads above, we'll track the amount the memory changes
 as we increase the number of pairs. */
GTEST_TEST(CollisionFilterMemoryTest, MemoryGrowth) {
  constexpr int num_geometries = 100;

  CollisionFilter filter;
  std::vector<GeometryId> ids;
  for (int i = 0; i < num_geometries; ++i) {
    ids.push_back(GeometryId::get_new_id());
    filter.AddGeometry(ids.back());
  }

  // Add 100 pairs.
  int a = 0;
  int b = 1;
  auto add_100 = [&a, &b, &ids, &filter]() {
    int i = 0;
    for (; a < num_geometries; ++a) {
      b = b >= num_geometries ? a + 1 : b;
      for (; b < num_geometries; ++b) {
        filter.Apply(CollisionFilterDeclaration().ExcludeWithin(
                         GeometrySet({ids[a], ids[b]})),
                     &GeometrySetTester::geometries);
        if (++i >= 100) {
          return;
        }
      }
    }
  };

  const size_t memory0 = MemoryTracker::current_usage;
  add_100();
  const size_t memory1 = MemoryTracker::current_usage;
  add_100();
  const size_t memory2 = MemoryTracker::current_usage;

  const size_t first_100_cost = memory1 - memory0;
  const size_t second_100_cost = memory2 - memory1;
  // Confirm we were actually able to measure the memory growth after adding
  // 100 pairs.
  EXPECT_GT(first_100_cost, 0);

  // When we double the population size, we want to show that the memory
  // growth is linear. However, STL's memory allocation strategy pads its
  // allocations to limit calls to the heap. So, we can't expect exact equality.
  // However, if the footprint grew as O(N²), then we'd expect the new footprint
  // to be more than 4x the original (double the size, quadruple the footprint).
  // Allowing for STL's 2X allocation strategy, as long as we can show the
  // footprint for the second set of 100 pairs grew by less than 2X the first
  // 100 pairs, then we're squarely within the linear regime.
  EXPECT_LT(second_100_cost, 2 * first_100_cost);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
