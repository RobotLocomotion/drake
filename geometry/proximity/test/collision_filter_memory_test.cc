#include <atomic>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/geometry/proximity/collision_filter.h"

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

using drake::test::LimitMalloc;
using drake::test::LimitMallocParams;

/* CollisionFilter is, conceptually, a matrix reporting the pairwise filter
 status between all geometry pairs. Implemented directly, we would expect the
 matrix size to grow quadratically with the number of geometries.

 CollisionFilter is specifically designed to avoid O(N^2) memory growth. This
 test verifies that the footprint has linear growth.

 It does so by counting the memory allocations performed. See implementation for
 more details. */
GTEST_TEST(CollisionFilterMemoryTest, MemoryGrowth) {
  constexpr int num_geometries = 10000;

  CollisionFilter filter;
  std::vector<GeometryId> ids;
  LimitMallocParams unlimited;

  {
    LimitMalloc guard(unlimited);

    for (int i = 0; i < num_geometries; ++i) {
      ids.push_back(GeometryId::get_new_id());
      filter.AddGeometry(ids.back());
    }

    /* If we generally allow CollisionFilter to make 10 allocations per
     added geometry, that will still be far below O(N^2). */
    ASSERT_LT(guard.num_allocations(), 10 * num_geometries);
  }

  // Adds N unique pairs to the collision filter.
  int a = 0;
  int b = 1;
  auto addNPairs = [&a, &b, &ids, &filter](int n) {
    int i = 0;
    for (; a < num_geometries; ++a) {
      b = b >= num_geometries ? a + 1 : b;
      for (; b < num_geometries; ++b) {
        filter.Apply(CollisionFilterDeclaration().ExcludeWithin(
                         GeometrySet({ids[a], ids[b]})),
                     &GeometrySetTester::geometries);
        if (++i >= n) {
          return;
        }
      }
    }
  };

  const int N = 10000;
  int first_n_allocations = 0;
  int second_n_allocations = 0;
  {
    LimitMalloc guard(unlimited);
    addNPairs(N);
    first_n_allocations = guard.num_allocations();
  }
  {
    LimitMalloc guard(unlimited);
    addNPairs(N);
    second_n_allocations = guard.num_allocations();
  }

  /* To be linear, the amount of work in allocating the second set of N should
   be approximately the same as the first set of N. We'll provide generous
   bounds to accommodate STL's memory allocation strategies but, nevertheless,
   lie far away from O(N^2) patterns. */
  EXPECT_GT(second_n_allocations, 0.5 * first_n_allocations);
  EXPECT_LT(second_n_allocations, 2 * first_n_allocations);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
