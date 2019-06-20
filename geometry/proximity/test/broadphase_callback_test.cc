#include "drake/geometry/proximity/broadphase_callback.h"

#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using fcl::Boxd;
using fcl::CollisionObjectd;
using std::make_shared;
using std::vector;

// Confirms that the callback properly returns a sorted pair for the
// corresponding geometry ids.
GTEST_TEST(BroadphaseCallback, PairsProperlyFormed) {
  // New ids are created in increasing order. Thus we know in which order they
  // are reported in a sorted pair.
  std::vector<GeometryId> geometry_map{GeometryId::get_new_id(),
                                       GeometryId::get_new_id()};
  CollisionFilterLegacy collision_filter;

  EncodedData data_A(GeometryIndex{0}, true);
  EncodedData data_B(GeometryIndex{1}, true);
  collision_filter.AddGeometry(data_A.encoding());
  collision_filter.AddGeometry(data_B.encoding());

  CollisionObjectd box_A(make_shared<Boxd>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  CollisionObjectd box_B(make_shared<Boxd>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  vector<SortedPair<GeometryId>> pairs;
  BroadphaseCallbackData data(&geometry_map, &collision_filter, &pairs);
  BroadphaseCallback(&box_A, &box_B, &data);
  ASSERT_EQ(pairs.size(), 1u);
  // Order is guaranteed.
  EXPECT_EQ(pairs[0].first(), geometry_map[0]);
  EXPECT_EQ(pairs[0].second(), geometry_map[1]);
}

// This test verifies that the broad-phase callback respects filtering.
GTEST_TEST(BroadphaseCallback, RespectsCollisionFilter) {
  std::vector<GeometryId> geometry_map{GeometryId::get_new_id(),
                                       GeometryId::get_new_id()};
  CollisionFilterLegacy collision_filter;

  EncodedData data_A(GeometryIndex{0}, true);
  EncodedData data_B(GeometryIndex{1}, true);
  collision_filter.AddGeometry(data_A.encoding());
  collision_filter.AddGeometry(data_B.encoding());
  // Filter the pair (A, B) by adding them to the same clique.
  collision_filter.AddToCollisionClique(data_A.encoding(), 1);
  collision_filter.AddToCollisionClique(data_B.encoding(), 1);

  CollisionObjectd box_A(make_shared<Boxd>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  CollisionObjectd box_B(make_shared<Boxd>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  vector<SortedPair<GeometryId>> pairs;
  BroadphaseCallbackData data(&geometry_map, &collision_filter, &pairs);
  BroadphaseCallback(&box_A, &box_B, &data);
  EXPECT_EQ(pairs.size(), 0u);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
