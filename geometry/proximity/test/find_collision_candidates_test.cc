#include "drake/geometry/proximity/find_collision_candidates.h"

#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace find_collision_candidates {
namespace {

using fcl::Boxd;
using fcl::CollisionObjectd;
using std::make_shared;
using std::vector;

// Confirms that the callback properly returns a sorted pair for the
// corresponding geometry ids.
GTEST_TEST(Callback, PairsProperlyFormed) {
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  CollisionFilterLegacy collision_filter;

  EncodedData data_A(id_A, true);
  EncodedData data_B(id_B, true);
  collision_filter.AddGeometry(data_A.encoding());
  collision_filter.AddGeometry(data_B.encoding());

  CollisionObjectd box_A(make_shared<Boxd>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  CollisionObjectd box_B(make_shared<Boxd>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  vector<SortedPair<GeometryId>> pairs;
  CallbackData data(&collision_filter, &pairs);
  Callback(&box_A, &box_B, &data);
  ASSERT_EQ(pairs.size(), 1u);
  const SortedPair<GeometryId> expected_pair{id_A, id_B};
  EXPECT_EQ(pairs[0], expected_pair);

  // We verify that the order the callback receives it doesn't affect the
  // results.
  pairs.clear();
  Callback(&box_B, &box_A, &data);
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0], expected_pair);
}

// This test verifies that the broad-phase callback respects filtering.
GTEST_TEST(Callback, RespectsCollisionFilter) {
  CollisionFilterLegacy collision_filter;

  EncodedData data_A(GeometryId::get_new_id(), true);
  EncodedData data_B(GeometryId::get_new_id(), true);
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
  CallbackData data(&collision_filter, &pairs);
  Callback(&box_A, &box_B, &data);
  EXPECT_EQ(pairs.size(), 0u);
}

}  // namespace
}  // namespace find_collision_candidates
}  // namespace internal
}  // namespace geometry
}  // namespace drake
