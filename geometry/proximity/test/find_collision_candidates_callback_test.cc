#include "drake/geometry/proximity/find_collision_candidates_callback.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/coal_backend.h"
#include "drake/geometry/proximity/fcl_backend.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace find_collision_candidates {
namespace {

/* This callback only reports candidate pairs; it runs no narrowphase, so
 both backends must agree exactly. */
template <typename Backend>
class FindCollisionCandidatesTest : public ::testing::Test {};
using Backends = ::testing::Types<FclBackend, CoalBackend>;
TYPED_TEST_SUITE(FindCollisionCandidatesTest, Backends);
using std::make_shared;
using std::vector;

// Confirms that the callback properly returns a sorted pair for the
// corresponding geometry ids.
TYPED_TEST(FindCollisionCandidatesTest, PairsProperlyFormed) {
  using Backend = TypeParam;
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  CollisionFilter collision_filter;

  EncodedData data_A(id_A, true);
  EncodedData data_B(id_B, true);
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());

  typename Backend::Object box_A(
      make_shared<typename Backend::Box>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  typename Backend::Object box_B(
      make_shared<typename Backend::Box>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  vector<SortedPair<GeometryId>> pairs;
  CallbackData data(&collision_filter, &pairs);
  Callback<Backend>(&box_A, &box_B, &data);
  ASSERT_EQ(pairs.size(), 1u);
  const SortedPair<GeometryId> expected_pair{id_A, id_B};
  EXPECT_EQ(pairs[0], expected_pair);

  // We verify that the order the callback receives it doesn't affect the
  // results.
  pairs.clear();
  Callback<Backend>(&box_B, &box_A, &data);
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0], expected_pair);
}

// This test verifies that the broad-phase callback respects filtering.
TYPED_TEST(FindCollisionCandidatesTest, RespectsCollisionFilter) {
  using Backend = TypeParam;
  CollisionFilter collision_filter;

  EncodedData data_A(GeometryId::get_new_id(), true);
  EncodedData data_B(GeometryId::get_new_id(), true);
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());
  // Filter the pair (A, B); we'll put the ids in a set and simply return that
  // set for the extract ids function.
  std::unordered_set<GeometryId> ids{data_A.id(), data_B.id()};
  CollisionFilter::ExtractIds extract = [&ids](const GeometrySet&,
                                               CollisionFilterScope) {
    return ids;
  };
  collision_filter.Apply(CollisionFilterDeclaration().ExcludeWithin(
                             GeometrySet{data_A.id(), data_B.id()}),
                         extract, false /* is_invariant */);

  typename Backend::Object box_A(
      make_shared<typename Backend::Box>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  typename Backend::Object box_B(
      make_shared<typename Backend::Box>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  vector<SortedPair<GeometryId>> pairs;
  CallbackData data(&collision_filter, &pairs);
  Callback<Backend>(&box_A, &box_B, &data);
  EXPECT_EQ(pairs.size(), 0u);
}

}  // namespace
}  // namespace find_collision_candidates
}  // namespace internal
}  // namespace geometry
}  // namespace drake
