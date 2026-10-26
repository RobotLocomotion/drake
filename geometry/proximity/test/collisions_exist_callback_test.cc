#include "drake/geometry/proximity/collisions_exist_callback.h"

#include <memory>
#include <unordered_set>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/proximity/coal_backend.h"
#include "drake/geometry/proximity/fcl_backend.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace has_collisions {
namespace {

using std::make_shared;

/* These tests exercise bookkeeping -- the collision filter and the
 "did anything touch" flag -- not numerics, so both backends must agree
 exactly. */
template <typename Backend>
class CollisionsExistCallbackTest : public ::testing::Test {};
using Backends = ::testing::Types<FclBackend, CoalBackend>;
TYPED_TEST_SUITE(CollisionsExistCallbackTest, Backends);

// TODO(tehbelinda - #10227): Add a test to ensure broad-phase culling exits
// early after finding the first contact. Ideally there's a nice way to
// mock/fake the callback and check it only gets called once despite multiple
// collisions.

// This test confirms that the callback correctly returns whether collisions
// exist. The pair of geometries (A, B) are created such that they collide.
// Geometry C is created like geometry B but translated beyond any of the
// existing box dimensions such that the pair (A, C) do not collide.
//
// Simplified 2D representation:
//
//          A
//    ┏━━━┓
//    ┃   ┃
// ┌──╂───╂──┐ B
// │  ┃   ┃  │
// │  ┃   ┃  │
// └──╂───╂──┘
//    ┃   ┃
//    ┗━━━┛
//                  ┌┄┄┄┄┄┄┄┄┐ C
//                  ┆        ┆
//                  ┆        ┆
//                  └┄┄┄┄┄┄┄┄┘
//
TYPED_TEST(CollisionsExistCallbackTest, Exist) {
  using Backend = TypeParam;
  CollisionFilter collision_filter;

  const EncodedData data_A(GeometryId::get_new_id(), true);
  const EncodedData data_B(GeometryId::get_new_id(), true);
  const EncodedData data_C(GeometryId::get_new_id(), true);
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());
  collision_filter.AddGeometry(data_C.id());

  typename Backend::Object box_A(
      make_shared<typename Backend::Box>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  typename Backend::Object box_B(
      make_shared<typename Backend::Box>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);
  typename Backend::Object box_C(
      make_shared<typename Backend::Box>(0.4, 0.3, 0.2));
  box_C.setTranslation(Vector3<double>(1., 1., 1.));
  data_C.write_to(&box_C);

  CallbackData<Backend> data(&collision_filter);
  Callback<Backend>(&box_A, &box_B, &data);
  EXPECT_TRUE(data.collisions_exist);

  Callback<Backend>(&box_A, &box_C, &data);
  EXPECT_FALSE(data.collisions_exist);
}

// This test verifies that the broad-phase callback respects filtering by
// creating a pair of geometries (A, B) that are in collision, then checking
// that they no longer collide once filtered out.
//
// Simplified 2D representation:
//
//          A
//    ┏━━━┓
//    ┃   ┃
// ┌──╂───╂──┐ B
// │  ┃   ┃  │
// │  ┃   ┃  │
// └──╂───╂──┘
//    ┃   ┃
//    ┗━━━┛
//
TYPED_TEST(CollisionsExistCallbackTest, RespectsCollisionFilter) {
  using Backend = TypeParam;
  CollisionFilter collision_filter;

  EncodedData data_A(GeometryId::get_new_id(), true);
  EncodedData data_B(GeometryId::get_new_id(), true);
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());

  typename Backend::Object box_A(
      make_shared<typename Backend::Box>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  typename Backend::Object box_B(
      make_shared<typename Backend::Box>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  // Make sure the pair collide.
  CallbackData<Backend> data_before(&collision_filter);
  Callback<Backend>(&box_A, &box_B, &data_before);
  EXPECT_TRUE(data_before.collisions_exist);

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

  // Make sure the pair no longer collides.
  CallbackData<Backend> data_after(&collision_filter);
  Callback<Backend>(&box_A, &box_B, &data_after);
  EXPECT_FALSE(data_after.collisions_exist);
}

}  // namespace
}  // namespace has_collisions
}  // namespace internal
}  // namespace geometry
}  // namespace drake
