#include "drake/geometry/proximity/collisions_exist_callback.h"

#include <utility>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace has_collisions {
namespace {

using fcl::Boxd;
using fcl::CollisionObjectd;
using std::make_shared;

// TODO(tehbelinda - #10227): Add a test to ensure broad-phase culling exits
// early after finding the first contact. Ideally there's a nice way to
// mock/fake the callback and check it only gets called once despite multiple
// collisions.

// This test confirms that the callback correctly returns whether collisions
// exist. The pair of geometries (A, B) are created such that they collide.
// Geometry C is created like geometry B but translated beyond any of the
// existing box dimensions such that the pair (A, C) do not collide.
//
// Simplifed 2D representation:
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
GTEST_TEST(CollisionsExistCallback, Exist) {
  CollisionFilter collision_filter;

  const EncodedData data_A(GeometryId::get_new_id(), true);
  const EncodedData data_B(GeometryId::get_new_id(), true);
  const EncodedData data_C(GeometryId::get_new_id(), true);
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());
  collision_filter.AddGeometry(data_C.id());

  CollisionObjectd box_A(make_shared<Boxd>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  CollisionObjectd box_B(make_shared<Boxd>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);
  CollisionObjectd box_C(make_shared<Boxd>(0.4, 0.3, 0.2));
  box_C.setTranslation(Vector3<double>(1., 1., 1.));
  data_C.write_to(&box_C);

  CallbackData data(&collision_filter);
  Callback(&box_A, &box_B, &data);
  EXPECT_TRUE(data.collisions_exist);

  Callback(&box_A, &box_C, &data);
  EXPECT_FALSE(data.collisions_exist);
}

// This test verifies that the broad-phase callback respects filtering by
// creating a pair of geometries (A, B) that are in collision, then checking
// that they no longer collide once filtered out.
//
// Simplifed 2D representation:
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
GTEST_TEST(CollisionsExistCallback, RespectsCollisionFilter) {
  CollisionFilter collision_filter;

  EncodedData data_A(GeometryId::get_new_id(), true);
  EncodedData data_B(GeometryId::get_new_id(), true);
  collision_filter.AddGeometry(data_A.id());
  collision_filter.AddGeometry(data_B.id());

  CollisionObjectd box_A(make_shared<Boxd>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  CollisionObjectd box_B(make_shared<Boxd>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);

  // Make sure the pair collide.
  CallbackData data_before(&collision_filter);
  Callback(&box_A, &box_B, &data_before);
  EXPECT_TRUE(data_before.collisions_exist);

  // Filter the pair (A, B); we'll put the ids in a set and simply return that
  // set for the extract ids function.
  std::unordered_set<GeometryId> ids{data_A.id(), data_B.id()};
  CollisionFilter::ExtractIds extract = [&ids](const GeometrySet&) {
    return ids;
  };
  collision_filter.Apply(CollisionFilterDeclaration().ExcludeWithin(
                             GeometrySet{data_A.id(), data_B.id()}),
                         extract, false /* is_invariant */);

  // Make sure the pair no longer collides.
  CallbackData data_after(&collision_filter);
  Callback(&box_A, &box_B, &data_after);
  EXPECT_FALSE(data_after.collisions_exist);
}

}  // namespace
}  // namespace has_collisions
}  // namespace internal
}  // namespace geometry
}  // namespace drake
