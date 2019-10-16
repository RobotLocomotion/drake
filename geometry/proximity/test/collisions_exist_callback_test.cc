#include "drake/geometry/proximity/collisions_exist_callback.h"

#include <utility>
#include <vector>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace collisions_exist {
namespace {

using fcl::Boxd;
using fcl::CollisionObjectd;
using std::make_shared;
using std::vector;

// Confirms that the callback returns whether collisions exist.
GTEST_TEST(CollisionsExistCallback, Exist) {
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  const GeometryId id_C = GeometryId::get_new_id();
  CollisionFilterLegacy collision_filter;

  EncodedData data_A(id_A, true);
  EncodedData data_B(id_B, true);
  EncodedData data_C(id_C, true);
  collision_filter.AddGeometry(data_A.encoding());
  collision_filter.AddGeometry(data_B.encoding());
  collision_filter.AddGeometry(data_C.encoding());

  CollisionObjectd box_A(make_shared<Boxd>(0.25, 0.3, 0.4));
  data_A.write_to(&box_A);
  CollisionObjectd box_B(make_shared<Boxd>(0.4, 0.3, 0.2));
  data_B.write_to(&box_B);
  CollisionObjectd box_C(make_shared<Boxd>(0.4, 0.3, 0.2));
  box_C.setTranslation(Vector3<double>(1., 1., 1.));
  data_C.write_to(&box_C);

  CallbackData data(&collision_filter);
  Callback(&box_A, &box_B, &data);
  ASSERT_TRUE(data.exist);

  Callback(&box_A, &box_C, &data);
  ASSERT_FALSE(data.exist);
}

// This test verifies that the broad-phase callback respects filtering.
GTEST_TEST(CollisionsExistCallback, RespectsCollisionFilter) {
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

  CallbackData data(&collision_filter);
  Callback(&box_A, &box_B, &data);
  ASSERT_FALSE(data.exist);
}

}  // namespace
}  // namespace collisions_exist
}  // namespace internal
}  // namespace geometry
}  // namespace drake
