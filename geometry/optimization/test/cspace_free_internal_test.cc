#include "drake/geometry/optimization/cspace_free_internal.h"

#include <unordered_set>

#include <gtest/gtest.h>

#include "drake/geometry/optimization/test/c_iris_test_utilities.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {

TEST_F(CIrisToyRobotTest, GetCollisionGeometries) {
  const auto link_geometries = GetCollisionGeometries(*plant_, *scene_graph_);
  // Each link has some geometries.
  EXPECT_EQ(link_geometries.size(), plant_->num_bodies());

  auto check_link_geometries =
      [&link_geometries](const multibody::BodyIndex body,
                         const std::unordered_set<geometry::GeometryId>&
                             geometry_ids_expected) {
        auto it = link_geometries.find(body);
        std::unordered_set<geometry::GeometryId> geometry_ids;
        for (const auto& geometry : it->second) {
          EXPECT_EQ(geometry->body_index(), body);
          geometry_ids.emplace(geometry->id());
        }
        EXPECT_EQ(geometry_ids.size(), geometry_ids_expected.size());
        for (const auto id : geometry_ids) {
          EXPECT_TRUE(geometry_ids_expected.contains(id));
        }
      };

  check_link_geometries(plant_->world_body().index(),
                        {world_box_, world_cylinder_});
  check_link_geometries(body_indices_[0], {body0_box_, body0_sphere_});
  check_link_geometries(body_indices_[1], {body1_convex_, body1_capsule_});
  check_link_geometries(body_indices_[2], {body2_sphere_, body2_capsule_});
  check_link_geometries(body_indices_[3], {body3_box_, body3_cylinder_});
}

}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
