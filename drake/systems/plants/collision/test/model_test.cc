#include <cmath>
#include <vector>

#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/collision/Model.h"
#include "gtest/gtest.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;

namespace DrakeCollision {
namespace {

/*
 * Three bodies (cube (1 m edges) and two spheres (0.5 m radii) arranged like
 *this
 *
 *                             *****
 *                           **     **
 *                           *   3   * --+--
 *                           **     **   |
 *                             *****     |
 *                                       |
 *          ^                           2 m
 *        y |                            |
 *          |                            |
 *      +---+---+              *****     |
 *      |   |   |            **      *   |
 *      |   +---+---->       *   2   * --+--
 *      | 1     |   x        **     **
 *      +-------+              *****
 *          |                    |
 *          +------ 2 m ---------+
 *          |                    |
 *
 */
TEST(ModelTest, ClosestPointsAllToAll) {
  // Set up the geometry.
  Isometry3d T_body1_to_world, T_body2_to_world, T_body3_to_world,
      T_elem2_to_body;
  T_body1_to_world.setIdentity();
  T_body2_to_world.setIdentity();
  T_body3_to_world.setIdentity();
  T_body2_to_world.translation() << 1, 0, 0;
  T_elem2_to_body.setIdentity();
  T_elem2_to_body.translation() << 1, 0, 0;
  T_body3_to_world.translation() << 2, 2, 0;
  T_body3_to_world.matrix().block<2, 2>(0, 0) << 0, -1, 1,
      0;  // rotate 90 degrees in z

  DrakeShapes::Box geometry_1(Vector3d(1, 1, 1));
  DrakeShapes::Sphere geometry_2(0.5);
  DrakeShapes::Sphere geometry_3(0.5);
  CollisionElement element_1(geometry_1);
  CollisionElement element_2(geometry_2, T_elem2_to_body);
  CollisionElement element_3(geometry_3);

  // Populate the model.
  std::shared_ptr<Model> model = newModel();
  ElementId id1 = model->addElement(element_1);
  ElementId id2 = model->addElement(element_2);
  ElementId id3 = model->addElement(element_3);
  model->updateElementWorldTransform(id1, T_body1_to_world);
  model->updateElementWorldTransform(id2, T_body2_to_world);
  model->updateElementWorldTransform(id3, T_body3_to_world);

  // Compute the closest points.
  const std::vector<ElementId> ids_to_check = {id1, id2, id3};
  std::vector<PointPair> points;
  model->closestPointsAllToAll(ids_to_check, true, points);
  ASSERT_EQ(3, points.size());

  // Check the closest point between object 1 and object 2.
  // TODO(david-german-tri): Migrate this test to use Eigen matchers once
  // they are available.
  EXPECT_EQ(id1, points[0].getIdA());
  EXPECT_EQ(id2, points[0].getIdB());
  EXPECT_EQ(1.0, points[0].getDistance());
  EXPECT_EQ(Vector3d(-1, 0, 0), points[0].getNormal());
  EXPECT_TRUE((Vector3d(0.5, 0, 0) - points[0].getPtA()).isZero());
  EXPECT_TRUE((Vector3d(0.5, 0, 0) - points[0].getPtB()).isZero());

  // Check the closest point between object 1 and object 3.
  EXPECT_EQ(id1, points[1].getIdA());
  EXPECT_EQ(id3, points[1].getIdB());
  EXPECT_EQ(1.6213203435596428, points[1].getDistance());
  EXPECT_EQ(Vector3d(-sqrt(2) / 2, -sqrt(2) / 2, 0), points[1].getNormal());
  EXPECT_TRUE((Vector3d(0.5, 0.5, 0) - points[1].getPtA()).isZero());
  EXPECT_TRUE(
      (Vector3d(-sqrt(2) / 4, sqrt(2) / 4, 0) - points[1].getPtB()).isZero());

  // Check the closest point between object 2 and object 3.
  EXPECT_EQ(id2, points[2].getIdA());
  EXPECT_EQ(id3, points[2].getIdB());
  EXPECT_EQ(1.0, points[2].getDistance());
  EXPECT_EQ(Vector3d(0, -1, 0), points[2].getNormal());
  EXPECT_TRUE((Vector3d(1, 0.5, 0) - points[2].getPtA()).isZero());
  EXPECT_TRUE((Vector3d(-0.5, 0, 0) - points[2].getPtB()).isZero());
}

TEST(ModelTest, CollisionGroups) {
  CollisionElement element_1, element_2, element_3;

  // Adds element 1 to its own set of groups.
  element_1.add_to_collision_group(2);
  element_1.add_to_collision_group(23);
  element_1.add_to_collision_group(11);
  element_1.add_to_collision_group(15);
  element_1.add_to_collision_group(9);
  std::vector<int> element_1_set = std::vector<int>({2, 9, 11, 15, 23});

  // Tests the situation where the same collision groups are added to a
  // collision element multiple times.
  // If a collision element is added to a group it already belongs to, the
  // addition has no effect. This is tested by asserting the total number of
  // elements in the test below.
  element_1.add_to_collision_group(11);
  element_1.add_to_collision_group(23);

  // Adds element 2 to its own set of groups.
  element_2.add_to_collision_group(11);
  element_2.add_to_collision_group(9);
  element_2.add_to_collision_group(13);
  element_2.add_to_collision_group(13);
  element_2.add_to_collision_group(11);

  // Adds element 3 to its own set of groups.
  element_3.add_to_collision_group(1);
  element_3.add_to_collision_group(13);
  element_3.add_to_collision_group(13);
  element_3.add_to_collision_group(8);
  element_3.add_to_collision_group(1);

  // Checks the correctness of each element's collision groups set.
  EXPECT_EQ(std::vector<int>({2, 9, 11, 15, 23}), element_1.collision_groups());
  EXPECT_EQ(std::vector<int>({9, 11, 13}), element_2.collision_groups());
  EXPECT_EQ(std::vector<int>({1, 8, 13}), element_3.collision_groups());

  // Groups cannot be repeated. Therefore expect 5 groups instead of 7.
  ASSERT_EQ(5, element_1.number_of_groups());

  // Groups cannot be repeated for element_2 either.
  ASSERT_EQ(3, element_2.number_of_groups());

  // Groups cannot be repeated for element_3 either.
  ASSERT_EQ(3, element_3.number_of_groups());

  // element_2 does not collide with element_1 (groups 9 and 11 in common).
  EXPECT_FALSE(element_2.CollidesWith(&element_1));

  // element_2 does not collide with element_3 (group 13 in common).
  EXPECT_FALSE(element_2.CollidesWith(&element_3));

  // element_3 does collide with element_1 (no groups in common).
  EXPECT_TRUE(element_3.CollidesWith(&element_1));

  // element_3 does not collide with element_2 (group 13 in common).
  EXPECT_FALSE(element_3.CollidesWith(&element_2));
}

}  // namespace
}  // namespace DrakeCollision
