#include <cmath>
#include <vector>

#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/collision/Model.h"
#include "gtest/gtest.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::AngleAxisd;

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
GTEST_TEST(ModelTest, closestPointsAllToAll) {
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
  // rotate 90 degrees in z
  T_body3_to_world.linear() =
      AngleAxisd(M_PI_2, Vector3d::UnitZ()).toRotationMatrix();

  // Numerical precision tolerance to perform floating point comparisons.
  // For these very simple setup tests are expected to pass to machine
  // precision. More complex geometries might require a looser tolerance.
  const double tolerance = Eigen::NumTraits<double>::epsilon();

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
  EXPECT_NEAR(1.0, points[0].getDistance(), tolerance);
  // Normal is on body B expressed in the world's frame.
  // Points are in the local frame of the body.
  EXPECT_TRUE(points[0].getNormal().isApprox(Vector3d(-1, 0, 0)));
  EXPECT_TRUE(points[0].getPtA().isApprox(Vector3d(0.5, 0, 0)));
  EXPECT_TRUE(points[0].getPtB().isApprox(Vector3d(0.5, 0, 0)));

  // Check the closest point between object 1 and object 3.
  EXPECT_EQ(id1, points[1].getIdA());
  EXPECT_EQ(id3, points[1].getIdB());
  // exact_distance =
  // distance_between_centers -
  // box_center_to_corner_distance -
  // sphere_center_to_surface_distance =
  // = sqrt(8.0) - 1.0/sqrt(2.0) - 1/2.
  double exact_distance = sqrt(8.0) - 1.0 / sqrt(2.0) - 0.5;
  EXPECT_NEAR(exact_distance, points[1].getDistance(), tolerance);
  // Normal is on body B expressed in the world's frame.
  // Points are in the local frame of the body.
  EXPECT_TRUE(
      points[1].getNormal().isApprox(Vector3d(-sqrt(2) / 2, -sqrt(2) / 2, 0)));
  EXPECT_TRUE(points[1].getPtA().isApprox(Vector3d(0.5, 0.5, 0)));
  // Notice the y component is positive given that the body's frame is rotated
  // 90 degrees around the z axis.
  // Therefore x_body = y_world, y_body=-x_world and z_body=z_world
  EXPECT_TRUE(
      points[1].getPtB().isApprox(Vector3d(-sqrt(2) / 4, sqrt(2) / 4, 0)));

  // Check the closest point between object 2 and object 3.
  EXPECT_EQ(id2, points[2].getIdA());
  EXPECT_EQ(id3, points[2].getIdB());
  EXPECT_NEAR(1.0, points[2].getDistance(), tolerance);
  // Normal is on body B expressed in the world's frame.
  // Points are in the local frame of the body.
  EXPECT_TRUE(points[2].getNormal().isApprox(Vector3d(0, -1, 0)));
  EXPECT_TRUE(points[2].getPtA().isApprox(Vector3d(1, 0.5, 0)));
  EXPECT_TRUE(points[2].getPtB().isApprox(Vector3d(-0.5, 0, 0)));
}

GTEST_TEST(ModelTest, CollisionGroups) {
  CollisionElement element_1, element_2, element_3;

  // Adds element 1 to its own set of groups.
  element_1.AddToCollisionClique(2);
  element_1.AddToCollisionClique(23);
  element_1.AddToCollisionClique(11);
  element_1.AddToCollisionClique(15);
  element_1.AddToCollisionClique(9);
  std::vector<int> element_1_set = std::vector<int>{2, 9, 11, 15, 23};

  // Tests the situation where the same collision groups are added to a
  // collision element multiple times.
  // If a collision element is added to a group it already belongs to, the
  // addition has no effect. This is tested by asserting the total number of
  // elements in the test below.
  element_1.AddToCollisionClique(11);
  element_1.AddToCollisionClique(23);

  // Adds element 2 to its own set of groups.
  element_2.AddToCollisionClique(11);
  element_2.AddToCollisionClique(9);
  element_2.AddToCollisionClique(13);
  element_2.AddToCollisionClique(13);
  element_2.AddToCollisionClique(11);

  // Adds element 3 to its own set of groups.
  element_3.AddToCollisionClique(1);
  element_3.AddToCollisionClique(13);
  element_3.AddToCollisionClique(13);
  element_3.AddToCollisionClique(8);
  element_3.AddToCollisionClique(1);

  // Checks the correctness of each element's collision groups set.
  EXPECT_EQ(std::vector<int>({2, 9, 11, 15, 23}),
            element_1.collision_cliques());
  EXPECT_EQ(std::vector<int>({9, 11, 13}), element_2.collision_cliques());
  EXPECT_EQ(std::vector<int>({1, 8, 13}), element_3.collision_cliques());

  // Groups cannot be repeated. Therefore expect 5 groups instead of 7.
  ASSERT_EQ(5, element_1.number_of_cliques());

  // Groups cannot be repeated for element_2 either.
  ASSERT_EQ(3, element_2.number_of_cliques());

  // Groups cannot be repeated for element_3 either.
  ASSERT_EQ(3, element_3.number_of_cliques());

  // element_2 does not collide with element_1 (groups 9 and 11 in common).
  EXPECT_FALSE(element_2.CanCollideWith(&element_1));

  // element_2 does not collide with element_3 (group 13 in common).
  EXPECT_FALSE(element_2.CanCollideWith(&element_3));

  // element_3 does collide with element_1 (no groups in common).
  EXPECT_TRUE(element_3.CanCollideWith(&element_1));

  // element_3 does not collide with element_2 (group 13 in common).
  EXPECT_FALSE(element_3.CanCollideWith(&element_2));
}

}  // namespace
}  // namespace DrakeCollision
