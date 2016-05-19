#include <cmath>
#include <vector>

#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/collision/Model.h"
#include "gtest/gtest.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::AngleAxisd;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

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
TEST(ModelTest, closestPointsAllToAll) {
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
      AngleAxisd(M_PI_2,Vector3d::UnitZ()).toRotationMatrix();

  // Numerical precision tolerance to perform floating point comparisons.
  // For these very simple setup tests are expected to pass to machine
  // precision. More complex geometries might require a looser tolerance.
  const double tolerance = Eigen::NumTraits<double>::epsilon();

  DrakeShapes::Box geometry_1(Vector3d(1, 1, 1));
  DrakeShapes::Sphere geometry_2(0.5);
  DrakeShapes::Sphere geometry_3(0.5);
  Element element_1(geometry_1);
  Element element_2(geometry_2, T_elem2_to_body);
  Element element_3(geometry_3);

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
  EXPECT_NEAR(1.6213203435596428, points[1].getDistance(), tolerance);
  // Normal is on body B expressed in the world's frame.
  // Points are in the local frame of the body.
  EXPECT_TRUE(points[1].getNormal().isApprox(
      Vector3d(-sqrt(2) / 2, -sqrt(2) / 2, 0)));
  EXPECT_TRUE(points[1].getPtA().isApprox(
      Vector3d(0.5, 0.5, 0)));
  // Notice the y component is positive given that the body's frame is rotated
  // 90 degrees around the z axis.
  // Therefore x_body = y_world, y_body=-x_world and z_body=z_world
  EXPECT_TRUE(points[1].getPtB().isApprox(
      Vector3d(-sqrt(2) / 4, sqrt(2) / 4, 0)));

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

TEST(ModelTest, collisionPointsAllToAll_Box_vs_Sphere) {
  // Numerical precision tolerance to perform floating point comparisons.
  // For these very simple setup tests are expected to pass to machine
  // precision. More complex geometries might require a looser tolerance.
  const double tolerance = 2.0e-9;

  DrakeShapes::Box box(Vector3d(1.0, 1.0, 1.0));
  DrakeShapes::Sphere sphere(0.5);

  Element colliding_box(box);
  Element colliding_sphere(sphere);

  // Populate the model.
  std::unique_ptr<Model> model(newModel());
  ElementId box_id = model->addElement(colliding_box);
  ElementId sphere_id = model->addElement(colliding_sphere);

  // Body 1 pose
  Isometry3d box_pose;
  box_pose.setIdentity();
  box_pose.translation() = Vector3d(0.0,0.5,0.0);
  model->updateElementWorldTransform(box_id, box_pose);

  // Body 2 pose
  Isometry3d sphere_pose;
  sphere_pose.setIdentity();
  sphere_pose.translation() = Vector3d(0.0,1.25,0.0);
  model->updateElementWorldTransform(sphere_id, sphere_pose);

  // Compute collision points.
  std::vector<PointPair> points;

  // TODO(amcastro-tri): with `use_margins = true` the results are wrong. It
  // looks like the margins are not appropriatelysubtracted.
  model->collisionPointsAllToAll(false, points);
  //const std::vector<ElementId> ids_to_check = {box_id, sphere_id};
  //model->closestPointsAllToAll(ids_to_check, true, points);

  // Only one contact point is expected when colliding with a sphere.
  ASSERT_EQ(1, points.size());

  // Check the closest point between object 2 and object 3.
  EXPECT_EQ(box_id, points[0].getIdA());
  EXPECT_EQ(sphere_id, points[0].getIdB());
  EXPECT_NEAR(-0.25, points[0].getDistance(), tolerance);
  // Points are in the world frame on the surface of the corresponding body.
  // That is why getPtA() is generally different from getPtB(), unless there is
  // an exact non-penetrating collision.
  // WARNING:
  // This convention is different from the one used by closestPointsAllToAll
  // which computes points in the local frame of the body.
  // TODO(amcastro-tri): make these two conventions match? does this interfere
  // with any Matlab functionality?
  EXPECT_TRUE(points[0].getNormal().isApprox(Vector3d(0.0, -1.0, 0.0)));
  EXPECT_TRUE(points[0].getPtA().isApprox(Vector3d(0.0, 1.0, 0.0)));
  EXPECT_TRUE(points[0].getPtB().isApprox(Vector3d(0.0, 0.75, 0.0)));

  for(auto& pt_pair: points) {
    // Normal is on body B.
    PRINT_VAR(pt_pair.getNormal().transpose());
    PRINT_VAR(pt_pair.getDistance());

    PRINT_VAR(pt_pair.getIdA());
    PRINT_VAR(pt_pair.getPtA().transpose());

    PRINT_VAR(pt_pair.getIdB());
    PRINT_VAR(pt_pair.getPtB().transpose());
  }

}

TEST(ModelTest, collisionPointsAllToAll_SmallBoxSittingOnLargeBox) {
  // Numerical precision tolerance to perform floating point comparisons.
  // For these very simple setup tests are expected to pass to machine
  // precision. More complex geometries might require a looser tolerance.
  const double tolerance = 2.0e-9; //Eigen::NumTraits<double>::epsilon();

  DrakeShapes::Box large_box(Vector3d(5.0, 5.0, 5.0));
  DrakeShapes::Box small_box(Vector3d(1.0, 1.0, 1.0));

  Element colliding_large_box(large_box);
  Element colliding_small_box(small_box);

  // Populate the model.
  std::unique_ptr<Model> model(newModel());
  ElementId large_box_id = model->addElement(colliding_large_box);
  ElementId small_box_id = model->addElement(colliding_small_box);

  // Large body pose
  Isometry3d large_box_pose;
  large_box_pose.setIdentity();
  large_box_pose.translation() = Vector3d(0.0,2.5,0.0);
  model->updateElementWorldTransform(large_box_id, large_box_pose);

  // Small body pose
  Isometry3d small_box_pose;
  small_box_pose.setIdentity();
  small_box_pose.translation() = Vector3d(0.0,5.4,0.0);
  model->updateElementWorldTransform(small_box_id, small_box_pose);

  // Compute collision points.
  std::vector<PointPair> points;

  // TODO(amcastro-tri): with `use_margins = true` the results are wrong. It
  // looks like the margins are not appropriately subtracted.
  model->collisionPointsAllToAll(false, points);

  // Unfortunately Bullet's manifold has one point for this case.
  // Best for physics simulations would be Bullet to return at least the four
  // corners of the smaller box. However it randomly picks one corner.
  ASSERT_EQ(1, points.size());

  EXPECT_NEAR(-0.1, points[0].getDistance(), tolerance);

  EXPECT_TRUE(points[0].getNormal().isApprox(Vector3d(0.0, -1.0, 0.0)));

  // Bullet randomly picks one corner. Therefore it is impossible to assert
  // if that choice would change with future releases (say just because
  // tolerances changed).
  // Commented out below are the results from a previous run:
  //EXPECT_TRUE(points[0].getPtA().isApprox(Vector3d(0.5, 5.0, 0.5)));
  //EXPECT_TRUE(points[0].getPtB().isApprox(Vector3d(0.5, 4.9, 0.5)));

  std::cout << "Small box sitting on large box" << std::endl;
  for(auto& pt_pair: points) {
    // Normal is on body B.
    PRINT_VAR(pt_pair.getNormal().transpose());
    PRINT_VAR(pt_pair.getDistance());

    PRINT_VAR(pt_pair.getIdA());
    PRINT_VAR(pt_pair.getPtA().transpose());

    PRINT_VAR(pt_pair.getIdB());
    PRINT_VAR(pt_pair.getPtB().transpose());
  }

}

}  // namespace
}  // namespace DrakeCollision
