#include "drake/multibody/collision/model.h"

#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/collision/drake_collision.h"

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Matrix3Xd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace DrakeCollision {
namespace {

// Structure used to hold the analytical solution of the tests.
// It stores the collision point on the surface of a collision body in both
// world and body frames.
struct SurfacePoint {
  SurfacePoint() {}
  SurfacePoint(Vector3d wf, Vector3d bf) : world_frame(wf), body_frame(bf) {}
  // Eigen variables are left uninitalized by default.
  Vector3d world_frame;
  Vector3d body_frame;
};

// Solutions are accessed by collision element id using an std::unordered_set.
// DrakeCollision::Model returns the collision detection results as a vector of
// DrakeCollision::PointPair entries. Each entry holds a reference to the pair
// of collision elements taking part in the collision. Collision elements are
// referenced by their id.
// The order in which the pair of elements is stored in a PointPair cannot
// be guaranteed, and therefore we cannot guarantee the return of
// PointPair::idA and PointPair::idB in our tests.
// This means we cannot guarantee that future versions of the underlying
// implementation (say Bullet, FCL) won't change this order (since unfortunately
// id's are merely a memory address cast to an integer).
// The user only has access to collision elements by id.
// To provide a unique mapping between id's and the analytical solution to the
// contact point on a specific element here we use an `std::unordered_set` to
// map id's to a `SurfacePoint` structure holding the analytical solution on
// both body and world frames.
typedef std::unordered_map<const DrakeCollision::Element*, SurfacePoint>
    ElementToSurfacePointMap;

// GENERAL REMARKS ON THE TESTS PERFORMED
// A series of canonical tests are performed. These are Box_vs_Sphere,
// SmallBoxSittingOnLargeBox and NonAlignedBoxes.
// These tests are performed using the following algorithms:
// - closestPointsAllToAll: O(N^2) checking all pairs of collision elements.
//   Results are in bodies' frames.
// - ComputeMaximumDepthCollisionPoints: Uses collision library dispatching.
//   Results are in world frame.
// - potentialCollisionPoints: randomly samples collision elements' pose
//   searching for potential collision points. Results are in the bodies'
//   frames.

// CLEARING CACHED RESULTS TEST
// Test ClearCachedResults tests the model is not caching results when a series
// of collision dispatch queries is performed. This ensures results do not
// depend on the past history and that the user gets a fresh query every time.

// REMARKS ON MULTICONTACT
// Results from Model::potentialCollisionPoints are affected by previous calls
// to Model::ComputeMaximumDepthCollisionPoints even if cached results are
// cleared with Model::ClearCachedResults. This is the reason why multicontact
// tests using Model::potentialCollisionPoints are performed separately in their
// own tests.
// For instance, for the NonAlignedBoxes there is a separate test called
// NonAlignedBoxes_multi performed with Model::potentialCollisionPoints.
// It seems like Bullet is storing some additional configuration setup with the
// call to Model::ComputeMaximumDepthCollisionPoints that persists throughout
// subsequent calls to Model::potentialCollisionPoints.
// Therefore, DO NOT mix these calls since they might produce erroneous results.
// Notice also that the tolerance used for these tests is much higher. The
// reason is that Bullet randomly changes the pose of the collision elements
// searching for potential collision points. This is the additional source of
// error introduced.

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

  // Populate the model.
  unique_ptr<Model> model = newModel();
  auto element_1 = model->AddElement(make_unique<Element>(geometry_1));
  auto element_2 = model->AddElement(
      make_unique<Element>(geometry_2, T_elem2_to_body));
  auto element_3 = model->AddElement(make_unique<Element>(geometry_3));
  ElementId id1 = element_1->getId();
  ElementId id2 = element_2->getId();
  ElementId id3 = element_3->getId();
  model->updateElementWorldTransform(id1, T_body1_to_world);
  model->updateElementWorldTransform(id2, T_body2_to_world);
  model->updateElementWorldTransform(id3, T_body3_to_world);

  // Compute the closest points.
  const std::vector<ElementId> ids_to_check = {id1, id2, id3};
  std::vector<PointPair> points;
  model->closestPointsAllToAll(ids_to_check, true, points);
  ASSERT_EQ(3u, points.size());

  // Check the closest point between object 1 and object 2.
  // TODO(david-german-tri): Migrate this test to use Eigen matchers once
  // they are available.
  EXPECT_EQ(id1, points[0].idA);
  EXPECT_EQ(id2, points[0].idB);
  EXPECT_NEAR(1.0, points[0].distance, tolerance);
  // Normal is on body B expressed in the world's frame.
  // Points are in the local frame of the body.
  EXPECT_TRUE(points[0].normal.isApprox(Vector3d(-1, 0, 0)));
  EXPECT_TRUE(points[0].ptA.isApprox(Vector3d(0.5, 0, 0)));
  EXPECT_TRUE(points[0].ptB.isApprox(Vector3d(0.5, 0, 0)));

  // Check the closest point between object 1 and object 3.
  EXPECT_EQ(id1, points[1].idA);
  EXPECT_EQ(id3, points[1].idB);
  // exact_distance =
  // distance_between_centers -
  // box_center_to_corner_distance -
  // sphere_center_to_surface_distance =
  // = sqrt(8.0) - 1.0/sqrt(2.0) - 1/2.
  double exact_distance = sqrt(8.0) - 1.0 / sqrt(2.0) - 0.5;
  EXPECT_NEAR(exact_distance, points[1].distance, tolerance);
  // Normal is on body B expressed in the world's frame.
  // Points are in the local frame of the body.
  EXPECT_TRUE(
      points[1].normal.isApprox(Vector3d(-sqrt(2) / 2, -sqrt(2) / 2, 0)));
  EXPECT_TRUE(points[1].ptA.isApprox(Vector3d(0.5, 0.5, 0)));
  // Notice the y component is positive given that the body's frame is rotated
  // 90 degrees around the z axis.
  // Therefore x_body = y_world, y_body=-x_world and z_body=z_world
  EXPECT_TRUE(
      points[1].ptB.isApprox(Vector3d(-sqrt(2) / 4, sqrt(2) / 4, 0)));

  // Check the closest point between object 2 and object 3.
  EXPECT_EQ(id2, points[2].idA);
  EXPECT_EQ(id3, points[2].idB);
  EXPECT_NEAR(1.0, points[2].distance, tolerance);
  // Normal is on body B expressed in the world's frame.
  // Points are in the local frame of the body.
  EXPECT_TRUE(points[2].normal.isApprox(Vector3d(0, -1, 0)));
  EXPECT_TRUE(points[2].ptA.isApprox(Vector3d(1, 0.5, 0)));
  EXPECT_TRUE(points[2].ptB.isApprox(Vector3d(-0.5, 0, 0)));
}

GTEST_TEST(ModelTest, CollisionCliques) {
  Element element_1, element_2, element_3;

  // Adds element 1 to its own set of cliques.
  element_1.AddToCollisionClique(2);
  element_1.AddToCollisionClique(23);
  element_1.AddToCollisionClique(11);
  element_1.AddToCollisionClique(15);
  element_1.AddToCollisionClique(9);

  // Tests the situation where the same collision cliques are added to a
  // collision element multiple times.
  // If a collision element is added to a clique it already belongs to, the
  // addition has no effect. This is tested by asserting the total number of
  // elements in the test below.
  element_1.AddToCollisionClique(11);
  element_1.AddToCollisionClique(23);

  // Cliques cannot be repeated. Therefore expect 5 cliques instead of 7.
  ASSERT_EQ(5, element_1.get_num_cliques());
  // Checks the correctness of the entire set.
  EXPECT_EQ(std::vector<int>({2, 9, 11, 15, 23}),
            element_1.collision_cliques());

  // Adds element 2 to its own set of cliques.
  element_2.AddToCollisionClique(11);
  element_2.AddToCollisionClique(9);
  element_2.AddToCollisionClique(13);
  element_2.AddToCollisionClique(13);
  element_2.AddToCollisionClique(11);

  // Cliques cannot be repeated. Therefore expect 3 cliques instead of 5.
  ASSERT_EQ(3, element_2.get_num_cliques());
  // Checks the correctness of the entire set.
  EXPECT_EQ(std::vector<int>({9, 11, 13}), element_2.collision_cliques());

  // Adds element 3 to its own set of cliques.
  element_3.AddToCollisionClique(1);
  element_3.AddToCollisionClique(13);
  element_3.AddToCollisionClique(13);
  element_3.AddToCollisionClique(8);
  element_3.AddToCollisionClique(1);

  // Cliques cannot be repeated. Therefore expect 3 cliques instead of 5.
  ASSERT_EQ(3, element_3.get_num_cliques());
  // Checks the correctness of the entire set.
  EXPECT_EQ(std::vector<int>({1, 8, 13}), element_3.collision_cliques());

  // element_2 does not collide with element_1 (cliques 9 and 11 in common).
  EXPECT_FALSE(element_2.CanCollideWith(&element_1));

  // element_2 does not collide with element_3 (clique 13 in common).
  EXPECT_FALSE(element_2.CanCollideWith(&element_3));

  // element_3 does collide with element_1 (no cliques in common).
  EXPECT_TRUE(element_3.CanCollideWith(&element_1));

  // element_3 does not collide with element_2 (clique 13 in common).
  EXPECT_FALSE(element_3.CanCollideWith(&element_2));
}

// A sphere of diameter 1.0 is placed on top of a box with sides of length 1.0.
// The sphere overlaps with the box with its deepest penetration point (the
// bottom) 0.25 units into the box (negative distance). Only one contact point
// is expected when colliding with a sphere.
class BoxVsSphereTest : public ::testing::Test {
 public:
  void SetUp() override {
    DrakeShapes::Box box(Vector3d(1.0, 1.0, 1.0));
    DrakeShapes::Sphere sphere(0.5);

    // Populate the model.
    model_ = newModel();
    box_ = model_->AddElement(make_unique<Element>(box));
    sphere_ = model_->AddElement(make_unique<Element>(sphere));

    // Access the analytical solution to the contact point on the surface of
    // each collision element by element id.
    // Solutions are expressed in world and body frames.
    solution_ = {
        /*           world frame     , body frame  */
        {box_,    {{0.0,  1.0, 0.0}, {0.0,  0.5, 0.0}}},
        {sphere_, {{0.0, 0.75, 0.0}, {0.0, -0.5, 0.0}}}};

    // Body 1 pose
    Isometry3d box_pose;
    box_pose.setIdentity();
    box_pose.translation() = Vector3d(0.0, 0.5, 0.0);
    model_->updateElementWorldTransform(box_->getId(), box_pose);

    // Body 2 pose
    Isometry3d sphere_pose;
    sphere_pose.setIdentity();
    sphere_pose.translation() = Vector3d(0.0, 1.25, 0.0);
    model_->updateElementWorldTransform(sphere_->getId(), sphere_pose);
  }

 protected:
  double tolerance_;
  std::unique_ptr<Model> model_;
  Element* box_, * sphere_;
  ElementToSurfacePointMap solution_;
};

TEST_F(BoxVsSphereTest, SingleContact) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 2.0e-9;

  // List of collision points.
  std::vector<PointPair> points;

  // Collision test performed with Model::closestPointsAllToAll.
  const std::vector<ElementId> ids_to_check = {box_->getId(), sphere_->getId()};
  model_->closestPointsAllToAll(ids_to_check, true, points);
  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.25, points[0].distance, tolerance_);
  // Points are in the bodies' frame on the surface of the corresponding body.
  EXPECT_TRUE(points[0].normal.isApprox(Vector3d(0.0, -1.0, 0.0)));
  EXPECT_TRUE(
      points[0].ptA.isApprox(solution_[points[0].elementA].body_frame));
  EXPECT_TRUE(
      points[0].ptB.isApprox(solution_[points[0].elementB].body_frame));

  // Collision test performed with Model::ComputeMaximumDepthCollisionPoints.
  // Not using margins.
  points.clear();
  model_->ComputeMaximumDepthCollisionPoints(false, points);
  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.25, points[0].distance, tolerance_);
  // Points are in the world frame on the surface of the corresponding body.
  // That is why ptA is generally different from ptB, unless there is
  // an exact non-penetrating collision.
  // WARNING:
  // This convention is different from the one used by closestPointsAllToAll
  // which computes points in the local frame of the body.
  // TODO(amcastro-tri): make these two conventions match? does this interfere
  // with any Matlab functionality?
  EXPECT_TRUE(points[0].normal.isApprox(Vector3d(0.0, -1.0, 0.0)));
  EXPECT_TRUE(
      points[0].ptA.isApprox(solution_[points[0].elementA].world_frame));
  EXPECT_TRUE(
      points[0].ptB.isApprox(solution_[points[0].elementB].world_frame));

  // Collision test performed with Model::ComputeMaximumDepthCollisionPoints.
  // Using margins.
  points.clear();
  model_->ComputeMaximumDepthCollisionPoints(true, points);
  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.25, points[0].distance, tolerance_);
  // Points are in the world frame on the surface of the corresponding body.
  // That is why ptA is generally different from ptB, unless there is
  // an exact non-penetrating collision.
  // WARNING:
  // This convention is different from the one used by closestPointsAllToAll
  // which computes points in the local frame of the body.
  // TODO(amcastro-tri): make these two conventions match? does this interfere
  // with any Matlab functionality?
  EXPECT_TRUE(points[0].normal.isApprox(Vector3d(0.0, -1.0, 0.0)));
  EXPECT_TRUE(
      points[0].ptA.isApprox(solution_[points[0].elementA].world_frame));
  EXPECT_TRUE(
      points[0].ptB.isApprox(solution_[points[0].elementB].world_frame));

  // Calls to BulletModel::potentialCollisionPoints cannot be mixed.
  // Therefore throw an exception if user attempts to call
  // potentialCollisionPoints after calling another dispatch method.
  points.clear();
  EXPECT_THROW(points = model_->potentialCollisionPoints(false),
               std::runtime_error);
}

// This test is exactly the same as the previous Box_vs_Sphere test.
// The difference is that a multi-contact algorithm is being used.
// See REMARKS ON MULTICONTACT at the top of this file.
TEST_F(BoxVsSphereTest, MultiContact) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 2.0e-4;

  // List of collision points.
  std::vector<PointPair> points;

  // Collision test performed with Model::potentialCollisionPoints.
  points.clear();
  points = model_->potentialCollisionPoints(false);

  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.25, points[0].distance, tolerance_);
  // Points are in the bodies' frame on the surface of the corresponding body.
  EXPECT_TRUE(points[0].normal.isApprox(Vector3d(0.0, -1.0, 0.0)));

  // Ensures the vertical coordinate is computed within tolerance_.
  EXPECT_NEAR(points[0].ptA.y(),
              solution_[points[0].elementA].body_frame.y(), tolerance_);
  EXPECT_NEAR(points[0].ptB.y(),
              solution_[points[0].elementB].body_frame.y(), tolerance_);

  // Notice however that the x and z coordinates are computed with a much
  // larger error.
  EXPECT_TRUE(points[0].ptA.isApprox(
      solution_[points[0].elementA].body_frame, tolerance_ * 200));
  EXPECT_TRUE(points[0].ptB.isApprox(
      solution_[points[0].elementB].body_frame, tolerance_ * 200));
}

// This test seeks to find out whether DrakeCollision::Model can report
// collision manifolds. To this end, a small cube with unit length sides is
// placed on top of a large cube with sides of length 5.0. The smaller cube is
// placed such that it intersects the large box. Therefore the intersection
// between the two boxes is not just a single point but the (squared) perimeter
// all around the smaller box (the manifold).
//
// Unfortunately these tests show that DrakeCollision::Model only reports a
// single (randomly chosen) point at one of the smaller box corners. In previous
// runs this was the corner at (0.5, 0.5, z) where z = 5.0 for the top of the
// large box and z = 4.9 for the bottom of the smaller box.
class SmallBoxSittingOnLargeBox: public ::testing::Test {
 public:
  void SetUp() override {
    // Boxes centered around the origin in their local frames.
    DrakeShapes::Box large_box(Vector3d(5.0, 5.0, 5.0));
    DrakeShapes::Box small_box(Vector3d(1.0, 1.0, 1.0));

    // Populate the model.
    model_ = newModel();
    large_box_ = model_->AddElement(make_unique<Element>(large_box));
    small_box_ = model_->AddElement(make_unique<Element>(small_box));

    // Access the analytical solution to the contact point on the surface of
    // each collision element by element id.
    // Solutions are expressed in world and body frames.
    solution_ = {
        /*              world frame    , body frame  */
        {large_box_, {{0.0, 5.0, 0.0}, {0.0,  2.5, 0.0}}},
        {small_box_, {{0.0, 4.9, 0.0}, {0.0, -0.5, 0.0}}}};

    // Large body pose
    Isometry3d large_box_pose;
    large_box_pose.setIdentity();
    large_box_pose.translation() = Vector3d(0.0, 2.5, 0.0);
    model_->updateElementWorldTransform(large_box_->getId(), large_box_pose);

    // Small body pose
    Isometry3d small_box_pose;
    small_box_pose.setIdentity();
    small_box_pose.translation() = Vector3d(0.0, 5.4, 0.0);
    model_->updateElementWorldTransform(small_box_->getId(), small_box_pose);
  }

 protected:
  double tolerance_;
  std::unique_ptr<Model> model_;
  Element* small_box_, * large_box_;
  ElementToSurfacePointMap solution_;
};

TEST_F(SmallBoxSittingOnLargeBox, SingleContact) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 2.0e-9;

  // List of collision points.
  std::vector<PointPair> points;

  // Unfortunately DrakeCollision::Model is randomly selecting one of the small
  // box's corners instead of reporting a manifold describing the perimeter of
  // the square where both boxes intersect.
  // Therefore it is impossible to assert if that choice would change with
  // future releases (say just because tolerances changed).
  // What we can test for sure is:
  // 1. The penetration depth.
  // 2. The vertical position of the collision point (since for any of the four
  //    corners of the small box is the same.

  // Collision test performed with Model::closestPointsAllToAll.
  const std::vector<ElementId> ids_to_check = {large_box_->getId(),
                                               small_box_->getId()};
  model_->closestPointsAllToAll(ids_to_check, true, points);
  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.1, points[0].distance, tolerance_);
  EXPECT_TRUE(points[0].normal.isApprox(Vector3d(0.0, -1.0, 0.0)));
  // Collision points are reported on each of the respective bodies' frames.
  // Only test for vertical position.
  EXPECT_NEAR(points[0].ptA.y(),
              solution_[points[0].elementA].body_frame.y(), tolerance_);
  EXPECT_NEAR(points[0].ptB.y(),
              solution_[points[0].elementB].body_frame.y(), tolerance_);

  // Collision test performed with Model::ComputeMaximumDepthCollisionPoints.
  // Not using margins.
  points.clear();
  model_->ComputeMaximumDepthCollisionPoints(false, points);

  // Unfortunately DrakeCollision::Model's manifold has one point for this case.
  // Best for physics simulations would be DrakeCollision::Model to return at
  // least the four corners of the smaller box. However it randomly picks one
  // corner.
  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.1, points[0].distance, tolerance_);
  // Collision points are reported in the world's frame.
  // Only test for vertical position.
  EXPECT_NEAR(points[0].ptA.y(),
              solution_[points[0].elementA].world_frame.y(), tolerance_);
  EXPECT_NEAR(points[0].ptB.y(),
              solution_[points[0].elementB].world_frame.y(), tolerance_);

  // Collision test performed with Model::ComputeMaximumDepthCollisionPoints.
  // Using margins.
  points.clear();
  model_->ComputeMaximumDepthCollisionPoints(true, points);

  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.1, points[0].distance, tolerance_);
  // Collision points are reported in the world's frame.
  // Only test for vertical position.
  EXPECT_NEAR(points[0].ptA.y(),
              solution_[points[0].elementA].world_frame.y(), tolerance_);
  EXPECT_NEAR(points[0].ptB.y(),
              solution_[points[0].elementB].world_frame.y(), tolerance_);

  points.clear();
  EXPECT_THROW(points = model_->potentialCollisionPoints(false),
               std::runtime_error);
}

// This test is exactly the same as the previous SmallBoxSittingOnLargeBox.
// The difference is that a multi-contact algorithm is being used.
// See REMARKS ON MULTICONTACT at the top of this file.
TEST_F(SmallBoxSittingOnLargeBox, MultiContact) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 2.0e-9;

  // List of collision points.
  std::vector<PointPair> points;

  // Unfortunately DrakeCollision::Model is randomly selecting the contact
  // points. It is not even clear at this stage how many points in the manifold
  // we should expect.
  // What we can test for sure are:
  // 1. The penetration depth.
  // 2. The vertical position of the collision point (since for any of the four
  //    corners of the small box is the same).

  // Collision test performed with Model::potentialCollisionPoints.
  points = model_->potentialCollisionPoints(false);
  ASSERT_EQ(4u, points.size());
  for (const PointPair& point : points) {
    EXPECT_NEAR(-0.1, point.distance, tolerance_);
    EXPECT_TRUE(point.normal.isApprox(Vector3d(0.0, -1.0, 0.0)));
    // Collision points are reported on each of the respective bodies' frames.
    // This is consistent with the return by Model::closestPointsAllToAll.
    // Only test for vertical position.
    EXPECT_NEAR(point.ptA.y(),
                solution_[point.elementA].body_frame.y(), tolerance_);
    EXPECT_NEAR(point.ptB.y(),
                solution_[point.elementB].body_frame.y(), tolerance_);
  }
}

// This test seeks to find out whether DrakeCollision::Model can report
// collision manifolds. To this end two unit length boxes are placed on top of
// one another. The box sitting on top is rotated by 45 degrees so that the
// contact area would consist of an octagon. If DrakeCollision::Model can report
// manifolds, the manifold would consist of the perimeter of this octagon.

// Unfortunately these tests show that DrakeCollision::Model only reports a
// single (randomly chosen) point within this octagonal contact area.
class NonAlignedBoxes: public ::testing::Test {
 public:
  void SetUp() override {
    // Boxes centered around the origin in their local frames.
    DrakeShapes::Box box1(Vector3d(1.0, 1.0, 1.0));
    DrakeShapes::Box box2(Vector3d(1.0, 1.0, 1.0));

    // Populate the model.
    model_ = newModel();
    box1_ = model_->AddElement(make_unique<Element>(box1));
    box2_ = model_->AddElement(make_unique<Element>(box2));

    // Access the analytical solution to the contact point on the surface of
    // each collision element by element id.
    // Solutions are expressed in world and body frames.
    solution_ = {
        /*         world frame    , body frame  */
        {box1_, {{0.0, 1.0, 0.0}, {0.0,  0.5, 0.0}}},
        {box2_, {{0.0, 0.9, 0.0}, {0.0, -0.5, 0.0}}}};

    // Box 1 pose.
    Isometry3d box1_pose;
    box1_pose.setIdentity();
    box1_pose.translation() = Vector3d(0.0, 0.5, 0.0);
    model_->updateElementWorldTransform(box1_->getId(), box1_pose);

    // Box 2 pose.
    // Rotate box 2 45 degrees around the y axis so that it does not alight with
    // box 1.
    Isometry3d box2_pose;
    box2_pose.setIdentity();
    box2_pose.translation() = Vector3d(0.0, 1.4, 0.0);
    box2_pose.linear() =
        AngleAxisd(M_PI_4, Vector3d::UnitY()).toRotationMatrix();
    model_->updateElementWorldTransform(box2_->getId(), box2_pose);
  }

 protected:
  double tolerance_;
  std::unique_ptr<Model> model_;
  Element* box1_, * box2_;
  ElementToSurfacePointMap solution_;
};

TEST_F(NonAlignedBoxes, SingleContact) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 3.0e-9;

  // List of collision points.
  std::vector<PointPair> points;

  // Unfortunately DrakeCollision::Model is randomly selecting one of the small
  // box's corners instead of reporting a manifold describing the perimeter of
  // the square where both boxes intersect.
  // Therefore it is impossible to assert if that choice would change with
  // future releases (say just because tolerances changed).
  // What we can test for sure is:
  // 1. The penetration depth.
  // 2. The vertical position of the collision point (since for any of the four
  //    corners of the small box is the same.
  // Collision test performed with Model::closestPointsAllToAll.
  const std::vector<ElementId> ids_to_check = {box1_->getId(), box2_->getId()};
  model_->closestPointsAllToAll(ids_to_check, true, points);
  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.1, points[0].distance, tolerance_);
  EXPECT_TRUE(points[0].normal.isApprox(Vector3d(0.0, -1.0, 0.0)));
  // Collision points are reported on each of the respective bodies' frames.
  // Only test for vertical position.
  EXPECT_NEAR(points[0].ptA.y(),
              solution_[points[0].elementA].body_frame.y(), tolerance_);
  EXPECT_NEAR(points[0].ptB.y(),
              solution_[points[0].elementB].body_frame.y(), tolerance_);

  // Collision test performed with Model::ComputeMaximumDepthCollisionPoints.
  points.clear();
  model_->ComputeMaximumDepthCollisionPoints(false, points);
  // Unfortunately DrakeCollision::Model's manifold has one point for this case.
  // Best for physics simulations would be DrakeCollision::Model to return at
  // least the four corners of the smaller box. However it randomly picks one
  // corner.
  ASSERT_EQ(1u, points.size());
  EXPECT_NEAR(-0.1, points[0].distance, tolerance_);
  EXPECT_TRUE(points[0].normal.isApprox(Vector3d(0.0, -1.0, 0.0)));
  // Collision points are reported in the world's frame.
  // Only test for vertical position.
  EXPECT_NEAR(points[0].ptA.y(),
              solution_[points[0].elementA].world_frame.y(), tolerance_);
  EXPECT_NEAR(points[0].ptB.y(),
              solution_[points[0].elementB].world_frame.y(), tolerance_);

  points.clear();
  EXPECT_THROW(points = model_->potentialCollisionPoints(false),
               std::runtime_error);
}

// This test is exactly the same as the previous NonAlignedBoxes.
// The difference is that a multi-contact algorithm is being used.
// See REMARKS ON MULTICONTACT at the top of this file.
TEST_F(NonAlignedBoxes, MultiContact) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 5.0e-4;

  // List of collision points.
  std::vector<PointPair> points;

  // Collision test performed with Model::potentialCollisionPoints.
  points.clear();
  points = model_->potentialCollisionPoints(false);
  ASSERT_EQ(4u, points.size());

  for (const PointPair& point : points) {
    EXPECT_NEAR(-0.1, point.distance, tolerance_);
    EXPECT_TRUE(point.normal.isApprox(Vector3d(0.0, -1.0, 0.0),
                                               tolerance_ * 50));
    // Collision points are reported on each of the respective bodies' frames.
    // This is consistent with the return by Model::closestPointsAllToAll.
    // Only test for vertical position.
    EXPECT_NEAR(point.ptA.y(),
                solution_[points[0].elementA].body_frame.y(), tolerance_);
    EXPECT_NEAR(point.ptB.y(),
                solution_[points[0].elementB].body_frame.y(), tolerance_);
  }
}

// Tests the model is not caching results from a series of different queries.
// Internally the model is using the method Model::ClearCachedResults
// to this end.
TEST_F(SmallBoxSittingOnLargeBox, ClearCachedResults) {
  // Numerical precision tolerance to perform floating point comparisons.
  // Its magnitude was chosen to be the minimum value for which these tests can
  // successfully pass.
  tolerance_ = 2.0e-9;

  // Large body pose
  Isometry3d large_box_pose;
  large_box_pose.setIdentity();
  large_box_pose.translation() = Vector3d(0.0, 2.5, 0.0);
  model_->updateElementWorldTransform(large_box_->getId(), large_box_pose);

  // Small body pose
  Isometry3d small_box_pose;
  small_box_pose.setIdentity();
  small_box_pose.translation() = Vector3d(0.0, 5.4, 0.0);
  model_->updateElementWorldTransform(small_box_->getId(), small_box_pose);

  // List of collision points.
  std::vector<PointPair> points;

  // Not clearing cached results
  for (int i = 0; i < 4; ++i) {
    // Small disturbance so that tests are slightly different causing Bullet's
    // dispatcher to cache these results.
    if (i == 0) small_box_pose.translation() = Vector3d(   0.0, 5.4,    0.0);
    if (i == 1) small_box_pose.translation() = Vector3d(1.0e-3, 5.4,    0.0);
    if (i == 2) small_box_pose.translation() = Vector3d(   0.0, 5.4, 1.0e-3);
    if (i == 3) small_box_pose.translation() = Vector3d(1.0e-3, 5.4, 1.0e-3);
    model_->updateElementWorldTransform(small_box_->getId(), small_box_pose);

    // Notice that the results vector is cleared every time so that results
    // do not accumulate.
    points.clear();

    model_->ComputeMaximumDepthCollisionPoints(false, points);
  }

  // Check that the model is not caching results even after four queries.
  // If the model does not cache results there should only be one result.
  ASSERT_EQ(1u, points.size());

  for (const PointPair& point : points) {
    EXPECT_NEAR(-0.1, point.distance, tolerance_);
    EXPECT_TRUE(
        point.normal.isApprox(Vector3d(0.0, -1.0, 0.0), tolerance_));
    // Only test for vertical position.
    EXPECT_NEAR(point.ptA.y(),
                solution_[points[0].elementA].world_frame.y(), tolerance_);
    EXPECT_NEAR(point.ptB.y(),
                solution_[points[0].elementB].world_frame.y(), tolerance_);
  }
}

// Tests that anchored collision elements do not collide with other anchored
// collision elements.
// This test sets four spheres in a box arrangement so that they collide with
// their neighboring spheres. In this configuration, the collision dispatcher
// will return four collision points.
// However, two collision elements (ball1 and ball4) are flagged as anchored and
// therefore only three collision points are reported. Dynamic (non-anchored)
// collision elements can still collide with anchored elements.
GTEST_TEST(ModelTest, AnchoredElements) {
  DrakeShapes::Sphere sphere(0.5);
  Isometry3d pose = Isometry3d::Identity();

  // Creates collision elements.
  // Positions the balls on the corners of squares.  This positioning is done
  // on the *elements* themselves.  This confirms that this information is
  // properly inherited by the underlying collision model
  // Flags ball1 and ball4 to be anchored.

  // NOTE: These elements are being instantiated here so that their anchored
  // state can be set *before* adding to the collision model.
  auto ball1 = make_unique<Element>(sphere);
  ball1->set_anchored();
  pose.translation() = Vector3d(0.45, 0.45, 0.0);
  ball1->updateWorldTransform(pose);

  auto ball2 = make_unique<Element>(sphere);
  pose.translation() = Vector3d(-0.45, 0.45, 0.0);
  ball2->updateWorldTransform(pose);

  auto ball3 = make_unique<Element>(sphere);
  pose.translation() = Vector3d(-0.45, -0.45, 0.0);
  ball3->updateWorldTransform(pose);

  auto ball4 = make_unique<Element>(sphere);
  ball4->set_anchored();
  pose.translation() = Vector3d(0.45, -0.45, 0.0);
  ball4->updateWorldTransform(pose);

  // Populate the model.
  unique_ptr<Model> model = newModel();
  model->AddElement(move(ball1));
  model->AddElement(move(ball2));
  model->AddElement(move(ball3));
  model->AddElement(move(ball4));

  // List of collision points.
  std::vector<PointPair> points;

  // Compute all points of contact.
  model->ComputeMaximumDepthCollisionPoints(false, points);

  // Only three points are expected (instead of four) since ball1 and ball4 are
  // flagged as anchored.
  ASSERT_EQ(3u, points.size());
}

/*
 This tests a sphere against a non-convex mesh

      ______         \    /        _____    <---- z = 0.2
     /      \         \__/        /     \   <---- z = 0.1
    /        \_________/\________/       \
   /                                      \

 The sphere is positioned such that it should collide with the inner spike
 at a height of z = 0.1.  If the mesh is rendered as a convex hull, it
 would intersect with the hull at z = 0.2.
 This tests confirms two things:
  1) The non-convex mesh is being instantiated (and *not* a convex hull).
  2) Collision with the non-convex mesh is producing the expected answer.
 */
GTEST_TEST(ModelTest, AnchoredMeshes) {
  // Numerical precision tolerance to perform floating point comparisons.
  const double tolerance = 1.0e-9;

  DrakeShapes::Sphere sphere_shape(0.5);
  Isometry3d pose = Isometry3d::Identity();

  std::string file_name = drake::GetDrakePath() +
      "/multibody/collision/test/ripple_cap.obj";
  DrakeShapes::Mesh cap_shape(file_name, file_name);

  // Creates collision elements.
  // Flag cap_element to be anchored. If not, Drake will create a convex hull
  // for dynamic collision elements out of the points in the mesh.

  // NOTE: The elements are being instantiated here so that the anchored state
  // can be set *before* registering with the collision model.
  auto sphere_element = make_unique<Element>(sphere_shape);
  Element *sphere = sphere_element.get();
  auto cap_element = make_unique<Element>(cap_shape);
  Element *cap = cap_element.get();
  cap_element->set_anchored();

  // Populate the model.
  unique_ptr<Model> model = newModel();
  model->AddElement(move(sphere_element));
  model->AddElement(move(cap_element));

  // Access the analytical solution to the contact point on the surface of
  // each collision element by element id.
  // Solutions are expressed in world and body frames.
  ElementToSurfacePointMap solution = {
      /*           world frame     , body frame  */
      {sphere, {{0.0, 0.0, 0.09}, {0.0, 0.0, -0.5}}},
      {cap, {{0.0, 0.0, 0.1}, {0.0, 0.0, 0.1}}}};

  // Sets the collision elements' pose.
  pose.translation() = Vector3d(0.0, 0.0, 0.59);
  model->updateElementWorldTransform(sphere->getId(), pose);

  pose.translation() = Vector3d(0.0, 0.0, 0.0);
  model->updateElementWorldTransform(cap->getId(), pose);

  // List of collision points.
  std::vector<PointPair> points;

  // Computes all points of contact.
  model->ComputeMaximumDepthCollisionPoints(false, points);

  // Expects one collision point for this test.
  ASSERT_EQ(1u, points.size());

  EXPECT_NEAR(-0.01, points[0].distance, tolerance);

  EXPECT_TRUE(points[0].normal.isApprox(Vector3d(0, 0, 1), tolerance));

  EXPECT_TRUE(points[0].ptA.isApprox(
      solution[points[0].elementA].world_frame, tolerance));

  EXPECT_TRUE(CompareMatrices(points[0].ptB,
                              solution[points[0].elementB].world_frame,
                              tolerance, drake::MatrixCompareType::absolute));
}

// This test confirms that point queries against non-convex objects (e.g.,
// a static mesh) will *not* produce distance values.
GTEST_TEST(ModelTest, PointDistanceToNonConvex) {
  std::string file_name = drake::GetDrakePath() +
      "/multibody/collision/test/ripple_cap.obj";
  DrakeShapes::Mesh cap_shape(file_name, file_name);

  // NOTE: The elements are being instantiated here so that the anchored state
  // can be set *before* registering with the collision model.
  auto cap_element = make_unique<Element>(cap_shape);
  cap_element->set_anchored();

  // Populate the model.
  unique_ptr<Model> model = newModel();
  model->AddElement(move(cap_element));

  const int kPointCount = 4;
  Eigen::RowVectorXd cx(kPointCount), cy(kPointCount), cz(kPointCount);

  // A collection of points -- all should be at infinite distance.
  cx << 0, 0, 0, 1;
  cy << 0.2499, 0, -0.25, -0.25;
  cz << 1, 1, 1, 1.75;
  Matrix3Xd points;
  points.resize(Eigen::NoChange, kPointCount);
  points << cx, cy, cz;

  std::vector<PointPair> results;
  model->collisionDetectFromPoints(points, false, results);

  ASSERT_EQ(results.size(), 4u);
  const double inf = std::numeric_limits<double>::infinity();
  for (auto& pair : results) {
    EXPECT_EQ(pair.distance, inf);
    EXPECT_EQ(pair.elementA, nullptr);
    EXPECT_EQ(pair.elementB, nullptr);
    EXPECT_EQ(pair.normal, Vector3d(0, 0, 1));
    EXPECT_EQ(pair.ptA, Vector3d(0, 0, inf));
    EXPECT_EQ(pair.ptB, Vector3d(0, 0, inf));
  }
}

// This test confirms that point queries against an empty world will produce
// infinite values.
GTEST_TEST(ModelTest, PointDistanceToEmptyWorld) {
  unique_ptr<Model> model = newModel();

  const int kPointCount = 4;
  Eigen::RowVectorXd cx(kPointCount), cy(kPointCount), cz(kPointCount);

  // A collection of points -- none of which should produce results.
  cx << 0, 0, 0, 1;
  cy << 0.2499, 0, -0.25, -0.25;
  cz << 1, 1, 1, 1.75;
  Matrix3Xd points;
  points.resize(Eigen::NoChange, kPointCount);
  points << cx, cy, cz;

  std::vector<PointPair> results;
  model->collisionDetectFromPoints(points, false, results);

  ASSERT_EQ(results.size(), 4u);
  const double inf = std::numeric_limits<double>::infinity();
  for (auto& pair : results) {
    EXPECT_EQ(pair.distance, inf);
    EXPECT_EQ(pair.elementA, nullptr);
    EXPECT_EQ(pair.elementB, nullptr);
    EXPECT_EQ(pair.normal, Vector3d(0, 0, 1));
    EXPECT_EQ(pair.ptA, Vector3d(0, 0, inf));
    EXPECT_EQ(pair.ptB, Vector3d(0, 0, inf));
  }
}

// This tests that the query for distance between two objects (where at least
// one is non-convex) is gracefully handled -- no distance is reported.
GTEST_TEST(ModelTest, DistanceToNonConvex) {
  DrakeShapes::Sphere sphere_shape(0.5);
  Isometry3d pose = Isometry3d::Identity();

  std::string file_name = drake::GetDrakePath() +
      "/multibody/collision/test/ripple_cap.obj";
  DrakeShapes::Mesh cap_shape(file_name, file_name);

  // Creates collision elements.
  // Flag cap_element to be anchored. If not, Drake will create a convex hull
  // for dynamic collision elements out of the points in the mesh.
  // NOTE: The elements are being instantiated here so that the anchored state
  // can be set *before* registering with the collision model.
  auto cap_element = make_unique<Element>(cap_shape);
  Element *cap = cap_element.get();
  cap_element->set_anchored();

  // Populate the model.
  unique_ptr<Model> model = newModel();
  Element *sphere = model->AddElement(make_unique<Element>(sphere_shape));
  model->AddElement(move(cap_element));

  // Sets the collision elements' pose.
  pose.translation() = Vector3d(0.0, 0.0, 0.7);
  model->updateElementWorldTransform(sphere->getId(), pose);

  pose.translation() = Vector3d(0.0, 0.0, 0.0);
  model->updateElementWorldTransform(cap->getId(), pose);

  std::vector<PointPair> results;
  std::vector<ElementIdPair> pairs;
  pairs.emplace_back(sphere->getId(), cap->getId());
  model->closestPointsPairwise(pairs, true, results);
  EXPECT_EQ(results.size(), 0u);
}

}  // namespace
}  // namespace DrakeCollision
