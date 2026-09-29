/* Smoke test for Drake's vendored build of Coal (@coal_internal).

 This is a build test, not a characterization of Coal's numerics. It exercises
 the parts of Coal that Drake's proximity engine will depend on, and -- more to
 the point -- the parts of Coal that Drake's build recipe is most likely to get
 wrong:

   - the "pi" path, which no_boost.patch rewrote from boost::math onto
     std::numbers;
   - the std::span path (contact patches), the other half of that patch;
   - the narrowphase entry points, collide() and distance();
   - the dynamic AABB tree broadphase, whose translation units reach private
     headers under src/ via include paths that our vendored layout has to
     preserve;
   - convex geometry, which pulls in convex.hxx. */

#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#include <coal/collision.h>
#include <coal/contact_patch.h>
#include <coal/distance.h>
#include <coal/shape/convex.h>
#include <coal/shape/geometric_shapes.h>
#include <gtest/gtest.h>

namespace drake {
namespace {

using coal::CollisionObject;

/* Coal's shape volumes go through what used to be boost::math's pi. */
GTEST_TEST(CoalBuildTest, ShapeVolume) {
  const coal::Sphere sphere(2.0);
  EXPECT_NEAR(sphere.computeVolume(), 4.0 * std::numbers::pi * 8.0 / 3.0,
              1e-14);

  const coal::Cylinder cylinder(1.5, 4.0);
  EXPECT_NEAR(cylinder.computeVolume(), std::numbers::pi * 1.5 * 1.5 * 4.0,
              1e-14);

  const coal::Ellipsoid ellipsoid(1.0, 2.0, 3.0);
  EXPECT_NEAR(ellipsoid.computeVolume(), 4.0 * std::numbers::pi * 6.0 / 3.0,
              1e-14);
}

/* Narrowphase penetration. Note that Coal reports penetration_depth as a
 signed distance (negative when overlapping), the opposite of FCL's sign
 convention. */
GTEST_TEST(CoalBuildTest, Collide) {
  const coal::Box box1(1, 1, 1);
  const coal::Box box2(1, 1, 1);
  const coal::Transform3s X_WB1(coal::Vec3s(0, 0, 0));
  const coal::Transform3s X_WB2(coal::Vec3s(0.75, 0, 0));

  coal::CollisionRequest request(coal::CONTACT, 1);
  request.enable_contact = true;
  coal::CollisionResult result;
  ASSERT_EQ(coal::collide(&box1, X_WB1, &box2, X_WB2, request, result), 1);
  ASSERT_EQ(result.numContacts(), 1u);
  // The depth is a GJK/EPA result, so it carries the solver's default
  // tolerance (1e-6) rather than machine precision.
  EXPECT_NEAR(result.getContact(0).penetration_depth, -0.25, 1e-9);
}

/* Narrowphase separation distance. */
GTEST_TEST(CoalBuildTest, Distance) {
  const coal::Box box1(1, 1, 1);
  const coal::Box box2(1, 1, 1);
  const coal::Transform3s X_WB1(coal::Vec3s(0, 0, 0));
  const coal::Transform3s X_WB2(coal::Vec3s(3.0, 0, 0));

  const coal::DistanceRequest request;
  coal::DistanceResult result;
  coal::distance(&box1, X_WB1, &box2, X_WB2, request, result);
  EXPECT_NEAR(result.min_distance, 2.0, 1e-9);
}

/* Contact patches go through what used to be boost::span. */
GTEST_TEST(CoalBuildTest, ContactPatch) {
  const coal::Box box1(1, 1, 1);
  const coal::Box box2(1, 1, 1);
  const coal::Transform3s X_WB1(coal::Vec3s(0, 0, 0));
  const coal::Transform3s X_WB2(coal::Vec3s(0, 0, 0.99));

  coal::CollisionRequest collision_request(coal::CONTACT, 1);
  collision_request.enable_contact = true;
  coal::CollisionResult collision_result;
  ASSERT_EQ(coal::collide(&box1, X_WB1, &box2, X_WB2, collision_request,
                          collision_result),
            1);

  const coal::ContactPatchRequest patch_request;
  coal::ContactPatchResult patch_result;
  coal::computeContactPatch(&box1, X_WB1, &box2, X_WB2, collision_result,
                            patch_request, patch_result);
  ASSERT_EQ(patch_result.numContactPatches(), 1u);
  // The boxes meet face to face, so the patch is a (square) quadrilateral.
  EXPECT_EQ(patch_result.getContactPatch(0).size(), 4u);
}

/* A tetrahedron, as convex geometry. */
GTEST_TEST(CoalBuildTest, Convex) {
  auto points = std::make_shared<std::vector<coal::Vec3s>>(
      std::vector<coal::Vec3s>{coal::Vec3s(0, 0, 0), coal::Vec3s(1, 0, 0),
                               coal::Vec3s(0, 1, 0), coal::Vec3s(0, 0, 1)});
  auto faces = std::make_shared<std::vector<coal::Triangle32>>(
      std::vector<coal::Triangle32>{
          coal::Triangle32(0, 2, 1), coal::Triangle32(0, 1, 3),
          coal::Triangle32(0, 3, 2), coal::Triangle32(1, 2, 3)});
  const coal::Convex<coal::Triangle32> tet(points, 4, faces, 4);
  EXPECT_EQ(tet.getNodeType(), coal::GEOM_CONVEX32);

  const coal::Sphere sphere(0.25);
  const coal::Transform3s X_WT = coal::Transform3s::Identity();
  const coal::Transform3s X_WS(coal::Vec3s(0, 0, 3));

  coal::DistanceRequest request;
  coal::DistanceResult result;
  coal::distance(&tet, X_WT, &sphere, X_WS, request, result);
  // The sphere's nearest point to the tetrahedron's apex at (0, 0, 1).
  EXPECT_NEAR(result.min_distance, 3.0 - 1.0 - 0.25, 1e-9);
}

/* The dynamic AABB tree broadphase. */
GTEST_TEST(CoalBuildTest, Broadphase) {
  struct CountingCallback final : public coal::CollisionCallBackBase {
    bool collide(CollisionObject*, CollisionObject*) final {
      ++count;
      return false;  // Keep going; don't stop the broadphase.
    }
    int count{0};
  };

  auto sphere = std::make_shared<coal::Sphere>(1.0);
  // Three spheres in a row; the outer two are too far apart to be candidates.
  CollisionObject object_a(sphere, coal::Transform3s(coal::Vec3s(0, 0, 0)));
  CollisionObject object_b(sphere, coal::Transform3s(coal::Vec3s(1.5, 0, 0)));
  CollisionObject object_c(sphere, coal::Transform3s(coal::Vec3s(3.0, 0, 0)));

  coal::DynamicAABBTreeCollisionManager manager;
  manager.registerObject(&object_a);
  manager.registerObject(&object_b);
  manager.registerObject(&object_c);
  manager.setup();
  ASSERT_EQ(manager.size(), 3u);

  CountingCallback callback;
  manager.collide(&callback);
  EXPECT_EQ(callback.count, 2);
}

}  // namespace
}  // namespace drake
