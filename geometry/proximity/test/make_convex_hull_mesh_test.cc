#include "drake/geometry/proximity/make_convex_hull_mesh.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

/* For the detailed tests of the calculations, see
 make_convex_hull_mesh_impl_test.cc. */

/* Confirm that unsupported shapes throw errors reporting such but supported
 shapes happily go through. */
GTEST_TEST(MakeConvexHullMeshTest, ShapeSupport) {
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Box(1, 1, 1)),
                              ".* only applies to Mesh and Convex .* Box.");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Capsule(1, 1)),
                              ".* only applies to .* Capsule.");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Cylinder(1, 1)),
                              ".* only applies .*");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Ellipsoid(1, 1, 1)),
                              ".* only applies .*");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(HalfSpace()),
                              ".* only applies .*");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(MeshcatCone(1)),
                              ".* only applies .*");
  DRAKE_EXPECT_THROWS_MESSAGE(MakeConvexHull(Sphere(1)), ".* only applies .*");

  /* Supported shapes. */
  const std::string mesh_file =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");

  const Eigen::Vector3d scale(2, 3, 4);
  auto validate_hull = [&scale](auto&& shape) {
    SCOPED_TRACE(shape.type_name());
    PolygonSurfaceMesh<double> hull = MakeConvexHull(shape);
    const auto& [_, size] = hull.CalcBoundingBox();
    EXPECT_TRUE(CompareMatrices(size, 2 * scale));
  };
  validate_hull(Convex(mesh_file, scale));
  validate_hull(Mesh(mesh_file, scale));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
