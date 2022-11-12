#include "drake/geometry/proximity/meshing_utilities.h"

#include <algorithm>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(SplitTriangularPrismToTetrahedraTest, SimpleTest) {
  std::vector<VolumeElement> tetrahedra =
      SplitTriangularPrismToTetrahedra(0, 1, 2, 3, 4, 5);

  EXPECT_EQ(tetrahedra.size(), 3);

  // The following statements check each of the three tetrahedra and also the
  // three face diagonals (0,5), (0,4), and (1,5) that we promised in the
  // API contract. A tetrahedron always connect its four vertices into all
  // possible pairs of vertices to make six edges (4 choose 2 = 6). Therefore,
  // checking for existence of tetrahedron *,u,*,v,* implies the diagonal u,v
  // exists in the mesh.
  //
  // Each test is specific to the implementation of the function
  // SplitTriangularPrismToTetrahedra() that chooses a particular vertex
  // ordering. If implementation of the ordering changes, these tests will
  // need to change too.
  EXPECT_TRUE(tetrahedra.end() != std::find(tetrahedra.begin(),
                                            tetrahedra.end(),
                                            VolumeElement(3, 4, 0, 5)));
  EXPECT_TRUE(tetrahedra.end() != std::find(tetrahedra.begin(),
                                            tetrahedra.end(),
                                            VolumeElement(4, 1, 0, 5)));
  EXPECT_TRUE(tetrahedra.end() != std::find(tetrahedra.begin(),
                                            tetrahedra.end(),
                                            VolumeElement(1, 2, 0, 5)));
}

GTEST_TEST(SplitPyramidToTetrahedraTest, SimpleTest) {
  std::vector<VolumeElement> tetrahedra =
      SplitPyramidToTetrahedra(0, 1, 2, 3, 4);

  EXPECT_EQ(tetrahedra.size(), 2);
  EXPECT_TRUE(tetrahedra.end() != std::find(tetrahedra.begin(),
                                            tetrahedra.end(),
                                            VolumeElement(3, 4, 0, 2)));
  EXPECT_TRUE(tetrahedra.end() != std::find(tetrahedra.begin(),
                                            tetrahedra.end(),
                                            VolumeElement(4, 1, 0, 2)));
}

GTEST_TEST(CalcEnclosedVolumeTest, SimpleTest) {
  std::string obj = FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  TriangleSurfaceMesh<double> surface_mesh =
      ReadObjToTriangleSurfaceMesh(obj, 1.0);
  EXPECT_NEAR(CalcEnclosedVolume(surface_mesh), 8.0, 1e-14);

  // Translating the box so the origin is not in the interior should not change
  // the volume.
  surface_mesh.TransformVertices(
      math::RigidTransform<double>(Eigen::Vector3d{4, 4, 4}));
  EXPECT_NEAR(CalcEnclosedVolume(surface_mesh), 8.0, 1e-14);

  obj = FindResourceOrThrow("drake/geometry/test/octahedron.obj");
  const double meshlab_volume{3.771236};
  surface_mesh = ReadObjToTriangleSurfaceMesh(obj, 1.0);
  // Note: The large tolerance is because we are only matching Meshlab's volume
  // calculation as printed to the console with limited precision.
  EXPECT_NEAR(CalcEnclosedVolume(surface_mesh), meshlab_volume, 1e-6);

  obj = FindResourceOrThrow("drake/geometry/test/non_convex_mesh.obj");
  // meshlab reports volume 0.1.
  surface_mesh = ReadObjToTriangleSurfaceMesh(obj, 1.0);
  EXPECT_NEAR(CalcEnclosedVolume(surface_mesh), 0.1, 1e-14);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
