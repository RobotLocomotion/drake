#include "drake/geometry/proximity/meshing_utilities.h"

#include <algorithm>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(SplitTriangularPrismToTetrahedraTest, SimpleTest) {
  const VolumeVertexIndex v0(0);
  const VolumeVertexIndex v1(1);
  const VolumeVertexIndex v2(2);
  const VolumeVertexIndex v3(3);
  const VolumeVertexIndex v4(4);
  const VolumeVertexIndex v5(5);

  std::vector<VolumeElement> tetrahedra =
      SplitTriangularPrismToTetrahedra(v0, v1, v2, v3, v4, v5);

  EXPECT_EQ(tetrahedra.size(), 3);

  // The following statements check each of the three tetrahedra and also the
  // three face diagonals (v0,v5), (v0,v4), and (v1,v5) that we promised in the
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
                                            VolumeElement(v3, v4, v0, v5)));
  EXPECT_TRUE(tetrahedra.end() != std::find(tetrahedra.begin(),
                                            tetrahedra.end(),
                                            VolumeElement(v4, v1, v0, v5)));
  EXPECT_TRUE(tetrahedra.end() != std::find(tetrahedra.begin(),
                                            tetrahedra.end(),
                                            VolumeElement(v1, v2, v0, v5)));
}

GTEST_TEST(SplitPyramidToTetrahedraTest, SimpleTest) {
  const VolumeVertexIndex v0(0);
  const VolumeVertexIndex v1(1);
  const VolumeVertexIndex v2(2);
  const VolumeVertexIndex v3(3);
  const VolumeVertexIndex v4(4);

  std::vector<VolumeElement> tetrahedra =
      SplitPyramidToTetrahedra(v0, v1, v2, v3, v4);

  EXPECT_EQ(tetrahedra.size(), 2);
  EXPECT_TRUE(tetrahedra.end() != std::find(tetrahedra.begin(),
                                            tetrahedra.end(),
                                            VolumeElement(v3, v4, v0, v2)));
  EXPECT_TRUE(tetrahedra.end() != std::find(tetrahedra.begin(),
                                            tetrahedra.end(),
                                            VolumeElement(v4, v1, v0, v2)));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
