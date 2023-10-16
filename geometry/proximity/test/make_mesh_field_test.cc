#include "drake/geometry/proximity/make_mesh_field.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

using ScalarTypes = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(MakeVolumeMeshPressureFieldTest, ScalarTypes);

template <typename T>
class MakeVolumeMeshPressureFieldTest : public ::testing::Test {};

// Tests that we can make a pressure field from a non-convex tetrahedral mesh.
TYPED_TEST(MakeVolumeMeshPressureFieldTest, PressureOnNonConvexMesh) {
  using T = TypeParam;

  VolumeMesh<T> non_convex_mesh = MakeVolumeMeshFromVtk<T>(
      Mesh(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk")));

  const T kHydroelasticModulus = 1e7;
  VolumeMeshFieldLinear<T, T> field =
      MakeVolumeMeshPressureField(&non_convex_mesh, kHydroelasticModulus);

  // The first five vertices are on the boundary, so they have zero pressure.
  for (int v = 0; v < 5; ++v) {
    SCOPED_TRACE(fmt::format("v: {}", v));
    EXPECT_EQ(field.EvaluateAtVertex(v), T(0.0));
  }
  // Only the last vertex is an interior vertex, so its pressure value is the
  // hydroelastic modulus.
  if constexpr (std::is_same_v<T, double>) {
    EXPECT_EQ(field.EvaluateAtVertex(5), kHydroelasticModulus);
  } else {
    static_assert(std::is_same_v<T, AutoDiffXd>);
    EXPECT_EQ(field.EvaluateAtVertex(5).value(), kHydroelasticModulus.value());
  }
}

// Tests that an input mesh without interior vertices will throw. For
// simplicity, use double as the representative scalar type.
GTEST_TEST(MakeVolumeMeshPressureFieldTest, NoInteriorVertex) {
  const VolumeMesh<double> mesh_without_interior_vertex{
      {
          {0, 1, 2, 3},
      },
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ()}};

  const double kHydroelasticModulus = 1e7;
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeVolumeMeshPressureField(&mesh_without_interior_vertex,
                                  kHydroelasticModulus),
      "MakeVolumeMeshPressureField.*: "
      "the computed max distance to boundary among "
      "all mesh vertices is non-positive. Perhaps "
      "the mesh lacks interior vertices.");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
