#include "drake/geometry/proximity/make_nonconvex_field.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

GTEST_TEST(MakeNonConvexMeshPressureFieldTest, Double) {
  const VolumeMesh<double> non_convex_mesh {
      {
       {5, 4, 1, 2},
       {5, 4, 2, 3},
       {5, 4, 3, 1},
       {5, 0, 3, 2},
       {5, 0, 2, 1},
       {5, 0, 1, 3},
      },
      {Vector3d::Zero(), Vector3d::UnitX(), Vector3d::UnitY(),
       Vector3d::UnitZ(), 0.2 * Vector3d::Ones(), 0.1 * Vector3d::Ones()}
  };

  const double kHydroelasticModulus = 1e7;
  const double kEps = 1e-14;
  VolumeMeshFieldLinear<double, double> field =
      MakeNonConvexMeshPressureField(&non_convex_mesh, kHydroelasticModulus);

  // All vertices are on the boundary except the last vertex.
  for (int v = 0; v < 5; ++v) {
    SCOPED_TRACE(fmt::format("v: {}", v));
    EXPECT_NEAR(field.EvaluateAtVertex(v), 0, kEps);
  }
  EXPECT_NEAR(field.EvaluateAtVertex(5), kHydroelasticModulus,
              kEps * kHydroelasticModulus);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
