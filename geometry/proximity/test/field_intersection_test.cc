#include "drake/geometry/proximity/field_intersection.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;

// This fixture is for intersection of two tetrahedra with linear functions.
// We set up the tetrahedra and linear functions in such a way that
// the two functions have the equilibrium plane that intersects the two
// tetrahedra into an octagon. See the picture:
// geometry/proximity/images/two_linear_tetrahedra_intersect_into_octagon.png
// TODO(DamrongGuoy): Complete the code that can generate the above picture.
//  We will need a few more PRs.
class FieldIntersectionLowLevelTest : public ::testing::Test {
 public:
  FieldIntersectionLowLevelTest()
      : mesh0_M_(std::vector<VolumeElement>{{0, 1, 2, 3}},
                 std::vector<Vector3d>{{2.0, 0.0, 2.0},
                                       {-2.0, 0.0, 2.0},
                                       {0.0, 2.0, -2.0},
                                       {0.0, -2.0, -2.0}}),
        field0_M_({8e4, 4e4, 4e4, 0}, &mesh0_M_),  // (4 + x + y + z)*10⁴
        mesh1_N_(std::vector<VolumeElement>{{0, 1, 2, 3}},
                 std::vector<Vector3d>{{1.5, 1.5, 2.5},
                                       {-1.5, -1.5, 2.5},
                                       {-1.5, 1.5, -2.5},
                                       {1.5, -1.5, -2.5}}),
        field1_N_({7e4, 1e4, 4e4, 4e4}, &mesh1_N_)  // (4 + x + y)*10⁴
  {}

 protected:
  // Returns the mesh expressed in frame F given X_FG and the mesh expressed
  // in frame G.
  static VolumeMesh<double> TransformVolumeMesh(
      const RigidTransformd& X_FG, const VolumeMesh<double>& mesh_G) {
    std::vector<Vector3<double>> p_FVs;
    for (const Vector3<double>& p_GV : mesh_G.vertices()) {
      p_FVs.emplace_back(X_FG * p_GV);
    }
    return {std::vector<VolumeElement>(mesh_G.tetrahedra()), std::move(p_FVs)};
  }

  const VolumeMesh<double> mesh0_M_;
  const VolumeMeshFieldLinear<double, double> field0_M_;
  const VolumeMesh<double> mesh1_N_;
  const VolumeMeshFieldLinear<double, double> field1_N_;

  double tolerance(double scale = 1) {
    const double kEps = 1e-14;
    return (scale > 1) ? kEps * scale : kEps;
  }
};

// For simplicity, this test identifies the two frames of the two meshes to
// be the same, so we can easily calculate the expected equilibrium plane for
// verification. The next test will set up a complex rigid transform between
// two frames.
TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlaneIdenticalFrames) {
  const auto X_MN = RigidTransformd::Identity();
  //     f0(x,y,z) = (4 + x + y + z)*10⁴   (1)
  //     f1(x,y,z) = (4 + x + y)*10⁴       (2)
  // The equilibrium plane satisfies:
  //       f0 - f1 = z*10⁴ = 0      (1)-(2) = 0.
  // Therefore, z = 0 is the equilibrium plane. We pick the plane normal
  // +UnitZ() instead of -UnitZ(), so that it points out of f1 and into f0.
  const Plane<double> expected_plane_M{Vector3d::UnitZ(), Vector3d::Zero()};

  // Initialize the plane to be different from the expected plane.
  Plane<double> plane_M{Vector3d::UnitX(), Vector3d(1, 2, 3)};
  const bool success =
      CalcEquilibriumPlane(0, field0_M_, 0, field1_N_, X_MN, &plane_M);

  ASSERT_TRUE(success);
  EXPECT_TRUE(CompareMatrices(plane_M.normal(), expected_plane_M.normal(),
                              tolerance()));
  // The choice of this query point is arbitrary.
  const Vector3d p_MQ(0.1, 0.2, 0.3);
  const double expected_height = expected_plane_M.CalcHeight<double>(p_MQ);
  EXPECT_NEAR(plane_M.CalcHeight<double>(p_MQ), expected_height,
              tolerance(expected_height));
}

// Expresses the two meshes in two different frames F and G for a
// stronger test. Correctness of this test also depends on the previous test.
//
//     mesh0_F <--> mesh0_M  <--M=N--> mesh1_N <--> mesh1_G
//
TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlaneComplexTransform) {
  const auto X_FG = RigidTransformd(RollPitchYawd(M_PI_4, M_PI/3, M_PI_2),
                                    Vector3d(-1., -2., -3.));
  const auto X_FM = RigidTransformd(RollPitchYawd(M_PI, M_PI_2, M_PI/3),
                                    Vector3d(2, -6, 4));
  // From the previous test, we know that identifying frame M with frame N
  // will give a simple expression of the expected equilibrium plane in frame M.
  const RigidTransformd X_MN = RigidTransformd::Identity();
  const Vector3d expected_normal_M = Vector3d::UnitZ();
  const Vector3d expected_p_M = Vector3d::Zero();
  Plane<double> expected_plane_F(X_FM.rotation() * expected_normal_M,
                                 X_FM * expected_p_M);
  const RigidTransformd X_GN = X_FG.InvertAndCompose(X_FM) * X_MN;

  const VolumeMesh<double> mesh0_F = TransformVolumeMesh(X_FM, mesh0_M_);
  const VolumeMeshFieldLinear<double, double> field0_F(
      std::vector<double>(field0_M_.values()), &mesh0_F);

  const VolumeMesh<double> mesh1_G = TransformVolumeMesh(X_GN, mesh1_N_);
  const VolumeMeshFieldLinear<double, double> field1_G(
      std::vector<double>(field1_N_.values()), &mesh1_G);

  // Initialize the plane arbitrarily.
  Plane<double> plane_F{Vector3d(-1, -2, -3), Vector3d(-4, -5, -6)};
  bool success = CalcEquilibriumPlane(0, field0_F, 0, field1_G, X_FG, &plane_F);

  ASSERT_TRUE(success);
  EXPECT_TRUE(CompareMatrices(plane_F.normal(), expected_plane_F.normal(),
                              tolerance()));
  // The choice of this query point is arbitrary.
  const Vector3d p_FQ(0.1, 0.2, 0.3);
  const double expected_height = expected_plane_F.CalcHeight<double>(p_FQ);
  EXPECT_NEAR(plane_F.CalcHeight<double>(p_FQ), expected_height,
              tolerance(expected_height));
}

// Tests special cases when the two linear functions have no equilibrium
// plane. It can happen when their gradients are deemed identical, which means
// the two functions are equal everywhere, or they are nowhere equal.
TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlaneNone) {
  // Same function in the same frame are equal everywhere, so there is no
  // equilibrium plane. (Mathematically speaking, every plane is an
  // equilibrium plane.)
  {
    // An arbitrary initial value of the plane.
    const Plane<double> init_plane_M{Vector3d::UnitX(), Vector3d(1, 2, 3)};

    Plane<double> plane_M(init_plane_M);
    const bool success = CalcEquilibriumPlane(
        0, field0_M_, 0, field0_M_, RigidTransformd::Identity(), &plane_M);

    ASSERT_FALSE(success);
    // Verify that the plane has not changed from its initial value.
    EXPECT_EQ(plane_M.normal(), init_plane_M.normal());
    // The choice of this query point is arbitrary.
    const Vector3d p_FQ(0.1, 0.2, 0.3);
    const double expected_height = init_plane_M.CalcHeight<double>(p_FQ);
    EXPECT_EQ(plane_M.CalcHeight<double>(p_FQ), expected_height);
  }
  // Use a translational frame L with respect to frame M to make two functions
  // that are nowhere equal.
  {
    const RigidTransformd X_ML(Vector3d(0.4, 0.3, 0.5));
    VolumeMeshFieldLinear<double, double> field2_L(field0_M_);

    // An arbitrary initial value of the plane.
    const Plane<double> init_plane_M{Vector3d::UnitX(), Vector3d(1, 2, 3)};

    Plane<double> plane_M(init_plane_M);
    const bool success =
        CalcEquilibriumPlane(0, field0_M_, 0, field2_L, X_ML, &plane_M);

    ASSERT_FALSE(success);
    // Verify that the plane has not changed from its initial value.
    EXPECT_EQ(plane_M.normal(), init_plane_M.normal());
    // The choice of this query point is arbitrary.
    const Vector3d p_FQ(0.1, 0.2, 0.3);
    const double expected_height = init_plane_M.CalcHeight<double>(p_FQ);
    EXPECT_EQ(plane_M.CalcHeight<double>(p_FQ), expected_height);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
