#include "drake/geometry/proximity/calc_obb.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using internal::MakeObb;

constexpr double kTol = 1.0e-13;

GTEST_TEST(CalcObbFromShapeTest, CalcObb) {
  // Box
  {
    const Box box(1, 2, 3);
    const std::optional<Obb> obb = CalcObb(box);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    EXPECT_TRUE(CompareMatrices(obb->half_width(),
                                Vector3<double>(0.5, 1.0, 1.5), kTol));
  }

  // Capsule
  {
    const double radius = 1.23;
    const double length = 2.4;
    const Capsule capsule(radius, length);
    const std::optional<Obb> obb = CalcObb(capsule);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    const double half_length = length / 2.0 + radius;
    EXPECT_TRUE(CompareMatrices(
        obb->half_width(), Vector3<double>(radius, radius, half_length), kTol));
  }

  // Cylinder
  {
    const double radius = 1.3;
    const double length = 2.1;
    const Cylinder cylinder(radius, length);
    const std::optional<Obb> obb = CalcObb(cylinder);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    const double half_length = length / 2.0;
    EXPECT_TRUE(CompareMatrices(
        obb->half_width(), Vector3<double>(radius, radius, half_length), kTol));
  }

  // Ellipsoid
  {
    const Ellipsoid ellipsoid(1, 2, 3);
    const std::optional<Obb> obb = CalcObb(ellipsoid);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    EXPECT_TRUE(
        CompareMatrices(obb->half_width(), Vector3<double>(1, 2, 3), kTol));
  }

  // Sphere
  {
    const double radius = 3.0;
    const Sphere sphere(radius);
    const std::optional<Obb> obb = CalcObb(sphere);
    ASSERT_TRUE(obb.has_value());
    EXPECT_TRUE(obb->pose().IsExactlyIdentity());
    EXPECT_TRUE(CompareMatrices(obb->half_width(),
                                Vector3<double>(radius, radius, radius), kTol));
  }

  // HalfSpace
  {
    const HalfSpace hs;
    const std::optional<Obb> obb = CalcObb(hs);
    EXPECT_FALSE(obb.has_value());
  }

  // MeshcatCone
  {
    const MeshcatCone cone(4, 2, 3);
    const std::optional<Obb> obb = CalcObb(cone);
    EXPECT_FALSE(obb.has_value());
  }

  // For Mesh and Convex, we'll use a simple cube and confirm that the OBB is
  // as expected.
  {
    const std::string cube_obj =
        FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    const Mesh mesh(cube_obj);
    const std::optional<Obb> obb = CalcObb(mesh);
    ASSERT_TRUE(obb.has_value());
    // The quad_cube is an axis-aligned 2x2x2 cube centered at the origin.
    // So, its OBB should be an axis-aligned box with half_width of (1, 1, 1).
    // PCA may lead to arbitrary axis directions. We check that the translation
    // is zero and that the rotation is a signed permutation matrix.
    const auto& pose = obb->pose();
    EXPECT_TRUE(
        CompareMatrices(pose.translation(), Vector3<double>::Zero(), kTol));
    const Matrix3<double>& R_abs = pose.rotation().matrix().cwiseAbs();
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(R_abs.row(i).sum(), 1.0, kTol);
      EXPECT_NEAR(R_abs.col(i).sum(), 1.0, kTol);
    }
    EXPECT_TRUE(
        CompareMatrices(obb->half_width(), Vector3<double>(1, 1, 1), kTol));
  }
  {
    const std::string cube_obj =
        FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
    const Convex convex(cube_obj);
    const std::optional<Obb> obb = CalcObb(convex);
    ASSERT_TRUE(obb.has_value());
    const auto& pose = obb->pose();
    EXPECT_TRUE(
        CompareMatrices(pose.translation(), Vector3<double>::Zero(), kTol));
    const Matrix3<double>& R_abs = pose.rotation().matrix().cwiseAbs();
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(R_abs.row(i).sum(), 1.0, kTol);
      EXPECT_NEAR(R_abs.col(i).sum(), 1.0, kTol);
    }
    EXPECT_TRUE(
        CompareMatrices(obb->half_width(), Vector3<double>(1, 1, 1), kTol));
  }
}

class MakeObbFromMeshTest : public ::testing::Test {
 protected:
  void SetUp() override {
    obj_path_ = FindResourceOrThrow("drake/geometry/test/cube_with_hole.obj");
    vtk_path_ = FindResourceOrThrow("drake/geometry/test/cube_as_volume.vtk");
    gltf_path_ = FindResourceOrThrow("drake/geometry/test/cube_with_hole.gltf");
  }

  // Helper function to verify that the OBB has the correct half-widths and
  // pose.
  void ValidateObb(const Obb& obb, const Vector3<double>& half_width) {
    const Matrix3<double>& R_abs = obb.pose().rotation().matrix().cwiseAbs();
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(R_abs.row(i).sum(), 1.0, kTol);
      EXPECT_NEAR(R_abs.col(i).sum(), 1.0, kTol);
    }
    // Verify that the half-widths are correct. We don't know the exact
    // ordering of the half-widths, so we order them and then compare.
    Vector3d expected = half_width;
    Vector3d computed = obb.half_width();
    std::sort(expected.data(), expected.data() + expected.size());
    std::sort(computed.data(), computed.data() + computed.size());
    EXPECT_TRUE(CompareMatrices(computed, expected, kTol));
  }

  // Each of the test files is a cube (with or without a hole) that has an
  // obb with half-widths (1, 1, 1) and with identity pose.
  std::string obj_path_;
  std::string vtk_path_;
  std::string gltf_path_;
};

TEST_F(MakeObbFromMeshTest, Obj) {
  const Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  const std::optional<Obb> obj_obb = MakeObb(MeshSource(obj_path_), scale);
  ASSERT_TRUE(obj_obb.has_value());
  ValidateObb(*obj_obb, scale);
}

TEST_F(MakeObbFromMeshTest, Vtk) {
  const Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  const std::optional<Obb> vtk_obb = MakeObb(MeshSource(vtk_path_), scale);
  ASSERT_TRUE(vtk_obb.has_value());
  ValidateObb(*vtk_obb, scale);
}

// The OBB for the gltf cube suffers from #14067. We just check that the
// half-widths are positive.
TEST_F(MakeObbFromMeshTest, Gltf) {
  const Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  const std::optional<Obb> gltf_obb = MakeObb(MeshSource(gltf_path_), scale);
  ASSERT_TRUE(gltf_obb.has_value());
  EXPECT_GT(gltf_obb->half_width().x(), 0.0);
  EXPECT_GT(gltf_obb->half_width().y(), 0.0);
  EXPECT_GT(gltf_obb->half_width().z(), 0.0);
}

// Test scaling effects.
TEST_F(MakeObbFromMeshTest, ScalingEffect) {
  const MeshSource mesh_source(obj_path_);
  const Eigen::Vector3d double_scale = Eigen::Vector3d(2.0, 2.0, 2.0);
  const std::optional<Obb> scaled_obb = MakeObb(mesh_source, double_scale);
  ASSERT_TRUE(scaled_obb.has_value());
  ValidateObb(*scaled_obb, double_scale);
}

// Test non-uniform scaling.
TEST_F(MakeObbFromMeshTest, NonUniformScaling) {
  const MeshSource mesh_source(vtk_path_);
  const Eigen::Vector3d scale(2.0, 3.0, 4.0);
  const std::optional<Obb> obb = MakeObb(mesh_source, scale);
  ASSERT_TRUE(obb.has_value());
  ValidateObb(*obb, scale);
}

// Test unsupported file extension.
TEST_F(MakeObbFromMeshTest, UnsupportedExtension) {
  // Create a mesh source with an unsupported extension.
  const MeshSource mesh_source("fake_file.stl");
  const Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  const std::optional<Obb> obb = MakeObb(mesh_source, scale);
  EXPECT_FALSE(obb.has_value());
}

// Test with zero scale (should still work but produce degenerate OBB).
TEST_F(MakeObbFromMeshTest, ZeroScale) {
  const MeshSource mesh_source(obj_path_);
  const Eigen::Vector3d zero_scale = Eigen::Vector3d::Zero();
  const std::optional<Obb> obb = MakeObb(mesh_source, zero_scale);
  ASSERT_TRUE(obb.has_value());
  ValidateObb(*obb, zero_scale);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
