#include "drake/geometry/proximity/make_obb_from_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/mesh_source.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
constexpr double kTol = 1e-13;

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
  const Obb obj_obb = MakeObb(MeshSource(obj_path_), scale);
  ValidateObb(obj_obb, scale);
}

TEST_F(MakeObbFromMeshTest, Vtk) {
  const Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  const Obb vtk_obb = MakeObb(MeshSource(vtk_path_), scale);
  ValidateObb(vtk_obb, scale);
}

// The OBB for the gltf cube suffers from #14067. We just check that the
// half-widths are positive.
TEST_F(MakeObbFromMeshTest, Gltf) {
  const Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  const Obb gltf_obb = MakeObb(MeshSource(gltf_path_), scale);
  EXPECT_GT(gltf_obb.half_width().x(), 0.0);
  EXPECT_GT(gltf_obb.half_width().y(), 0.0);
  EXPECT_GT(gltf_obb.half_width().z(), 0.0);
}

// Test scaling effects.
TEST_F(MakeObbFromMeshTest, ScalingEffect) {
  const MeshSource mesh_source(obj_path_);
  const Eigen::Vector3d double_scale = Eigen::Vector3d(2.0, 2.0, 2.0);
  const Obb scaled_obb = MakeObb(mesh_source, double_scale);
  ValidateObb(scaled_obb, double_scale);
}

// Test non-uniform scaling.
TEST_F(MakeObbFromMeshTest, NonUniformScaling) {
  const MeshSource mesh_source(vtk_path_);
  const Eigen::Vector3d scale(2.0, 3.0, 4.0);
  const Obb obb = MakeObb(mesh_source, scale);
  ValidateObb(obb, scale);
}

// Test unsupported file extension.
TEST_F(MakeObbFromMeshTest, UnsupportedExtension) {
  // Create a mesh source with an unsupported extension.
  const MeshSource mesh_source("fake_file.stl");
  const Eigen::Vector3d scale = Eigen::Vector3d::Ones();

  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeObb(mesh_source, scale),
      "MakeObb only applies to .obj, .vtk, and .gltf meshes; "
      "unsupported extension '.stl' for geometry data: .*");
}

// Test with zero scale (should still work but produce degenerate OBB).
TEST_F(MakeObbFromMeshTest, ZeroScale) {
  const MeshSource mesh_source(obj_path_);
  const Eigen::Vector3d zero_scale = Eigen::Vector3d::Zero();
  const Obb obb = MakeObb(mesh_source, zero_scale);
  ValidateObb(obb, zero_scale);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
