#include "drake/geometry/proximity/field_intersection.h"

#include <memory>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;
using math::RigidTransformd;
using math::RotationMatrixd;

class FieldIntersectionLowLevelTest : public ::testing::Test {
 public:
  FieldIntersectionLowLevelTest()
      : mesh0_M_(
            std::vector<VolumeElement>{
                VolumeElement(0, 1, 2, 3)},
            std::vector<Vector3d>{Vector3d(2, 0, 2), Vector3d(-2, 0, 2),
                                  Vector3d(0, 2, -2), Vector3d(0, -2, -2)}),
        // (4 + x + y + z)e4
        field0_M_(vector<double>({8e4, 4e4, 4e4, 0}), &mesh0_M_),
        mesh1_N_(std::vector<VolumeElement>{VolumeElement(0, 1, 2, 3)},
                 std::vector<Vector3d>{
                     Vector3d(1.5, 1.5, 2.5), Vector3d(-1.5, -1.5, 2.5),
                     Vector3d(-1.5, 1.5, -2.5), Vector3d(1.5, -1.5, -2.5)}),
        // (4 + x + y)e4
        field1_N_(vector<double>({7e4, 1e4, 4e4, 4e4}), &mesh1_N_) {}

 protected:
  void SetUp() override {
    //    WriteVolumeMeshFieldLinearToVtk(
    //        "field0_M.vtk", field0_M_,
    //        "Pressure distribution on compliant tetrahedron in frame M");
    //    WriteVolumeMeshFieldLinearToVtk(
    //        "field1_N.vtk", field1_N_,
    //        "Pressure distribution on compliant tetrahedron in frame N");
    ASSERT_TRUE(CompareMatrices(field0_M_.EvaluateGradient(0),
                                Vector3d(1, 1, 1) * 1e4, 1e-10));
    ASSERT_TRUE(CompareMatrices(field1_N_.EvaluateGradient(0),
                                Vector3d(1, 1, 0) * 1e4, 1e-10));
  }

  const VolumeMesh<double> mesh0_M_;
  const VolumeMeshFieldLinear<double, double> field0_M_;
  const VolumeMesh<double> mesh1_N_;
  const VolumeMeshFieldLinear<double, double> field1_N_;
};

TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlane) {
  const int first_element_in_field0{0};
  const int first_element_in_field1{0};
  const RigidTransformd identity_X_MN = RigidTransformd::Identity();
  Plane<double> plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
  bool success = CalcEquilibriumPlane(first_element_in_field0, field0_M_,
                                      first_element_in_field1, field1_N_,
                                      identity_X_MN, &plane_M);
  ASSERT_TRUE(success);

  // Verify that the equilibrium plane is the X-Y plane in frame M.
  // This is the because the two fields are:
  //     f0(x,y,z) = (4 + x + y + z)e4
  //     f1(x,y,z) = (4 + x + y)e4,
  // so the plane of f0 = f1 is:
  //     (4 + x + y + z)e4 - (4 + x + y)e4 = 0 ==> The plane z = 0.
  EXPECT_TRUE(CompareMatrices(plane_M.normal(), Vector3d::UnitZ(), 1e-14));
  EXPECT_NEAR(plane_M.CalcHeight<double>(Vector3d::Zero()), 0.0, 1e-14);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
