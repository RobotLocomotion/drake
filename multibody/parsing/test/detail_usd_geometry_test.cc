#include "drake/multibody/parsing/detail_usd_geometry.h"

#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/cube.h"
#include "pxr/usd/usdGeom/cylinder.h"
#include "pxr/usd/usdGeom/sphere.h"
#include <gtest/gtest.h>

#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/multibody/parsing/detail_usd_parser.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

class UsdGeometryTest : public test::DiagnosticPolicyTestBase {
 public:
  UsdGeometryTest() {
    // Construct a parser object so that it initializes the USD library.
    UsdParserWrapper parser;

    stage_ = pxr::UsdStage::CreateInMemory();
    meters_per_unit_ = 0.01;
    stage_up_axis_ = pxr::TfToken("Z");
  }
 protected:
  pxr::UsdStageRefPtr stage_;
  pxr::TfToken stage_up_axis_;
  double meters_per_unit_;
};

TEST_F(UsdGeometryTest, GetBoxDimensionTest) {
  pxr::UsdGeomCube box = pxr::UsdGeomCube::Define(
    stage_, pxr::SdfPath("/Box"));

  double size = 49.0;
  EXPECT_TRUE(box.CreateSizeAttr().Set(size));

  pxr::VtVec3fArray extent;
  EXPECT_TRUE(pxr::UsdGeomCube::ComputeExtent(size, &extent));
  EXPECT_TRUE(box.CreateExtentAttr().Set(extent));

  pxr::GfVec3d scale_factor = pxr::GfVec3d(0.4, 0.5, 0.6);
  auto scale_op = box.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  EXPECT_TRUE(scale_op.Set(scale_factor));

  std::optional<Eigen::Vector3d> dimension = GetBoxDimension(
    box.GetPrim(), meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(dimension.has_value());

  Eigen::Vector3d correct_dimension =
    UsdVec3dToEigen(scale_factor) * size * meters_per_unit_;
  EXPECT_EQ(dimension.value(), correct_dimension);
}

TEST_F(UsdGeometryTest, GetEllipsoidDimensionTest) {
  pxr::UsdGeomSphere ellipsoid = pxr::UsdGeomSphere::Define(
    stage_, pxr::SdfPath("/Ellipsoid"));

  double radius = 17.0;
  EXPECT_TRUE(ellipsoid.CreateRadiusAttr().Set(radius));

  pxr::VtVec3fArray extent;
  EXPECT_TRUE(pxr::UsdGeomSphere::ComputeExtent(radius, &extent));
  EXPECT_TRUE(ellipsoid.CreateExtentAttr().Set(extent));

  pxr::GfVec3d scale_factor = pxr::GfVec3d(0.6, 1.1, 2.9);
  auto scale_op = ellipsoid.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  EXPECT_TRUE(scale_op.Set(scale_factor));

  std::optional<Eigen::Vector3d> dimension = GetEllipsoidDimension(
    ellipsoid.GetPrim(), meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(dimension.has_value());

  Eigen::Vector3d correct_dimension =
    UsdVec3dToEigen(scale_factor) * radius * meters_per_unit_;
  EXPECT_EQ(dimension.value(), correct_dimension);
}

TEST_F(UsdGeometryTest, GetCylinderDimensionTest) {
  pxr::UsdGeomCylinder cylinder = pxr::UsdGeomCylinder::Define(
    stage_, pxr::SdfPath("/Cylinder"));

  double radius = 62.0;
  double height = 199.0;
  pxr::TfToken axis = pxr::TfToken("Z");
  EXPECT_TRUE(cylinder.CreateRadiusAttr().Set(radius));
  EXPECT_TRUE(cylinder.CreateHeightAttr().Set(height));
  EXPECT_TRUE(cylinder.CreateAxisAttr().Set(axis));

  pxr::VtVec3fArray extent;
  EXPECT_TRUE(pxr::UsdGeomCylinder::ComputeExtent(
    height, radius, axis, &extent));
  EXPECT_TRUE(cylinder.CreateExtentAttr().Set(extent));

  pxr::GfVec3d scale_factor = pxr::GfVec3d(0.7, 0.7, 0.9);
  auto scale_op = cylinder.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);
  EXPECT_TRUE(scale_op.Set(scale_factor));

  std::optional<Eigen::Vector2d> dimension = GetCylinderDimension(
    cylinder.GetPrim(), meters_per_unit_, stage_up_axis_, diagnostic_policy_);
  EXPECT_TRUE(dimension.has_value());

  double correct_radius = scale_factor[0] * radius * meters_per_unit_;
  double correct_height = scale_factor[2] * height * meters_per_unit_;
  auto correct_dimension = Eigen::Vector2d(correct_radius, correct_height);
  EXPECT_EQ(dimension.value(), correct_dimension);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
