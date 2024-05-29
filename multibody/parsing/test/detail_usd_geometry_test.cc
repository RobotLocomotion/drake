#include "drake/multibody/parsing/detail_usd_geometry.h"

#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/cube.h"
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
  }
 protected:
  pxr::UsdStageRefPtr stage_;
  double meters_per_unit_;
};

TEST_F(UsdGeometryTest, GetBoxDimensionTest) {
  pxr::UsdGeomCube cube = pxr::UsdGeomCube::Define(
    stage_, pxr::SdfPath("/Cube"));

  double size = 49.0;
  EXPECT_TRUE(cube.CreateSizeAttr().Set(size));

  pxr::VtVec3fArray extent;
  EXPECT_TRUE(pxr::UsdGeomCube::ComputeExtent(size, &extent));
  EXPECT_TRUE(cube.CreateExtentAttr().Set(extent));

  double scale_factor = 0.5;
  pxr::GfMatrix4d transform = pxr::GfMatrix4d();
  transform.SetScale(scale_factor);
  EXPECT_TRUE(cube.AddTransformOp().Set(transform));

  std::optional<Eigen::Vector3d> dimension = GetBoxDimension(
    cube.GetPrim(), meters_per_unit_, diagnostic_policy_);
  EXPECT_TRUE(dimension.has_value());
  double final_size = size * scale_factor * meters_per_unit_;
  EXPECT_EQ(dimension, Eigen::Vector3d(final_size, final_size, final_size));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
