#include "drake/geometry/render_vtk/internal_vtk_util.h"

#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkCellData.h>      // vtkCommonDataModel
#include <vtkNew.h>           // vtkCommonCore
#include <vtkPlaneSource.h>   // vtkFiltersSources
#include <vtkSmartPointer.h>  // vtkCommonCore

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {
namespace {

const double kTolerance = 1e-15;
const double kSize = 10.0;
const double kHalfSize = kSize * 0.5;
const int kNumPoints = 4;

struct Point {
  double x;
  double y;
  double z;
};

// clang-format off
const Point kExpectedPointSet[kNumPoints] = {
  Point{ kHalfSize, -kHalfSize, 0.0},
  Point{ kHalfSize,  kHalfSize, 0.0},
  Point{-kHalfSize, -kHalfSize, 0.0},
  Point{-kHalfSize,  kHalfSize, 0.0}
};
// clang-format on

GTEST_TEST(PointsCorrespondenceTest, PlaneCreationTest) {
  vtkSmartPointer<vtkPlaneSource> dut = CreateSquarePlane(kSize);

  EXPECT_EQ(kNumPoints, dut->GetOutput()->GetPoints()->GetNumberOfPoints());
  for (int i = 0; i < kNumPoints; ++i) {
    double dut_point[3];
    dut->GetOutput()->GetPoints()->GetPoint(i, dut_point);

    EXPECT_NEAR(kExpectedPointSet[i].x, dut_point[0], kTolerance);
    EXPECT_NEAR(kExpectedPointSet[i].y, dut_point[1], kTolerance);
    EXPECT_NEAR(kExpectedPointSet[i].z, dut_point[2], kTolerance);
  }
}

// Verifies whether the conversion is correct.
GTEST_TEST(ConvertToVtkTransformTest, ConversionTest) {
  const math::RigidTransformd X_AB(
      Eigen::AngleAxisd(1.0, Eigen::Vector3d::Constant(1.0 / std::sqrt(3.0))),
      Eigen::Vector3d(1.0, 2.0, 3.0));

  auto dut_X_AB = ConvertToVtkTransform(X_AB);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(X_AB.GetAsMatrix4()(i, j),
                  dut_X_AB->GetMatrix()->GetElement(i, j), kTolerance);
    }
  }

  const Vector3<double> scale(2, 3, 4);
  auto dut_T_AB = ConvertToVtkTransform(X_AB, scale);

  // The transform T_AB is the concatenation of translation, scale, and rotation
  // (T * S * R). We get S * R by multiplying the _columns_ of R by the
  // corresponding scale factor.
  Eigen::Matrix4d T_AB_expected = X_AB.GetAsMatrix4();
  T_AB_expected.block<1, 3>(0, 0) *= scale.x();
  T_AB_expected.block<1, 3>(1, 0) *= scale.y();
  T_AB_expected.block<1, 3>(2, 0) *= scale.z();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(T_AB_expected(i, j), dut_T_AB->GetMatrix()->GetElement(i, j),
                  kTolerance);
    }
  }
}

GTEST_TEST(MakeVtkPointerArrayTest, ValidTest) {
  vtkNew<vtkCellData> data0;
  vtkNew<vtkCellData> data1;
  vtkNew<vtkCellData> data2;

  const auto dut1 = MakeVtkPointerArray(data0);
  EXPECT_EQ(dut1.size(), 1);
  EXPECT_EQ(dut1[0], data0.GetPointer());

  const auto dut2 = MakeVtkPointerArray(data0, data1);
  EXPECT_EQ(dut2.size(), 2);
  EXPECT_EQ(dut2[0], data0.GetPointer());
  EXPECT_EQ(dut2[1], data1.GetPointer());

  const auto dut3 = MakeVtkPointerArray(data0, data1, data2);
  EXPECT_EQ(dut3.size(), 3);
  EXPECT_EQ(dut3[0], data0.GetPointer());
  EXPECT_EQ(dut3[1], data1.GetPointer());
  EXPECT_EQ(dut3[2], data2.GetPointer());
}

}  // namespace
}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
