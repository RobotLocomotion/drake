#include "drake/geometry/render/vtk_util.h"

#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <vtkCellData.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>

namespace drake {
namespace geometry {
namespace render {
namespace vtk_util {
namespace {

const double kTolerance = 1e-15;
const double kSize = 10.;
const double kHalfSize = kSize * 0.5;
const int kNumPoints = 4;

struct Point {
  double x;
  double y;
  double z;
};

const Point kExpectedPointSet[kNumPoints] = {
  Point{ kHalfSize, -kHalfSize, 0.},
  Point{ kHalfSize,  kHalfSize, 0.},
  Point{-kHalfSize, -kHalfSize, 0.},
  Point{-kHalfSize,  kHalfSize, 0.}
};

GTEST_TEST(PointsCorrespondenceTest, PlaneCreationTest) {
  vtkSmartPointer<vtkPlaneSource> dut =
      CreateSquarePlane(kSize);

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
  const math::RigidTransformd expected(
       Eigen::AngleAxisd(1., Eigen::Vector3d(1. / std::sqrt(3.),
                                             1. / std::sqrt(3.),
                                             1. / std::sqrt(3.))),
       Eigen::Vector3d(1., 2., 3.));

  auto dut = ConvertToVtkTransform(expected);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(expected.GetAsMatrix4()(i, j),
                  dut->GetMatrix()->GetElement(i, j),
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
}  // namespace vtk_util
}  // namespace render
}  // namespace geometry
}  // namespace drake
