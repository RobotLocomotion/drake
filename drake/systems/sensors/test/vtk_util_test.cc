#include "drake/systems/sensors/vtk_util.h"

#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <vtkCellData.h>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>

namespace drake {
namespace systems {
namespace sensors {
namespace {

const double kTolerance = std::numeric_limits<double>::epsilon();
const double kSize = 10.;
const double kHalfSize = kSize * 0.5;
const int kNumPoints = 4;

struct Point {
  double x;
  double y;
  double z;
};

const Point kExpectedPointSet[kNumPoints] = {
  Point{-kHalfSize, -kHalfSize, 0.},
  Point{-kHalfSize,  kHalfSize, 0.},
  Point{ kHalfSize, -kHalfSize, 0.},
  Point{ kHalfSize,  kHalfSize, 0.}
};

GTEST_TEST(PointsCorrespondenceTest, PlaneCreationTest) {
  vtkSmartPointer<vtkPlaneSource> dut =
      VtkUtil::CreateSquarePlane(kSize);

  EXPECT_EQ(kNumPoints, dut->GetOutput()->GetPoints()->GetNumberOfPoints());
  for (int i = 0; i < kNumPoints; ++i) {
    double dut_point[3];
    dut->GetOutput()->GetPoints()->GetPoint(i, dut_point);

    EXPECT_EQ(kExpectedPointSet[i].x, dut_point[0]);
    EXPECT_EQ(kExpectedPointSet[i].y, dut_point[1]);
    EXPECT_EQ(kExpectedPointSet[i].z, dut_point[2]);
  }
}

// Verifies whether the conversion is correct.
GTEST_TEST(ConvertToVtkTransformTest, ConversionTest) {
  const Eigen::Isometry3d expected(
      (Eigen::Translation3d(1., 2., 3.) *
       Eigen::AngleAxisd(1., Eigen::Vector3d(1. / std::sqrt(3.),
                                             1. / std::sqrt(3.),
                                             1. / std::sqrt(3.)))));

  auto dut = VtkUtil::ConvertToVtkTransform(expected);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(expected.matrix()(i, j),
                  dut->GetMatrix()->GetElement(i, j),
                  kTolerance);
    }
  }
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
