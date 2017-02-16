#include "drake/systems/sensors/vtk_util.h"

#include <limits>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "gtest/gtest.h"

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
  Point{ kHalfSize,  kHalfSize, 0.},
  Point{ kHalfSize, -kHalfSize, 0.},
  Point{-kHalfSize,  kHalfSize, 0.},
  Point{-kHalfSize, -kHalfSize, 0.}
};


class PointsCorrespondenceTest : public ::testing::Test {
 public:
  PointsCorrespondenceTest() {
    for (auto point : kExpectedPointSet) {
      point_set_.push_back(point);
    }
  }

  void SetUp() {}

  bool MatchAndRemovePoint(double point[3]) {
    bool matched = false;
    for (auto it = point_set_.begin(); it != point_set_.end(); ++it) {
      const auto& a_point = *it;
      if ((std::abs(a_point.x - point[0]) < kTolerance) &&
          (std::abs(a_point.y - point[1]) < kTolerance) &&
          (std::abs(a_point.z - point[2]) < kTolerance)) {
        matched = true;
        point_set_.erase(it);
        break;
      }
    }
    return matched;
  }

 private:
  std::vector<Point> point_set_;
};

// Verifies whether the created plane has the expected set of 3D points.
TEST_F(PointsCorrespondenceTest, PlaneCreationTest) {
  vtkSmartPointer<vtkPlaneSource> dut =
      VtkUtil::CreateSquarePlane(kSize);

  EXPECT_EQ(kNumPoints, dut->GetOutput()->GetPoints()->GetNumberOfPoints());

  for (int i = 0; i < kNumPoints; ++i) {
    double dut_point[3];
    dut->GetOutput()->GetPoints()->GetPoint(i, dut_point);
    EXPECT_TRUE(MatchAndRemovePoint(dut_point));
  }
}

// Verifies whether conversion is correct.
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
