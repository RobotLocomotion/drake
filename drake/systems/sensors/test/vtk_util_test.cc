#include "drake/systems/sensors/vtk_util.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

const double kTolerance = std::numeric_limits<double>::epsilon();
const int kNumPoints = 4;
const int kSize = 10.;

struct Point {
  double x;
  double y;
  double z;
};

const Point kExpectedPointSet[kNumPoints] = {
  Point{ kSize, kSize, 0.},
  Point{ kSize,-kSize, 0.},
  Point{-kSize, kSize, 0.},
  Point{-kSize,-kSize, 0.}
};


class VtkUtilTest : public ::testing::Test {
 public:
  VtkUtilTest() {
    for (auto point : kExpectedPointSet) {
      point_set_.push_back(point);
    }
  }

  void SetUp() {};

  void MatchAndRemovePoint(double point[3]) {
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
    EXPECT_TRUE(matched);
  }

 private:
  std::vector<Point> point_set_;
};


TEST_F(VtkUtilTest, PointsCorrespoindenceTest) {
  unsigned char color[3] = {255, 128, 0};
  vtkSmartPointer<vtkPolyData> dut = VtkUtil::CreateSquarePlane(kSize, color);

  EXPECT_EQ(kNumPoints, dut->GetPoints()->GetNumberOfPoints());

  for (int i = 0; i < kNumPoints; ++i) {
    double dut_point[3];
    dut->GetPoints()->GetPoint(i, dut_point);
    MatchAndRemovePoint(dut_point);
  }
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
