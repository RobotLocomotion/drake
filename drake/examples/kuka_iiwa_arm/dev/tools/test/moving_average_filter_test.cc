#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/examples/kuka_iiwa_arm/dev/tools/moving_average_filter.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace tools {

namespace {

const unsigned int kWindowSize{3};

GTEST_TEST(testMAFilter, instantiation_test) {
  std::unique_ptr<MovingAverageFilter<double>> ma_filter1, ma_filter2;
  EXPECT_NO_THROW(ma_filter1 =
                      std::make_unique<MovingAverageFilter<double>>(2));
  EXPECT_ANY_THROW(ma_filter2 =
                       std::make_unique<MovingAverageFilter<double>>(1));
}

GTEST_TEST(MovingAverageDoubleTest, Compute) {
  auto filter = std::make_unique<MovingAverageFilter<double>>(kWindowSize);
  std::vector<double> window;
  window.push_back(0.45);
  window.push_back(7.8);
  window.push_back(3.2);
  double sum = 0;

  for (size_t i = 0; i < window.size(); ++i) {
    sum += window[i];
    EXPECT_EQ((1 / 3) * sum, filter->compute(window[i]));
  }
  double new_data_point = -23.2;
  EXPECT_EQ((1 / 3) * (sum + new_data_point - window[0]),
            filter->compute(new_data_point));
}

GTEST_TEST(MovingAverageVectorTest, compute_vector_test) {
  auto filter =std::make_unique<MovingAverageFilter<Eigen::Vector3d>>(
      kWindowSize);
  std::vector<Eigen::Vector3d> window;
  window.push_back(Eigen::Vector3d(0.3, 0.45, 0.76));
  window.push_back(Eigen::Vector3d(-0.67, 2.45, -0.98));
  window.push_back(Eigen::Vector3d(42.0, -10.45, 4.65));

  Eigen::VectorXd sum = Eigen::Vector3d::Zero();

  // output represents moving average.
  for (size_t i = 0; i < window.size(); ++i) {
    sum += window[i];
    EXPECT_EQ((1 / 3) * sum, filter->compute(window[i]));
  }

  Eigen::Vector3d new_data_point(0.67, -78.9, 3.6);

  EXPECT_EQ((1 / 3) * (sum + new_data_point - window[0]),
            filter->compute(new_data_point));
}

}  // namespace

}  // namespace tools
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
