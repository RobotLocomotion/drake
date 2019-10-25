#include "drake/manipulation/util/moving_average_filter.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

namespace drake {
namespace manipulation {
namespace util {
namespace test {
namespace {

const unsigned int kWindowSize{3};

GTEST_TEST(MovingAverageFilterTest, InstantiationTest) {
  DRAKE_EXPECT_NO_THROW(MovingAverageFilter<double>(2));
  EXPECT_ANY_THROW(MovingAverageFilter<double>(0));
}

GTEST_TEST(MovingAverageDoubleTest, UpdateTest) {
  MovingAverageFilter<double> filter(kWindowSize);
  std::vector<double> window{-4.5, 1.0, 21.6};
  double sum = 0;
  for (size_t i = 0; i < window.size(); ++i) {
    sum += window[i];
    EXPECT_EQ((1.0 / (i + 1)) * sum, filter.Update(window[i]));
  }
  const double new_data_point = -12.8;
  EXPECT_EQ((1.0 / kWindowSize) * (sum + new_data_point - window[0]),
            filter.Update(new_data_point));
}

GTEST_TEST(MovingAverageVectorTest, UpdateVectorTest) {
  MovingAverageFilter<VectorX<double>> filter(kWindowSize);
  std::vector<VectorX<double>> window;
  window.push_back(
      (VectorX<double>(5) << 0.3, 0.45, 0.76, -0.45, -0.32).finished());

  window.push_back(
      (VectorX<double>(5) << 0.2, -5.2, 0.0, 0.45, -0.21).finished());

  window.push_back(
      (VectorX<double>(5) << 0.5, 0.32, -5.0, -0.91, 0.54).finished());

  VectorX<double> sum(5);
  sum.setZero();

  // Output represents moving average.
  for (size_t i = 0; i < window.size(); ++i) {
    sum += window[i];
    EXPECT_TRUE(CompareMatrices(((1.0 / (i + 1)) * sum),
                                (filter.Update(window[i])), 1e-12,
                                drake::MatrixCompareType::absolute));
  }

  const VectorX<double> new_data_point =
      (VectorX<double>(5) << 0.67, -78.9, 3.6, 3.5, -10.0).finished();
  EXPECT_TRUE(CompareMatrices(
      ((1.0 / kWindowSize) * (sum + new_data_point - window[0])),
      (filter.Update(new_data_point)), 1e-12,
      drake::MatrixCompareType::absolute));

  // Check for death on wrong sized data.
  EXPECT_ANY_THROW(filter.Update((VectorX<double>(2) << -5.6, 9.0).finished()));
}

}  // namespace
}  // namespace test
}  // namespace util
}  // namespace manipulation
}  // namespace drake
