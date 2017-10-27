#include "drake/common/make_coherent.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace common {
namespace {

GTEST_TEST(MakeCoherentTest, AutoDiff) {
  using T = Eigen::AutoDiffScalar<Eigen::VectorXd>;
  const T donor(7., Eigen::Vector2d{1., 2.});
  T recipient(4.);
  make_coherent(donor, &recipient);
  EXPECT_EQ(4., recipient.value());
  EXPECT_TRUE(
      CompareMatrices(recipient.derivatives(), Eigen::Vector2d::Zero()));
}

GTEST_TEST(MakeCoherentTest, Double) {
  const double donor(7.);
  double recipient(4.);
  EXPECT_NO_THROW(make_coherent(donor, &recipient));
}

GTEST_TEST(MakeCoherentTest, Symbolic) {
  const symbolic::Expression donor(7.);
  symbolic::Expression recipient(4.);
  EXPECT_NO_THROW(make_coherent(donor, &recipient));
}

}  // namespace
}  // namespace common
}  // namespace drake
