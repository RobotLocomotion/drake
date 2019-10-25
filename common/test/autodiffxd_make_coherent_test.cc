#include "drake/common/autodiffxd_make_coherent.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

namespace drake {
namespace common {
namespace {

GTEST_TEST(AutoDiffXdMakeCoherentTest, AutoDiffEmptyDerivatives) {
  const AutoDiffXd donor(7., Eigen::Vector2d{1., 2.});
  AutoDiffXd recipient(4.);  // Recipient's derivatives are empty.
  autodiffxd_make_coherent(donor, &recipient);
  EXPECT_EQ(4., recipient.value());
  EXPECT_TRUE(
      CompareMatrices(recipient.derivatives(), Eigen::Vector2d::Zero()));
}

GTEST_TEST(AutoDiffXdMakeCoherentTest, AutoDiffNonemptyDerivatives) {
  const AutoDiffXd donor(7., Eigen::Vector2d{1., 2.});
  AutoDiffXd recipient(4., Vector1d{1.});  // Nonempty but incompatibly-sized
                                           // derivatives.
  EXPECT_THROW(autodiffxd_make_coherent(donor, &recipient), std::runtime_error);
}

GTEST_TEST(AutoDiffXdMakeCoherentTest, Double) {
  const double donor(7.);
  double recipient(4.);
  DRAKE_EXPECT_NO_THROW(autodiffxd_make_coherent(donor, &recipient));
}

GTEST_TEST(AutoDiffXdMakeCoherentTest, Symbolic) {
  const symbolic::Expression donor(7.);
  symbolic::Expression recipient(4.);
  DRAKE_EXPECT_NO_THROW(autodiffxd_make_coherent(donor, &recipient));
}

}  // namespace
}  // namespace common
}  // namespace drake
