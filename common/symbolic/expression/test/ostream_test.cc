// Drake never uses operator<< for Eigen::Matix data, but we'd like users to be
// able to do so at their discretion. Therefore, we test it here to ensure that
// our NumTraits specialization is sufficient for this purpose.
#undef EIGEN_NO_IO

#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/symbolic/expression/all.h"

namespace drake {
namespace symbolic {
namespace {

using std::ostringstream;

// Provides common variables and matrices that are used by the
// following tests.
class VariableOverloadingTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};

  Eigen::Matrix<Variable, 2, 2> var_mat_;
  Eigen::Matrix<Expression, 2, 2> expr_mat_;

  void SetUp() override {
    // clang-format off
    var_mat_ << x_, y_,
                z_, w_;
    expr_mat_ << (x_ + z_), (x_ + w_),
                 (y_ + z_), (y_ + w_);
    // clang-format on
  }
};

// XXX Mat<var> as well.

TEST_F(VariableOverloadingTest, EigenExpressionMatrixOutputStream) {
  // The following fails if we do not provide
  // `Eigen::NumTraits<drake::symbolic::DerivedA>`
  ostringstream oss1;
  oss1 << expr_mat_;

  ostringstream oss2;
  oss2 << "      (x + z)       (x + w)\n"
       << "      (y + z)       (y + w)";

  EXPECT_EQ(oss1.str(), oss2.str());
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
