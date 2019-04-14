#include "drake/solvers/evaluator_base.h"

#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using std::cout;
using std::endl;
using std::make_shared;
using std::make_unique;
using std::numeric_limits;
using std::runtime_error;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

using Eigen::Ref;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using ::testing::AssertionResult;
using ::testing::AssertionSuccess;
using ::testing::AssertionFailure;

namespace drake {
namespace solvers {
namespace {

GTEST_TEST(EvaluatorBaseTest, FunctionEvaluatorTest) {
  // Test that we can construct FunctionCosts with different signatures.

  auto callable = [](const auto& x, auto* y) {
    const Eigen::Vector3d c(1, 2, 3);
    *y = c * c.transpose() * x;
  };

  // Test that the various versions of the constructor all succeed.
  FunctionEvaluator evaluator_double(3, 3, callable);
  FunctionEvaluator evaluator_nonsymbolic(3, 3, callable, callable);
  FunctionEvaluator evaluator(3, 3, callable, callable, callable,
                              "all scalar types");

  { // Test double.
    const Eigen::Vector3d x(-10, -20, -30);
    Eigen::VectorXd y(3);
    const Eigen::Vector3d y_expected{-140, -280, -420};

    evaluator.Eval(x, &y);
    EXPECT_TRUE(CompareMatrices(y, y_expected));
  }

  { // Test AutoDiffXd.
    const Vector3<AutoDiffXd> x =
        math::initializeAutoDiff(Eigen::Vector3d{-10, -20, -30});
    VectorX<AutoDiffXd> y(3);
    const Eigen::Vector3d y_expected{-140, -280, -420};
    Eigen::Matrix3d grad_expected;
    grad_expected << 1, 2, 3, 2, 4, 6, 3, 6, 9;

    evaluator.Eval(x, &y);
    EXPECT_TRUE(CompareMatrices(math::autoDiffToValueMatrix(y), y_expected));
    EXPECT_TRUE(CompareMatrices(math::autoDiffToGradientMatrix(y),
        grad_expected));

    EXPECT_THROW(evaluator_double.Eval(x, &y), std::logic_error);
  }

  { // Test symbolic.
    const symbolic::Variable x{"x"}, y{"y"}, z{"z"};
    Vector3<symbolic::Variable> in{x, y, z};
    VectorX<symbolic::Expression> out(3);

    evaluator.Eval(in, &out);
    EXPECT_TRUE(out[2].EqualTo(3*x + 6*y + 9*z));

    EXPECT_THROW(evaluator_double.Eval(in, &out), std::logic_error);
    EXPECT_THROW(evaluator_nonsymbolic.Eval(in, &out), std::logic_error);
  }
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
