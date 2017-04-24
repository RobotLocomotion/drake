#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/test/is_dynamic_castable.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/test/generic_trivial_costs.h"

using std::shared_ptr;
using std::make_shared;
using std::unique_ptr;
using std::make_unique;
using Eigen::Ref;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using drake::Vector1d;
using drake::solvers::test::GenericTrivialCost1;
using drake::solvers::test::GenericTrivialCost2;

namespace drake {
namespace solvers {
namespace {

template<typename T, bool is_dereferencable>
struct deref {
  static const auto& to_const_rvalue(T&& x) {
    return x;
  }
};
template<typename T>
struct deref<T, true> {
  static const auto& to_const_rvalue(T&& x) {
    return *x;
  }
};

template<bool is_dereferencable, typename T>
const auto& to_const_rvalue(T&& t) {
  using impl = deref<T, is_dereferencable>;
  return impl::to_const_rvalue(std::forward<T>(t));
}

// Verifies that FunctionCost form can be constructed correctly
template<bool is_pointer, typename F>
void VerifyFunctionCost(F&& f, const Ref<const VectorXd>& x_value) {
  auto cost = CreateFunctionCost(std::forward<F>(f));
  EXPECT_TRUE(is_dynamic_castable<Cost>(cost));
  // TODO(eric.cousineau): Remove when shim is removed
  EXPECT_TRUE(is_dynamic_castable<Constraint>(cost));
  // Compare values
  Eigen::VectorXd y_expected(1), y;
  to_const_rvalue<is_pointer>(f).eval(x_value, y_expected);
  cost->Eval(x_value, y);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

GTEST_TEST(testCost, testFunctionCost) {
  // Test that we can construct FunctionCosts with different signatures
  Eigen::Vector2d x(1, 2);
  VerifyFunctionCost<false>(GenericTrivialCost2(), x);
  const GenericTrivialCost2 obj_const;
  VerifyFunctionCost<false>(obj_const, x);
  VerifyFunctionCost<true>(make_shared<GenericTrivialCost2>(), x);
  VerifyFunctionCost<true>(make_unique<GenericTrivialCost2>(), x);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
