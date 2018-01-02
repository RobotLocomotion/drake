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

// Generic dereferencing for a value type, or a managed pointer.
template <typename T>
const T& deref(const T& x) {
  return x;
}
template <typename T>
const T& deref(const shared_ptr<T>& x) {
  return *x;
}
template <typename T>
const T& deref(const unique_ptr<T>& x) {
  return *x;
}

struct GenericTrivialFunctor {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GenericTrivialFunctor)

  GenericTrivialFunctor() {}

  int numInputs() const { return 3; }
  int numOutputs() const { return 3; }

  template <typename T>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(const detail::VecIn<T>& x, detail::VecOut<T>& y) const {
    Eigen::Vector3d c(1, 2, 3);
    y = c * x.transpose() * c;
  }
};

AssertionResult CompareAutodiff(const AutoDiffVecXd& tx_expected,
                                const AutoDiffVecXd& tx_actual,
                                double tolerance = 0.0) {
  const VectorXd x_expected = math::autoDiffToValueMatrix(tx_expected);
  const VectorXd x_actual = math::autoDiffToValueMatrix(tx_actual);
  AssertionResult value_result =
      CompareMatrices(x_expected, x_actual, tolerance);
  if (!value_result) {
    return value_result << "(value)";
  }
  const MatrixXd dx_expected = math::autoDiffToGradientMatrix(tx_expected);
  const MatrixXd dx_actual = math::autoDiffToGradientMatrix(tx_actual);
  AssertionResult grad_result =
      CompareMatrices(dx_expected, dx_actual, tolerance);
  if (!grad_result) {
    return grad_result << "(gradient)";
  }
  return AssertionSuccess();
}

// Verifies that FunctionEvaluator can be constructed correctly with
// different callable objects (r/l-value, shared/unique_ptr).
// TODO(eric.cousineau): Share these function-based test utilities with
// cost_test.
template <typename F>
void VerifyFunctionEvaluator(F&& f, const VectorXd& x) {
  // Compute expected value prior to forwarding `f` (which may involve
  // move'ing `unique_ptr<>` or `shared_ptr<>`, making `f` a nullptr).
  Eigen::VectorXd y_expected(3);
  // Manually specialize the call to `eval` because compiler may have issues
  // inferring T from Eigen::Ref<VectorX<T>>. It works in FunctionEvaluator
  // because Ref<VectorX<T>> is already determined by the function signature.
  deref(f).template eval<double>(x, y_expected);
  const AutoDiffVecXd tx = math::initializeAutoDiff(x);
  AutoDiffVecXd ty_expected(3);
  deref(f).template eval<AutoDiffXd>(tx, ty_expected);
  Eigen::MatrixXd dy_expected = math::autoDiffToGradientMatrix(ty_expected);

  // Construct evaluator, moving `f` if applicable.
  shared_ptr<EvaluatorBase> evaluator =
      MakeFunctionEvaluator(std::forward<F>(f));
  EXPECT_TRUE(is_dynamic_castable<EvaluatorBase>(evaluator));

  // Compare double.
  Eigen::VectorXd y(3);
  evaluator->Eval(x, y);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
  // Check AutoDif.
  AutoDiffVecXd ty(3);
  evaluator->Eval(tx, ty);
  EXPECT_TRUE(CompareAutodiff(ty, ty_expected));
}

// Store generic callable (e.g. a lambda), and assign sizes to it manually.
// TODO(eric.cousineau): Migrate this to function.h or evaluator_base.h.
template <typename Callable>
class FunctionWrapper {
 public:
  template <typename CallableF>
  FunctionWrapper(CallableF&& callable, int num_outputs, int num_vars)
      : num_outputs_(num_outputs),
        num_vars_(num_vars),
        callable_(std::forward<CallableF>(callable)) {}

  int numInputs() const { return num_vars_; }
  int numOutputs() const { return num_outputs_; }

  template <typename T>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(const detail::VecIn<T>& x, detail::VecOut<T>& y) const {
    callable_(x, y);
  }

 public:
  int num_outputs_{};
  int num_vars_{};
  Callable callable_;
};

template <typename CallableF>
auto MakeFunctionWrapped(CallableF&& c, int num_outputs, int num_vars) {
  using Callable = std::decay_t<CallableF>;
  using Wrapped = FunctionWrapper<Callable>;
  return Wrapped(std::forward<CallableF>(c), num_outputs, num_vars);
}

GTEST_TEST(EvaluatorBaseTest, FunctionEvaluatorTest) {
  // Test that we can construct FunctionCosts with different signatures.
  Eigen::Vector3d x(-10, -20, -30);
  VerifyFunctionEvaluator(GenericTrivialFunctor(), x);
  const GenericTrivialFunctor obj_const{};
  VerifyFunctionEvaluator(obj_const, x);
  VerifyFunctionEvaluator(make_shared<GenericTrivialFunctor>(), x);
  VerifyFunctionEvaluator(make_unique<GenericTrivialFunctor>(), x);

  auto callable = [](const auto& x1, auto& y1) {
    Eigen::Vector3d c(1, 2, 3);
    y1 = c * x1.transpose() * c;
  };
  VerifyFunctionEvaluator(MakeFunctionWrapped(callable, 3, 3), x);
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
