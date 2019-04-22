#include "drake/math/compute_numerical_gradient.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace math {
namespace {

GTEST_TEST(ComputeNumericalGradientTest, TestAffineFunction) {
  // Compute the numerical gradient of an affine function y = A * x + b. The
  // numerical gradient be A.
  Eigen::Matrix<double, 3, 2> A;
  // Arbitrary A matrix.
  // clang-format off
  A << 1.2, 3,
       2.5, -4.25,
       1.5, 0;
  // clang-format on
  // Arbitrary b vector.
  Eigen::Vector3d b(1, 2, 3);
  // I need to create a std::function, instead of using a lambda directly, as
  // template deduction doesn't work when converting lambda to std::function,
  // explained in
  // https://stackoverflow.com/questions/26665152/compiler-does-not-deduce-template-parameters-map-stdvector-stdvector
  std::function<void(const Eigen::Vector2d& x, Eigen::Vector3d*)> calc_fun =
      [&A, &b](const Eigen::Matrix<double, 2, 1>& x,
               Eigen::Matrix<double, 3, 1>* y) -> void { *y = A * x + b; };

  auto check_gradient = [&A, &calc_fun](const Eigen::Vector2d& x) {
    // forward difference
    auto J = ComputeNumericalGradient(
        calc_fun, x,
        NumericalGradientOption{NumericalGradientMethod::kForward});
    // The big tolerance is caused by the cancellation error in computing
    // f(x +  Δx) - f(x)), coming from the numerical roundoff error.
    const double tol = 1.1E-7;
    EXPECT_TRUE(CompareMatrices(J, A, tol));
    // backward difference
    J = ComputeNumericalGradient(
        calc_fun, x,
        NumericalGradientOption{NumericalGradientMethod::kBackward});
    EXPECT_TRUE(CompareMatrices(J, A, tol));
    // central difference
    J = ComputeNumericalGradient<Eigen::Vector2d, Eigen::Vector3d,
                                 Eigen::Vector2d>(
        calc_fun, x,
        NumericalGradientOption{NumericalGradientMethod::kCentral});
    EXPECT_TRUE(CompareMatrices(J, A, tol));
  };

  // Check gradient at different points.
  check_gradient(Eigen::Vector2d::Zero());
  check_gradient(Eigen::Vector2d(1, 2));
  check_gradient(Eigen::Vector2d(-1, 2E-10));
}

template <typename T>
void ToyFunction(const Vector3<T>& x, Vector2<T>* y) {
  using std::sin;
  (*y)(0) = x(0) + sin(x(1)) + x(0) * x(1) + 2;
  (*y)(1) = 2 * x(0) * x(2) + 3;
}

GTEST_TEST(ComputeNumericalGradientTest, TestToyFunction) {
  auto check_gradient = [](const Eigen::Vector3d& x) {
    // I need to create a std::function, rather than passing ToyFunction<double>
    // to ComputeNumericalGradient directly, as template deduction doesn't work
    // in the implicit conversion from function object to std::function.
    std::function<void(const Eigen::Vector3d&, Eigen::Vector2d*)>
        ToyFunctionDouble = ToyFunction<double>;
    auto J = ComputeNumericalGradient(
        ToyFunctionDouble, x,
        NumericalGradientOption{NumericalGradientMethod::kForward});
    auto x_autodiff = math::initializeAutoDiff<3>(x);
    Vector2<AutoDiffd<3>> y_autodiff;
    ToyFunction(x_autodiff, &y_autodiff);
    const double tol = 1E-7;
    EXPECT_TRUE(CompareMatrices(J, autoDiffToGradientMatrix(y_autodiff), tol));

    // backward difference
    J = ComputeNumericalGradient(
        ToyFunctionDouble, x,
        NumericalGradientOption{NumericalGradientMethod::kBackward});
    EXPECT_TRUE(CompareMatrices(J, autoDiffToGradientMatrix(y_autodiff), tol));

    // central difference
    // We could use the function object ToyFunction<double> instead of the
    // std::function ToyFunctionDouble, but then we have to explicitly
    // instantiate the template parameters for ComputeNumericalGradient.
    J = ComputeNumericalGradient<Eigen::Vector3d, Eigen::Vector2d,
                                 Eigen::Vector3d>(
        ToyFunction<double>, x,
        NumericalGradientOption{NumericalGradientMethod::kCentral});
    EXPECT_TRUE(CompareMatrices(J, autoDiffToGradientMatrix(y_autodiff), tol));
  };

  check_gradient(Eigen::Vector3d::Zero());
  check_gradient(Eigen::Vector3d(0, 1, 2));
  check_gradient(Eigen::Vector3d(0, -1, -2));
}

class ToyEvaluator : public solvers::EvaluatorBase {
 public:
  ToyEvaluator() : solvers::EvaluatorBase(2, 3) {}

  ~ToyEvaluator() override {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    Eigen::Vector2d y_double;
    ToyFunction(Eigen::Vector3d(x), &y_double);
    *y = y_double;
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    AutoDiffVecd<Eigen::Dynamic, 2> y_autodiff;
    ToyFunction(AutoDiffVecd<Eigen::Dynamic, 3>(x), &y_autodiff);
    *y = y_autodiff;
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    Vector2<symbolic::Expression> y_sym;
    ToyFunction(Vector3<symbolic::Expression>(x), &y_sym);
    *y = y_sym;
  }
};

GTEST_TEST(ComputeNumericalGradientTest, TestEvaluator) {
  // Test compute numerical gradient for solvers::EvaluatorBase
  ToyEvaluator evaluator;
  std::function<void(const Eigen::Ref<const Eigen::VectorXd>&,
                     Eigen::VectorXd*)>
      evaluator_eval =
          [&evaluator](const Eigen::Ref<const Eigen::VectorXd>& x,
                       Eigen::VectorXd* y) { return evaluator.Eval(x, y); };
  Eigen::Vector3d x(0, 1, 2);
  const auto J = ComputeNumericalGradient(evaluator_eval, x);
  AutoDiffVecXd y_autodiff;
  evaluator.Eval(math::initializeAutoDiff(x), &y_autodiff);
  const double tol = 1E-7;
  EXPECT_TRUE(
      CompareMatrices(J, math::autoDiffToGradientMatrix(y_autodiff), tol));
}

}  // namespace
}  // namespace math
}  // namespace drake
