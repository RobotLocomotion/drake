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
    auto x_autodiff = math::InitializeAutoDiff<3>(x);
    Vector2<AutoDiffd<3>> y_autodiff;
    ToyFunction(x_autodiff, &y_autodiff);
    const double tol = 1E-7;
    EXPECT_TRUE(CompareMatrices(J, ExtractGradient(y_autodiff), tol));

    // backward difference
    J = ComputeNumericalGradient(
        ToyFunctionDouble, x,
        NumericalGradientOption{NumericalGradientMethod::kBackward});
    EXPECT_TRUE(CompareMatrices(J, ExtractGradient(y_autodiff), tol));

    // central difference
    // We could use the function object ToyFunction<double> instead of the
    // std::function ToyFunctionDouble, but then we have to explicitly
    // instantiate the template parameters for ComputeNumericalGradient.
    J = ComputeNumericalGradient<Eigen::Vector3d, Eigen::Vector2d,
                                 Eigen::Vector3d>(
        ToyFunction<double>, x,
        NumericalGradientOption{NumericalGradientMethod::kCentral});
    EXPECT_TRUE(CompareMatrices(J, ExtractGradient(y_autodiff), tol));
  };

  check_gradient(Eigen::Vector3d::Zero());
  check_gradient(Eigen::Vector3d(0, 1, 2));
  check_gradient(Eigen::Vector3d(0, -1, -2));
}

// Implements f(x, y) = 0.5 * kHessian * (x^2 + y^2).
const double kHessian = 5.0;
template <typename T>
void Paraboloid(const Vector2<T>& x, Vector1<T>* y) {
  (*y)(0) = 0.5 * kHessian * x.squaredNorm();
}

// This test verifies the correct implementation of ComputeNumericalGradient()
// by using it to compute the gradient of the simple Paraboloid() function
// defined above. We use automatic differentiation to compute a reference
// solution. In addition, we test we can propagate automatically computed
// gradients through ComputeNumericalGradient() when using AutoDiffd. The
// result is an approximation to the Hessian of the Paraboloid() function.
GTEST_TEST(ComputeNumericalGradientTest, TestParaboloid) {
  // We define a lambda so that we can perform a number of tests at different
  // points x in the plane.
  auto check_gradient = [](const Vector2<double>& x) {
    // I need to create a std::function, rather than passing ToyFunction<double>
    // to ComputeNumericalGradient directly, as template deduction doesn't work
    // in the implicit conversion from function object to std::function.
    std::function<void(const Vector2<double>&, Vector1<double>*)>
        ParaboloidDouble = Paraboloid<double>;
    auto J = ComputeNumericalGradient(
        ParaboloidDouble, x,
        NumericalGradientOption{NumericalGradientMethod::kForward});

    typedef AutoDiffd<2> AD2d;
    typedef Vector2<AD2d> Vector2AD2d;
    typedef Vector1<AD2d> Vector1AD2d;

    Vector2AD2d x_autodiff = math::InitializeAutoDiff<2>(x);
    Vector1AD2d y_autodiff;
    Paraboloid(x_autodiff, &y_autodiff);
    const double tol = 2.0e-7;
    EXPECT_TRUE(CompareMatrices(J, ExtractGradient(y_autodiff), tol));

    // Compute the Hessian using the autodiff version of
    // ComputeNumericalGradient().
    std::function<void(const Vector2AD2d&, Vector1AD2d*)> ParaboloidAD3d =
        Paraboloid<AD2d>;

    // With forward differencing.
    auto JAD3d = ComputeNumericalGradient(
        ParaboloidAD3d, x_autodiff,
        NumericalGradientOption{NumericalGradientMethod::kForward});
    Matrix2<double> H = ExtractGradient(JAD3d);
    Matrix2<double> H_exact = kHessian * Matrix2<double>::Identity();
    EXPECT_TRUE(CompareMatrices(H, H_exact, tol));

    // With backward differencing.
    JAD3d = ComputeNumericalGradient(
        ParaboloidAD3d, x_autodiff,
        NumericalGradientOption{NumericalGradientMethod::kBackward});
    H = ExtractGradient(JAD3d);
    H_exact = kHessian * Matrix2<double>::Identity();
    EXPECT_TRUE(CompareMatrices(H, H_exact, tol));

    // With central differencing.
    JAD3d = ComputeNumericalGradient(
        ParaboloidAD3d, x_autodiff,
        NumericalGradientOption{NumericalGradientMethod::kCentral});
    H = ExtractGradient(JAD3d);
    H_exact = kHessian * Matrix2<double>::Identity();
    // We can tighten the tolerance significantly when using central
    // differences.
    EXPECT_TRUE(CompareMatrices(H, H_exact, tol / 1000));

    // Test for symbolic::Expression, at least for constant expressions.
    typedef Vector2<symbolic::Expression> Vector2Expr;
    typedef Vector1<symbolic::Expression> Vector1Expr;
    std::function<void(const Vector2Expr&, Vector1Expr*)> ParaboloidExpr =
        Paraboloid<symbolic::Expression>;

    // With forward differencing.
    const symbolic::Expression x1(x(0));
    const symbolic::Expression x2(x(1));
    const Vector2Expr x_symbolic(x1, x2);
    auto JExpr = ComputeNumericalGradient(
        ParaboloidExpr, x_symbolic,
        NumericalGradientOption{NumericalGradientMethod::kForward});
    const RowVector2<double> JExpr_to_double(ExtractDoubleOrThrow(JExpr(0)),
                                             ExtractDoubleOrThrow(JExpr(1)));
    EXPECT_TRUE(CompareMatrices(JExpr_to_double, J, 0));
  };

  // We perform our tests on a set of arbitrary points.
  check_gradient(Eigen::Vector2d::Zero());
  check_gradient(Eigen::Vector2d(0, 1));
  check_gradient(Eigen::Vector2d(-1, -2));
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
  evaluator.Eval(math::InitializeAutoDiff(x), &y_autodiff);
  const double tol = 1E-7;
  EXPECT_TRUE(CompareMatrices(J, math::ExtractGradient(y_autodiff), tol));
}

}  // namespace
}  // namespace math
}  // namespace drake
