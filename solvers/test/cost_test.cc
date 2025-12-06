#include "drake/solvers/cost.h"

#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/evaluator_base.h"
#include "drake/solvers/test/generic_trivial_costs.h"

using std::cout;
using std::endl;
using std::make_shared;
using std::make_unique;
using std::numeric_limits;
using std::runtime_error;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

using drake::Vector1d;
using drake::solvers::test::GenericTrivialCost2;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Ref;
using Eigen::RowVector2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace drake {

using symbolic::Expression;
using symbolic::Variable;
using symbolic::test::ExprEqual;

namespace solvers {
namespace {

// For a given Constraint, return the equivalent Cost type
template <typename C>
struct related_cost {
  using type = void;
};
template <>
struct related_cost<LinearConstraint> {
  using type = LinearCost;
};
template <>
struct related_cost<QuadraticConstraint> {
  using type = QuadraticCost;
};
template <>
struct related_cost<PolynomialConstraint> {
  using type = PolynomialCost;
};

// Utility to explicitly pass a vector rather than an initializer list.
// This must be done since you cannot forward std::initializer_list's via
// parameter packs. (GCC 4.9.3 is non-standards compliant and permits this,
// but this is fixed in GCC 4.9.4.)
template <typename T>
auto make_vector(std::initializer_list<T> items) {
  return vector<std::decay_t<T>>(items);
}

GTEST_TEST(testCost, testLinearCost) {
  const double tol = numeric_limits<double>::epsilon();

  // Simple ground truth test.
  Eigen::Vector2d a(1, 2);
  const Eigen::Vector2d x0(3, 4);
  const double obj_expected = 11.;

  auto cost = make_shared<LinearCost>(a);
  Eigen::VectorXd y(1);
  cost->Eval(x0, &y);
  EXPECT_EQ(y.rows(), 1);
  EXPECT_NEAR(y(0), obj_expected, tol);

  // Test Eval with AutoDiff scalar.
  Eigen::Matrix2Xd x_grad(2, 1);
  x_grad << 5, 6;
  const AutoDiffVecXd x_autodiff = math::InitializeAutoDiff(x0, x_grad);
  AutoDiffVecXd y_autodiff;
  cost->Eval(x_autodiff, &y_autodiff);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff),
                              Vector1<double>(obj_expected), tol));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff),
                              a.transpose() * x_grad, tol));
  // Test Eval with identity gradient.
  const AutoDiffVecXd x_autodiff_identity = math::InitializeAutoDiff(x0);
  AutoDiffVecXd y_autodiff_identity;
  cost->Eval(x_autodiff_identity, &y_autodiff_identity);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff_identity),
                              Vector1<double>(obj_expected), tol));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff_identity),
                              a.transpose(), tol));
  // Test Eval with empty gradient.
  const AutoDiffVecXd x_autodiff_empty = x0.cast<AutoDiffXd>();
  AutoDiffVecXd y_autodiff_empty;
  cost->Eval(x_autodiff_empty, &y_autodiff_empty);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff_empty),
                              Vector1<double>(obj_expected), tol));
  EXPECT_EQ(math::ExtractGradient(y_autodiff_empty).size(), 0);

  // Test Eval/CheckSatisfied using Expression.
  const VectorX<Variable> x_sym{symbolic::MakeVectorContinuousVariable(2, "x")};
  VectorX<Expression> y_sym;
  cost->Eval(x_sym, &y_sym);
  EXPECT_EQ(y_sym.size(), 1);
  EXPECT_PRED2(ExprEqual, y_sym[0], 1 * x_sym[0] + 2 * x_sym[1]);

  // Update with a constant term.
  double b = 100;
  cost->UpdateCoefficients(a, b);
  cost->Eval(x0, &y);
  EXPECT_NEAR(y(0), obj_expected + b, tol);
  b = 200;
  cost->update_constant_term(b);
  cost->Eval(x0, &y);
  EXPECT_NEAR(y(0), obj_expected + b, tol);

  EXPECT_THROW(cost->UpdateCoefficients(Eigen::Vector3d::Ones(), b),
               runtime_error);

  // Update one entry in a.
  cost->update_coefficient_entry(0, 10);
  cost->Eval(x0, &y);
  EXPECT_NEAR(y(0), 10 * x0(0) + a(1) * x0(1) + b, tol);

  // Reconstruct the same cost with the constant term.
  auto new_cost = make_shared<LinearCost>(a, b);
  new_cost->Eval(x0, &y);
  EXPECT_NEAR(y(0), obj_expected + b, tol);

  new_cost->set_description("simple linear cost");
  EXPECT_EQ(
      fmt::format("{}", *new_cost),
      "LinearCost (200 + $(0) + 2 * $(1)) described as 'simple linear cost'");

  EXPECT_TRUE(cost->is_thread_safe());
}

GTEST_TEST(TestQuadraticCost, NonconvexCost) {
  const double tol = numeric_limits<double>::epsilon();

  // Simple ground truth test.
  Eigen::Matrix2d Q;
  Q << 1, 2, 3, 4;
  const Eigen::Vector2d b(5, 6);
  const Eigen::Vector2d x0(7, 8);
  const double obj_expected = 375.5;

  auto cost = make_shared<QuadraticCost>(Q, b);
  Eigen::VectorXd y(1);

  EXPECT_FALSE(cost->is_convex());

  EXPECT_TRUE(CompareMatrices(cost->Q(), (Q + Q.transpose()) / 2, 1E-10,
                              MatrixCompareType::absolute));

  cost->Eval(x0, &y);
  EXPECT_EQ(y.rows(), 1);
  EXPECT_NEAR(y(0), obj_expected, tol);

  // Test Eval with AutoDiff scalar.
  Eigen::Matrix2Xd x_grad(2, 1);
  x_grad << 5, 6;
  const AutoDiffVecXd x_autodiff = math::InitializeAutoDiff(x0, x_grad);
  AutoDiffVecXd y_autodiff;
  cost->Eval(x_autodiff, &y_autodiff);
  const AutoDiffXd y_autodiff_expected =
      0.5 * x_autodiff.dot(Q * x_autodiff) + b.dot(x_autodiff);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff),
                              Vector1<double>(obj_expected), tol));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff),
                              y_autodiff_expected.derivatives(), tol));
  // Test Eval with identity gradient.
  const AutoDiffVecXd x_autodiff_identity = math::InitializeAutoDiff(x0);
  AutoDiffVecXd y_autodiff_identity;
  cost->Eval(x_autodiff_identity, &y_autodiff_identity);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff_identity),
                              Vector1<double>(obj_expected), tol));
  EXPECT_TRUE(CompareMatrices(
      math::ExtractGradient(y_autodiff_identity),
      x0.transpose() * (Q + Q.transpose()) / 2 + b.transpose(), tol));
  // Test Eval with empty gradient.
  const AutoDiffVecXd x_autodiff_empty = x0.cast<AutoDiffXd>();
  AutoDiffVecXd y_autodiff_empty;
  cost->Eval(x_autodiff_empty, &y_autodiff_empty);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff_empty),
                              Vector1<double>(obj_expected), tol));
  EXPECT_EQ(math::ExtractGradient(y_autodiff_empty).size(), 0);

  // Test Eval/CheckSatisfied using Expression.
  {
    const VectorX<Variable> x_sym{
        symbolic::MakeVectorContinuousVariable(2, "x")};
    VectorX<Expression> y_sym;
    cost->Eval(x_sym, &y_sym);
    EXPECT_EQ(y_sym.size(), 1);
    const Variable& x_0{x_sym[0]};
    const Variable& x_1{x_sym[1]};
    EXPECT_PRED2(ExprEqual, y_sym[0].Expand(),
                 // 0.5 x'Qx + bx
                 (0.5 * (x_0 * x_0 + (2 + 3) * x_0 * x_1 + 4 * x_1 * x_1) +
                  5 * x_0 + 6 * x_1)
                     .Expand());
  }

  // Update with an asymmetric Q
  cost->UpdateCoefficients(2 * Q, b);
  EXPECT_TRUE(CompareMatrices(cost->Q(), (Q + Q.transpose()), 1E-10,
                              MatrixCompareType::absolute));
  EXPECT_FALSE(cost->is_convex());

  // Update the Hessian to make it convex.
  cost->UpdateHessianEntry(0, 1, 1, /*is_hessian_psd=*/std::nullopt);
  Eigen::Matrix2d Q_expected = (Eigen::Matrix2d() << 2, 1, 1, 8).finished();
  EXPECT_TRUE(CompareMatrices(cost->Q(), Q_expected, 1E-10));
  EXPECT_TRUE(cost->is_convex());

  // Update the Hessian to make it non-convex
  cost->UpdateHessianEntry(0, 1, 5, /*is_hessian_psd=*/false);
  Q_expected << 2, 5, 5, 8;
  EXPECT_TRUE(CompareMatrices(cost->Q(), Q_expected, 1E-10));
  EXPECT_FALSE(cost->is_convex());

  // Update the diagonal entry of the Hessian.
  cost->UpdateHessianEntry(0, 0, 1, /*is_hessian_psd=*/std::nullopt);
  Q_expected << 1, 5, 5, 8;
  EXPECT_TRUE(CompareMatrices(cost->Q(), Q_expected, 1E-10));
  EXPECT_FALSE(cost->is_convex());

  // Update an entry in the linear coefficient.
  cost->update_linear_coefficient_entry(0, 10);
  cost->update_linear_coefficient_entry(1, 3);
  EXPECT_TRUE(CompareMatrices(cost->b(), Eigen::Vector2d(10, 3)));

  // Update with a constant term.
  const double c = 100;
  cost->UpdateCoefficients(Q, b, c);
  cost->Eval(x0, &y);
  EXPECT_NEAR(y(0), obj_expected + c, tol);
  cost->update_constant_term(200);
  EXPECT_EQ(cost->c(), 200);

  EXPECT_THROW(cost->UpdateCoefficients(Eigen::Matrix3d::Identity(), b, c),
               runtime_error);
  EXPECT_THROW(cost->UpdateCoefficients(Q, Eigen::Vector3d::Ones(), c),
               runtime_error);

  // Reconstruct the same cost with the constant term.
  auto new_cost = make_shared<QuadraticCost>(Q, b, c);
  new_cost->Eval(x0, &y);
  EXPECT_NEAR(y(0), obj_expected + c, tol);

  // Now construct cost with is_hessian_psd=false.
  cost = std::make_shared<QuadraticCost>(Q, b, 0., false);
  EXPECT_FALSE(cost->is_convex());

  // Now update the Hessian such that it is PSD.
  // UpdateCoefficients function will check if the new Hessian is PSD.
  cost->UpdateCoefficients(Q + 100 * Eigen::Matrix2d::Identity(),
                           Eigen::Vector2d::Zero());
  EXPECT_TRUE(cost->is_convex());
  // By specifying is_hessian_psd=true, UpdateCoefficients() function will
  // bypass checking if the new Hessian is PSD.
  cost->UpdateCoefficients(Q + 100 * Eigen::Matrix2d::Identity(),
                           Eigen::Vector2d::Zero(), 0., true);
  EXPECT_TRUE(cost->is_convex());

  // By specifying is_hessian_psd=false, UpdateCoefficients() function will
  // bypass checking if the new Hessian is PSD. Although the new Hessian is
  // truly PSD, our code takes is_hessian_psd at a face value, and skip the PSD
  // check entirely. The user should never lie about is_hessian_psd flag, we do
  // it here just to check if the code behaves as we expect when that lie
  // occurs.
  cost->UpdateCoefficients(Q + 100 * Eigen::Matrix2d::Identity(),
                           Eigen::Vector2d::Zero(), 0., false);
  EXPECT_FALSE(cost->is_convex());

  EXPECT_TRUE(cost->is_thread_safe());
}

GTEST_TEST(TestQuadraticCost, ConvexCost) {
  Eigen::Matrix2d Q;
  Q << 1, 2, 3, 10;
  const Eigen::Vector2d b(5, 6);
  auto cost = std::make_shared<QuadraticCost>(Q, b, 0.5);
  EXPECT_TRUE(cost->is_convex());

  cost = std::make_shared<QuadraticCost>(Q, b, 0.5, true);
  EXPECT_TRUE(cost->is_convex());

  // Call UpdateCoefficients(). This function determines if the new Hessian is
  // psd.
  cost->UpdateCoefficients(-Q, b, 0.1);
  EXPECT_FALSE(cost->is_convex());

  // Call UpdateCoefficients() and tell the function that the new Hessian is not
  // psd.
  cost->UpdateCoefficients(-Q, b, 0.1, false);
  EXPECT_FALSE(cost->is_convex());

  // Call UpdateCoefficients() and tell the function that the new Hessian is
  // psd by setting is_hessian_psd=true. Although the actual Hessian is not psd,
  // our function will take the face value of is_hessian_psd and bypass the
  // matrix psd check.
  cost->UpdateCoefficients(-Q, b, 0.1, true);
  EXPECT_TRUE(cost->is_convex());

  // Call Make2NormSquaredCost.
  cost = Make2NormSquaredCost((Eigen::Matrix2d() << 1, 2, 3, 4).finished(),
                              Eigen::Vector2d(2, 3));
  EXPECT_TRUE(cost->is_convex());

  EXPECT_TRUE(cost->is_thread_safe());
}

// TODO(eric.cousineau): Move QuadraticErrorCost and L2NormCost tests here from
// MathematicalProgram.

template <typename C, typename BoundType, typename... Args>
void VerifyRelatedCost(const Ref<const VectorXd>& x_value, Args&&... args) {
  // Ensure that a constraint constructed in a particular fashion yields
  // equivalent results to its shim, and the related cost.
  const double inf = std::numeric_limits<double>::infinity();
  BoundType lb = -BoundType(-inf);
  BoundType ub = BoundType(inf);
  C constraint(std::forward<Args>(args)..., lb, ub);
  typename related_cost<C>::type cost(std::forward<Args>(args)...);
  VectorXd y_expected, y;
  constraint.Eval(x_value, &y);
  cost.Eval(x_value, &y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

GTEST_TEST(testCost, testCostShim) {
  // Test CostShim's by means of the related constraints.

  VerifyRelatedCost<LinearConstraint, Vector1d>(Vector1d(2), Vector1d(3));

  VerifyRelatedCost<QuadraticConstraint, double>(Vector1d(2), Vector1d(3),
                                                 Vector1d(4));

  const Polynomiald x("x");
  const auto poly = (x - 1) * (x - 1);
  const auto var_mapping = make_vector({x.GetSimpleVariable()});
  VerifyRelatedCost<PolynomialConstraint, Vector1d>(
      Vector1d(2), VectorXPoly::Constant(1, poly), var_mapping);
}

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

// Verifies that FunctionCost form can be constructed correctly.
template <typename F>
void VerifyFunctionCost(F&& f, const Ref<const VectorXd>& x_value) {
  // Compute expected value prior to forwarding `f` (which may involve
  // move'ing `unique_ptr<>` or `shared_ptr<>`, making `f` a nullptr).
  Eigen::VectorXd y_expected(1);
  deref(f).eval(x_value, &y_expected);
  // Construct cost, moving `f`, if applicable.
  auto cost = MakeFunctionCost(std::forward<F>(f));
  EXPECT_TRUE(is_dynamic_castable<Cost>(cost));
  // Compare values.
  Eigen::VectorXd y(1);
  cost->Eval(x_value, &y);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

GTEST_TEST(testCost, testFunctionCost) {
  // Test that we can construct FunctionCosts with different signatures.
  Eigen::Vector2d x(1, 2);
  VerifyFunctionCost(GenericTrivialCost2(), x);
  // Ensure that we explicitly call the default constructor for a const class.
  // @ref http://stackoverflow.com/a/28338123/7829525
  const GenericTrivialCost2 obj_const{};
  VerifyFunctionCost(obj_const, x);
  VerifyFunctionCost(make_shared<GenericTrivialCost2>(), x);
  VerifyFunctionCost(make_unique<GenericTrivialCost2>(), x);
}

GTEST_TEST(TestL1NormCost, Eval) {
  Matrix<double, 2, 4> A;
  // clang-format off
  A << .32,  2.0, 1.3, -4.,
       2.3, -2.0, 7.1, 1.3;
  // clang-format on
  const Vector2d b{.42, -3.2};

  L1NormCost cost(A, b);
  EXPECT_TRUE(CompareMatrices(A, cost.A()));
  EXPECT_TRUE(CompareMatrices(b, cost.b()));

  const Vector4d x0{5.2, 3.4, -1.3, 2.1};
  const Vector2d z = A * x0 + b;

  // Test double.
  {
    VectorXd y;
    cost.Eval(x0, &y);
    EXPECT_NEAR(z.cwiseAbs().sum(), y[0], 1e-15);
  }

  // Test AutoDiffXd.
  {
    const Vector4<AutoDiffXd> x = math::InitializeAutoDiff(x0);
    VectorX<AutoDiffXd> y;
    cost.Eval(x, &y);
    EXPECT_NEAR(z.cwiseAbs().sum(), math::ExtractValue(y)[0], 1e-15);
    const Matrix<double, 1, 4> grad_expected =
        z.cwiseQuotient(z.cwiseAbs()).transpose() * A;
    EXPECT_TRUE(
        CompareMatrices(math::ExtractGradient(y), grad_expected, 1e-15));
  }

  // Test Symbolic.
  {
    auto x = symbolic::MakeVectorVariable(4, "x");
    VectorX<Expression> y;
    cost.Eval(x, &y);
    symbolic::Environment env;
    env.insert(x, x0);
    EXPECT_NEAR(z.cwiseAbs().sum(), y[0].Evaluate(env), 1e-14);
  }
}

GTEST_TEST(TestL1NormCost, UpdateCoefficients) {
  L1NormCost cost(Matrix2d::Identity(), Vector2d::Zero());

  cost.UpdateCoefficients(Matrix<double, 4, 2>::Identity(), Vector4d::Zero());
  EXPECT_EQ(cost.A().rows(), 4);
  EXPECT_EQ(cost.b().rows(), 4);

  // Can't change the number of variables.
  EXPECT_THROW(cost.UpdateCoefficients(Matrix3d::Identity(), Vector3d::Zero()),
               std::exception);

  // A and b must have the same number of rows.
  EXPECT_THROW(cost.UpdateCoefficients(Matrix3d::Identity(), Vector4d::Zero()),
               std::exception);

  cost.update_A_entry(0, 1, 0.5);
  EXPECT_TRUE(CompareMatrices(
      cost.A(),
      (Eigen::Matrix<double, 4, 2>() << 1, 0.5, 0, 1, 0, 0, 0, 0).finished()));

  cost.update_b_entry(0, 1.5);
  EXPECT_TRUE(CompareMatrices(cost.b(), Eigen::Vector4d(1.5, 0, 0, 0)));
}

GTEST_TEST(TestL1NormCost, Display) {
  L1NormCost cost(Matrix2d::Identity(), Vector2d::Ones());
  std::ostringstream os;
  cost.Display(os, symbolic::MakeVectorContinuousVariable(2, "x"));
  EXPECT_EQ(fmt::format("{}", os.str()),
            "L1NormCost (abs((1 + x(0))) + abs((1 + x(1))))");
}

GTEST_TEST(TestL2NormCost, Eval) {
  Matrix<double, 2, 4> A;
  // clang-format off
  A << .32,  2.0, 1.3, -4.,
       2.3, -2.0, 7.1, 1.3;
  // clang-format on
  const Vector2d b{.42, -3.2};

  L2NormCost cost(A, b);
  EXPECT_TRUE(CompareMatrices(A, cost.GetDenseA()));
  EXPECT_TRUE(CompareMatrices(A, cost.get_sparse_A().toDense()));
  EXPECT_TRUE(CompareMatrices(b, cost.b()));

  const Vector4d x0{5.2, 3.4, -1.3, 2.1};
  const Vector2d z = A * x0 + b;

  // Test double.
  {
    VectorXd y;
    cost.Eval(x0, &y);
    EXPECT_NEAR(std::sqrt(z.dot(z)), y[0], 1e-16);
  }

  // Test AutoDiffXd.
  {
    const Vector4<AutoDiffXd> x = math::InitializeAutoDiff(x0);
    VectorX<AutoDiffXd> y;
    cost.Eval(x, &y);
    EXPECT_NEAR(z.norm(), math::ExtractValue(y)[0], 1e-15);
    const Matrix<double, 1, 4> grad_expected =
        (x0.transpose() * A.transpose() * A + b.transpose() * A) / (z.norm());
    EXPECT_TRUE(
        CompareMatrices(math::ExtractGradient(y), grad_expected, 1e-14));
  }

  // Test Symbolic.
  {
    auto x = symbolic::MakeVectorVariable(4, "x");
    VectorX<Expression> y;
    cost.Eval(x, &y);
    symbolic::Environment env;
    env.insert(x, x0);
    EXPECT_NEAR(z.norm(), y[0].Evaluate(env), 1e-14);
  }

  {
    // Test constructor with sparse A.
    L2NormCost cost_sparse(A.sparseView(), b);
    EXPECT_TRUE(CompareMatrices(cost_sparse.GetDenseA(), A));
    EXPECT_TRUE(CompareMatrices(cost_sparse.get_sparse_A().toDense(), A));
    EXPECT_TRUE(CompareMatrices(cost_sparse.b(), b));
  }
}

GTEST_TEST(TestL2NormCost, UpdateCoefficients) {
  L2NormCost cost(Matrix2d::Identity(), Vector2d::Zero());

  Matrix<double, 4, 2> new_A = Matrix<double, 4, 2>::Identity();
  // Call UpdateCoefficients with a dense A.
  cost.UpdateCoefficients(new_A, Vector4d::Zero());
  EXPECT_TRUE(CompareMatrices(cost.get_sparse_A().toDense(), new_A));
  EXPECT_TRUE(CompareMatrices(cost.GetDenseA(), new_A));
  EXPECT_EQ(cost.b().rows(), 4);

  // Call UpdateCoefficients with a sparse A.
  new_A << 1, 2, 3, 0, 1, 3, 5, 0;
  cost.UpdateCoefficients(new_A.sparseView(), Vector4d::Zero());
  EXPECT_TRUE(CompareMatrices(cost.get_sparse_A().toDense(), new_A));
  EXPECT_TRUE(CompareMatrices(cost.GetDenseA(), new_A));
  EXPECT_EQ(cost.b().rows(), 4);

  // Can't change the number of variables.
  EXPECT_THROW(cost.UpdateCoefficients(Matrix3d::Identity(), Vector3d::Zero()),
               std::exception);

  // A and b must have the same number of rows.
  EXPECT_THROW(cost.UpdateCoefficients(Matrix3d::Identity(), Vector4d::Zero()),
               std::exception);
}

GTEST_TEST(TestL2NormCost, Display) {
  L2NormCost cost(Matrix2d::Identity(), Vector2d::Ones());
  std::ostringstream os;
  cost.Display(os, symbolic::MakeVectorContinuousVariable(2, "x"));
  EXPECT_EQ(fmt::format("{}", os.str()),
            "L2NormCost sqrt((pow((1 + x(0)), 2) + pow((1 + x(1)), 2)))");
}

GTEST_TEST(TestLInfNormCost, Eval) {
  Matrix<double, 2, 4> A;
  // clang-format off
  A << .32,  2.0, 1.3, -4.,
       2.3, -2.0, 7.1, 1.3;
  // clang-format on
  const Vector2d b{.42, -3.2};

  LInfNormCost cost(A, b);
  EXPECT_TRUE(CompareMatrices(A, cost.A()));
  EXPECT_TRUE(CompareMatrices(b, cost.b()));

  const Vector4d x0{5.2, 3.4, -1.3, 2.1};
  const Vector2d z = A * x0 + b;

  // Test double.
  {
    VectorXd y;
    cost.Eval(x0, &y);
    EXPECT_NEAR(z.cwiseAbs().maxCoeff(), y[0], 1e-16);
  }

  // Test AutoDiffXd.
  {
    const Vector4<AutoDiffXd> x = math::InitializeAutoDiff(x0);
    VectorX<AutoDiffXd> y;
    cost.Eval(x, &y);
    int max_row;
    EXPECT_NEAR(z.cwiseAbs().maxCoeff(&max_row), math::ExtractValue(y)[0],
                1e-15);
    const Matrix<double, 1, 4> grad_expected =
        (z.cwiseQuotient(z.cwiseAbs()))(max_row)*A.row(max_row);
    EXPECT_TRUE(
        CompareMatrices(math::ExtractGradient(y), grad_expected, 1e-15));
  }

  // Test Symbolic.
  {
    auto x = symbolic::MakeVectorVariable(4, "x");
    VectorX<Expression> y;
    cost.Eval(x, &y);
    symbolic::Environment env;
    env.insert(x, x0);
    EXPECT_NEAR(z.cwiseAbs().maxCoeff(), y[0].Evaluate(env), 1e-14);
  }
}

GTEST_TEST(TestLInfNormCost, UpdateCoefficients) {
  LInfNormCost cost(Matrix2d::Identity(), Vector2d::Zero());

  cost.UpdateCoefficients(Matrix<double, 4, 2>::Identity(), Vector4d::Zero());
  EXPECT_EQ(cost.A().rows(), 4);
  EXPECT_EQ(cost.b().rows(), 4);

  // Can't change the number of variables.
  EXPECT_THROW(cost.UpdateCoefficients(Matrix3d::Identity(), Vector3d::Zero()),
               std::exception);

  // A and b must have the same number of rows.
  EXPECT_THROW(cost.UpdateCoefficients(Matrix3d::Identity(), Vector4d::Zero()),
               std::exception);

  cost.update_A_entry(0, 1, 0.5);
  EXPECT_TRUE(CompareMatrices(
      cost.A(),
      (Eigen::Matrix<double, 4, 2>() << 1, 0.5, 0, 1, 0, 0, 0, 0).finished()));

  cost.update_b_entry(0, 1.5);
  EXPECT_TRUE(CompareMatrices(cost.b(), Eigen::Vector4d(1.5, 0, 0, 0)));
}

GTEST_TEST(TestLInfNormCost, Display) {
  LInfNormCost cost(Matrix2d::Identity(), Vector2d::Ones());
  std::ostringstream os;
  cost.Display(os, symbolic::MakeVectorContinuousVariable(2, "x"));
  EXPECT_EQ(fmt::format("{}", os.str()),
            "LInfNormCost max(abs((1 + x(0))), abs((1 + x(1))))");
}

GTEST_TEST(TestPerspectiveQuadraticCost, Eval) {
  Matrix<double, 2, 4> A;
  // clang-format off
  A << .32,  2.0, 1.3, -4.,
       2.3, -2.0, 7.1, 1.3;
  // clang-format on
  const Vector2d b{.42, -3.2};

  PerspectiveQuadraticCost cost(A, b);
  EXPECT_TRUE(CompareMatrices(A, cost.A()));
  EXPECT_TRUE(CompareMatrices(b, cost.b()));

  const Vector4d x0{5.2, 3.4, -1.3, 2.1};
  const Vector2d z = A * x0 + b;

  // Test double.
  {
    VectorXd y;
    cost.Eval(x0, &y);
    EXPECT_DOUBLE_EQ(z(1) * z(1) / z(0), y[0]);
  }

  // Test AutoDiffXd.
  {
    const Vector4<AutoDiffXd> x = math::InitializeAutoDiff(x0);
    VectorX<AutoDiffXd> y;
    cost.Eval(x, &y);
    EXPECT_DOUBLE_EQ(z(1) * z(1) / z(0), math::ExtractValue(y)[0]);
    const Matrix<double, 1, 4> grad_expected =
        RowVector2d(-(z(1) * z(1)) / (z(0) * z(0)), 2 * z(1) / z(0)) * A;
    EXPECT_TRUE(
        CompareMatrices(math::ExtractGradient(y), grad_expected, 1e-13));
  }

  // Test Symbolic.
  {
    auto x = symbolic::MakeVectorVariable(4, "x");
    VectorX<Expression> y;
    cost.Eval(x, &y);
    symbolic::Environment env;
    env.insert(x, x0);
    EXPECT_DOUBLE_EQ(z(1) * z(1) / z(0), y[0].Evaluate(env));
  }
}

GTEST_TEST(TestPerspectiveQuadraticCost, UpdateCoefficients) {
  PerspectiveQuadraticCost cost(Matrix2d::Identity(), Vector2d::Zero());

  cost.UpdateCoefficients(Matrix<double, 4, 2>::Identity(), Vector4d::Zero());
  EXPECT_EQ(cost.A().rows(), 4);
  EXPECT_EQ(cost.b().rows(), 4);

  // Can't change the number of variables.
  EXPECT_THROW(cost.UpdateCoefficients(Matrix3d::Identity(), Vector3d::Zero()),
               std::exception);

  // A and b must have the same number of rows.
  EXPECT_THROW(cost.UpdateCoefficients(Matrix3d::Identity(), Vector4d::Zero()),
               std::exception);

  cost.update_A_entry(0, 1, 0.5);
  EXPECT_TRUE(CompareMatrices(
      cost.A(),
      (Eigen::Matrix<double, 4, 2>() << 1, 0.5, 0, 1, 0, 0, 0, 0).finished()));

  cost.update_b_entry(0, 1.5);
  EXPECT_TRUE(CompareMatrices(cost.b(), Eigen::Vector4d(1.5, 0, 0, 0)));
}

GTEST_TEST(TestPerspectiveQuadraticCost, Display) {
  PerspectiveQuadraticCost cost(Matrix2d::Identity(), Vector2d::Ones());
  std::ostringstream os;
  cost.Display(os, symbolic::MakeVectorContinuousVariable(2, "x"));
  EXPECT_EQ(fmt::format("{}", os.str()),
            "PerspectiveQuadraticCost (pow((1 + x(1)), 2) / (1 + x(0)))");
}

class Evaluator2In1Out : public EvaluatorBase {
 public:
  Evaluator2In1Out() : EvaluatorBase(1, 2) {}

  ~Evaluator2In1Out() override {}

 private:
  template <typename T, typename S>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<S>* y) const {
    y->resize(1);
    using std::sin;
    (*y)(0) = x(0) + sin(x(1));
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              VectorXd* y) const override {
    this->DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    this->DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    this->DoEvalGeneric(x, y);
  }
};

class Evaluator3In2Out : public EvaluatorBase {
 public:
  Evaluator3In2Out() : EvaluatorBase(3, 2) {}

  ~Evaluator3In2Out() override {}

 private:
  template <typename T, typename S>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<S>* y) const {
    y->resize(3);
    using std::sin;
    (*y)(0) = x(0) + 3 * x(1) * x(0);
    (*y)(1) = sin(x(0) + x(1));
    (*y)(2) = 2 + x(0);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              VectorXd* y) const override {
    this->DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    this->DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    this->DoEvalGeneric(x, y);
  }
};

GTEST_TEST(EvaluatorCost, Eval) {
  // Test with a single-output evaluator.
  auto evaluator_2in_1out = std::make_shared<Evaluator2In1Out>();
  EvaluatorCost<Evaluator2In1Out> dut1(evaluator_2in_1out);
  Eigen::Vector2d x1(0.2, 0.3);
  VectorXd y1;
  dut1.Eval(x1, &y1);
  Eigen::VectorXd y1_expected;
  evaluator_2in_1out->Eval(x1, &y1_expected);
  EXPECT_TRUE(CompareMatrices(y1, y1_expected));

  // Test a linear transformation of the evaluator output.
  auto evaluator_3in_2out = std::make_shared<Evaluator3In2Out>();
  const Eigen::Vector3d a(1, 2, 3);
  const double b = 4;
  EvaluatorCost<Evaluator3In2Out> dut2(evaluator_3in_2out, a, b);
  const Eigen::Vector2d x2(2, 3);
  Eigen::VectorXd y2;
  dut2.Eval(x2, &y2);
  Eigen::VectorXd evaluator2_y;
  evaluator_3in_2out->Eval(x2, &evaluator2_y);
  ASSERT_EQ(y2.rows(), 1);
  EXPECT_NEAR(y2(0), a.dot(evaluator2_y) + b, 1E-12);
}

GTEST_TEST(ExpressionCost, Basic) {
  using std::cos;
  using std::sin;
  Variable x("x"), y("y");
  symbolic::Expression e = x * sin(y);
  ExpressionCost cost(e);

  EXPECT_TRUE(e.EqualTo(cost.expression()));
  EXPECT_EQ(cost.vars().size(), 2);
  EXPECT_EQ(cost.vars()[0], x);
  EXPECT_EQ(cost.vars()[1], y);

  EXPECT_EQ(cost.num_vars(), 2);
  Vector2d x_d(1.2, 3.5);
  VectorXd y_d;
  Vector1d y_expected(1.2 * sin(3.5));
  cost.Eval(x_d, &y_d);
  EXPECT_TRUE(CompareMatrices(y_d, y_expected));

  AutoDiffVecXd x_ad = math::InitializeAutoDiff(x_d);
  AutoDiffVecXd y_ad;
  RowVector2d y_deriv_expected(sin(3.5), 1.2 * cos(3.5));
  cost.Eval(x_ad, &y_ad);
  EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_ad), y_expected));
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_ad), y_deriv_expected));

  Variable x_test("x"), y_test("y");
  Vector2<Variable> x_e(x_test, y_test);
  VectorX<Expression> y_e;
  Expression e_expected = x_test * sin(y_test);
  cost.Eval(x_e, &y_e);
  EXPECT_EQ(y_e.size(), 1);
  EXPECT_TRUE(y_e[0].EqualTo(e_expected));

  std::ostringstream os;
  cost.Display(os, x_e);
  EXPECT_EQ(os.str(), "ExpressionCost (x * sin(y))");
}

GTEST_TEST(ToLatex, GenericCost) {
  test::GenericTrivialCost1 c{false};
  c.set_description("test");
  Vector3<Variable> vars = symbolic::MakeVectorVariable<3>("x");
  EXPECT_EQ(c.ToLatex(vars),
            "\\text{GenericTrivialCost1}(x_{0}, x_{1}, x_{2}) \\tag{test}");
  EXPECT_FALSE(c.is_thread_safe());
}

GTEST_TEST(ToLatex, LinearCost) {
  LinearCost c(Vector3d(1, 2, 3), 4);
  c.set_description("test");
  Vector3<Variable> vars = symbolic::MakeVectorVariable<3>("x");
  EXPECT_EQ(c.ToLatex(vars), "(4 + x_{0} + 2x_{1} + 3x_{2}) \\tag{test}");
}

GTEST_TEST(ToLatex, QuadraticCost) {
  QuadraticCost c(Matrix3d::Identity(), Vector3d(1, 2, 3), 4);
  c.set_description("test");
  Vector3<Variable> vars = symbolic::MakeVectorVariable<3>("x");
  EXPECT_EQ(c.ToLatex(vars),
            "(4 + x_{0} + 2x_{1} + 3x_{2} + 0.500x_{0}^{2} + 0.500x_{1}^{2} + "
            "0.500x_{2}^{2}) \\tag{test}");
}

GTEST_TEST(ToLatex, L1NormCost) {
  L1NormCost c(Matrix3d::Identity(), Vector3d(1, 2, 3));
  c.set_description("test");
  Vector3<Variable> vars = symbolic::MakeVectorVariable<3>("x");
  EXPECT_EQ(c.ToLatex(vars),
            "\\left|\\begin{bmatrix} (1 + x_{0}) \\\\ (2 + x_{1}) \\\\ (3 + "
            "x_{2}) \\end{bmatrix}\\right|_1 \\tag{test}");
}

GTEST_TEST(ToLatex, L2NormCost) {
  L2NormCost c(Matrix3d::Identity(), Vector3d(1, 2, 3));
  c.set_description("test");
  Vector3<Variable> vars = symbolic::MakeVectorVariable<3>("x");
  EXPECT_EQ(c.ToLatex(vars),
            "\\left|\\begin{bmatrix} (1 + x_{0}) \\\\ (2 + x_{1}) \\\\ (3 + "
            "x_{2}) \\end{bmatrix}\\right|_2 \\tag{test}");
}

GTEST_TEST(ToLatex, LInfNormCost) {
  LInfNormCost c(Matrix3d::Identity(), Vector3d(1, 2, 3));
  c.set_description("test");
  Vector3<Variable> vars = symbolic::MakeVectorVariable<3>("x");
  EXPECT_EQ(c.ToLatex(vars),
            "\\left|\\begin{bmatrix} (1 + x_{0}) \\\\ (2 + x_{1}) \\\\ (3 + "
            "x_{2}) \\end{bmatrix}\\right|_\\infty \\tag{test}");
}

GTEST_TEST(ToLatex, PerspectiveQuadraticCost) {
  PerspectiveQuadraticCost c(Matrix3d::Identity(), Vector3d(1, 2, 3));
  c.set_description("test");
  Vector3<Variable> vars = symbolic::MakeVectorVariable<3>("x");
  EXPECT_EQ(
      c.ToLatex(vars),
      "\\frac{((2 + x_{1})^{2} + (3 + x_{2})^{2})}{(1 + x_{0})} \\tag{test}");
}

GTEST_TEST(ToLatex, ExpressionCost) {
  Variable x("x"), y("y");
  Expression e = x * sin(y);
  ExpressionCost c(e);
  c.set_description("test");
  Vector2<Variable> vars(x, y);
  EXPECT_EQ(c.ToLatex(vars), "x \\sin{y} \\tag{test}");
}

GTEST_TEST(IsThreadSafe, GenericCost) {
  test::GenericTrivialCost1 c1{true};
  EXPECT_TRUE(c1.is_thread_safe());
  test::GenericTrivialCost1 c2{false};
  EXPECT_FALSE(c2.is_thread_safe());
}

GTEST_TEST(IsThreadSafe, LinearCost) {
  LinearCost c(Vector3d(1, 2, 3), 4);
  EXPECT_TRUE(c.is_thread_safe());
}

GTEST_TEST(IsThreadSafe, QuadraticCost) {
  QuadraticCost c(Matrix3d::Identity(), Vector3d(1, 2, 3), 4);
  EXPECT_TRUE(c.is_thread_safe());
}

GTEST_TEST(IsThreadSafe, L1NormCost) {
  L1NormCost c(Matrix3d::Identity(), Vector3d(1, 2, 3));
  EXPECT_TRUE(c.is_thread_safe());
}

GTEST_TEST(IsThreadSafe, L2NormCost) {
  L2NormCost c(Matrix3d::Identity(), Vector3d(1, 2, 3));
  EXPECT_TRUE(c.is_thread_safe());
}

GTEST_TEST(IsThreadSafe, LInfNormCost) {
  LInfNormCost c(Matrix3d::Identity(), Vector3d(1, 2, 3));
  EXPECT_TRUE(c.is_thread_safe());
}

GTEST_TEST(IsThreadSafe, PerspectiveQuadraticCost) {
  PerspectiveQuadraticCost c(Matrix3d::Identity(), Vector3d(1, 2, 3));
  EXPECT_TRUE(c.is_thread_safe());
}

GTEST_TEST(IsThreadSafe, ExpressionCost) {
  Variable x("x"), y("y");
  Expression e = x * sin(y);
  ExpressionCost c(e);
  EXPECT_FALSE(c.is_thread_safe());
}

}  // namespace
}  // namespace solvers
}  // namespace drake
