#include "drake/solvers/cost.h"

#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/create_cost.h"
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

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Ref;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using drake::solvers::test::GenericTrivialCost2;

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
  const AutoDiffVecXd x_autodiff =
      math::InitializeAutoDiff(x0, x_grad);
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
  const double b = 100;
  cost->UpdateCoefficients(a, b);
  cost->Eval(x0, &y);
  EXPECT_NEAR(y(0), obj_expected + b, tol);
  EXPECT_THROW(cost->UpdateCoefficients(Eigen::Vector3d::Ones(), b),
               runtime_error);

  // Reconstruct the same cost with the constant term.
  auto new_cost = make_shared<LinearCost>(a, b);
  new_cost->Eval(x0, &y);
  EXPECT_NEAR(y(0), obj_expected + b, tol);

  new_cost->set_description("simple linear cost");
  EXPECT_EQ(
      fmt::format("{}", *new_cost),
      "LinearCost (100 + $(0) + 2 * $(1)) described as 'simple linear cost'");
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
  const AutoDiffVecXd x_autodiff =
      math::InitializeAutoDiff(x0, x_grad);
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

  // Update with a constant term.
  const double c = 100;
  cost->UpdateCoefficients(Q, b, c);
  cost->Eval(x0, &y);
  EXPECT_NEAR(y(0), obj_expected + c, tol);

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

GTEST_TEST(TestL2NormCost, Eval) {
  Matrix<double, 2, 4> A;
  // clang-format off
  A << .32,  2.0, 1.3, -4.,
       2.3, -2.0, 7.1, 1.3;
  // clang-format on
  const Vector2d b{.42, -3.2};

  L2NormCost cost(A, b);
  EXPECT_TRUE(CompareMatrices(A, cost.A()));
  EXPECT_TRUE(CompareMatrices(b, cost.b()));

  const Vector4d x0{5.2, 3.4, -1.3, 2.1};
  const Vector2d z = A*x0 + b;

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
    EXPECT_NEAR(z.norm(), math::ExtractValue(y)[0], 1e-16);
    const Matrix<double, 1, 4> grad_expected =
        (x0.transpose() * A.transpose() * A + b.transpose() * A) / (z.norm());
    EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y),
                                grad_expected, 1e-15));
  }

  // Test Symbolic.
  {
    auto x = symbolic::MakeVectorVariable(4, "x");
    VectorX<Expression> y;
    cost.Eval(x, &y);
    symbolic::Environment env;
    env.insert(x, x0);
    EXPECT_NEAR(z.norm(), y[0].Evaluate(env), 1e-15);
  }
}

GTEST_TEST(TestL2NormCost, UpdateCoefficients) {
  L2NormCost cost(Matrix2d::Identity(), Vector2d::Zero());

  cost.UpdateCoefficients(Matrix<double, 4, 2>::Identity(), Vector4d::Zero());
  EXPECT_EQ(cost.A().rows(), 4);
  EXPECT_EQ(cost.b().rows(), 4);

  // Can't change the number of variables.
  EXPECT_THROW(
      cost.UpdateCoefficients(Matrix3d::Identity(), Vector3d::Zero()),
      std::exception);

  // A and b must have the same number of rows.
  EXPECT_THROW(
      cost.UpdateCoefficients(Matrix3d::Identity(), Vector4d::Zero()),
      std::exception);
}

GTEST_TEST(TestL2NormCost, Display) {
  L2NormCost cost(Matrix2d::Identity(), Vector2d::Ones());
  std::ostringstream os;
  cost.Display(os, symbolic::MakeVectorContinuousVariable(2, "x"));
  EXPECT_EQ(fmt::format("{}", os.str()),
            "L2NormCost sqrt((pow((1 + x(0)), 2) + pow((1 + x(1)), 2)))");
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
