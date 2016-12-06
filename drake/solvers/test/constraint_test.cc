#include "drake/solvers/constraint.h"

#include <type_traits>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace solvers {
namespace {
// This function is used to return a temporary object, so that we
// can test move constructor.
template <typename T>
T pipe(T constraint) {
  return constraint;
}

// Tests if the copied and moved constraints behaves the same as the original
// constraint.
// is_copyable is true if the constraint can be constructed using T(const T&
// rhs).
// is_movable is true if the constraint can be constructed using T(T&& rhs).
// is_copy_assignable is true if T can be copy-assigned from an lvalue
// expression.
// is_move_assignable is true if T can be move-assigned from an rvalue
// expression.
template <typename T>
void TestMovableCopyableAssignableFun(const T& constraint,
                                      const Eigen::Ref<const Eigen::VectorXd> x,
                                      bool is_copyable, bool is_movable,
                                      bool is_copy_assignable,
                                      bool is_move_assignable) {
  static_assert(std::is_base_of<Constraint, T>::value,
                "T should be a Constraint type");
  std::vector<T> constraints;
  if (is_copyable) {
    EXPECT_TRUE(std::is_copy_constructible<T>::value);
    T constraint_copied(constraint);
    constraints.push_back(constraint_copied);
  }
  if (is_movable) {
    EXPECT_TRUE(std::is_move_constructible<T>::value);
    T constraint_moved(pipe(constraint));
    constraints.push_back(constraint_moved);
  }

  if (is_copy_assignable) {
    EXPECT_TRUE(std::is_copy_assignable<T>::value);
    T constraint_assigned_copy = constraint;
    constraints.push_back(constraint_assigned_copy);
  }

  if (is_move_assignable) {
    EXPECT_TRUE(std::is_move_assignable<T>::value);
    T constraint_assigned_move = pipe(constraint);
    constraints.push_back(constraint_assigned_move);
  }

  Eigen::VectorXd y_expected;
  constraint.Eval(x, y_expected);

  for (const auto& con : constraints) {
    EXPECT_EQ(con.num_constraints(), constraint.num_constraints());
    EXPECT_TRUE(CompareMatrices(con.lower_bound(), constraint.lower_bound()));
    EXPECT_TRUE(CompareMatrices(con.upper_bound(), constraint.upper_bound()));
    EXPECT_EQ(con.get_description(), constraint.get_description());
    Eigen::VectorXd y;
    con.Eval(x, y);
    EXPECT_TRUE(CompareMatrices(y, y_expected));
  }
}

GTEST_TEST(TestConstraint, TestMovableCopyable) {
  QuadraticConstraint quadratic_constraint(Eigen::Matrix2d::Identity(),
                                           Eigen::Vector2d::Ones(), -1, 1);
  TestMovableCopyableAssignableFun(
      quadratic_constraint, Eigen::Vector2d(1.2, 0.2), true, true, true, true);
  LorentzConeConstraint lorentz_cone_constraint;
  TestMovableCopyableAssignableFun(lorentz_cone_constraint,
                                   Eigen::Vector3d(1.2, 0.3, 0.3), true, true,
                                   true, true);
  RotatedLorentzConeConstraint rotated_lorentz_cone_constraint;
  TestMovableCopyableAssignableFun(rotated_lorentz_cone_constraint,
                                   Eigen::Vector4d(2.0, 3.0, 1.2, 0.3), true,
                                   true, true, true);
  Polynomiald x("x");
  std::vector<Polynomiald::VarType> var_mapping = {x.GetSimpleVariable()};
  PolynomialConstraint polynomial_constraint(VectorXPoly::Constant(1, x),
                                             var_mapping, Vector1d::Constant(2),
                                             Vector1d::Constant(2));
  TestMovableCopyableAssignableFun(polynomial_constraint, Vector1d::Constant(1),
                                   true, true, false, false);
  LinearConstraint linear_constraint(Eigen::RowVector2d(1, 2), Vector1d(0),
                                     Vector1d(1));
  TestMovableCopyableAssignableFun(linear_constraint, Eigen::Vector2d(2, 3),
                                   true, true, true, true);
  LinearEqualityConstraint linear_equality_constraint(Eigen::RowVector2d(1, 2),
                                                      Vector1d(0));
  TestMovableCopyableAssignableFun(linear_equality_constraint,
                                   Eigen::Vector2d(2, 3), true, true, true,
                                   true);
  BoundingBoxConstraint bounding_box_constraint(Eigen::Vector2d(0, 1),
                                                Eigen::Vector2d(1, 2));
  TestMovableCopyableAssignableFun(bounding_box_constraint,
                                   Eigen::Vector2d(0.5, 1.5), true, true, true,
                                   true);
  LinearComplementarityConstraint linear_complementarity_constraint(
      Eigen::Matrix2d::Identity(), Eigen::Vector2d::Ones());
  TestMovableCopyableAssignableFun(linear_complementarity_constraint,
                                   Eigen::Vector2d(2, 3), true, true, true,
                                   true);
  PositiveSemidefiniteConstraint positive_semidefinite_constraint(2);
  TestMovableCopyableAssignableFun(positive_semidefinite_constraint,
                                   Eigen::Vector4d(1, 0, 0, 1), true, true,
                                   true, true);
  LinearMatrixInequalityConstraint linear_matrix_inequality_constraint(
      {Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Ones()});
  TestMovableCopyableAssignableFun(linear_matrix_inequality_constraint,
                                   Vector1d(1), true, true, false, false);
}

// Tests if the Lorentz Cone constraint is imposed correctly.
void TestLorentzConeEval(const VectorXd& x_test, bool is_in_cone) {
  auto cnstr = LorentzConeConstraint();
  VectorXd y;
  // Test Eval with VectorXd.
  cnstr.Eval(x_test, y);
  Vector2d y_expected;
  y_expected(0) = x_test(0);
  y_expected(1) =
      x_test(0) * x_test(0) - x_test.tail(x_test.size() - 1).squaredNorm();
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected = (y(0) >= 0) & (y(1) >= 0);
  EXPECT_TRUE(is_in_cone == is_in_cone_expected);

  auto tx = drake::math::initializeAutoDiff(x_test);
  TaylorVecXd x_taylor = tx;
  TaylorVecXd y_taylor;
  // Test Eval with TaylorVar.
  cnstr.Eval(x_taylor, y_taylor);

  // Check if the gradient is correct.
  Eigen::Matrix<double, 2, Eigen::Dynamic> y_grad_expected(2, x_test.size());
  y_grad_expected.setZero();
  y_grad_expected(0, 0) = 1.0;
  y_grad_expected(1, 0) = 2 * x_test(0);
  for (int i = 1; i < x_test.size(); i++) {
    y_grad_expected(1, i) = -2 * x_test(i);
  }
  EXPECT_TRUE(CompareMatrices(y_grad_expected,
                              drake::math::autoDiffToGradientMatrix(y_taylor),
                              1E-10, MatrixCompareType::absolute));
}

void TestRotatedLorentzConeEval(const VectorXd& x_test, bool is_in_cone) {
  auto cnstr = RotatedLorentzConeConstraint();
  VectorXd y;
  cnstr.Eval(x_test, y);
  Vector3d y_expected;
  y_expected(0) = x_test(0);
  y_expected(1) = x_test(1);
  y_expected(2) =
      x_test(0) * x_test(1) - x_test.tail(x_test.size() - 2).squaredNorm();
  EXPECT_TRUE(
      CompareMatrices(y, y_expected, 1E-10, MatrixCompareType::absolute));

  bool is_in_cone_expected =
      (x_test(0) >= 0) & (x_test(1) >= 0) &
      (x_test(0) * x_test(1) >= x_test.tail(x_test.size() - 2).norm());
  EXPECT_TRUE(is_in_cone == is_in_cone_expected);

  // Eval with taylor var.
  auto tx = drake::math::initializeAutoDiff(x_test);
  TaylorVecXd x_taylor = tx;
  TaylorVecXd y_taylor;
  cnstr.Eval(x_taylor, y_taylor);

  // Check if the gradient is correct.
  Eigen::Matrix<double, 3, Eigen::Dynamic> y_grad_expected(3, x_test.size());
  y_grad_expected.setZero();
  y_grad_expected(0, 0) = 1.0;
  y_grad_expected(1, 1) = 1.0;
  y_grad_expected(2, 0) = x_test(1);
  y_grad_expected(2, 1) = x_test(0);
  for (int i = 2; i < x_test.size(); ++i) {
    y_grad_expected(2, i) = -2 * x_test(i);
  }
  EXPECT_TRUE(CompareMatrices(y_grad_expected,
                              drake::math::autoDiffToGradientMatrix(y_taylor),
                              1E-10, MatrixCompareType::absolute));
}

GTEST_TEST(testConstraint, testLorentzConeConstraint) {
  auto cnstr = LorentzConeConstraint();
  auto lb = cnstr.lower_bound();
  auto ub = cnstr.upper_bound();
  EXPECT_TRUE(CompareMatrices(Eigen::Vector2d(0.0, 0.0), lb, 1E-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()), ub,
      1e-10, MatrixCompareType::absolute));

  // [3;1;1] is in the interior of the Lorentz cone.
  Eigen::Vector3d x1(3.0, 1.0, 1.0);
  TestLorentzConeEval(x1, true);

  // [3;2;2;1] is on the boundary of the Lorentz cone.
  Eigen::Vector4d x2(3.0, 2.0, 2.0, 1.0);
  TestLorentzConeEval(x2, true);

  // [3; 3; 1] is outside of the Lorentz cone.
  Eigen::Vector3d x3(3.0, 3.0, 1.0);
  TestLorentzConeEval(x3, false);

  // [-3; 1; 1] is outside of the Lorentz cone.
  Eigen::Vector3d x4(-3.0, 1.0, 1.0);
  TestLorentzConeEval(x4, false);
}

GTEST_TEST(testConstraint, testRotatedLorentzConeConstraint) {
  auto cnstr = RotatedLorentzConeConstraint();
  auto lb = cnstr.lower_bound();
  auto ub = cnstr.upper_bound();
  EXPECT_TRUE(CompareMatrices(Eigen::Vector3d::Zero(), lb, 1E-10,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()), ub,
      1e-10, MatrixCompareType::absolute));

  // [1;2;1] is in the interior of the rotated lorentz cone.
  TestRotatedLorentzConeEval(Vector3d(1, 2, 1), true);

  // [1;2;1;1] is on the boundary of the rotated Lorentz cone.
  TestRotatedLorentzConeEval(Eigen::Vector4d(1, 2, 1, 1), true);

  // [1;2;2;2] is outside of the rotated Lorentz cone.
  TestRotatedLorentzConeEval(Eigen::Vector4d(1, 2, 2, 2), false);

  // [-1; -2; 1] is outside of the rotated Lorentz cone.
  TestRotatedLorentzConeEval(Vector3d(-1, -2, 1), false);
}

GTEST_TEST(testConstraint, testPositiveSemidefiniteConstraint) {
  auto cnstr = PositiveSemidefiniteConstraint(3);

  Eigen::Matrix<double, 9, 1> X1;
  // clang-format off
  X1 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
  // clang-format on
  Eigen::VectorXd y;
  cnstr.Eval(X1, y);
  EXPECT_TRUE((y.array() >= cnstr.lower_bound().array()).all());
  EXPECT_TRUE((y.array() <= cnstr.upper_bound().array()).all());

  Eigen::Matrix<double, 9, 1> X2;
  // clang-format off
  X2 << 1, 2, 0,
        2, -2, -1,
        0, -1, -2;
  // clang-format on
  cnstr.Eval(X2, y);
  EXPECT_TRUE((y.array() < cnstr.lower_bound().array()).any() ||
              (y.array() > cnstr.upper_bound().array()).any());
}

GTEST_TEST(testConstraint, testLinearMatrixInequalityConstraint) {
  Eigen::Matrix2d F0 = 2 * Eigen::Matrix2d::Identity();
  Eigen::Matrix2d F1;
  F1 << 1, 1, 1, 1;
  Eigen::Matrix2d F2;
  F2 << 1, 2, 2, 1;
  auto cnstr = LinearMatrixInequalityConstraint({F0, F1, F2});

  // [4, 3]
  // [3, 4] is positive semidefinite
  Eigen::VectorXd y;
  cnstr.Eval(Eigen::Vector2d(1, 1), y);
  EXPECT_TRUE((y.array() >= cnstr.lower_bound().array()).all());
  EXPECT_TRUE((y.array() <= cnstr.upper_bound().array()).all());

  // [1 -2]
  // [-2 1] is not p.s.d
  cnstr.Eval(Eigen::Vector2d(0, -1), y);
  EXPECT_TRUE((y.array() < cnstr.lower_bound().array()).any() ||
              (y.array() > cnstr.upper_bound().array()).any());
}
}  // namespace
}  // namespace solvers
}  // namespace drake
