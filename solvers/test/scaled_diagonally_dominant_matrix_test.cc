#include <limits>

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

using drake::symbolic::test::ExprEqual;

namespace drake {
namespace solvers {
bool is_zero(const symbolic::Variable& v) {
  return v.is_dummy();
}

template <typename T>
void CheckOffDiagonalTerms(int nx) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables(nx);

  auto M = prog.AddScaledDiagonallyDominantMatrixConstraint(X.cast<T>());
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < nx; ++j) {
      // M[i][j] should be a zero matrix if i >= j.
      if (i >= j) {
        for (int m = 0; m < 2; ++m) {
          for (int n = 0; n < 2; ++n) {
            EXPECT_TRUE(is_zero(M[i][j](m, n)));
          }
        }
      } else {
        EXPECT_PRED2(ExprEqual, symbolic::Expression(M[i][j](0, 1)),
                     symbolic::Expression(X(i, j)));
        EXPECT_PRED2(ExprEqual, symbolic::Expression(M[i][j](1, 0)),
                     symbolic::Expression(X(j, i)));
      }
    }
  }
}

GTEST_TEST(ScaledDiagonallyDominantMatrixTest, AddConstraint) {
  CheckOffDiagonalTerms<symbolic::Expression>(2);
  CheckOffDiagonalTerms<symbolic::Variable>(2);
  CheckOffDiagonalTerms<symbolic::Expression>(4);
  CheckOffDiagonalTerms<symbolic::Variable>(4);
}

template <typename T>
void CheckSDDMatrix(const Eigen::Ref<const Eigen::MatrixXd>& X_val,
                    bool is_sdd) {
  // Test if a sdd matrix satisfies the constraint.
  const int nx = X_val.rows();
  DRAKE_DEMAND(X_val.cols() == nx);
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables(nx);
  auto M = prog.AddScaledDiagonallyDominantMatrixConstraint(X.cast<T>());

  for (int i = 0; i < nx; ++i) {
    prog.AddBoundingBoxConstraint(X_val.col(i), X_val.col(i), X.col(i));
  }

  const auto result = Solve(prog);
  if (is_sdd) {
    EXPECT_TRUE(result.is_success());
    // Since X = ∑ᵢⱼ Mⁱʲ according to the definition of scaled diagonally
    // dominant matrix, we evaluate the summation of M, and compare that with X.
    std::vector<std::vector<Eigen::MatrixXd>> M_val(nx);
    symbolic::Environment env;
    for (int i = 0; i < prog.num_vars(); ++i) {
      env.insert(prog.decision_variable(i),
                 result.GetSolution(prog.decision_variable(i)));
    }
    Eigen::MatrixXd M_sum(nx, nx);
    M_sum.setZero();
    const double tol = 1E-6;
    for (int i = 0; i < nx; ++i) {
      M_val[i].resize(nx);
      for (int j = i + 1; j < nx; ++j) {
        M_val[i][j].resize(nx, nx);
        M_val[i][j].setZero();
        M_val[i][j](i, i) = symbolic::Expression(M[i][j](0, 0)).Evaluate(env);
        M_val[i][j](i, j) = symbolic::Expression(M[i][j](0, 1)).Evaluate(env);
        M_val[i][j](j, i) = M_val[i][j](i, j);
        M_val[i][j](j, j) = symbolic::Expression(M[i][j](1, 1)).Evaluate(env);
        M_sum += M_val[i][j];
        // (M[i][j](0, 0); M[i][j](1, 1); M[i][j](0, 1)) should be in the
        // rotated Lorentz cone.
        EXPECT_GE(M_val[i][j](i, i), -tol);
        EXPECT_GE(M_val[i][j](j, j), -tol);
        EXPECT_GE(M_val[i][j](i, i) * M_val[i][j](j, j),
                  std::pow(M_val[i][j](i, j), 2) - tol);
      }
    }
    EXPECT_TRUE(CompareMatrices(M_sum, X_val, tol));
  } else {
    EXPECT_FALSE(result.is_success());
    EXPECT_TRUE(result.get_solution_result() ==
                    SolutionResult::kInfeasibleConstraints ||
                result.get_solution_result() ==
                    SolutionResult::kInfeasibleOrUnbounded);
  }
}

bool IsMatrixSDD(const Eigen::Ref<Eigen::MatrixXd>& X) {
  // A matrix X is scaled diagonally dominant, if there exists a positive vector
  // d, such that the matrix A defined as A(i, j) = d(j) * X(i, j) is diagonally
  // dominant with positive diagonals.
  // This is explained as Remark 6 of "DSOS and SDSOS optimization: more
  // tractable alternatives to sum of squares and semidefinite optimization" by
  // Amir Ali Ahmadi and Anirudha Majumdar, with arXiv link
  // https://arxiv.org/abs/1706.02586.
  const int nx = X.rows();
  MathematicalProgram prog;
  auto d = prog.NewContinuousVariables(nx);
  MatrixX<symbolic::Expression> A(nx, nx);
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < nx; ++j) {
      A(i, j) = d(j) * X(i, j);
    }
  }
  prog.AddPositiveDiagonallyDominantMatrixConstraint(A);
  prog.AddBoundingBoxConstraint(1, std::numeric_limits<double>::infinity(), d);

  const auto result = Solve(prog);
  return result.is_success();
}

GTEST_TEST(ScaledDiagonallyDominantMatrixTest, TestSDDMatrix) {
  Eigen::Matrix4d dd_X;
  // A diagonally dominant matrix.
  // clang-format off
  dd_X << 1, -0.2, 0.3, -0.45,
           -0.2, 2, 0.5, 1,
           0.3, 0.5, 3, 2,
           -0.45, 1, 2, 4;
  // clang-format on
  CheckSDDMatrix<symbolic::Expression>(dd_X, true);
  CheckSDDMatrix<symbolic::Variable>(dd_X, true);

  Eigen::Matrix4d D = Eigen::Vector4d(1, 2, 3, 4).asDiagonal();
  Eigen::Matrix4d sdd_X = D * dd_X * D;
  CheckSDDMatrix<symbolic::Expression>(sdd_X, true);
  CheckSDDMatrix<symbolic::Variable>(sdd_X, true);

  D = Eigen::Vector4d(0.2, -1, -0.5, 1.2).asDiagonal();
  sdd_X = D * dd_X * D;
  CheckSDDMatrix<symbolic::Expression>(sdd_X, true);
  CheckSDDMatrix<symbolic::Variable>(sdd_X, true);

  // not_dd_X is not diagonally dominant (dd), but is scaled diagonally
  // dominant.
  Eigen::Matrix4d not_dd_X;
  not_dd_X << 1, -0.2, 0.3, -0.55, -0.2, 2, 0.5, 1, 0.3, 0.5, 3, 2, -0.55, 1, 2,
      4;
  DRAKE_DEMAND(IsMatrixSDD(not_dd_X));
  CheckSDDMatrix<symbolic::Expression>(not_dd_X, true);
  CheckSDDMatrix<symbolic::Variable>(not_dd_X, true);
}

GTEST_TEST(ScaledDiagonallyDominantMatrixTest, TestNotSDDMatrix) {
  Eigen::Matrix4d not_sdd_X;
  // Not a diagonally dominant matrix.
  // clang-format off
  not_sdd_X << 1, -0.2, 0.3, -1.55,
               -0.2, 2, 0.5, 1,
               0.3, 0.5, 3, 2,
               -1.55, 1, 2, 4;
  // clang-format on
  DRAKE_DEMAND(!IsMatrixSDD(not_sdd_X));
  CheckSDDMatrix<symbolic::Expression>(not_sdd_X, false);
  CheckSDDMatrix<symbolic::Variable>(not_sdd_X, false);

  Eigen::Matrix4d D = Eigen::Vector4d(1, 2, 3, 4).asDiagonal();
  not_sdd_X = D * not_sdd_X * D;
  CheckSDDMatrix<symbolic::Expression>(not_sdd_X, false);
  CheckSDDMatrix<symbolic::Variable>(not_sdd_X, false);

  D = Eigen::Vector4d(0.2, -1, -0.5, 1.2).asDiagonal();
  not_sdd_X = D * not_sdd_X * D;
  CheckSDDMatrix<symbolic::Expression>(not_sdd_X, false);
  CheckSDDMatrix<symbolic::Variable>(not_sdd_X, false);
}

GTEST_TEST(SdsosTest, SdsosPolynomial) {
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates<2>("x");
  Vector4<symbolic::Monomial> m;
  m << symbolic::Monomial(), symbolic::Monomial(x(0)), symbolic::Monomial(x(1)),
      symbolic::Monomial({{x(0), 1}, {x(1), 1}});

  symbolic::Polynomial p;
  std::tie(p, std::ignore) = prog.NewSosPolynomial(
      m, MathematicalProgram::NonnegativePolynomial::kSdsos);

  Eigen::Matrix4d dd_X;
  // A diagonally dominant matrix.
  // clang-format off
  dd_X << 1, -0.2, 0.3, -0.45,
           -0.2, 2, 0.5, 1,
           0.3, 0.5, 3, 2,
           -0.45, 1, 2, 4;
  // clang-format on

  const Eigen::Matrix4d D = Eigen::Vector4d(1, 2, 3, 4).asDiagonal();
  const Eigen::Matrix4d sdd_X = D * dd_X * D;
  symbolic::Polynomial p_expected;
  for (int i = 0; i < sdd_X.rows(); ++i) {
    for (int j = 0; j < sdd_X.cols(); ++j) {
      p_expected.AddProduct(sdd_X(i, j), m(i) * m(j));
    }
  }
  prog.AddLinearEqualityConstraint(p == p_expected);

  const MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());
}

GTEST_TEST(SdsosTest, NotSdsosPolynomial) {
  MathematicalProgram prog;
  auto x = prog.NewIndeterminates<2>("x");
  Vector3<symbolic::Monomial> m;
  m << symbolic::Monomial(), symbolic::Monomial(x(0)), symbolic::Monomial(x(1));

  // This polynomial is not sdsos, since at x = (0, -1) the polynomial is
  // negative.
  symbolic::Polynomial non_sdsos_poly(
      1 + x(0) + 4 * x(1) + x(0) * x(0) + x(1) * x(1), symbolic::Variables(x));
  const auto Q = prog.AddSosConstraint(
      non_sdsos_poly, m, MathematicalProgram::NonnegativePolynomial::kSdsos);

  const MathematicalProgramResult result = Solve(prog);
  EXPECT_FALSE(result.is_success());
  EXPECT_TRUE(
      result.get_solution_result() == SolutionResult::kInfeasibleConstraints ||
      result.get_solution_result() == SolutionResult::kInfeasibleOrUnbounded);
}

GTEST_TEST(ReplacePSDConstraintWithSDDConstraint,
           SinglePsdConstraint) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto psd_constraint = prog.AddPositiveSemidefiniteConstraint(X);

  // Add an arbitrary linear constraint on X.
  Eigen::MatrixXd A(2, 3);
  // clang-format off
  A << 1, 0, 1,
      0, -1, 1;
  // clang-format on
  Eigen::VectorXd lb(2);
  lb << -10, -7;
  Eigen::VectorXd ub(2);
  ub << 11, 9;
  auto affine_constraint_upper =
      prog.AddLinearConstraint(A * X * Eigen::VectorXd::Ones(3) <= ub);
  auto affine_constraint_lower =
      prog.AddLinearConstraint(A * X * Eigen::VectorXd::Ones(3) >= lb);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog.linear_constraints()), 2);

  auto sdd_constraint =
      prog.TightenPSDConstraintToSDDConstraint(
          psd_constraint);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  EXPECT_EQ(ssize(prog.linear_constraints()), 2);
  EXPECT_EQ(ssize(prog.rotated_lorentz_cone_constraints()), 3);
}

GTEST_TEST(ReplacePSDConstraintWithSDDConstraint,
           MultiPsdConstraint) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto Y = prog.NewSymmetricContinuousVariables<4>();
  auto psd_constraint_X = prog.AddPositiveSemidefiniteConstraint(X);
  auto psd_constraint_Y = prog.AddPositiveSemidefiniteConstraint(Y);

  // Add an arbitrary linear constraint on X.
  Eigen::MatrixXd A(2, 3);
  // clang-format off
  A << 1, 0, 1,
      0, -1, 1;
  // clang-format on
  Eigen::VectorXd lb(2);
  lb << -10, -7;
  Eigen::VectorXd ub(2);
  ub << 11, 9;
  auto affine_constraint_upper =
      prog.AddLinearConstraint(A * X * Eigen::VectorXd::Ones(3) <= ub);
  auto affine_constraint_lower =
      prog.AddLinearConstraint(A * X * Eigen::VectorXd::Ones(3) >= lb);
  auto Y_eq_constraint =
      prog.AddLinearEqualityConstraint(Y == Eigen::MatrixXd::Identity(4, 4));

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 2);
  EXPECT_EQ(ssize(prog.linear_constraints()), 2);
  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 1);

  auto sdd_constraint_X =
      prog.TightenPSDConstraintToSDDConstraint(
          psd_constraint_X);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 1);
  EXPECT_EQ(ssize(prog.linear_constraints()), 2);
  // An sdd constraint on X adds an equality constraints on the upper diagonal
  // of X to represent slack variables
  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 2);
  // 3 choose 2 rotated lorentz cone constraints for constraint X to be sdd
  EXPECT_EQ(ssize(prog.rotated_lorentz_cone_constraints()), 3);

  auto sdd_constraint_Y =
      prog.TightenPSDConstraintToSDDConstraint(
          psd_constraint_Y);

  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  EXPECT_EQ(ssize(prog.linear_constraints()), 2);
  // An sdd constraint on X adds an equality constraints on the upper diagonal
  // of Y to represent slack variables
  EXPECT_EQ(ssize(prog.linear_equality_constraints()), 3);
  // 3 choose 2 rotated lorentz cone constraints for the constraint that X be
  // sdd and 4 choose 2 for the constraint that Y be sdd.
  EXPECT_EQ(ssize(prog.rotated_lorentz_cone_constraints()), 9);
}

GTEST_TEST(ReplacePSDConstraintWithSDDConstraint,
           ReplacePsdConstraintNotInProgramVariableNotInProgram) {
  MathematicalProgram prog1;
  auto X1 = prog1.NewSymmetricContinuousVariables<3>();
  auto psd_constraint1 = prog1.AddPositiveSemidefiniteConstraint(X1);

  MathematicalProgram prog2;
  auto X2 = prog2.NewSymmetricContinuousVariables<3>();
  auto psd_constraint2 = prog2.AddPositiveSemidefiniteConstraint(X2);

  DRAKE_EXPECT_THROWS_MESSAGE(
      prog1.TightenPSDConstraintToSDDConstraint(psd_constraint2),
      ".*is not a decision variable.*");
}

GTEST_TEST(ReplacePSDConstraintWithSDDConstraint,
           ReplacePsdConstraintNotInProgramVariableInProgram) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  // A constraint not in the program.
  auto constraint = internal::CreateBinding(
      std::make_shared<PositiveSemidefiniteConstraint>(X.rows()),
      Eigen::Map<VectorXDecisionVariable>(X.data(), X.size()));
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  prog.TightenPSDConstraintToSDDConstraint(constraint);
  EXPECT_EQ(ssize(prog.positive_semidefinite_constraints()), 0);
  // Still adds the DD constraint even though the constraint was not found in
  // the program.
  EXPECT_EQ(ssize(prog.rotated_lorentz_cone_constraints()), 3);
}

}  // namespace solvers
}  // namespace drake
