#include "drake/solvers/get_program_type.h"

#include <gtest/gtest.h>

namespace drake {
namespace solvers {
// We don't exhaustively test for "false positives" in these tests for
// GetProgramType(). The argument for not doing so is based on the idea that
// mathematical programs are uniquely characterized as a single type. It is
// impossible for a program that would classify as one type to ever provide a
// misclassificaiton as a different type. We rely on the sampling of
// mathematical programs to provide sufficient coverage of meaningful
// mathematical programs to render testing false positives unnecessary (as those
// candidates for false positives test positively elsewhere).
GTEST_TEST(GetProgramTypeTest, LP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x[0] + x[1] == 1);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kLP);
  prog.AddLinearCost(x[0] + x[1]);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kLP);
  prog.AddLinearConstraint(x[0] >= 0);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kLP);
  prog.AddQuadraticCost(x[0] * x[0]);
  EXPECT_NE(GetProgramType(prog), ProgramType::kLP);
}

GTEST_TEST(GetProgramTypeTest, QP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddQuadraticCost(x[0] * x[0] + x[1], true);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kQP);
  prog.AddLinearConstraint(x[0] + x[1] == 1);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kQP);
  prog.AddLinearCost(x[0] + x[1]);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kQP);

  // Add a non-convex quadratic cost.
  auto nonconvex_cost =
      prog.AddQuadraticCost(-x[1] * x[1], false /* non-convex */);
  EXPECT_NE(GetProgramType(prog), ProgramType::kQP);
  prog.RemoveCost(nonconvex_cost);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kQP);

  prog.AddLorentzConeConstraint(
      Vector3<symbolic::Expression>(x[0] + 2, 2 * x[0] + 1, x[0] + x[1]));
  EXPECT_NE(GetProgramType(prog), ProgramType::kQP);
}

GTEST_TEST(GetProgramTypeTest, SOCP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();
  prog.AddLorentzConeConstraint(x.cast<symbolic::Expression>());
  EXPECT_EQ(GetProgramType(prog), ProgramType::kSOCP);
  prog.AddRotatedLorentzConeConstraint(x.cast<symbolic::Expression>());
  EXPECT_EQ(GetProgramType(prog), ProgramType::kSOCP);
  prog.AddLinearConstraint(x[0] >= 1);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kSOCP);
  auto quadratic_cost = prog.AddQuadraticCost(x[0] * x[0]);
  EXPECT_NE(GetProgramType(prog), ProgramType::kSOCP);
  prog.RemoveCost(quadratic_cost);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kSOCP);
  prog.AddPositiveSemidefiniteConstraint(x[0] * Eigen::Matrix2d::Identity() +
                                         x[1] * Eigen::Matrix2d::Ones());
  EXPECT_NE(GetProgramType(prog), ProgramType::kSOCP);
}

GTEST_TEST(GetProgramTypeTest, SDP) {
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddLinearCost(X(0, 0) + X(1, 1));
  EXPECT_EQ(GetProgramType(prog), ProgramType::kSDP);
  prog.AddLorentzConeConstraint(
      Vector3<symbolic::Expression>(X(0, 0) + 1, X(1, 1), X(1, 2) + 2));
  EXPECT_EQ(GetProgramType(prog), ProgramType::kSDP);
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearMatrixInequalityConstraint(
      {Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Ones(),
       2 * Eigen::Matrix2d::Ones()},
      x);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kSDP);
  auto quadratic_cost = prog.AddQuadraticCost(x[0] * x[0]);
  EXPECT_NE(GetProgramType(prog), ProgramType::kSDP);
}

GTEST_TEST(GetProgramTypeTest, GP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  Eigen::SparseMatrix<double> A(3, 3);
  A.setIdentity();
  prog.AddExponentialConeConstraint(A, Eigen::Vector3d(0, 1, 2), x);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kGP);
  // Adding a Lorentz cone constraint, now this program cannot be modelled as
  // GP, but a CGP.
  prog.AddLorentzConeConstraint(x.cast<symbolic::Expression>());
  EXPECT_NE(GetProgramType(prog), ProgramType::kGP);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kCGP);
}

GTEST_TEST(GetProgramTypeTest, NLP) {
  {
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<2>();
    auto quadratic_cost =
        prog.AddQuadraticCost(x(0) * x(0) - x(1) * x(1), false);
    EXPECT_EQ(GetProgramType(prog), ProgramType::kNLP);
  }
  {
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<2>();
    prog.AddPolynomialCost(x(0) * x(0) * x(1) + 2 * x(1));
    EXPECT_EQ(GetProgramType(prog), ProgramType::kNLP);
  }
  {
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<2>();
    prog.AddConstraint(
        std::make_shared<QuadraticConstraint>(Eigen::Matrix2d::Identity(),
                                              Eigen::Vector2d(1, 0), 0, 1),
        x);
    EXPECT_EQ(GetProgramType(prog), ProgramType::kNLP);
    auto b = prog.NewBinaryVariables<2>();
    EXPECT_NE(GetProgramType(prog), ProgramType::kNLP);
  }
  {
    // A problem with linear complementarity constraint and a cost.
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<2>();
    prog.AddConstraint(std::make_shared<LinearComplementarityConstraint>(
                           Eigen::Matrix2d::Identity(), Eigen::Vector2d(1, 1)),
                       x);
    prog.AddLinearCost(x(0) + x(1));
    EXPECT_EQ(GetProgramType(prog), ProgramType::kNLP);
  }
  {
    // A problem with linear complementarity constraint and a lorentz cone
    // constraint.
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<3>();
    prog.AddConstraint(
        std::make_shared<LinearComplementarityConstraint>(
            Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 1, 0)),
        x);
    prog.AddLorentzConeConstraint(x);
    EXPECT_EQ(GetProgramType(prog), ProgramType::kNLP);
  }
}

GTEST_TEST(GetProgramTypeTest, LCP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearComplementarityConstraint(Eigen::Matrix2d::Identity(),
                                          Eigen::Vector2d(1, 2), x);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kLCP);
  // LCP doesn't accept linear constraint.
  prog.AddLinearConstraint(x[0] + x[1] >= 1);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kNLP);
}

GTEST_TEST(GetProgramTypeTest, MILP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto b = prog.NewBinaryVariables<2>();
  prog.AddLinearConstraint(x[0] + x[1] + b[1] + b[0] == 3);
  prog.AddLinearCost(x[0] + x[1]);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kMILP);
  prog.AddQuadraticCost(x[0] * x[0]);
  EXPECT_NE(GetProgramType(prog), ProgramType::kMILP);
}

GTEST_TEST(GetProgramTypeTest, MIQP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto b = prog.NewBinaryVariables<2>();
  prog.AddLinearConstraint(x[0] + x[1] + b[1] + b[0] == 3);
  prog.AddLinearCost(x[0] + x[1]);
  prog.AddQuadraticCost(x[0] * x[0], true);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kMIQP);
  // Add a non-convex quadratic cost.
  prog.AddQuadraticCost(-x[1] * x[1], false /* non-convex */);
  EXPECT_NE(GetProgramType(prog), ProgramType::kMIQP);
}

GTEST_TEST(GetProgramTypeTest, MISOCP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto b = prog.NewBinaryVariables<2>();
  prog.AddLinearConstraint(x[0] + x[1] + b[1] + b[0] == 3);
  prog.AddLorentzConeConstraint(x.cast<symbolic::Expression>());
  prog.AddLinearCost(x[0] + x[1]);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kMISOCP);
  prog.AddPositiveSemidefiniteConstraint(x[0] * Eigen::Matrix2d::Identity() +
                                         b[0] * Eigen::Matrix2d::Ones());
  EXPECT_NE(GetProgramType(prog), ProgramType::kMISOCP);
}

GTEST_TEST(GetProgramTypeTest, MISDP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto b = prog.NewBinaryVariables<2>();
  prog.AddLinearConstraint(x[0] + x[1] + b[1] + b[0] == 3);
  prog.AddLorentzConeConstraint(x.cast<symbolic::Expression>());
  prog.AddLinearCost(x[0] + x[1]);
  auto X = prog.NewSymmetricContinuousVariables<3>();
  prog.AddPositiveSemidefiniteConstraint(X);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kMISDP);
  prog.AddQuadraticCost(x[0] * x[0], true);
  EXPECT_NE(GetProgramType(prog), ProgramType::kMISDP);
}

GTEST_TEST(GetProgramTypeTest, QuadraticCostConicConstraint) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();
  prog.AddLorentzConeConstraint(x.cast<symbolic::Expression>());
  prog.AddRotatedLorentzConeConstraint(x.cast<symbolic::Expression>());
  auto quadratic_cost = prog.AddQuadraticCost(x(0) * x(0) + x(3) * x(3), true);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kQuadraticCostConicConstraint);
  prog.RemoveCost(quadratic_cost);
  EXPECT_NE(GetProgramType(prog), ProgramType::kQuadraticCostConicConstraint);
}

GTEST_TEST(GetProgramTypeTest, Unknown) {
  // Nonlinear constraint with binary variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  prog.AddConstraint(x[0] * x[0] * x[1] + 3 * x[1] * x[0] * x[2] == 1);
  auto b = prog.NewBinaryVariables<1>();
  prog.AddLinearComplementarityConstraint(Eigen::Matrix3d::Identity(),
                                          Eigen::Vector3d::Ones(), x);
  EXPECT_EQ(GetProgramType(prog), ProgramType::kUnknown);
}
}  // namespace solvers
}  // namespace drake
