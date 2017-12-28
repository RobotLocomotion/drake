#include "drake/solvers/branch_and_bound.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/scs_solver.h"

namespace drake {
namespace solvers {
namespace {
// Construct a mixed-integer linear program
// min x(0) + 2x(1) - 3x(3) + 1
// s.t x(0) + x(1) + 2x(3) = 2
//     x(1) - 3.1x(2) >= 1
//     x(2) + 1.2x(3) - x(0) <= 5
//     x(0) , x(2) are binary
// At the root node, the optimizer should obtain an integral solution. So the root node does not need to branch.
// The optimal solution is (0, 1, 0, 0.5), the optimal cost is 1.5
std::unique_ptr<MathematicalProgram> ConstructMathematicalProgram1() {
  auto prog = std::make_unique<MathematicalProgram>();
  VectorDecisionVariable<4> x;
  x(0) = symbolic::Variable("x0", symbolic::Variable::Type::BINARY);
  x(1) = symbolic::Variable("x1", symbolic::Variable::Type::CONTINUOUS);
  x(2) = symbolic::Variable("x2", symbolic::Variable::Type::BINARY);
  x(3) = symbolic::Variable("x3", symbolic::Variable::Type::CONTINUOUS);
  prog->WithVariables(x);
  prog->AddCost(x(0) + 2 * x(1) - 3 * x(3) + 1);
  prog->AddLinearEqualityConstraint(x(0) + x(1) + 2 * x(3) == 2);
  prog->AddLinearConstraint(x(1) - 3.1 * x(2) >= 1);
  prog->AddLinearConstraint(x(2) + 1.2 * x(3) - x(0) <= 5);
  return prog;
}

SolutionResult SolveWithGurobiOrMosekOrScs(MathematicalProgram* prog) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    return gurobi_solver.Solve(*prog);
  }
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    return mosek_solver.Solve(*prog);
  }
  ScsSolver scs_solver;
  if (scs_solver.available()) {
    return scs_solver.Solve(*prog);
  }
  throw std::runtime_error("None of the required solvers is available.");
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructor) {
  auto prog = ConstructMathematicalProgram1();
  auto x = prog->decision_variables();
  MixedIntegerBranchAndBoundNode node(*prog);

  // The left and right childs are empty.
  EXPECT_FALSE(node.left_child());
  EXPECT_FALSE(node.right_child());
  // The parent node is empty.
  EXPECT_TRUE(node.IsRoot());
  // None of the binary variables are fixed.
  EXPECT_EQ(node.binary_var_index(), -1);
  EXPECT_EQ(node.binary_var_value(), -1);

  SolutionResult result = SolveWithGurobiOrMosekOrScs(node.prog());
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  Eigen::Vector4d x_expected(0, 1, 0, 0.5);
  EXPECT_TRUE(CompareMatrices(node.prog()->GetSolution(x), x_expected, 1E-5, MatrixCompareType::absolute));
}
}  // namespace
}  // namespace solvers
}  // namespace drake
