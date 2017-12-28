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
// At the root node, the optimizer should obtain an integral solution. So the
// root node does not need to branch.
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

// Construct the problem data for the mixed-integer linear program
// min x₀ + 2x₁ - 3x₂ - 4x₃ + 4.5x₄ + 1
// s.t 2x₀ + x₂ + 1.5x₃ + x₄ = 4.5
//     1 ≤ 2x₀ + 4x₃ + x₄ ≤ 7
//     -2 ≤ 3x₁ + 2x₂ - 5x₃ + x₄ ≤ 7
//     -5 ≤ x₁ + x₂ + 2x₃ ≤ 10
//     -10 ≤ x₁ ≤ 10
//     x₀, x₂, x₄ are binary variables.
// The optimal solution is (1, 1/3, 1, 1, 0), with optimal cost -13/3.
std::unique_ptr<MathematicalProgram> ConstructMathematicalProgram2() {
  auto prog = std::make_unique<MathematicalProgram>();
  VectorDecisionVariable<5> x;
  x(0) = symbolic::Variable("x0", symbolic::Variable::Type::BINARY);
  x(1) = symbolic::Variable("x1", symbolic::Variable::Type::CONTINUOUS);
  x(2) = symbolic::Variable("x2", symbolic::Variable::Type::BINARY);
  x(3) = symbolic::Variable("x3", symbolic::Variable::Type::CONTINUOUS);
  x(4) = symbolic::Variable("x4", symbolic::Variable::Type::BINARY);
  prog->WithVariables(x);
  prog->AddCost(x(0) + 2 * x(1) - 3 * x(2) - 4 * x(3) + 4.5 * x(4) + 1);
  prog->AddLinearEqualityConstraint(2 * x(0) + x(2) + 1.5 * x(3) + x(4) == 4.5);
  prog->AddLinearConstraint(2 * x(0) + 4 * x(3) + x(4), 1, 7);
  prog->AddLinearConstraint(3 * x(1) + 2 * x(2) - 5 * x(3) + x(4), -2, 7);
  prog->AddLinearConstraint(x(1) + x(2) + 2 * x(3), -5, 10);
  prog->AddBoundingBoxConstraint(-10, 10, x(1));
  return prog;
}

SolutionResult SolveWithGurobiOrMosek(MathematicalProgram* prog) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    return gurobi_solver.Solve(*prog);
  }
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    return mosek_solver.Solve(*prog);
  }
  throw std::runtime_error("None of the required solvers is available.");
}

void CheckRootNode(const MixedIntegerBranchAndBoundNode& root) {
  // The left and right childs are empty.
  EXPECT_FALSE(root.left_child());
  EXPECT_FALSE(root.right_child());
  // The parent node is empty.
  EXPECT_TRUE(root.IsRoot());
  // None of the binary variables are fixed.
  EXPECT_EQ(root.binary_var_index(), -1);
  EXPECT_EQ(root.binary_var_value(), -1);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot1) {
  auto prog = ConstructMathematicalProgram1();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable>
      map_old_vars_to_new_vars;
  std::tie(root, map_old_vars_to_new_vars) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog);
  VectorDecisionVariable<4> x;
  for (int i = 0; i < 4; ++i) {
    x(i) = map_old_vars_to_new_vars.at(prog->decision_variable(i).get_id());
    EXPECT_TRUE(x(i).equal_to(root->prog()->decision_variable(i)));
  }

  CheckRootNode(*root);

  const SolutionResult result = SolveWithGurobiOrMosek(root->prog());
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  const Eigen::Vector4d x_expected(0, 1, 0, 0.5);
  EXPECT_TRUE(CompareMatrices(root->prog()->GetSolution(x), x_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_NEAR(root->prog()->GetOptimalCost(), 1.5, 1E-5);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot2) {
  auto prog = ConstructMathematicalProgram2();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable>
      map_old_vars_to_new_vars;
  std::tie(root, map_old_vars_to_new_vars) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog);
  VectorDecisionVariable<5> x;
  for (int i = 0; i < 5; ++i) {
    x(i) = map_old_vars_to_new_vars.at(prog->decision_variable(i).get_id());
    EXPECT_TRUE(x(i).equal_to(root->prog()->decision_variable(i)));
  }

  CheckRootNode(*root);
  
  const SolutionResult result = SolveWithGurobiOrMosek(root->prog());
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  Eigen::Matrix<double, 5, 1> x_expected;
  x_expected << 0.7, 1, 1, 1.4, 0;
  EXPECT_TRUE(CompareMatrices(root->prog()->GetSolution(x), x_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_NEAR(root->prog()->GetOptimalCost(), -4.9, 1E-5);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
