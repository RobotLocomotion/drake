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

// Construct an unbounded mixed-integer optimization problem.
// min x(0) + 2*x(1) + 3 * x(2) + 2.5*x(3) + 2
// s.t x(0) + x(1) - x(2) + x(3) <= 3
//     1 <= x(0) + 2 * x(1) - 2 * x(2) + 4 * x(3) <= 3
//     x(0), x(2) are binary.
std::unique_ptr<MathematicalProgram> ConstructMathematicalProgram3() {
  auto prog = std::make_unique<MathematicalProgram>();
  VectorDecisionVariable<4> x;
  x(0) = symbolic::Variable("x0", symbolic::Variable::Type::BINARY);
  x(1) = symbolic::Variable("x1", symbolic::Variable::Type::CONTINUOUS);
  x(2) = symbolic::Variable("x2", symbolic::Variable::Type::BINARY);
  x(3) = symbolic::Variable("x3", symbolic::Variable::Type::CONTINUOUS);
  prog->WithVariables(x);
  prog->AddCost(x(0) + 2 * x(1) + 3 * x(2) + 2.5 * x(3) + 2);
  prog->AddLinearConstraint(x(0) + x(1) - x(2) + x(3) <= 3);
  prog->AddLinearConstraint(x(0) + 2 * x(1) - 2 * x(2) + 4 * x(3), 1, 3);
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

void CheckNewRootNode(
    const MixedIntegerBranchAndBoundNode& root,
    const std::list<symbolic::Variable>& binary_vars_expected) {
  // The left and right childs are empty.
  EXPECT_FALSE(root.left_child());
  EXPECT_FALSE(root.right_child());
  // The parent node is empty.
  EXPECT_TRUE(root.IsRoot());
  // None of the binary variables are fixed.
  EXPECT_TRUE(root.fixed_binary_variable().is_dummy());
  EXPECT_EQ(root.fixed_binary_value(), -1);
  EXPECT_EQ(root.remaining_binary_variables(), binary_vars_expected);
  EXPECT_TRUE(root.IsLeaf());
}

void TestProgSolve(MathematicalProgram* prog,
                   const Eigen::Ref<const VectorXDecisionVariable>& x,
                   const Eigen::Ref<const Eigen::VectorXd>& x_expected,
                   double optimal_cost, double tol = 1E-4) {
  const SolutionResult result = SolveWithGurobiOrMosek(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(prog->GetSolution(x), x_expected, tol,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(prog->GetOptimalCost(), optimal_cost, tol);
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

  CheckNewRootNode(*root, {x(0), x(2)});

  EXPECT_THROW(root->IsOptimalSolutionIntegral(), std::runtime_error);

  const Eigen::Vector4d x_expected(0, 1, 0, 0.5);
  TestProgSolve(root->prog(), x, x_expected, 1.5, 1E-5);
  EXPECT_TRUE(root->IsOptimalSolutionIntegral());
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

  CheckNewRootNode(*root, {x(0), x(2), x(4)});

  Eigen::Matrix<double, 5, 1> x_expected;
  x_expected << 0.7, 1, 1, 1.4, 0;
  TestProgSolve(root->prog(), x, x_expected, -4.9, 1E-5);
  EXPECT_FALSE(root->IsOptimalSolutionIntegral());
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot3) {
  auto prog = ConstructMathematicalProgram3();

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

  CheckNewRootNode(*root, {x(0), x(2)});

  const SolutionResult result = SolveWithGurobiOrMosek(root->prog());
  EXPECT_EQ(result, SolutionResult::kUnbounded);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRootError) {
  // The optimization program does not have a binary variable.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  prog.AddCost(x.cast<symbolic::Expression>().sum());

  EXPECT_THROW(MixedIntegerBranchAndBoundNode::ConstructRootNode(prog), std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestBranch1) {
  auto prog = ConstructMathematicalProgram1();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::tie(root, std::ignore) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog);
  VectorDecisionVariable<4> x = root->prog()->decision_variables();

  // Branch on variable x(0).
  root->Branch(x(0));

  const Eigen::Vector4d x_expected_l0(0, 1, 0, 0.5);
  const Eigen::Vector4d x_expected_r0(1, 1, 0, 0);
  TestProgSolve(root->left_child()->prog(), x, x_expected_l0, 1.5, 1E-5);
  TestProgSolve(root->right_child()->prog(), x, x_expected_r0, 4, 1E-5);
  EXPECT_TRUE(root->left_child()->IsOptimalSolutionIntegral());
  EXPECT_TRUE(root->right_child()->IsOptimalSolutionIntegral());

  // Branch on variable x(2). The child nodes created by branching on x(0) will
  // be deleted.
  root->Branch(x(2));
  const Eigen::Vector4d x_expected_l2(0, 1, 0, 0.5);
  const Eigen::Vector4d x_expected_r2(0, 4.1, 1, -1.05);
  TestProgSolve(root->left_child()->prog(), x, x_expected_l2, 1.5, 1E-5);
  TestProgSolve(root->right_child()->prog(), x, x_expected_r2, 12.35, 1E-5);
  EXPECT_TRUE(root->left_child()->IsOptimalSolutionIntegral());
  EXPECT_TRUE(root->right_child()->IsOptimalSolutionIntegral());

  // Branch on variable x(1) and x(3). Since these two variables are continuous,
  // expect a runtime error thrown.
  EXPECT_THROW(root->Branch(x(1)), std::runtime_error);
  EXPECT_THROW(root->Branch(x(3)), std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestBranch2) {
  auto prog = ConstructMathematicalProgram2();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::tie(root, std::ignore) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog);
  VectorDecisionVariable<5> x = root->prog()->decision_variables();

  // Branch on x(0)
  root->Branch(x(0));
  const SolutionResult result_l0 =
      SolveWithGurobiOrMosek(root->left_child()->prog());
  EXPECT_EQ(result_l0, SolutionResult::kInfeasibleConstraints);
  Eigen::Matrix<double, 5, 1> x_expected_r;
  x_expected_r << 1, 1.0 / 3.0, 1, 1, 0;
  TestProgSolve(root->right_child()->prog(), x, x_expected_r, -13.0 / 3.0,
                1E-5);
  EXPECT_TRUE(root->right_child()->IsOptimalSolutionIntegral());

  // Branch on x(2)
  root->Branch(x(2));
  Eigen::Matrix<double, 5, 1> x_expected_l;
  x_expected_l << 1, 2.0 / 3.0, 0, 1, 1;
  TestProgSolve(root->left_child()->prog(), x, x_expected_l, 23.0 / 6.0, 1E-5);
  EXPECT_TRUE(root->left_child()->IsOptimalSolutionIntegral());
  x_expected_r << 0.7, 1, 1, 1.4, 0;
  TestProgSolve(root->right_child()->prog(), x, x_expected_r, -4.9, 1E-5);
  EXPECT_FALSE(root->right_child()->IsOptimalSolutionIntegral());

  // Branch on x(4)
  root->Branch(x(4));
  x_expected_l << 0.7, 1, 1, 1.4, 0;
  x_expected_r << 0.2, 2.0 / 3.0, 1, 1.4, 1;
  TestProgSolve(root->left_child()->prog(), x, x_expected_l, -4.9, 1E-5);
  TestProgSolve(root->right_child()->prog(), x, x_expected_r, -47.0 / 30, 1E-5);
  EXPECT_FALSE(root->left_child()->IsOptimalSolutionIntegral());
  EXPECT_FALSE(root->right_child()->IsOptimalSolutionIntegral());

  // x(1) and x(3) are continuous variables, expect runtime_error thrown when we
  // branch on them.
  EXPECT_THROW(root->Branch(x(1)), std::runtime_error);
  EXPECT_THROW(root->Branch(x(3)), std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestBranch3) {
  auto prog = ConstructMathematicalProgram3();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::tie(root, std::ignore) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog);
  VectorDecisionVariable<4> x = root->prog()->decision_variables();

  // Branch on x(0) and x(2), the child nodes are all unbounded.
  for (const auto& x_binary : {x(0), x(2)}) {
    root->Branch(x_binary);
    const SolutionResult result_l =
        SolveWithGurobiOrMosek(root->left_child()->prog());
    const SolutionResult result_r =
        SolveWithGurobiOrMosek(root->right_child()->prog());
    EXPECT_EQ(result_l, SolutionResult::kUnbounded);
    EXPECT_EQ(result_r, SolutionResult::kUnbounded);
  }

  // Expect to throw a runtime_error when branching on x(1) and x(3), as they
  // are continuous variables.
  EXPECT_THROW(root->Branch(x(1)), std::runtime_error);
  EXPECT_THROW(root->Branch(x(3)), std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestNewVariable) {
  auto prog = ConstructMathematicalProgram1();

  MixedIntegerBranchAndBound bnb(*prog);
  const VectorDecisionVariable<4> prog_x = prog->decision_variables();

  const VectorDecisionVariable<4> bnb_x = bnb.root()->prog()->decision_variables();

  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(bnb_x(i).equal_to(bnb.NewVariable(prog_x(i))));
  }

  static_assert(std::is_same<decltype(bnb.NewVariables(prog_x.head<2>())), VectorDecisionVariable<2>>::value, "Should return VectorDecisionVariable<2> object.\n");
  static_assert(std::is_same<decltype(bnb.NewVariables(prog_x.head(2))), VectorXDecisionVariable>::value, "Should return VectorXDecisionVariable object.\n");
  MatrixDecisionVariable<2, 2> X;
  X << prog_x(0), prog_x(1), prog_x(2), prog_x(3);
  static_assert(std::is_same<decltype(bnb.NewVariables(X)), MatrixDecisionVariable<2, 2>>::value, "Should return MatrixDecisionVariable<2, 2> object.\n");
}
}  // namespace
}  // namespace solvers
}  // namespace drake
