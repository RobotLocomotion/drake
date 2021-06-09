#include "drake/solvers/branch_and_bound.h"

#include <algorithm>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mixed_integer_optimization_util.h"
#include "drake/solvers/scs_solver.h"

namespace drake {
namespace solvers {
// This class exposes the protected and private members of
//  MixedIntegerBranchAndBound, so that we can test its internal implementation.
class MixedIntegerBranchAndBoundTester {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MixedIntegerBranchAndBoundTester)

  explicit MixedIntegerBranchAndBoundTester(const MathematicalProgram& prog,
                                            const SolverId& solver_id)
      : bnb_{new MixedIntegerBranchAndBound(prog, solver_id)} {}

  MixedIntegerBranchAndBound* bnb() const { return bnb_.get(); }

  MixedIntegerBranchAndBoundNode* PickBranchingNode() const {
    return bnb_->PickBranchingNode();
  }

  const symbolic::Variable* PickBranchingVariable(
      const MixedIntegerBranchAndBoundNode& node) const {
    return bnb_->PickBranchingVariable(node);
  }

  void UpdateIntegralSolution(const Eigen::Ref<const Eigen::VectorXd>& solution,
                              double cost) {
    return bnb_->UpdateIntegralSolution(solution, cost);
  }

  void BranchAndUpdate(MixedIntegerBranchAndBoundNode* node,
                       const symbolic::Variable& branching_variable) {
    return bnb_->BranchAndUpdate(node, branching_variable);
  }

  MixedIntegerBranchAndBoundNode* mutable_root() { return bnb_->root_.get(); }

  void SearchIntegralSolutionByRounding(
      const MixedIntegerBranchAndBoundNode& node) {
    bnb_->SearchIntegralSolutionByRounding(node);
  }

 private:
  std::unique_ptr<MixedIntegerBranchAndBound> bnb_;
};

namespace {
// Construct a mixed-integer linear program
// min x₀ + 2x₁ - 3x₃ + 1
// s.t x₀ + x₁ + 2x₃ = 2
//     x₁ - 3.1x₂ ≥ 1
//     x₂ + 1.2x₃ - x₀ ≤ 5
//     x₀ , x₂ are binary
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
  prog->AddDecisionVariables(x);
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
  prog->AddDecisionVariables(x);
  prog->AddCost(x(0) + 2 * x(1) - 3 * x(2) - 4 * x(3) + 4.5 * x(4) + 1);
  prog->AddLinearEqualityConstraint(2 * x(0) + x(2) + 1.5 * x(3) + x(4) == 4.5);
  prog->AddLinearConstraint(2 * x(0) + 4 * x(3) + x(4), 1, 7);
  prog->AddLinearConstraint(3 * x(1) + 2 * x(2) - 5 * x(3) + x(4), -2, 7);
  prog->AddLinearConstraint(x(1) + x(2) + 2 * x(3), -5, 10);
  prog->AddBoundingBoxConstraint(-10, 10, x(1));
  return prog;
}

// Construct an unbounded mixed-integer optimization problem.
// min x₀ + 2*x₁ + 3 * x₂ + 2.5*x₃ + 2
// s.t x₀ + x₁ - x₂ + x₃ ≤ 3
//     1 ≤ x₀ + 2 * x₁ - 2 * x₂ + 4 * x₃ ≤ 3
//     x₀, x₂ are binary.
std::unique_ptr<MathematicalProgram> ConstructMathematicalProgram3() {
  auto prog = std::make_unique<MathematicalProgram>();
  VectorDecisionVariable<4> x;
  x(0) = symbolic::Variable("x0", symbolic::Variable::Type::BINARY);
  x(1) = symbolic::Variable("x1", symbolic::Variable::Type::CONTINUOUS);
  x(2) = symbolic::Variable("x2", symbolic::Variable::Type::BINARY);
  x(3) = symbolic::Variable("x3", symbolic::Variable::Type::CONTINUOUS);
  prog->AddDecisionVariables(x);
  prog->AddCost(x(0) + 2 * x(1) + 3 * x(2) + 2.5 * x(3) + 2);
  prog->AddLinearConstraint(x(0) + x(1) - x(2) + x(3) <= 3);
  prog->AddLinearConstraint(x(0) + 2 * x(1) - 2 * x(2) + 4 * x(3), 1, 3);
  return prog;
}

// An infeasible program.
// Given points P0 = (0, 3), P1 = (1, 1), and P2 = (2, 2), and the line segments
// connecting P0P1 and P1P2, we want to find if the point (1, 2) is on the
// line segments, and optimize some cost. This problem can be formulated as
// min x₀ + x₁ + 2 * x₂ // An arbitrary cost.
// s.t x₀ ≤ y₀
//     x₁ ≤ y₀ + y₁
//     x₂ ≤ y₁
//     x₀ + x₁ + x₂ = 1
//     x₁ + 2 * x₂ = 1
//     y₀ + y₁ = 1
//     3 * x₀ + x₁ + 2 * x₂ = 2
//     y₀, y₁ are binary
//     x₀, x₁, x₂ ≥ 0
// This formulation is obtained by using the special ordered set constraint
// with yᵢ being the indicator binary variable that the point is on the line
// segment PᵢPᵢ₊₁
std::unique_ptr<MathematicalProgram> ConstructMathematicalProgram4() {
  auto prog = std::make_unique<MathematicalProgram>();
  auto x = prog->NewContinuousVariables<3>("x");
  auto y = prog->NewBinaryVariables<2>("y");
  AddSos2Constraint(prog.get(), x.cast<symbolic::Expression>(),
                    y.cast<symbolic::Expression>());
  prog->AddLinearCost(x(0) + x(1) + 2 * x(2));
  prog->AddLinearConstraint(x(1) + 2 * x(2) == 1);
  prog->AddLinearConstraint(3 * x(0) + x(1) + 2 * x(2) == 2);
  prog->AddBoundingBoxConstraint(0, 1, x);
  return prog;
}

void CheckNewRootNode(
    const MixedIntegerBranchAndBoundNode& root,
    const std::list<symbolic::Variable>& binary_vars_expected) {
  // The left and right children are empty.
  EXPECT_FALSE(root.left_child());
  EXPECT_FALSE(root.right_child());
  // The parent node is empty.
  EXPECT_TRUE(root.IsRoot());
  // None of the binary variables are fixed.
  EXPECT_TRUE(root.fixed_binary_variable().is_dummy());
  EXPECT_EQ(root.fixed_binary_value(), -1);
  // Expect root.remaining_binary_variables() equal to binary_vars_expected.
  EXPECT_EQ(root.remaining_binary_variables().size(),
            binary_vars_expected.size());
  EXPECT_TRUE(std::equal(
      root.remaining_binary_variables().begin(),
      root.remaining_binary_variables().end(), binary_vars_expected.begin(),
      [](const symbolic::Variable& v1, const symbolic::Variable& v2) {
        return v1.equal_to(v2);
      }));
  EXPECT_TRUE(root.IsLeaf());
}

void CheckNodeSolution(const MixedIntegerBranchAndBoundNode& node,
                       const Eigen::Ref<const VectorXDecisionVariable>& x,
                       const Eigen::Ref<const Eigen::VectorXd>& x_expected,
                       double optimal_cost, double tol = 1E-4) {
  const SolutionResult result = node.solution_result();
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(node.prog_result()->GetSolution(x), x_expected,
                              tol, MatrixCompareType::absolute));
  EXPECT_NEAR(node.prog_result()->get_optimal_cost(), optimal_cost, tol);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot1) {
  // Test constructing root node for prog 1.
  auto prog = ConstructMathematicalProgram1();
  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable>
      map_old_vars_to_new_vars;
  std::tie(root, map_old_vars_to_new_vars) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog,
                                                        GurobiSolver::id());

  VectorDecisionVariable<4> x;
  for (int i = 0; i < 4; ++i) {
    x(i) = map_old_vars_to_new_vars.at(prog->decision_variable(i).get_id());
    EXPECT_TRUE(x(i).equal_to(root->prog()->decision_variable(i)));
  }

  CheckNewRootNode(*root, {x(0), x(2)});

  const Eigen::Vector4d x_expected(0, 1, 0, 0.5);
  const double tol{1E-5};
  const double cost_expected{1.5};
  CheckNodeSolution(*root, x, x_expected, cost_expected, tol);
  EXPECT_TRUE(root->optimal_solution_is_integral());
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot2) {
  // Test constructing root node for prog 2.
  auto prog = ConstructMathematicalProgram2();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable>
      map_old_vars_to_new_vars;
  std::tie(root, map_old_vars_to_new_vars) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog,
                                                        GurobiSolver::id());
  VectorDecisionVariable<5> x;
  for (int i = 0; i < 5; ++i) {
    x(i) = map_old_vars_to_new_vars.at(prog->decision_variable(i).get_id());
    EXPECT_TRUE(x(i).equal_to(root->prog()->decision_variable(i)));
  }

  CheckNewRootNode(*root, {x(0), x(2), x(4)});

  Eigen::Matrix<double, 5, 1> x_expected;
  x_expected << 0.7, 1, 1, 1.4, 0;
  const double tol{1E-5};
  const double cost_expected{-4.9};
  CheckNodeSolution(*root, x, x_expected, cost_expected, tol);
  EXPECT_FALSE(root->optimal_solution_is_integral());
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot3) {
  // Test constructing root node for prog 3.
  auto prog = ConstructMathematicalProgram3();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable>
      map_old_vars_to_new_vars;
  std::tie(root, map_old_vars_to_new_vars) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog,
                                                        GurobiSolver::id());
  VectorDecisionVariable<4> x;
  for (int i = 0; i < 4; ++i) {
    x(i) = map_old_vars_to_new_vars.at(prog->decision_variable(i).get_id());
    EXPECT_TRUE(x(i).equal_to(root->prog()->decision_variable(i)));
  }

  CheckNewRootNode(*root, {x(0), x(2)});

  EXPECT_EQ(root->solution_result(), SolutionResult::kUnbounded);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot4) {
  // Test constructing root node for prog 4.
  auto prog = ConstructMathematicalProgram4();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable>
      map_old_vars_to_new_vars;
  std::tie(root, map_old_vars_to_new_vars) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog,
                                                        GurobiSolver::id());
  VectorDecisionVariable<5> x;
  for (int i = 0; i < 5; ++i) {
    x(i) = map_old_vars_to_new_vars.at(prog->decision_variable(i).get_id());
    EXPECT_TRUE(x(i).equal_to(root->prog()->decision_variable(i)));
  }

  CheckNewRootNode(*root, {x(3), x(4)});

  EXPECT_EQ(root->solution_result(), SolutionResult::kSolutionFound);
  Eigen::Matrix<double, 5, 1> x_expected;
  x_expected << 1.0 / 3, 1.0 / 3, 1.0 / 3, 2.0 / 3, 1.0 / 3;
  const double tol{1E-5};
  const double cost_expected{4.0 / 3.0};
  CheckNodeSolution(*root, x, x_expected, cost_expected, tol);
  EXPECT_FALSE(root->optimal_solution_is_integral());
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRootError) {
  // The optimization program does not have a binary variable.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  prog.AddCost(x.cast<symbolic::Expression>().sum());

  EXPECT_THROW(MixedIntegerBranchAndBoundNode::ConstructRootNode(
                   prog, GurobiSolver::id()),
               std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestBranch1) {
  // Test branching on the root node for prog 1.
  auto prog = ConstructMathematicalProgram1();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::tie(root, std::ignore) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog,
                                                        GurobiSolver::id());
  VectorDecisionVariable<4> x = root->prog()->decision_variables();

  // Branch on variable x(0).
  root->Branch(x(0));

  const Eigen::Vector4d x_expected_l0(0, 1, 0, 0.5);
  const Eigen::Vector4d x_expected_r0(1, 1, 0, 0);
  const double tol{1E-5};
  CheckNodeSolution(*(root->left_child()), x, x_expected_l0, 1.5, tol);
  CheckNodeSolution(*(root->right_child()), x, x_expected_r0, 4, tol);
  EXPECT_TRUE(root->left_child()->optimal_solution_is_integral());
  EXPECT_TRUE(root->right_child()->optimal_solution_is_integral());

  // Branch on variable x(2). The child nodes created by branching on x(0) will
  // be deleted.
  root->Branch(x(2));
  const Eigen::Vector4d x_expected_l2(0, 1, 0, 0.5);
  const Eigen::Vector4d x_expected_r2(0, 4.1, 1, -1.05);
  CheckNodeSolution(*(root->left_child()), x, x_expected_l2, 1.5, tol);
  CheckNodeSolution(*(root->right_child()), x, x_expected_r2, 12.35, tol);
  EXPECT_TRUE(root->left_child()->optimal_solution_is_integral());
  EXPECT_TRUE(root->right_child()->optimal_solution_is_integral());

  // Branch on variable x(1) and x(3). Since these two variables are continuous,
  // expect a runtime error thrown.
  EXPECT_THROW(root->Branch(x(1)), std::runtime_error);
  EXPECT_THROW(root->Branch(x(3)), std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestBranch2) {
  // Test branching on the root node for prog 2.
  auto prog = ConstructMathematicalProgram2();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::tie(root, std::ignore) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog,
                                                        GurobiSolver::id());
  VectorDecisionVariable<5> x = root->prog()->decision_variables();

  // Branch on x(0)
  root->Branch(x(0));
  EXPECT_EQ(root->left_child()->solution_result(),
            SolutionResult::kInfeasibleConstraints);
  Eigen::Matrix<double, 5, 1> x_expected_r;
  x_expected_r << 1, 1.0 / 3.0, 1, 1, 0;
  const double tol{1E-5};
  CheckNodeSolution(*(root->right_child()), x, x_expected_r, -13.0 / 3.0, tol);
  EXPECT_TRUE(root->right_child()->optimal_solution_is_integral());

  // Branch on x(2)
  root->Branch(x(2));
  Eigen::Matrix<double, 5, 1> x_expected_l;
  x_expected_l << 1, 2.0 / 3.0, 0, 1, 1;
  CheckNodeSolution(*(root->left_child()), x, x_expected_l, 23.0 / 6.0, tol);
  EXPECT_TRUE(root->left_child()->optimal_solution_is_integral());
  x_expected_r << 0.7, 1, 1, 1.4, 0;
  CheckNodeSolution(*(root->right_child()), x, x_expected_r, -4.9, tol);
  EXPECT_FALSE(root->right_child()->optimal_solution_is_integral());

  // Branch on x(4)
  root->Branch(x(4));
  x_expected_l << 0.7, 1, 1, 1.4, 0;
  x_expected_r << 0.2, 2.0 / 3.0, 1, 1.4, 1;
  CheckNodeSolution(*(root->left_child()), x, x_expected_l, -4.9, tol);
  CheckNodeSolution(*(root->right_child()), x, x_expected_r, -47.0 / 30, tol);
  EXPECT_FALSE(root->left_child()->optimal_solution_is_integral());
  EXPECT_FALSE(root->right_child()->optimal_solution_is_integral());

  // x(1) and x(3) are continuous variables, expect runtime_error thrown when we
  // branch on them.
  EXPECT_THROW(root->Branch(x(1)), std::runtime_error);
  EXPECT_THROW(root->Branch(x(3)), std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestBranch3) {
  // Test branching on the root node for prog 3.
  auto prog = ConstructMathematicalProgram3();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::tie(root, std::ignore) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog,
                                                        GurobiSolver::id());
  VectorDecisionVariable<4> x = root->prog()->decision_variables();

  // Branch on x(0) and x(2), the child nodes are all unbounded.
  for (const auto& x_binary : {x(0), x(2)}) {
    root->Branch(x_binary);
    EXPECT_EQ(root->left_child()->solution_result(),
              SolutionResult::kUnbounded);
    EXPECT_EQ(root->right_child()->solution_result(),
              SolutionResult::kUnbounded);
  }

  // Expect to throw a runtime_error when branching on x(1) and x(3), as they
  // are continuous variables.
  EXPECT_THROW(root->Branch(x(1)), std::runtime_error);
  EXPECT_THROW(root->Branch(x(3)), std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestBranch4) {
  // Test branching on the root node for prog 4.
  auto prog = ConstructMathematicalProgram4();

  std::unique_ptr<MixedIntegerBranchAndBoundNode> root;
  std::tie(root, std::ignore) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(*prog,
                                                        GurobiSolver::id());
  VectorDecisionVariable<5> x = root->prog()->decision_variables();

  // Branch on x(3)
  root->Branch(x(3));
  EXPECT_EQ(root->left_child()->solution_result(),
            SolutionResult::kInfeasibleConstraints);
  EXPECT_EQ(root->right_child()->solution_result(),
            SolutionResult::kInfeasibleConstraints);

  // Branch on x(4)
  root->Branch(x(4));
  EXPECT_EQ(root->left_child()->solution_result(),
            SolutionResult::kInfeasibleConstraints);
  EXPECT_EQ(root->right_child()->solution_result(),
            SolutionResult::kInfeasibleConstraints);
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestNewVariable) {
  // Test GetNewVariable() function.
  auto prog = ConstructMathematicalProgram1();

  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  const VectorDecisionVariable<4> prog_x = prog->decision_variables();

  const VectorDecisionVariable<4> bnb_x =
      bnb.root()->prog()->decision_variables();

  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(bnb_x(i).equal_to(bnb.GetNewVariable(prog_x(i))));
  }

  static_assert(std::is_same_v<decltype(bnb.GetNewVariables(prog_x.head<2>())),
                               VectorDecisionVariable<2>>,
                "Should return VectorDecisionVariable<2> object.\n");
  static_assert(std::is_same_v<decltype(bnb.GetNewVariables(prog_x.head(2))),
                               VectorXDecisionVariable>,
                "Should return VectorXDecisionVariable object.\n");
  MatrixDecisionVariable<2, 2> X;
  X << prog_x(0), prog_x(1), prog_x(2), prog_x(3);
  static_assert(std::is_same_v<decltype(bnb.GetNewVariables(X)),
                               MatrixDecisionVariable<2, 2>>,
                "Should return MatrixDecisionVariable<2, 2> object.\n");
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestConstructor1) {
  // Test the constructor for MixedIntegerBranchAndBound for prog 1.
  auto prog = ConstructMathematicalProgram1();
  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  // The root node of the tree has integral optimal cost (0, 1, 0, 0.5), with
  // cost 1.5.
  const double tol = 1E-3;
  EXPECT_NEAR(bnb.best_upper_bound(), 1.5, tol);
  EXPECT_NEAR(bnb.best_lower_bound(), 1.5, tol);
  const auto& best_solutions = bnb.solutions();
  EXPECT_EQ(best_solutions.size(), 1);
  EXPECT_NEAR(best_solutions.begin()->first, 1.5, tol);
  const Eigen::Vector4d x_expected(0, 1, 0, 0.5);
  EXPECT_TRUE(CompareMatrices(best_solutions.begin()->second, x_expected, tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(bnb.IsLeafNodeFathomed(*(bnb.root())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestConstructor2) {
  // Test the constructor for MixedIntegerBranchAndBound for prog 2.
  auto prog = ConstructMathematicalProgram2();
  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  // The root node does not have an optimal integral solution.
  const double tol{1E-3};
  EXPECT_NEAR(bnb.best_lower_bound(), -4.9, tol);
  EXPECT_EQ(bnb.best_upper_bound(), std::numeric_limits<double>::infinity());
  EXPECT_TRUE(bnb.solutions().empty());
  EXPECT_FALSE(bnb.IsLeafNodeFathomed(*(bnb.root())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestConstructor3) {
  // Test the constructor for MixedIntegerBranchAndBound for prog 3.
  auto prog = ConstructMathematicalProgram3();
  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  // The root node is unbounded.
  EXPECT_EQ(bnb.best_lower_bound(), -std::numeric_limits<double>::infinity());
  EXPECT_EQ(bnb.best_upper_bound(), std::numeric_limits<double>::infinity());
  EXPECT_TRUE(bnb.solutions().empty());
  EXPECT_FALSE(bnb.IsLeafNodeFathomed(*(bnb.root())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestConstructor4) {
  // Test the constructor for MixedIntegerBranchAndBound for prog 4.
  auto prog = ConstructMathematicalProgram4();
  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  const double tol{1E-3};
  EXPECT_NEAR(bnb.best_lower_bound(), 4.0 / 3.0, tol);
  EXPECT_EQ(bnb.best_upper_bound(), std::numeric_limits<double>::infinity());
  EXPECT_TRUE(bnb.solutions().empty());
  EXPECT_FALSE(bnb.IsLeafNodeFathomed(*(bnb.root())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestLeafNodeFathomed1) {
  // Test IsLeafNodeFathomed for prog1.
  auto prog1 = ConstructMathematicalProgram1();
  MixedIntegerBranchAndBoundTester dut(*prog1, GurobiSolver::id());
  // The optimal solution to the root is integral. The node is fathomed.
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())));
  VectorDecisionVariable<4> x = dut.bnb()->root()->prog()->decision_variables();

  dut.mutable_root()->Branch(x(0));
  // Both left and right children have integral optimal solution. Both nodes are
  // fathomed.
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.mutable_root()->Branch(x(2));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  // The root node is not a leaf node. Expect a runtime error.
  EXPECT_THROW(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())),
               std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestLeafNodeFathomed2) {
  // Test IsLeafNodeFathomed for prog2.
  auto prog2 = ConstructMathematicalProgram2();
  MixedIntegerBranchAndBoundTester dut(*prog2, GurobiSolver::id());
  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  // The optimal solution to the root is not integral, the node is not fathomed.
  // The optimal cost is -4.9.
  EXPECT_FALSE(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())));

  dut.mutable_root()->Branch(x(0));
  // The left node is infeasible, thus fathomed.
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  // The right node has integral optimal solution, thus fathomed.
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.mutable_root()->Branch(x(2));
  // The solution to the left node is integral, with optimal cost 23.0 / 6.0.
  // The node is fathomed.
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  // The solution ot the right node is not integral, with optimal cost -4.9. The
  // node is not fathomed.
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.mutable_root()->Branch(x(4));
  // Neither the left nor the right child nodes has integral solution.
  // Neither of the nodes is fathomed.
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestLeafNodeFathomed3) {
  // Test IsLeafNodeFathomed for prog3.
  auto prog = ConstructMathematicalProgram3();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  const VectorDecisionVariable<4> x =
      dut.bnb()->root()->prog()->decision_variables();

  EXPECT_FALSE(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())));

  dut.mutable_root()->Branch(x(0));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.mutable_root()->mutable_left_child()->Branch(x(2));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->left_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->right_child())));

  dut.mutable_root()->mutable_right_child()->Branch(x(2));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->right_child()->left_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->right_child()->right_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->left_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->right_child())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestLeafNodeFathomed4) {
  // Test IsLeafNodeFathomed for prog4.
  auto prog = ConstructMathematicalProgram4();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  const VectorDecisionVariable<5> x =
      dut.bnb()->root()->prog()->decision_variables();

  EXPECT_FALSE(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())));

  dut.mutable_root()->Branch(x(3));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.mutable_root()->Branch(x(4));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestPickBranchingNode1) {
  // Test choosing the node with the minimal lower bound.
  auto prog = ConstructMathematicalProgram2();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  // The root node has optimal cost -4.9.
  dut.bnb()->SetNodeSelectionMethod(
      MixedIntegerBranchAndBound::NodeSelectionMethod::kMinLowerBound);
  // Pick the root node.
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root());

  // The left node has optimal cost 23.0 / 6.0, the right node has optimal cost
  // -4.9
  // Also the left node is fathomed.
  dut.mutable_root()->Branch(x(2));
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->right_child());

  dut.mutable_root()->Branch(x(4));
  // The left node has optimal cost -4.9, the right node has optimal cost -47.0
  // / 30
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->left_child());
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestPickBranchingNode2) {
  // Test choosing the deepest non-fathomed leaf node.
  auto prog = ConstructMathematicalProgram2();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  dut.bnb()->SetNodeSelectionMethod(
      MixedIntegerBranchAndBound::NodeSelectionMethod::kDepthFirst);
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root());

  dut.mutable_root()->Branch(x(2));
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->right_child());

  dut.mutable_root()->Branch(x(4));
  EXPECT_TRUE(dut.PickBranchingNode() == dut.bnb()->root()->left_child() ||
              dut.PickBranchingNode() == dut.bnb()->root()->right_child());

  dut.mutable_root()->mutable_left_child()->Branch(x(0));
  // root->left->left is infeasible, root->left->right has integral solution.
  // The only non-fathomed leaf node is root->right
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->right_child());
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestPickBranchingVariable1) {
  // Test picking branching variable for prog 2.
  auto prog = ConstructMathematicalProgram2();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  // The optimal solution at the root is (0.7, 1, 1, 1.4, 0)
  dut.bnb()->SetVariableSelectionMethod(
      MixedIntegerBranchAndBound::VariableSelectionMethod::kMostAmbivalent);
  EXPECT_TRUE(dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(0)));
  dut.bnb()->SetVariableSelectionMethod(
      MixedIntegerBranchAndBound::VariableSelectionMethod::kLeastAmbivalent);
  EXPECT_TRUE(dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(2)) ||
              dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(4)));

  dut.BranchAndUpdate(dut.mutable_root(), x(0));
  // The optimization at root->left is infeasible.
  dut.bnb()->SetVariableSelectionMethod(
      MixedIntegerBranchAndBound::VariableSelectionMethod::kMostAmbivalent);
  EXPECT_THROW(dut.PickBranchingVariable(*(dut.bnb()->root()->left_child())),
               std::runtime_error);
  dut.bnb()->SetVariableSelectionMethod(
      MixedIntegerBranchAndBound::VariableSelectionMethod::kLeastAmbivalent);
  EXPECT_THROW(dut.PickBranchingVariable(*(dut.bnb()->root()->left_child())),
               std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestPickBranchingVariable2) {
  // Test picking branching variable for prog 3.
  auto prog = ConstructMathematicalProgram3();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  VectorDecisionVariable<4> x = dut.bnb()->root()->prog()->decision_variables();

  // The problem is unbounded, any branching variable is acceptable.
  dut.bnb()->SetVariableSelectionMethod(
      MixedIntegerBranchAndBound::VariableSelectionMethod::kMostAmbivalent);
  EXPECT_TRUE(dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(0)) ||
              dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(2)));
  dut.bnb()->SetVariableSelectionMethod(
      MixedIntegerBranchAndBound::VariableSelectionMethod::kLeastAmbivalent);
  EXPECT_TRUE(dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(0)) ||
              dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(2)));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestBranchAndUpdate2) {
  // TestBranchAndUpdate for prog2.
  auto prog = ConstructMathematicalProgram2();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  EXPECT_TRUE(dut.bnb()->solutions().empty());
  const double tol = 1E-3;
  EXPECT_NEAR(dut.bnb()->best_lower_bound(), -4.9, tol);
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());

  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  dut.BranchAndUpdate(dut.mutable_root(), x(2));
  // The left node has optimal cost 23.0 / 6.0. with integral solution (1, 2 /
  // 3, 0, 1, 1,)
  // The right node has optimal cost -4.9, with non-integral solution (0.7, 1,
  // 1, 1.4, 0).
  EXPECT_NEAR(dut.bnb()->best_lower_bound(), -4.9, tol);
  EXPECT_NEAR(dut.bnb()->best_upper_bound(), 23.0 / 6.0, tol);
  EXPECT_EQ(dut.bnb()->solutions().size(), 1);
  EXPECT_NEAR(dut.bnb()->solutions().begin()->first, 23.0 / 6.0, tol);
  Eigen::Matrix<double, 5, 1> x_expected;
  x_expected << 1, 2.0 / 3.0, 0, 1, 1;
  EXPECT_TRUE(CompareMatrices(dut.bnb()->solutions().begin()->second,
                              x_expected, tol, MatrixCompareType::absolute));

  dut.BranchAndUpdate(dut.mutable_root(), x(4));
  // The left node has optimal cost -4.9, with non-integral solution (0.7, 1, 1,
  // 1.4, 0).
  // The right node has optimal cost -47/30, with  non-integral solution (0.2,
  // 2/3, 1, 1.4, 1).
  EXPECT_NEAR(dut.bnb()->best_lower_bound(), -4.9, tol);
  EXPECT_NEAR(dut.bnb()->best_upper_bound(), 23.0 / 6.0, tol);
  EXPECT_EQ(dut.bnb()->solutions().size(), 1);
  EXPECT_NEAR(dut.bnb()->solutions().begin()->first, 23.0 / 6.0, tol);
  EXPECT_TRUE(CompareMatrices(dut.bnb()->solutions().begin()->second,
                              x_expected, tol, MatrixCompareType::absolute));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestBranchAndUpdate3) {
  // TestBranchAndUpdate for prog3.
  auto prog = ConstructMathematicalProgram3();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  const VectorDecisionVariable<4> x =
      dut.bnb()->root()->prog()->decision_variables();

  dut.BranchAndUpdate(dut.mutable_root(), x(0));
  // Both left and right children are unbounded.
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(dut.bnb()->solutions().empty());
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));

  dut.BranchAndUpdate(dut.mutable_root()->mutable_left_child(), x(2));
  // Both root->l->l and root->l->r are unbounded.
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(dut.bnb()->solutions().empty());
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->left_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->right_child())));

  dut.BranchAndUpdate(dut.mutable_root()->mutable_right_child(), x(2));
  // Both root->r->l and root->r->r are unbounded.
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(dut.bnb()->solutions().empty());
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->right_child()->left_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->right_child()->right_child())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestBranchAndUpdate4) {
  // TestBranchAndUpdate for prog4.
  auto prog = ConstructMathematicalProgram4();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  const VectorDecisionVariable<5> x =
      dut.bnb()->root()->prog()->decision_variables();

  dut.BranchAndUpdate(dut.mutable_root(), x(3));
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            std::numeric_limits<double>::infinity());

  dut.BranchAndUpdate(dut.mutable_root(), x(4));
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            std::numeric_limits<double>::infinity());
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestSolve1) {
  // Test Solve() function for prog 1.
  auto prog = ConstructMathematicalProgram1();
  const VectorDecisionVariable<4> x = prog->decision_variables();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());

  const SolutionResult solution_result = dut.bnb()->Solve();
  EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
  const Eigen::Vector4d x_expected(0, 1, 0, 0.5);
  const double tol{1E-3};
  // The root node finds an optimal integral solution, and the bnb terminates.
  EXPECT_EQ(dut.bnb()->solutions().size(), 1);
  EXPECT_NEAR(dut.bnb()->GetOptimalCost(), 1.5, tol);
  EXPECT_TRUE(CompareMatrices(dut.bnb()->GetSolution(x, 0), x_expected, tol,
                              MatrixCompareType::absolute));
}

std::vector<MixedIntegerBranchAndBound::NodeSelectionMethod>
NonUserDefinedPickNodeMethods() {
  return {MixedIntegerBranchAndBound::NodeSelectionMethod::kDepthFirst,
          MixedIntegerBranchAndBound::NodeSelectionMethod::kMinLowerBound};
}

std::vector<MixedIntegerBranchAndBound::VariableSelectionMethod>
NonUserDefinedPickVariableMethods() {
  return {
      MixedIntegerBranchAndBound::VariableSelectionMethod::kMostAmbivalent,
      MixedIntegerBranchAndBound::VariableSelectionMethod::kLeastAmbivalent};
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestSolve2) {
  auto prog = ConstructMathematicalProgram2();
  const VectorDecisionVariable<5> x = prog->decision_variables();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  for (auto pick_variable : NonUserDefinedPickVariableMethods()) {
    for (auto pick_node : NonUserDefinedPickNodeMethods()) {
      dut.bnb()->SetNodeSelectionMethod(pick_node);
      dut.bnb()->SetVariableSelectionMethod(pick_variable);

      const SolutionResult solution_result = dut.bnb()->Solve();
      EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
      const double tol{1E-3};
      EXPECT_NEAR(dut.bnb()->GetOptimalCost(), -13.0 / 3, tol);
      Eigen::Matrix<double, 5, 1> x_expected0;
      x_expected0 << 1, 1.0 / 3.0, 1, 1, 0;
      EXPECT_TRUE(CompareMatrices(dut.bnb()->GetSolution(x, 0), x_expected0,
                                  tol, MatrixCompareType::absolute));
      // The costs are in the ascending order.
      double previous_cost = dut.bnb()->GetOptimalCost();
      for (int i = 1; i < static_cast<int>(dut.bnb()->solutions().size());
           ++i) {
        EXPECT_GE(dut.bnb()->GetSubOptimalCost(i - 1), previous_cost);
        previous_cost = dut.bnb()->GetSubOptimalCost(i - 1);
      }
    }
  }
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestSolve3) {
  auto prog = ConstructMathematicalProgram3();
  const VectorDecisionVariable<4> x = prog->decision_variables();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  for (auto pick_variable : NonUserDefinedPickVariableMethods()) {
    for (auto pick_node : NonUserDefinedPickNodeMethods()) {
      dut.bnb()->SetNodeSelectionMethod(pick_node);
      dut.bnb()->SetVariableSelectionMethod(pick_variable);

      const SolutionResult solution_result = dut.bnb()->Solve();
      EXPECT_EQ(solution_result, SolutionResult::kUnbounded);
    }
  }
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestSolve4) {
  auto prog = ConstructMathematicalProgram4();
  const VectorDecisionVariable<5> x = prog->decision_variables();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  for (auto pick_variable : NonUserDefinedPickVariableMethods()) {
    for (auto pick_node : NonUserDefinedPickNodeMethods()) {
      dut.bnb()->SetNodeSelectionMethod(pick_node);
      dut.bnb()->SetVariableSelectionMethod(pick_variable);

      const SolutionResult solution_result = dut.bnb()->Solve();
      EXPECT_EQ(solution_result, SolutionResult::kInfeasibleConstraints);
    }
  }
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestSteelBlendingProblem) {
  // This problem is taken from
  // "An application of Mixed Integer Programming in a Swedish Steel Mill"
  //  by  Carl-Henrik Westerberg, Bengt Bjorklund and Eskil Hultman
  //  on Interfaces, 1977
  //  The formulation is
  //  min 1750 * b₀ + 990 * b₁ + 1240 * b₂ + 1680 * b₃ + 550 * x₀ +
  //  450 * x₁ + 400 * x₂ + 100 * x₃
  //  s.t 5 * b₀ + 3 * b₁ + 4 * b₂ + 6 * b₃ + x₀ + x₁ + x₂ + x₃
  //  = 25
  //      0.25 * b₀ + 0.12 * b₁ + 0.2 * b₂ + 0.18 * b₃ + 0.08 * x₀ +
  //      0.07 * x₁ + 0.06 * x₂ + 0.03 * x₃ = 1.25
  //      0.15 * b₀ + 0.09 * b₁ + 0.16 * b₂ + 0.24 * b₃ + 0.06 * x₀ +
  //      0.07 * x₁ + 0.08 * x₂ + 0.09 * x₃ = 1.25
  //      x ≥ 0
  //      b are binary variables.
  MathematicalProgram prog;
  auto b = prog.NewBinaryVariables<4>("b");
  auto x = prog.NewContinuousVariables<4>("x");

  prog.AddLinearCost(1750 * b(0) + 990 * b(1) + 1240 * b(2) + 1680 * b(3) +
                     500 * x(0) + 450 * x(1) + 400 * x(2) + 100 * x(3));

  prog.AddLinearConstraint(5 * b(0) + 3 * b(1) + 4 * b(2) + 6 * b(3) + x(0) +
                               x(1) + x(2) + x(3) ==
                           25);
  prog.AddLinearConstraint(0.25 * b(0) + 0.12 * b(1) + 0.2 * b(2) +
                               0.18 * b(3) + 0.08 * x(0) + 0.07 * x(1) +
                               0.06 * x(2) + 0.03 * x(3) ==
                           1.25);
  prog.AddLinearConstraint(0.15 * b(0) + 0.09 * b(1) + 0.16 * b(2) +
                               0.24 * b(3) + 0.06 * x(0) + 0.07 * x(1) +
                               0.08 * x(2) + 0.09 * x(3) ==
                           1.25);
  prog.AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(), x);

  MixedIntegerBranchAndBound bnb(prog, GurobiSolver::id());

  for (auto pick_variable : NonUserDefinedPickVariableMethods()) {
    for (auto pick_node : NonUserDefinedPickNodeMethods()) {
      bnb.SetNodeSelectionMethod(pick_node);
      bnb.SetVariableSelectionMethod(pick_variable);
      SolutionResult solution_result = bnb.Solve();
      EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
      const double tol = 1E-3;
      const Eigen::Vector4d b_expected0(1, 1, 0, 1);
      const Eigen::Vector4d x_expected0(7.25, 0, 0.25, 3.5);
      EXPECT_TRUE(CompareMatrices(bnb.GetSolution(x), x_expected0, tol,
                                  MatrixCompareType::absolute));
      EXPECT_TRUE(CompareMatrices(bnb.GetSolution(b), b_expected0, tol,
                                  MatrixCompareType::absolute));
      EXPECT_NEAR(bnb.GetOptimalCost(), 8495, tol);
      double previous_cost = bnb.GetOptimalCost();
      for (int i = 1; i < static_cast<int>(bnb.solutions().size()); ++i) {
        EXPECT_GE(bnb.GetSubOptimalCost(i - 1), previous_cost);
        previous_cost = bnb.GetSubOptimalCost(i - 1);
      }
    }
  }
}

void CheckAllIntegralSolution(
    const MixedIntegerBranchAndBound& bnb,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const std::vector<std::pair<double, Eigen::VectorXd>>&
        all_integral_solutions,
    double tol) {
  EXPECT_LE(bnb.solutions().size(), all_integral_solutions.size());
  EXPECT_NEAR(bnb.GetOptimalCost(), all_integral_solutions[0].first, tol);
  EXPECT_TRUE(CompareMatrices(bnb.GetSolution(x),
                              all_integral_solutions[0].second, tol));
  double previous_cost = bnb.GetOptimalCost();
  for (int i = 1; i < static_cast<int>(bnb.solutions().size()); ++i) {
    const double cost_i = bnb.GetSubOptimalCost(i - 1);
    EXPECT_GE(cost_i, previous_cost - tol);
    previous_cost = cost_i;
    bool found_match = false;
    for (int j = 1; j < static_cast<int>(all_integral_solutions.size()); ++j) {
      if (std::abs(cost_i - all_integral_solutions[j].first) < tol &&
          (bnb.GetSolution(x, i) - all_integral_solutions[j].second).norm() <
              tol) {
        found_match = true;
        break;
      }
    }
    EXPECT_TRUE(found_match);
  }
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestMultipleIntegralSolution1) {
  // Test a program with multiple integral solutions.
  // Given points P1 = (0, 3), P2 = (1, 1), P3 = (2, 2), P4 = (4, 5),
  // P5 = (5, 1), and the line segments connecting P1P2, P2P3, P3P4, P4P5,
  // find the point with the smallest x coordinate, with y coordinate equal
  // to 4.
  // min x₁ + 2 * x₂ + 3 * x₃ + 4 * x(4)
  // s.t x₀ ≤ y₀
  //     x₁ ≤ y₀ + y₁
  //     x₂ ≤ y₁ + y₂
  //     x₃ ≤ y₂ + y₃
  //     x(4) ≤ y₃
  //     y₀ + y₁ + y₂ + y₃ = 1
  //     x₀ + x₁ + x₂ + x₃ + x(4) = 1
  //     3*x₀ + x₁ + 2*x₂ + 5*x₃ + x(4) = 4
  //     x ≥ 0
  //     y are binary variables.
  // This formulation is obtained by using the special ordered set constraint
  // with yᵢ being the indicator binary variable that the point is on the line
  // segment PᵢPᵢ₊₁
  // The optimal solution is x = (0, 0, 1/3, 2/3, 0), y = (0, 0, 1, 0), with
  // optimal cost 8/3.
  // There are other integral solutions,
  // x = (0, 0, 0, 0.25, 75), y = (0, 0, 0, 1), with cost 13/4
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>("x");
  auto y = prog.NewBinaryVariables<4>("y");
  AddSos2Constraint(&prog, x.cast<symbolic::Expression>(),
                    y.cast<symbolic::Expression>());
  prog.AddLinearCost(x(1) + 2 * x(2) + 3 * x(3) + 4 * x(4));
  prog.AddLinearConstraint(3 * x(0) + x(1) + 2 * x(2) + 5 * x(3) + x(4) == 4);
  prog.AddBoundingBoxConstraint(0, 1, x);

  MixedIntegerBranchAndBound bnb(prog, GurobiSolver::id());
  for (auto pick_variable : NonUserDefinedPickVariableMethods()) {
    for (auto pick_node : NonUserDefinedPickNodeMethods()) {
      bnb.SetNodeSelectionMethod(pick_node);
      bnb.SetVariableSelectionMethod(pick_variable);
      SolutionResult solution_result = bnb.Solve();
      EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
      std::vector<std::pair<double, Eigen::VectorXd>> best_solutions;
      Eigen::Matrix<double, 9, 1> xy_expected;
      xy_expected << 0, 0, 1.0 / 3, 2.0 / 3, 0, 0, 0, 1, 0;
      best_solutions.emplace_back(8.0 / 3, xy_expected);
      xy_expected << 0, 0, 0, 0.25, 75, 0, 0, 0, 1;
      best_solutions.emplace_back(13.0 / 4, xy_expected);

      const double tol{1E-3};
      VectorDecisionVariable<9> xy;
      xy << x, y;
      CheckAllIntegralSolution(bnb, xy, best_solutions, tol);
    }
  }
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestMultipleIntegralSolution2) {
  // Test a program with multiple integral solutions.
  // Given points P1 = (0, 3), P2 = (1, 1), P3 = (2, 4), P4 = (3, 2), and
  // the line segments connecting P1P2, P2P3, and P3P4. Find the closest point
  // on the line segment to the point (1.5, 2)
  // This problem can be formulated as
  // min (x₁ + 2*x₂+3*x₃ - 1.5)² + (3*x₀ + x₁ + 4*x₂ + 2*x₃)²
  // s.t x₀ ≤ y₀
  //     x₁ ≤ y₀ + y₁
  //     x₂ ≤ y₁ + y₂
  //     x₃ ≤ y₂
  //     y₀ + y₁ + y₂ = 1
  //     x₀ + x₁ + x₂ + x₃ = 1
  //     x ≥ 0
  //     y are binary variables.
  // This formulation is obtained by using the special ordered set constraint
  // with yᵢ being the indicator binary variable that the point is on the line
  // segment PᵢPᵢ₊₁
  // The optimal solution is x = (0, 0.65, 0.35, 0), y = (0, 1, 0), with optimal
  // cost 0.025
  // The other integral solutions are
  // x = (0.3, 0.7, 0, 0), y = (1, 0, 0), with cost 0.8
  // x = (0, 0, 0.3, 0.7), y = (0, 0, 1), with cost 1.8
  // Namely on each of the line segment, there is a closest point, corresponding
  // to one integral solution.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>("x");
  auto y = prog.NewBinaryVariables<3>("y");
  AddSos2Constraint(&prog, x.cast<symbolic::Expression>(),
                    y.cast<symbolic::Expression>());
  Eigen::Matrix<symbolic::Expression, 2, 1> pt;
  pt << x(1) + 2 * x(2) + 3 * x(3), 3 * x(0) + x(1) + 4 * x(2) + 2 * x(3);
  prog.AddQuadraticCost((pt - Eigen::Vector2d(1.5, 2)).squaredNorm());

  MixedIntegerBranchAndBound bnb(prog, GurobiSolver::id());
  for (auto pick_variable : NonUserDefinedPickVariableMethods()) {
    for (auto pick_node : NonUserDefinedPickNodeMethods()) {
      bnb.SetNodeSelectionMethod(pick_node);
      bnb.SetVariableSelectionMethod(pick_variable);
      const SolutionResult solution_result = bnb.Solve();
      EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
      std::vector<std::pair<double, Eigen::VectorXd>> integral_solutions;
      Eigen::Matrix<double, 7, 1> xy_expected;
      xy_expected << 0, 0.65, 0.35, 0, 0, 1, 0;
      integral_solutions.emplace_back(0.025, xy_expected);
      xy_expected << 0.3, 0.7, 0, 0, 1, 0, 0;
      integral_solutions.emplace_back(0.8, xy_expected);
      xy_expected << 0, 0, 0.3, 0.7, 0, 0, 1;
      integral_solutions.emplace_back(1.8, xy_expected);

      const double tol{1E-3};
      VectorDecisionVariable<7> xy;
      xy << x, y;
      CheckAllIntegralSolution(bnb, xy, integral_solutions, tol);
    }
  }
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, SearchIntegralSolutionByRounding2) {
  // Test searching an integral solution by rounding the fractional solution to
  // binary values, and solve the continuous variables.
  // Test on prog2.
  auto prog = ConstructMathematicalProgram2();
  VectorDecisionVariable<5> x = prog->decision_variables();
  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  // The solution to the root node is (0.7, 1, 1, 1.4, 0), with optimal cost
  // -4.9. We will add the constraint x₀ = 1, x₂ = 1, x₄ = 0, and then solve the
  // continuous variables.
  // The optimal solution to this new program is (1, 1/3, 1, 1, 0), with cost
  // -13 / 3. This is also the optimal solution to the MIP prog2.
  dut.SearchIntegralSolutionByRounding(*(dut.bnb()->root()));
  const double tol{1E-5};
  EXPECT_NEAR(dut.bnb()->best_upper_bound(), -13.0 / 3, tol);
  // The best lower bound is unchanged after solving the new program with fixed
  // binary variables.
  EXPECT_NEAR(dut.bnb()->best_lower_bound(), -4.9, tol);
  Eigen::Matrix<double, 5, 1> x_expected;
  x_expected << 1, 1.0 / 3, 1, 1, 0;
  EXPECT_TRUE(CompareMatrices(dut.bnb()->GetSolution(x), x_expected, tol,
                              MatrixCompareType::absolute));

  // Now test to solve the mip with searching for integral solution by rounding.
  // The branch-and-bound should terminate at the root node, as it should find
  // the optimal integral solution at the root.
  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  bnb.SetSearchIntegralSolutionByRounding(true);
  const SolutionResult result = bnb.Solve();
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, SearchIntegralSolutionByRounding3) {
  // Test searching an integral solution by rounding the fractional solution to
  // binary values, and solve the continuous variables.
  // Test on prog3.
  auto prog = ConstructMathematicalProgram3();
  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  // The solution to the root node is unbounded. If we fix the binary variables
  // and search for the continuous variables, the new program is still
  // unbounded.
  dut.SearchIntegralSolutionByRounding(*(dut.bnb()->root()));
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            -std::numeric_limits<double>::infinity());
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, SearchIntegralSolutionByRounding4) {
  // Test searching an integral solution by rounding the fractional solution to
  // binary values, and solve the continuous variables.
  // Test on prog4.
  auto prog = ConstructMathematicalProgram4();
  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  // The optimal solution to the root node is (1/3, 1/3, 1/3, 2/3, 1/3), with
  // optimal cost 4/3. We will add the constraint y₀ = 1, y₁ = 0, and solve for
  // the continuous variable x. The new problem is infeasible.
  dut.SearchIntegralSolutionByRounding(*(dut.bnb()->root()));
  const double tol{1E-5};
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  // This best lower bound is unchanged, after we search for the integral
  // solution, since the new program is infeasible.
  EXPECT_NEAR(dut.bnb()->best_lower_bound(), 4.0 / 3, tol);
}

MixedIntegerBranchAndBoundNode* LeftMostNodeInSubTree(
    const MixedIntegerBranchAndBound& branch_and_bound,
    const MixedIntegerBranchAndBoundNode& subtree_root) {
  // Find the left most leaf node that is not fathomed.
  if (subtree_root.IsLeaf()) {
    if (branch_and_bound.IsLeafNodeFathomed(subtree_root)) {
      return nullptr;
    } else {
      return const_cast<MixedIntegerBranchAndBoundNode*>(&subtree_root);
    }
  } else {
    auto left_most_node_left_tree =
        LeftMostNodeInSubTree(branch_and_bound, *(subtree_root.left_child()));
    if (!left_most_node_left_tree) {
      return LeftMostNodeInSubTree(branch_and_bound,
                                   *(subtree_root.right_child()));
    }
    return left_most_node_left_tree;
  }
}

GTEST_TEST(MixedIntegerBranchAndBoundTest,
           TestSetUserDefinedNodeSelectionFunction) {
  // Test setting user defined node selection function.
  // Here we set to choose the left-most unfathomed node.

  auto prog = ConstructMathematicalProgram2();
  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());

  dut.bnb()->SetNodeSelectionMethod(
      MixedIntegerBranchAndBound::NodeSelectionMethod::kUserDefined);
  dut.bnb()->SetUserDefinedNodeSelectionFunction([](
      const MixedIntegerBranchAndBound& branch_and_bound) {
    return LeftMostNodeInSubTree(branch_and_bound, *(branch_and_bound.root()));
  });

  // There is only one root node, so bnb has to pick the root node.
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root());

  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  dut.BranchAndUpdate(dut.mutable_root(), x(2));
  // root->left has integral solution, so it is fathomed.
  // root->right has non-integral solution, so it is not fathomed.
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));
  // The left-most un-fathomed node is root->right.
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->right_child());

  dut.BranchAndUpdate(dut.mutable_root(), x(4));
  // root->left has non-integral solution, so it is not fathomed.
  // root->right has non-integral solution, so it is not fathomed.
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));
  // The left-most un-fathomed node is root->left.
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->left_child());
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, NodeCallbackTest) {
  // Test node callback function.
  // In this trivial test, we just count how many nodes has been explored.

  int num_visited_nodes = 1;
  auto call_back_fun = [&num_visited_nodes](
      const MixedIntegerBranchAndBoundNode& node,
      MixedIntegerBranchAndBound* bnb) { ++num_visited_nodes; };
  auto prog = ConstructMathematicalProgram2();
  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  dut.bnb()->SetUserDefinedNodeCallbackFunction(call_back_fun);

  // Initially, only the root node has been visited.
  EXPECT_EQ(num_visited_nodes, 1);

  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();
  // Every branch increments the number of visited nodes by 2.
  dut.BranchAndUpdate(dut.mutable_root(), x(0));
  EXPECT_EQ(num_visited_nodes, 3);

  dut.BranchAndUpdate(dut.mutable_root(), x(2));
  EXPECT_EQ(num_visited_nodes, 5);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
