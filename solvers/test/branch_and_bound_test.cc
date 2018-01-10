#include "drake/solvers/branch_and_bound.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
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

 private:
  std::unique_ptr<MixedIntegerBranchAndBound> bnb_;
};

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
  prog->AddDecisionVariables(x);
  prog->AddCost(x(0) + 2 * x(1) + 3 * x(2) + 2.5 * x(3) + 2);
  prog->AddLinearConstraint(x(0) + x(1) - x(2) + x(3) <= 3);
  prog->AddLinearConstraint(x(0) + 2 * x(1) - 2 * x(2) + 4 * x(3), 1, 3);
  return prog;
}

// An infeasible program.
// min x(0) + x(1) + 2 * x(2)
// s.t x(0) <= y(0)
//     x(1) <= y(0) + y(1)
//     x(2) <= y(1)
//     x(0) + x(1) + x(2) = 1
//     x(1) + 2 * x(2) = 1
//     y(0) + y(1) = 1
//     3 * x(0) + x(1) + 2 * x(2) = 2
//     y(0), y(1) are binary
//     x(0), x(1), x(2) >= 0
std::unique_ptr<MathematicalProgram> ConstructMathematicalProgram4() {
  auto prog = std::make_unique<MathematicalProgram>();
  auto x = prog->NewContinuousVariables<3>("x");
  auto y = prog->NewBinaryVariables<2>("y");
  prog->AddLinearCost(x(0) + x(1) + 2 * x(2));
  prog->AddLinearConstraint(x(0) <= y(0));
  prog->AddLinearConstraint(x(1) <= y(0) + y(1));
  prog->AddLinearConstraint(x(2) <= y(1));
  prog->AddLinearConstraint(x.cast<symbolic::Expression>().sum() == 1);
  prog->AddLinearConstraint(y(0) + y(1) == 1);
  prog->AddLinearConstraint(x(1) + 2 * x(2) == 1);
  prog->AddLinearConstraint(3 * x(0) + x(1) + 2 * x(2) == 2);
  prog->AddBoundingBoxConstraint(0, 1, x);
  return prog;
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

void TestProgSolve(const MixedIntegerBranchAndBoundNode& node,
                   const Eigen::Ref<const VectorXDecisionVariable>& x,
                   const Eigen::Ref<const Eigen::VectorXd>& x_expected,
                   double optimal_cost, double tol = 1E-4) {
  const SolutionResult result = node.solution_result();
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(node.prog()->GetSolution(x), x_expected, tol,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(node.prog()->GetOptimalCost(), optimal_cost, tol);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot1) {
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
  TestProgSolve(*root, x, x_expected, 1.5, 1E-5);
  EXPECT_TRUE(root->optimal_solution_is_integral());
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot2) {
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
  TestProgSolve(*root, x, x_expected, -4.9, 1E-5);
  EXPECT_FALSE(root->optimal_solution_is_integral());
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestConstructRoot3) {
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
  TestProgSolve(*root, x, x_expected, 4.0 / 3.0, 1E-5);
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
  TestProgSolve(*(root->left_child()), x, x_expected_l0, 1.5, 1E-5);
  TestProgSolve(*(root->right_child()), x, x_expected_r0, 4, 1E-5);
  EXPECT_TRUE(root->left_child()->optimal_solution_is_integral());
  EXPECT_TRUE(root->right_child()->optimal_solution_is_integral());

  // Branch on variable x(2). The child nodes created by branching on x(0) will
  // be deleted.
  root->Branch(x(2));
  const Eigen::Vector4d x_expected_l2(0, 1, 0, 0.5);
  const Eigen::Vector4d x_expected_r2(0, 4.1, 1, -1.05);
  TestProgSolve(*(root->left_child()), x, x_expected_l2, 1.5, 1E-5);
  TestProgSolve(*(root->right_child()), x, x_expected_r2, 12.35, 1E-5);
  EXPECT_TRUE(root->left_child()->optimal_solution_is_integral());
  EXPECT_TRUE(root->right_child()->optimal_solution_is_integral());

  // Branch on variable x(1) and x(3). Since these two variables are continuous,
  // expect a runtime error thrown.
  EXPECT_THROW(root->Branch(x(1)), std::runtime_error);
  EXPECT_THROW(root->Branch(x(3)), std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestBranch2) {
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
  TestProgSolve(*(root->right_child()), x, x_expected_r, -13.0 / 3.0, 1E-5);
  EXPECT_TRUE(root->right_child()->optimal_solution_is_integral());

  // Branch on x(2)
  root->Branch(x(2));
  Eigen::Matrix<double, 5, 1> x_expected_l;
  x_expected_l << 1, 2.0 / 3.0, 0, 1, 1;
  TestProgSolve(*(root->left_child()), x, x_expected_l, 23.0 / 6.0, 1E-5);
  EXPECT_TRUE(root->left_child()->optimal_solution_is_integral());
  x_expected_r << 0.7, 1, 1, 1.4, 0;
  TestProgSolve(*(root->right_child()), x, x_expected_r, -4.9, 1E-5);
  EXPECT_FALSE(root->right_child()->optimal_solution_is_integral());

  // Branch on x(4)
  root->Branch(x(4));
  x_expected_l << 0.7, 1, 1, 1.4, 0;
  x_expected_r << 0.2, 2.0 / 3.0, 1, 1.4, 1;
  TestProgSolve(*(root->left_child()), x, x_expected_l, -4.9, 1E-5);
  TestProgSolve(*(root->right_child()), x, x_expected_r, -47.0 / 30, 1E-5);
  EXPECT_FALSE(root->left_child()->optimal_solution_is_integral());
  EXPECT_FALSE(root->right_child()->optimal_solution_is_integral());

  // x(1) and x(3) are continuous variables, expect runtime_error thrown when we
  // branch on them.
  EXPECT_THROW(root->Branch(x(1)), std::runtime_error);
  EXPECT_THROW(root->Branch(x(3)), std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundNodeTest, TestBranch3) {
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
  auto prog = ConstructMathematicalProgram1();

  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  const VectorDecisionVariable<4> prog_x = prog->decision_variables();

  const VectorDecisionVariable<4> bnb_x =
      bnb.root()->prog()->decision_variables();

  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(bnb_x(i).equal_to(bnb.GetNewVariable(prog_x(i))));
  }

  static_assert(std::is_same<decltype(bnb.GetNewVariables(prog_x.head<2>())),
                             VectorDecisionVariable<2>>::value,
                "Should return VectorDecisionVariable<2> object.\n");
  static_assert(std::is_same<decltype(bnb.GetNewVariables(prog_x.head(2))),
                             VectorXDecisionVariable>::value,
                "Should return VectorXDecisionVariable object.\n");
  MatrixDecisionVariable<2, 2> X;
  X << prog_x(0), prog_x(1), prog_x(2), prog_x(3);
  static_assert(std::is_same<decltype(bnb.GetNewVariables(X)),
                             MatrixDecisionVariable<2, 2>>::value,
                "Should return MatrixDecisionVariable<2, 2> object.\n");
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestConstructor1) {
  auto prog = ConstructMathematicalProgram1();
  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  // The root node of the tree has integral optimal cost (0, 1, 0, 0.5), with
  // cost 1.5.
  const double tol = 1E-3;
  EXPECT_NEAR(bnb.best_upper_bound(), 1.5, tol);
  EXPECT_NEAR(bnb.best_lower_bound(), 1.5, tol);
  const auto& best_solutions = bnb.best_solutions();
  EXPECT_EQ(best_solutions.size(), 1);
  EXPECT_NEAR(best_solutions.front().first, 1.5, tol);
  const Eigen::Vector4d x_expected(0, 1, 0, 0.5);
  EXPECT_TRUE(CompareMatrices(best_solutions.front().second, x_expected, tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(bnb.IsLeafNodeFathomed(*(bnb.root())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestConstructor2) {
  auto prog = ConstructMathematicalProgram2();
  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  // The root node does not have an optimal integral solution.
  const double tol{1E-3};
  EXPECT_NEAR(bnb.best_lower_bound(), -4.9, tol);
  EXPECT_EQ(bnb.best_upper_bound(), std::numeric_limits<double>::infinity());
  EXPECT_TRUE(bnb.best_solutions().empty());
  EXPECT_FALSE(bnb.IsLeafNodeFathomed(*(bnb.root())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestConstructor3) {
  auto prog = ConstructMathematicalProgram3();
  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  // The root node is unbounded.
  EXPECT_EQ(bnb.best_lower_bound(), -std::numeric_limits<double>::infinity());
  EXPECT_EQ(bnb.best_upper_bound(), std::numeric_limits<double>::infinity());
  EXPECT_TRUE(bnb.best_solutions().empty());
  EXPECT_FALSE(bnb.IsLeafNodeFathomed(*(bnb.root())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestConstructor4) {
  auto prog = ConstructMathematicalProgram4();
  MixedIntegerBranchAndBound bnb(*prog, GurobiSolver::id());
  const double tol{1E-3};
  EXPECT_NEAR(bnb.best_lower_bound(), 4.0 / 3.0, tol);
  EXPECT_EQ(bnb.best_upper_bound(), std::numeric_limits<double>::infinity());
  EXPECT_TRUE(bnb.best_solutions().empty());
  EXPECT_FALSE(bnb.IsLeafNodeFathomed(*(bnb.root())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestLeafNodeFathomed1) {
  auto prog1 = ConstructMathematicalProgram1();
  MixedIntegerBranchAndBoundTester dut(*prog1, GurobiSolver::id());
  // The optimal solution to the root is integral. The node is fathomed.
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())));
  VectorDecisionVariable<4> x = dut.bnb()->root()->prog()->decision_variables();

  dut.bnb()->root()->Branch(x(0));
  // Both left and right children have integral optimal solution. Both nodes are
  // fathomed.
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.bnb()->root()->Branch(x(2));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  // The root node is not a leaf node. Expect a runtime error.
  EXPECT_THROW(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())),
               std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestLeafNodeFathomed2) {
  auto prog2 = ConstructMathematicalProgram2();
  MixedIntegerBranchAndBoundTester dut(*prog2, GurobiSolver::id());
  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  // The optimal solution to the root is not integral, the node is not fathomed.
  // The optimal cost is -4.9.
  EXPECT_FALSE(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())));

  dut.bnb()->root()->Branch(x(0));
  // The left node is infeasible, thus fathomed.
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  // The right node has integral optimal solution, thus fathomed.
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.bnb()->root()->Branch(x(2));
  // The solution to the left node is integral, with optimal cost 23.0 / 6.0.
  // The node is fathomed.
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  // The solution ot the right node is not integral, with optimal cost -4.9. The
  // node is not fathomed.
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.bnb()->root()->Branch(x(4));
  // Neither the left nor the right child nodes has integral solution.
  // Neither of the nodes is fathomed.
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestLeafNodeFathomed3) {
  auto prog = ConstructMathematicalProgram3();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  const VectorDecisionVariable<4> x =
      dut.bnb()->root()->prog()->decision_variables();

  EXPECT_FALSE(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())));

  dut.bnb()->root()->Branch(x(0));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.bnb()->root()->left_child()->Branch(x(2));
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->left_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->right_child())));

  dut.bnb()->root()->right_child()->Branch(x(2));
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
  auto prog = ConstructMathematicalProgram4();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  const VectorDecisionVariable<5> x =
      dut.bnb()->root()->prog()->decision_variables();

  EXPECT_FALSE(dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root())));

  dut.bnb()->root()->Branch(x(3));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));
  EXPECT_TRUE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->right_child())));

  dut.bnb()->root()->Branch(x(4));
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
  dut.bnb()->SetPickBranchingNodeMethod(
      MixedIntegerBranchAndBound::PickNode::kMinLowerBound);
  // Pick the root node.
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root());

  // The left node has optimal cost 23.0 / 6.0, the right node has optimal cost
  // -4.9
  // Also the left node is fathomed.
  dut.bnb()->root()->Branch(x(2));
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->right_child());

  dut.bnb()->root()->Branch(x(4));
  // The left node has optimal cost -4.9, the right node has optimal cost -47.0
  // / 30
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->left_child());
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestPickBranchingNode2) {
  // Test choosing the deepest non-fathomed leaf node.
  auto prog = ConstructMathematicalProgram2();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  dut.bnb()->SetPickBranchingNodeMethod(
      MixedIntegerBranchAndBound::PickNode::kDepthFirst);
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root());

  dut.bnb()->root()->Branch(x(2));
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->right_child());

  dut.bnb()->root()->Branch(x(4));
  EXPECT_TRUE(dut.PickBranchingNode() == dut.bnb()->root()->left_child() ||
              dut.PickBranchingNode() == dut.bnb()->root()->right_child());

  dut.bnb()->root()->left_child()->Branch(x(0));
  // root->left->left is infeasible, root->left->right has integral solution.
  // The only non-fathomed leaf node is root->right
  EXPECT_EQ(dut.PickBranchingNode(), dut.bnb()->root()->right_child());
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestPickBranchingVariable1) {
  auto prog = ConstructMathematicalProgram2();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  // The optimal solution at the root is (0.7, 1, 1, 1.4, 0)
  dut.bnb()->SetPickBranchingVariableMethod(
      MixedIntegerBranchAndBound::PickVariable::kMostAmbivalent);
  EXPECT_TRUE(dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(0)));
  dut.bnb()->SetPickBranchingVariableMethod(
      MixedIntegerBranchAndBound::PickVariable::kLeastAmbivalent);
  EXPECT_TRUE(dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(2)) ||
              dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(4)));

  dut.BranchAndUpdate(dut.bnb()->root(), x(0));
  // The optimization at root->left is infeasible.
  dut.bnb()->SetPickBranchingVariableMethod(
      MixedIntegerBranchAndBound::PickVariable::kMostAmbivalent);
  EXPECT_THROW(dut.PickBranchingVariable(*(dut.bnb()->root()->left_child())),
               std::runtime_error);
  dut.bnb()->SetPickBranchingVariableMethod(
      MixedIntegerBranchAndBound::PickVariable::kLeastAmbivalent);
  EXPECT_THROW(dut.PickBranchingVariable(*(dut.bnb()->root()->left_child())),
               std::runtime_error);
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestPickBranchingVariable2) {
  auto prog = ConstructMathematicalProgram3();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  VectorDecisionVariable<4> x = dut.bnb()->root()->prog()->decision_variables();

  // The problem is unbounded, any branching variable is acceptable.
  dut.bnb()->SetPickBranchingVariableMethod(
      MixedIntegerBranchAndBound::PickVariable::kMostAmbivalent);
  EXPECT_TRUE(dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(0)) ||
              dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(2)));
  dut.bnb()->SetPickBranchingVariableMethod(
      MixedIntegerBranchAndBound::PickVariable::kLeastAmbivalent);
  EXPECT_TRUE(dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(0)) ||
              dut.PickBranchingVariable(*(dut.bnb()->root()))->equal_to(x(2)));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestBranchAndUpdate2) {
  auto prog = ConstructMathematicalProgram2();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  EXPECT_TRUE(dut.bnb()->best_solutions().empty());
  const double tol = 1E-3;
  EXPECT_NEAR(dut.bnb()->best_lower_bound(), -4.9, tol);
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());

  VectorDecisionVariable<5> x = dut.bnb()->root()->prog()->decision_variables();

  dut.BranchAndUpdate(dut.bnb()->root(), x(2));
  // The left node has optimal cost 23.0 / 6.0. with integral solution (1, 2 /
  // 3, 0, 1, 1,)
  // The right node has optimal cost -4.9, with non-integral solution (0.7, 1,
  // 1, 1.4, 0).
  EXPECT_NEAR(dut.bnb()->best_lower_bound(), -4.9, tol);
  EXPECT_NEAR(dut.bnb()->best_upper_bound(), 23.0 / 6.0, tol);
  EXPECT_EQ(dut.bnb()->best_solutions().size(), 1);
  EXPECT_NEAR(dut.bnb()->best_solutions().front().first, 23.0 / 6.0, tol);
  Eigen::Matrix<double, 5, 1> x_expected;
  x_expected << 1, 2.0 / 3.0, 0, 1, 1;
  EXPECT_TRUE(CompareMatrices(dut.bnb()->best_solutions().front().second,
                              x_expected, tol, MatrixCompareType::absolute));

  dut.BranchAndUpdate(dut.bnb()->root(), x(4));
  // The left node has optimal cost -4.9, with non-integral solution (0.7, 1, 1,
  // 1.4, 0).
  // The right node has optimal cost -47/30, with  non-integral solution (0.2,
  // 2/3, 1, 1.4, 1).
  EXPECT_NEAR(dut.bnb()->best_lower_bound(), -4.9, tol);
  EXPECT_NEAR(dut.bnb()->best_upper_bound(), 23.0 / 6.0, tol);
  EXPECT_EQ(dut.bnb()->best_solutions().size(), 1);
  EXPECT_NEAR(dut.bnb()->best_solutions().front().first, 23.0 / 6.0, tol);
  EXPECT_TRUE(CompareMatrices(dut.bnb()->best_solutions().front().second,
                              x_expected, tol, MatrixCompareType::absolute));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestBranchAndUpdate3) {
  auto prog = ConstructMathematicalProgram3();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  const VectorDecisionVariable<4> x =
      dut.bnb()->root()->prog()->decision_variables();

  dut.BranchAndUpdate(dut.bnb()->root(), x(0));
  // Both left and right childs are unbounded.
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(dut.bnb()->best_solutions().empty());
  EXPECT_FALSE(
      dut.bnb()->IsLeafNodeFathomed(*(dut.bnb()->root()->left_child())));

  dut.BranchAndUpdate(dut.bnb()->root()->left_child(), x(2));
  // Both root->l->l and root->l->r are unbounded.
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(dut.bnb()->best_solutions().empty());
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->left_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->left_child()->right_child())));

  dut.BranchAndUpdate(dut.bnb()->root()->right_child(), x(2));
  // Both root->r->l and root->r->r are unbounded.
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(dut.bnb()->best_solutions().empty());
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->right_child()->left_child())));
  EXPECT_TRUE(dut.bnb()->IsLeafNodeFathomed(
      *(dut.bnb()->root()->right_child()->right_child())));
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestBranchAndUpdate4) {
  auto prog = ConstructMathematicalProgram4();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  const VectorDecisionVariable<5> x =
      dut.bnb()->root()->prog()->decision_variables();

  dut.BranchAndUpdate(dut.bnb()->root(), x(3));
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            std::numeric_limits<double>::infinity());

  dut.BranchAndUpdate(dut.bnb()->root(), x(4));
  EXPECT_EQ(dut.bnb()->best_upper_bound(),
            std::numeric_limits<double>::infinity());
  EXPECT_EQ(dut.bnb()->best_lower_bound(),
            std::numeric_limits<double>::infinity());
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestSolve1) {
  auto prog = ConstructMathematicalProgram1();
  const VectorDecisionVariable<4> x = prog->decision_variables();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());

  const SolutionResult solution_result = dut.bnb()->Solve();
  EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
  const Eigen::Vector4d x_expected(0, 1, 0, 0.5);
  const double tol{1E-3};
  // The root node finds an optimal integral solution, and the bnb terminates.
  EXPECT_EQ(dut.bnb()->best_solutions().size(), 1);
  EXPECT_NEAR(dut.bnb()->GetOptimalCost(0), 1.5, tol);
  EXPECT_TRUE(CompareMatrices(dut.bnb()->GetSolution(x, 0), x_expected, tol,
                              MatrixCompareType::absolute));
}

std::vector<MixedIntegerBranchAndBound::PickNode>
NonUserDefinedPickNodeMethods() {
  return {MixedIntegerBranchAndBound::PickNode::kDepthFirst,
          MixedIntegerBranchAndBound::PickNode::kMinLowerBound};
}

std::vector<MixedIntegerBranchAndBound::PickVariable>
NonUserDefinedPickVariableMethods() {
  return {MixedIntegerBranchAndBound::PickVariable::kMostAmbivalent,
          MixedIntegerBranchAndBound::PickVariable::kLeastAmbivalent};
}

GTEST_TEST(MixedIntegerBranchAndBoundTest, TestSolve2) {
  auto prog = ConstructMathematicalProgram2();
  const VectorDecisionVariable<5> x = prog->decision_variables();

  MixedIntegerBranchAndBoundTester dut(*prog, GurobiSolver::id());
  for (auto pick_variable : NonUserDefinedPickVariableMethods()) {
    for (auto pick_node : NonUserDefinedPickNodeMethods()) {
      dut.bnb()->SetPickBranchingNodeMethod(pick_node);
      dut.bnb()->SetPickBranchingVariableMethod(pick_variable);

      const SolutionResult solution_result = dut.bnb()->Solve();
      EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
      const double tol{1E-3};
      EXPECT_NEAR(dut.bnb()->GetOptimalCost(0), -13.0 / 3, tol);
      Eigen::Matrix<double, 5, 1> x_expected0;
      x_expected0 << 1, 1.0 / 3.0, 1, 1, 0;
      EXPECT_TRUE(CompareMatrices(dut.bnb()->GetSolution(x, 0), x_expected0,
                                  tol, MatrixCompareType::absolute));
      // The costs are in the ascending order.
      for (int i = 1; i < dut.bnb()->best_solutions().size(); ++i) {
        EXPECT_GE(dut.bnb()->GetOptimalCost(i),
                  dut.bnb()->GetOptimalCost(i - 1));
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
      dut.bnb()->SetPickBranchingNodeMethod(pick_node);
      dut.bnb()->SetPickBranchingVariableMethod(pick_variable);

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
      dut.bnb()->SetPickBranchingNodeMethod(pick_node);
      dut.bnb()->SetPickBranchingVariableMethod(pick_variable);

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
  //  min 1750 * b(0) + 990 * b(1) + 1240 * b(2) + 1680 * b(3) + 550 * x(0) +
  //  450 * x(1) + 400 * x(2) + 100 * x(3)
  //  s.t 5 * b(0) + 3 * b(1) + 4 * b(2) + 6 * b(3) + x(0) + x(1) + x(2) + x(3)
  //  = 25
  //      0.25 * b(0) + 0.12 * b(1) + 0.2 * b(2) + 0.18 * b(3) + 0.08 * x(0) +
  //      0.07 * x(1) + 0.06 * x(2) + 0.03 * x(3) = 1.25
  //      0.15 * b(0) + 0.09 * b(1) + 0.16 * b(2) + 0.24 * b(3) + 0.06 * x(0) +
  //      0.07 * x(1) + 0.08 * x(2) + 0.09 * x(3) = 1.25
  //      x >= 0
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
      bnb.SetPickBranchingNodeMethod(pick_node);
      bnb.SetPickBranchingVariableMethod(pick_variable);
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
      for (int i = 1; i < bnb.best_solutions().size(); ++i) {
        EXPECT_GE(bnb.GetOptimalCost(i), bnb.GetOptimalCost(i - 1));
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
  EXPECT_LE(bnb.best_solutions().size(), all_integral_solutions.size());
  EXPECT_NEAR(bnb.GetOptimalCost(), all_integral_solutions[0].first, tol);
  EXPECT_TRUE(CompareMatrices(bnb.GetSolution(x),
                              all_integral_solutions[0].second, tol));
  for (int i = 1; i < static_cast<int>(bnb.best_solutions().size()); ++i) {
    const double cost_i = bnb.GetOptimalCost(i);
    EXPECT_GE(cost_i, bnb.GetOptimalCost(i - 1) - tol);
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
  // min x(1) + 2 * x(2) + 3 * x(3) + 4 * x(4)
  // s.t x(0) <= y(0)
  //     x(1) <= y(0) + y(1)
  //     x(2) <= y(1) + y(2)
  //     x(3) <= y(2) + y(3)
  //     x(4) <= y(3)
  //     y(0) + y(1) + y(2) + y(3) = 1
  //     x(0) + x(1) + x(2) + x(3) + x(4) = 1
  //     3*x(0) + x(1) + 2*x(2) + 5*x(3) + x(4) = 4
  //     x >= 0
  //     y are binary variables.
  // The optimal solution is x = (0, 0, 1/3, 2/3, 0), y = (0, 0, 1, 0), with
  // optimal cost 8/3.
  // There are other integral solutions,
  // x = (0, 0, 0, 0.25, 75), y = (0, 0, 0, 1), with cost 13/4
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<5>("x");
  auto y = prog.NewBinaryVariables<4>("y");
  prog.AddLinearCost(x(1) + 2 * x(2) + 3 * x(3) + 4 * x(4));
  prog.AddLinearConstraint(x(0) <= y(0));
  prog.AddLinearConstraint(x(1) <= y(0) + y(1));
  prog.AddLinearConstraint(x(2) <= y(1) + y(2));
  prog.AddLinearConstraint(x(3) <= y(2) + y(3));
  prog.AddLinearConstraint(x(4) <= y(3));
  prog.AddLinearConstraint(y.cast<symbolic::Expression>().sum() == 1);
  prog.AddLinearConstraint(x.cast<symbolic::Expression>().sum() == 1);
  prog.AddLinearConstraint(3 * x(0) + x(1) + 2 * x(2) + 5 * x(3) + x(4) == 4);
  prog.AddBoundingBoxConstraint(0, 1, x);

  MixedIntegerBranchAndBound bnb(prog, GurobiSolver::id());
  for (auto pick_variable :
       {MixedIntegerBranchAndBound::PickVariable::kMostAmbivalent,
        MixedIntegerBranchAndBound::PickVariable::kLeastAmbivalent}) {
    for (auto pick_node :
         {MixedIntegerBranchAndBound::PickNode::kDepthFirst,
          MixedIntegerBranchAndBound::PickNode::kMinLowerBound}) {
      bnb.SetPickBranchingNodeMethod(pick_node);
      bnb.SetPickBranchingVariableMethod(pick_variable);
      SolutionResult solution_result = bnb.Solve();
      EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
      std::vector<std::pair<double, Eigen::VectorXd>> best_solutions;
      Eigen::Matrix<double, 9, 1> xy_expected;
      xy_expected << 0, 0, 1.0/3, 2.0/3, 0, 0, 0, 1, 0;
      best_solutions.emplace_back(8.0/3, xy_expected);
      xy_expected << 0, 0, 0, 0.25, 75, 0, 0, 0, 1;
      best_solutions.emplace_back(13.0 / 4, xy_expected);

      const double tol{1E-3};
      VectorDecisionVariable<9> xy;
      xy << x, y;
      CheckAllIntegralSolution(bnb, xy, best_solutions, tol);
    }
  }
}
}  // namespace
}  // namespace solvers
}  // namespace drake
