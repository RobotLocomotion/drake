#include "drake/solvers/dual_convex_program.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"
#include "drake/solvers/test/semidefinite_program_examples.h"
#include "drake/solvers/test/sos_examples.h"

namespace drake {
namespace solvers {
namespace test {

namespace {

void CheckPrimalDualSolution(
    const MathematicalProgram& primal_prog,
    const MathematicalProgram& dual_prog,
    const std::unordered_map<Binding<Constraint>,
                             MatrixX<symbolic::Expression>>&
        constraint_to_dual_variable_map) {
  auto primal_result = Solve(primal_prog);
  auto dual_result = Solve(dual_prog);
  if (primal_result.get_solution_result() == SolutionResult::kSolutionFound) {
    EXPECT_EQ(dual_result.get_solution_result(),
              SolutionResult::kSolutionFound);
    // By strong duality the primal and dual should have equal optimal costs. We
    // need to negate the dual result since it is a maximization problem.
    EXPECT_NEAR(primal_result.get_optimal_cost(),
                -dual_result.get_optimal_cost(), 1e-12);
    // Check the dual solution is the same one we get from querying the solvers.
    for (const auto& binding : primal_prog.GetAllConstraints()) {
      const Eigen::MatrixXd interface_dual_vars =
          primal_result.GetDualSolution(binding);
      const MatrixX<symbolic::Expression> manual_dual_vars_expr =
          dual_result.GetSolution(constraint_to_dual_variable_map.at(binding));
      const Eigen::MatrixXd manual_dual_vars =
          manual_dual_vars_expr.unaryExpr([](const symbolic::Expression& e) {
            return e.Evaluate();
          });

      EXPECT_TRUE(CompareMatrices(interface_dual_vars, manual_dual_vars, 1e-12,
                                  MatrixCompareType::absolute));
    }
  } else if (primal_result.get_solution_result() ==
             SolutionResult::kInfeasibleConstraints) {
    // Primal infeasibility implies the dual being unbounded (or in a bad case
    // infeasible).
    EXPECT_TRUE(
        dual_result.get_solution_result() == SolutionResult::kUnbounded ||
        dual_result.get_solution_result() ==
            SolutionResult::kInfeasibleOrUnbounded ||
        // The dual of the dual is the primal
        dual_result.get_solution_result() == SolutionResult::kDualInfeasible);
  } else if (primal_result.get_solution_result() == kUnbounded ||
             primal_result.get_solution_result() == kDualInfeasible) {
    // Primal unboundedness implies the dual being infeasibility.
    EXPECT_TRUE(dual_result.get_solution_result() ==
                    SolutionResult::kInfeasibleConstraints ||
                dual_result.get_solution_result() ==
                    SolutionResult::kInfeasibleOrUnbounded);
  } else if (primal_result.get_solution_result() ==
             SolutionResult::kInfeasibleOrUnbounded) {
    EXPECT_NE(dual_result.get_solution_result(),
              SolutionResult::kSolutionFound);
  }
}
}  // namespace

GTEST_TEST(DualConvexProgramTest, LinearFeasibilityProgramTest) {
  LinearFeasibilityProgram prog(ConstraintForm::kNonSymbolic);
  auto primal_result = Solve(*(prog.prog()));

  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  std::unique_ptr<MathematicalProgram> dual_prog =
      CreateDualConvexProgram(*(prog.prog()), &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(*(prog.prog()), *dual_prog,
                          constraint_to_dual_variable_map);
}

GTEST_TEST(DualConvexProgramTest, LinearProgram0Test) {
  LinearProgram0 prog(CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic);
  auto primal_result = Solve(*(prog.prog()));

  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  auto dual_prog =
      CreateDualConvexProgram(*(prog.prog()), &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(*(prog.prog()), *dual_prog,
                          constraint_to_dual_variable_map);
  //  std::cout << *dual_prog << std::endl;
  //  auto dual_result = Solve(*dual_prog);
  //
  //  EXPECT_TRUE(primal_result.is_success());
  //  EXPECT_TRUE(dual_result.is_success());
  //
  //  // There are 5 non-infinite linear constraints in this program so there
  //  should
  //  // be 5 dual variables.
  //  EXPECT_EQ(ssize(dual_prog->decision_variables()), 5);
  //
  //  EXPECT_NEAR(primal_result.get_optimal_cost(),
  //  -dual_result.get_optimal_cost(),
  //              1e-12);
}

TEST_P(LinearProgramTest, TestLP) {
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  const MathematicalProgram& primal_prog = *prob()->prog();
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

INSTANTIATE_TEST_SUITE_P(
    DualConvexProgramTest, LinearProgramTest,
    ::testing::Combine(
        ::testing::ValuesIn(linear_cost_form()),
        ::testing::ValuesIn(linear_constraint_form()),
        //                       ::testing::ValuesIn(linear_problems())
        ::testing::ValuesIn(std::vector<LinearProblems>{

        })));

}  // namespace test
}  // namespace solvers
}  // namespace drake
