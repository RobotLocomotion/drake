#include "drake/solvers/get_infeasible_constraints.h"

#include <regex>

#include <gtest/gtest.h>

#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(SolveTest, GetInfeasibleConstraints) {
  if (SnoptSolver::is_available()) {
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<1>();
    auto b0 = prog.AddBoundingBoxConstraint(0, 0, x);
    auto b1 = prog.AddBoundingBoxConstraint(1, 1, x);

    SnoptSolver solver;
    MathematicalProgramResult result = solver.Solve(prog, {}, {});
    EXPECT_FALSE(result.is_success());

    std::vector<std::string> infeasible =
        GetInfeasibleConstraints(prog, result);
    EXPECT_EQ(infeasible.size(), 1);

    // If no description is set, we should see the NiceTypeName of the
    // Constraint.
    auto matcher = [](const std::string& s, const std::string& re) {
      return std::regex_match(s, std::regex(re));
    };
    EXPECT_PRED2(matcher, infeasible[0],
                 "drake::solvers::BoundingBoxConstraint.*");

    // If a description for the constraint has been set, then that description
    // should be returned instead. There is no reason a priori for b0 or b1 to
    // be the infeasible one, so set both descriptions.
    b0.evaluator()->set_description("Test");
    b1.evaluator()->set_description("Test");
    infeasible = GetInfeasibleConstraints(prog, result);
    EXPECT_PRED2(matcher, infeasible[0], "Test.*");
  }
}

GTEST_TEST(TestGetInfeasibleConstraintBindings, Test) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();
  auto constraint1 = prog.AddBoundingBoxConstraint(0, 0, x);
  auto constraint2 = prog.AddBoundingBoxConstraint(1, 1, x);
  SnoptSolver solver;
  if (solver.is_available()) {
    const auto result = solver.Solve(prog);
    EXPECT_FALSE(result.is_success());
    const std::vector<Binding<Constraint>> infeasible_bindings =
        GetInfeasibleConstraintBindings(prog, result);
    const std::unordered_set<Binding<Constraint>> infeasible_bindings_set(
        infeasible_bindings.begin(), infeasible_bindings.end());
    const double x_val = result.GetSolution(x)(0);
    EXPECT_FALSE(infeasible_bindings_set.empty());
    if (std::abs(x_val) > 1e-4) {
      EXPECT_GT(infeasible_bindings_set.count(constraint1), 0);
    }
    if (std::abs(x_val - 1) > 1e-4) {
      EXPECT_GT(infeasible_bindings_set.count(constraint2), 0);
    }
    // If I relax the tolerance, then GetInfeasibleConstraintBindings returns an
    // empty vector.
    const std::vector<Binding<Constraint>> infeasible_bindings_relaxed =
        GetInfeasibleConstraintBindings(prog, result, 2);
    EXPECT_EQ(infeasible_bindings_relaxed.size(), 0);
  }
}
}  // namespace
}  // namespace solvers
}  // namespace drake
