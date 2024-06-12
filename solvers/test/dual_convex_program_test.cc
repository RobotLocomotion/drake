#include "drake/solvers/dual_convex_program.h"

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"
#include "drake/solvers/test/semidefinite_program_examples.h"
#include "drake/solvers/test/sos_examples.h"

namespace drake {
namespace solvers {
namespace test {

namespace {
const double kInf = std::numeric_limits<double>::infinity();
void CheckPrimalDualSolution(
    const MathematicalProgram& primal_prog,
    const MathematicalProgram& dual_prog,
    const std::unordered_map<Binding<Constraint>,
                             MatrixX<symbolic::Expression>>&
        constraint_to_dual_variable_map) {
  // Choose a solver which outputs conic dual variables. Gurobi does not for
  // Socp so it is excluded from this list.
  std::vector<SolverId> conic_solvers;
  if (MosekSolver::is_enabled() && MosekSolver::is_available()) {
    conic_solvers.push_back(MosekSolver::id());
  }
  if (ssize(primal_prog.positive_semidefinite_constraints()) == 0 &&
      ssize(primal_prog.linear_matrix_inequality_constraints()) == 0) {
    conic_solvers.push_back(ClarabelSolver::id());
  }
  // If no solvers are available to run this test, skip the test.
  if (ssize(conic_solvers) == 0) {
    return;
  }
  auto solver = MakeFirstAvailableSolver(conic_solvers);

  // We need relatively loose tolerance for these tests as both the primal and
  // the dual will typically solve only to a precision of 1e-8.
  const double kTol = 1e-6;
  MathematicalProgramResult primal_result;
  MathematicalProgramResult dual_result;
  solver->Solve(primal_prog, std::nullopt, std::nullopt, &primal_result);
  solver->Solve(dual_prog, std::nullopt, std::nullopt, &dual_result);

  if (primal_result.get_solution_result() == SolutionResult::kSolutionFound) {
    EXPECT_EQ(dual_result.get_solution_result(),
              SolutionResult::kSolutionFound);
    // By strong duality the primal and dual should have equal optimal costs. We
    // need to negate the dual result since it is a maximization problem.
    EXPECT_NEAR(primal_result.get_optimal_cost(),
                -dual_result.get_optimal_cost(), kTol);

    // We need the aggregated matrices A and b to check the complementarity gap.
    internal::ConvexConstraintAggregationInfo info;
    internal::ConvexConstraintAggregationOptions aggregation_options;
    aggregation_options.cast_rotated_lorentz_to_lorentz = false;
    aggregation_options.preserve_psd_inner_product_vectorization = false;
    aggregation_options.parse_psd_using_upper_triangular = false;
    internal::DoAggregateConvexConstraints(primal_prog, aggregation_options,
                                           &info);
    Eigen::SparseMatrix<double> A(info.A_row_count,
                                  primal_prog.decision_variables().size());
    A.setFromTriplets(info.A_triplets.begin(), info.A_triplets.end());
    Eigen::VectorXd b =
        Eigen::Map<Eigen::VectorXd>(info.b_std.data(), info.b_std.size());
    const double complementarity_gap = dual_result.get_x_val().transpose() *
                                       (-A * primal_result.get_x_val() + b);

    // Check that the dual variables are the same as the ones we get from
    // querying the solvers. Since the dual solution may not be unique (i.e.
    // there are repeated constraints in the primal formulation or the primal
    // does not have a unique solution), this section of code should only be
    // expected to succeed when the dual solution is unique.
    EXPECT_TRUE(complementarity_gap < kTol);
    // We should only expect the dual variables from the solver on the primal
    // and the solver on the dual to agree up to the square root of the
    // precision that the two were solved. The complementarity gap captures this
    // precision approximately.
    const double variable_kTol = std::sqrt(complementarity_gap);
    for (const auto& binding : primal_prog.GetAllConstraints()) {
      Eigen::MatrixXd interface_dual_vars =
          primal_result.GetDualSolution(binding);
      if (const auto* l3c = dynamic_cast<const PositiveSemidefiniteConstraint*>(
              binding.evaluator().get())) {
        unused(l3c);
        interface_dual_vars = math::ToSymmetricMatrixFromLowerTriangularColumns(
            interface_dual_vars);
      }

      const MatrixX<symbolic::Expression> manual_dual_vars_expr =
          dual_result.GetSolution(constraint_to_dual_variable_map.at(binding));

      Eigen::MatrixXd manual_dual_vars =
          manual_dual_vars_expr.unaryExpr([](const symbolic::Expression& e) {
            return e.Evaluate();
          });

      // If this constraint is a linear equality, we need to flip the sign
      // convention of the dual variable. Drake uses the shadow price, while
      // CreateDualConvexProgram uses the conic standard form. See
      // SetDualSolution of scs_clarabel_common.cc for details.
      if (const auto* l1c = dynamic_cast<const LinearEqualityConstraint*>(
              binding.evaluator().get())) {
        unused(l1c);
        manual_dual_vars = -manual_dual_vars;
      }
      EXPECT_TRUE(CompareMatrices(interface_dual_vars, manual_dual_vars,
                                  variable_kTol, MatrixCompareType::relative));
    }

  } else if (primal_result.get_solution_result() ==
             SolutionResult::kInfeasibleConstraints) {
    // Primal infeasibility implies the dual is unbounded (or in a bad case
    // infeasible).
    EXPECT_TRUE(
        dual_result.get_solution_result() == SolutionResult::kUnbounded ||
        dual_result.get_solution_result() ==
            SolutionResult::kInfeasibleOrUnbounded ||
        // The dual of the dual is the primal
        dual_result.get_solution_result() == SolutionResult::kDualInfeasible);
  } else if (primal_result.get_solution_result() == kUnbounded ||
             primal_result.get_solution_result() == kDualInfeasible) {
    // Primal unboundedness implies the dual being infeasible.
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

TEST_P(LinearProgramTest, TestLP) {
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  const MathematicalProgram& primal_prog = *prob()->prog();
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

// We exclude kLinearFeasibilityProgram and kLinearProgram2 due to their duals
// being non-unique.
INSTANTIATE_TEST_SUITE_P(
    DualConvexProgramTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(std::vector<LinearProblems>{
                           LinearProblems::kLinearProgram0,
                           LinearProblems::kLinearProgram1,
                           LinearProblems::kLinearProgram3})));

TEST_F(InfeasibleLinearProgramTest0, TestInfeasible) {
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  const MathematicalProgram& primal_prog = *prog_;
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

TEST_F(UnboundedLinearProgramTest0, TestUnbounded) {
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  const MathematicalProgram& primal_prog = *prog_;
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

TEST_P(TestEllipsoidsSeparation, TestSOCP) {
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  const MathematicalProgram& primal_prog = prog_;
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);

  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

INSTANTIATE_TEST_SUITE_P(
    DualConvexProgramTest, TestEllipsoidsSeparation,
    ::testing::ValuesIn(GetEllipsoidsSeparationProblems()));

TEST_P(TestQPasSOCP, TestSOCP) {
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  const MathematicalProgram& primal_prog = prog_socp_;
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

INSTANTIATE_TEST_SUITE_P(DualConvexProgramTest, TestQPasSOCP,
                         ::testing::ValuesIn(GetQPasSOCPProblems()));

TEST_P(TestFindSpringEquilibrium, TestSOCP) {
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  const MathematicalProgram& primal_prog = prog_;
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

INSTANTIATE_TEST_SUITE_P(
    DualConvexProgramTest, TestFindSpringEquilibrium,
    ::testing::ValuesIn(GetFindSpringEquilibriumProblems()));

GTEST_TEST(TestSos, SimpleSos1) {
  SimpleSos1 dut;
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  const MathematicalProgram& primal_prog = dut.prog();
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

GTEST_TEST(TestSos, UnivariateNonnegative1) {
  UnivariateNonnegative1 dut;
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  const MathematicalProgram& primal_prog = dut.prog();
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

GTEST_TEST(TestSdp, TestTrivialSDP) {
  // TODO(Alexandre.Amice) get from semidefinite_program_example.h
  MathematicalProgram primal_prog;

  auto S = primal_prog.NewSymmetricContinuousVariables<2>("S");

  // S is p.s.d
  primal_prog.AddPositiveSemidefiniteConstraint(S);

  // S(1, 0) = 1
  primal_prog.AddBoundingBoxConstraint(1, 1, S(1, 0));

  // Min S.trace()
  primal_prog.AddLinearCost(S.cast<symbolic::Expression>().trace());
  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  auto dual_prog =
      CreateDualConvexProgram(primal_prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(primal_prog, *dual_prog,
                          constraint_to_dual_variable_map);
}

// This tests LMI constraints. Uncomment this once we can retrieve LMI dual
// variables.
// GTEST_TEST(TestSdp, SolveEigenvalueProblem) {
//  // TODO(Alexandre.Amice) get from semidefinite_program_example.h
//  MathematicalProgram prog;
//  auto x = prog.NewContinuousVariables<2>("x");
//  Eigen::Matrix3d F1;
//  // clang-format off
//  F1 << 1, 0.2, 0.3,
//      0.2, 2, -0.1,
//      0.3, -0.1, 4;
//   Eigen::Matrix3d F2;
//  F2 << 2, 0.4, 0.7,
//      0.4, -1, 0.1,
//      0.7, 0.1, 5;
//  // clang-format on
//  auto z = prog.NewContinuousVariables<1>("z");
//  prog.AddLinearMatrixInequalityConstraint(
//      {Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(), -F1, -F2}, {z,
//      x});
//
//  const Eigen::Vector2d x_lb(0.1, 1);
//  const Eigen::Vector2d x_ub(2, 3);
//  prog.AddBoundingBoxConstraint(x_lb, x_ub, x);
//
//  prog.AddLinearCost(z(0));
//
//  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
//      constraint_to_dual_variable_map;
//  auto dual_prog =
//      CreateDualConvexProgram(prog, &constraint_to_dual_variable_map);
//  CheckPrimalDualSolution(prog, *dual_prog, constraint_to_dual_variable_map);
//}

GTEST_TEST(TestSdp, SolveSDPwithSecondOrderConeExample1) {
  // TODO(Alexandre.Amice) get from semidefinite_program_example.h
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>();
  auto x = prog.NewContinuousVariables<3>();
  Eigen::Matrix3d C0;
  // clang-format off
  C0 << 2, 1, 0,
        1, 2, 1,
        0, 1, 2;
  // clang-format on
  prog.AddLinearCost((C0 * X.cast<symbolic::Expression>()).trace() + x(0));
  prog.AddLinearConstraint(
      (Eigen::Matrix3d::Identity() * X.cast<symbolic::Expression>()).trace() +
          x(0) ==
      1);
  prog.AddLinearConstraint(
      (Eigen::Matrix3d::Ones() * X.cast<symbolic::Expression>()).trace() +
          x(1) + x(2) ==
      0.5);
  prog.AddPositiveSemidefiniteConstraint(X);
  prog.AddLorentzConeConstraint(x.cast<symbolic::Expression>());

  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  auto dual_prog =
      CreateDualConvexProgram(prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(prog, *dual_prog, constraint_to_dual_variable_map);
}

GTEST_TEST(TestSdp, SolveSDPwithSecondOrderConeExample2) {
  // TODO(Alexandre.Amice) get from semidefinite_program_example.h
  MathematicalProgram prog;
  const auto X = prog.NewSymmetricContinuousVariables<3>();
  const auto x = prog.NewContinuousVariables<1>()(0);
  prog.AddLinearCost(X(0, 0) + X(1, 1) + x);
  prog.AddBoundingBoxConstraint(0, kInf, x);
  prog.AddLinearConstraint(X(0, 0) + 2 * X(1, 1) + X(2, 2) + 3 * x == 3);
  Vector3<symbolic::Expression> lorentz_cone_expr;
  lorentz_cone_expr << X(0, 0), X(1, 1) + x, X(1, 1) + X(2, 2);
  prog.AddLorentzConeConstraint(lorentz_cone_expr);
  prog.AddLinearConstraint(X(1, 0) + X(2, 1) == 1);
  prog.AddPositiveSemidefiniteConstraint(X);

  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  auto dual_prog =
      CreateDualConvexProgram(prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(prog, *dual_prog, constraint_to_dual_variable_map);
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
