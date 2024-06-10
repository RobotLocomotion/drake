#include "drake/solvers/dual_convex_program.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/gurobi_solver.h"
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
  std::cout << "PRIMAL" << std::endl;
  std::cout << primal_prog << std::endl;
  std::cout << "DUAL" << std::endl;
  std::cout << dual_prog << std::endl;
  //  ClarabelSolver solver;
  MosekSolver solver;
  MosekSolver mosek;
  //  GurobiSolver gurobi;
  // We need relatively loose tolerance for these tests as both the primal and
  // the dual will typically solve only to a precision of 1e-8.
  const double kTol = 1e-4;
  auto primal_result = solver.Solve(primal_prog);
  auto dual_result = solver.Solve(dual_prog);
  auto mosek_primal_result = mosek.Solve(primal_prog);
  auto mosek_dual_result = mosek.Solve(dual_prog);
  //  auto gurobi_primal_result = gurobi.Solve(primal_prog);
  //  auto gurobi_dual_result = gurobi.Solve(dual_prog);
  if (primal_result.get_solution_result() == SolutionResult::kSolutionFound) {
    EXPECT_EQ(dual_result.get_solution_result(),
              SolutionResult::kSolutionFound);
    // By strong duality the primal and dual should have equal optimal costs. We
    // need to negate the dual result since it is a maximization problem.
    EXPECT_NEAR(primal_result.get_optimal_cost(),
                -dual_result.get_optimal_cost(), kTol);

    //        double complementary =
    //            primal_result.get_x_val().transpose() *
    //            dual_result.get_x_val();

    // We need the aggregated matrices A and b to check the complementarity gap.
    internal::ConvexConstraintAggregationInfo info;
    internal::ConvexConstraintAggregationOptions aggregation_options;
    internal::DoAggregateConvexConstraints(primal_prog, aggregation_options,
                                           &info);
    Eigen::SparseMatrix<double> A(info.A_row_count,
                                  primal_prog.decision_variables().size());
    A.setFromTriplets(info.A_triplets.begin(), info.A_triplets.end());
    Eigen::VectorXd b =
        Eigen::Map<Eigen::VectorXd>(info.b_std.data(), info.b_std.size());
    const double complementarity_gap = dual_result.get_x_val().transpose() *
                                       (-A * primal_result.get_x_val() + b);
    std::cout << fmt::format("complementary gap = {}", complementarity_gap)
              << std::endl;
    //    double mosek_gap = mosek_dual_result.get_x_val().transpose() *
    //                       (-A * mosek_primal_result.get_x_val() + b);
    //    double gurobi_gap = gurobi_dual_result.get_x_val().transpose() *
    //                        (-A * gurobi_primal_result.get_x_val() + b);
    //     std::cout << fmt::format("Clarabel x={}\n",
    //                             fmt_eigen(primal_result.get_x_val().transpose()))
    //              << std::endl;
    //    std::cout << fmt::format(
    //                     "Mosek x={}\n",
    //                     fmt_eigen(mosek_primal_result.get_x_val().transpose()))
    //              << std::endl;
    //    std::cout << fmt::format(
    //                     "gurobi x={}\n",
    //                     fmt_eigen(gurobi_primal_result.get_x_val().transpose()))
    //              << std::endl;
    //    std::cout << fmt::format("Clarabel z={}\n",
    //                             fmt_eigen(dual_result.get_x_val().transpose()))
    //              << std::endl;
    //    std::cout << fmt::format(
    //                     "Mosek z={}\n",
    //                     fmt_eigen(mosek_dual_result.get_x_val().transpose()))
    //              << std::endl;
    //    std::cout << fmt::format(
    //                     "gurobi z={}\n",
    //                     fmt_eigen(gurobi_dual_result.get_x_val().transpose()))
    //              << std::endl;
    //    std::cout << fmt::format("complementary gap = {}",
    //    complementarity_gap)
    //              << std::endl;
    //    std::cout << fmt::format("mosek_gap= {}", mosek_gap) << std::endl;
    //    std::cout << fmt::format("gurobi_gap = {}", gurobi_gap) << std::endl;

    //    DRAKE_THROW_UNLESS(false);
    //    bool primal_dual_complimentary =
    //        std::abs((primal_result.get_x_val().transpose() *
    //                  dual_result.get_x_val())(0)) < kTol;
    //    std::cout << "COMPLIMENTARY Gap" << std::endl;
    //    std::cout << std::abs((primal_result.get_x_val().transpose() *
    //                           dual_result.get_x_val())(0))
    //              << std::endl;

    // Check that the dual variables are the same as the ones we got from
    // querying the solvers. Since its possible to write a lot of duals for a
    // given problem, we only compare to the Clarabel solver's solution since we
    // have a known conversion. Additionally, we can only perform this check if
    // solution is unique as otherwise the primal and dual may not solve to a
    // complementary solution.
    //    if (primal_result.get_solver_id() == ClarabelSolver().solver_id() &&
    //        primal_dual_complimentary) {
    if (complementarity_gap < kTol) {
      const double variable_kTol = std::sqrt(complementarity_gap);
      // The dual we produce is the same as that used by Clarabel
      // Check the dual solution is the same one we get from querying the
      // solvers.
      for (const auto& binding : primal_prog.GetAllConstraints()) {
        Eigen::MatrixXd interface_dual_vars =
            primal_result.GetDualSolution(binding);
        Eigen::MatrixXd mosek_interface_dual_vars =
            mosek_primal_result.GetDualSolution(binding);
        if (const auto* l3c =
                dynamic_cast<const PositiveSemidefiniteConstraint*>(
                    binding.evaluator().get())) {
          unused(l3c);
          interface_dual_vars =
              math::ToSymmetricMatrixFromLowerTriangularColumns(
                  interface_dual_vars);
          mosek_interface_dual_vars =
              math::ToSymmetricMatrixFromLowerTriangularColumns(
                  mosek_interface_dual_vars);
        }

        //        const Eigen::MatrixXd gurobi_interface_dual_vars =
        //            gurobi_primal_result.GetDualSolution(binding);
        const MatrixX<symbolic::Expression> manual_dual_vars_expr =
            dual_result.GetSolution(
                constraint_to_dual_variable_map.at(binding));

        Eigen::MatrixXd manual_dual_vars =
            manual_dual_vars_expr.unaryExpr([](const symbolic::Expression& e) {
              return e.Evaluate();
            });

        // If this constraint is a linear equality, we need to flip the sign
        // convention of the dual variable. See SetDualSolution of
        // scs_clarabel_common.cc for details.
        if (const auto* l1c = dynamic_cast<const LinearEqualityConstraint*>(
                binding.evaluator().get())) {
          unused(l1c);
          manual_dual_vars = -manual_dual_vars;
        }
        //                else if (const auto* l2c = dynamic_cast<const
        //                BoundingBoxConstraint*>(
        //                               binding.evaluator().get())) {
        //                  unused(l2c);
        //                  for (int j = 0; j < manual_dual_vars.rows(); ++j) {
        //                    if (binding.evaluator()->lower_bound()(j) ==
        //                        binding.evaluator()->upper_bound()(j)) {
        //                      manual_dual_vars(j) = -manual_dual_vars(j);
        //                    }
        //                  }
        //                }

        if (!CompareMatrices(interface_dual_vars, manual_dual_vars,
                             variable_kTol, MatrixCompareType::relative) ||
            !CompareMatrices(mosek_interface_dual_vars, manual_dual_vars,
                             variable_kTol, MatrixCompareType::relative)) {
          std::cout << dual_prog << std::endl;
          std::cout << fmt::format("b={}", fmt_eigen(b.transpose()))
                    << std::endl;
          std::cout << fmt::format("A={}", fmt_eigen(A.toDense())) << std::endl;
          std::cout << fmt::format(
                           "dual_vars={}",
                           fmt_eigen(dual_result.get_x_val().transpose()))
                    << std::endl;
          std::cout << fmt::format(
                           "mosek dual_vars={}",
                           fmt_eigen(mosek_dual_result.get_x_val().transpose()))
                    << std::endl;
          std::cout << fmt::format("binding={}", binding) << std::endl;
          std::cout << fmt::format(
                           "manual_dual_vars_expr={}",
                           fmt_eigen(constraint_to_dual_variable_map.at(binding)
                                         .transpose()))
                    << std::endl;
          std::cout << fmt::format(
                           "manual_dual_vars_expr={}",
                           fmt_eigen(constraint_to_dual_variable_map.at(binding)
                                         .transpose()))
                    << std::endl;
          std::cout << fmt::format("manual_dual_vars={}",
                                   fmt_eigen(manual_dual_vars.transpose()))
                    << std::endl;
          std::cout << fmt::format("interface_dual_vars={}",
                                   fmt_eigen(interface_dual_vars.transpose()))
                    << std::endl;
          std::cout << fmt::format(
                           "mosek_interface_dual_vars={}\n",
                           fmt_eigen(mosek_interface_dual_vars.transpose()))
                    << std::endl;
          //        std::cout << fmt::format("gurobi_interface_dual_vars={}\n",
          //        fmt_eigen(interface_dual_vars.transpose())) << std::endl;
          EXPECT_TRUE(CompareMatrices(interface_dual_vars, manual_dual_vars,
                                      variable_kTol,
                                      MatrixCompareType::relative));
          EXPECT_TRUE(CompareMatrices(mosek_interface_dual_vars,
                                      manual_dual_vars, variable_kTol,
                                      MatrixCompareType::relative));
        }
        //        EXPECT_TRUE(
        //            CompareMatrices(gurobi_interface_dual_vars,
        //            manual_dual_vars, kTol));
      }
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

// This tests LMI constraints.
GTEST_TEST(TestSdp, SolveEigenvalueProblem) {
  // TODO(Alexandre.Amice) get from semidefinite_program_example.h
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  Eigen::Matrix3d F1;
  // clang-format off
  F1 << 1, 0.2, 0.3,
      0.2, 2, -0.1,
      0.3, -0.1, 4;
   Eigen::Matrix3d F2;
  F2 << 2, 0.4, 0.7,
      0.4, -1, 0.1,
      0.7, 0.1, 5;
  // clang-format on
  auto z = prog.NewContinuousVariables<1>("z");
  prog.AddLinearMatrixInequalityConstraint(
      {Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity(), -F1, -F2}, {z, x});

  const Eigen::Vector2d x_lb(0.1, 1);
  const Eigen::Vector2d x_ub(2, 3);
  prog.AddBoundingBoxConstraint(x_lb, x_ub, x);

  prog.AddLinearCost(z(0));

  std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>
      constraint_to_dual_variable_map;
  auto dual_prog =
      CreateDualConvexProgram(prog, &constraint_to_dual_variable_map);
  CheckPrimalDualSolution(prog, *dual_prog, constraint_to_dual_variable_map);
}

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
