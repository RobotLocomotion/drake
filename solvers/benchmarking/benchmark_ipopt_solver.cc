#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace solvers {
namespace {

static void BenchmarkIpoptSolver(benchmark::State& state) {  // NOLINT
  // Number of decision variables.
  const int nx = state.range(0);
  // Create the mathematical program and the solver.
  MathematicalProgram prog;
  IpoptSolver solver;
  // Create decision variables.
  auto x = prog.NewContinuousVariables(nx);
  // Add bounding box constraints.
  auto lbx = -1e0 * Eigen::VectorXd::Ones(nx);
  auto ubx = +1e0 * Eigen::VectorXd::Ones(nx);
  prog.AddBoundingBoxConstraint(lbx, ubx, x);
  // Add random linear constraints.
  auto A = Eigen::MatrixXd::Random(nx, nx);
  auto lb = Eigen::VectorXd::Zero(nx);
  auto ub = 1e20 * Eigen::VectorXd::Ones(nx);
  prog.AddLinearConstraint(A, lb, ub, x);
  // Add linear equality constraints.
  auto Aeq = Eigen::MatrixXd::Identity(nx, nx);
  auto beq = Eigen::VectorXd::Zero(nx);
  prog.AddLinearEqualityConstraint(Aeq, beq, x);
  // Add a nonlinear constraint: closed unit disk.
  prog.AddConstraint(x.transpose() * x <= 1e0);
  // Add a quadratic cost.
  prog.AddQuadraticCost(Eigen::MatrixXd::Identity(nx, nx),
                        Eigen::VectorXd::Zero(nx), x);

  // Run the solver and measure the performance.
  MathematicalProgramResult result;
  for (auto _ : state) {
    result = solver.Solve(prog);
  }
  // Verify the success of the solver.
  DRAKE_DEMAND(result.is_success());
}

BENCHMARK(BenchmarkIpoptSolver)->ArgsProduct({{10, 100, 1000}});
}  // namespace
}  // namespace solvers
}  // namespace drake
