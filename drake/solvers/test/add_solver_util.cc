#include "drake/solvers/test/add_solver_util.h"

#include <utility>

#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
namespace test {
void AddSolverIfAvailable(
    SolverType solver_type,
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>*
    solver_list) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> all_solvers;
  auto gurobi_solver = std::make_unique<GurobiSolver>();
  all_solvers.push_back(std::move(gurobi_solver));
  auto mosek_solver = std::make_unique<MosekSolver>();
  all_solvers.push_back(std::move(mosek_solver));
  auto snopt_solver = std::make_unique<SnoptSolver>();
  all_solvers.push_back(std::move(snopt_solver));

  for (auto& solver : all_solvers) {
    if (solver->solver_type() == solver_type) {
      if (solver->available()) {
        solver_list->push_back(std::move(solver));
      }
      return;
    }
  }
  throw std::runtime_error("solver is not supported");
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
