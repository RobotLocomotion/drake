#pragma once

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
namespace test {
void CheckSolver(const MathematicalProgram& prog,
                 SolverType desired_solver_type) {
  SolverType solver_type;
  int solver_result;
  prog.GetSolverResult(&solver_type, &solver_result);
  EXPECT_EQ(solver_type, desired_solver_type);
}

void RunSolver(MathematicalProgram* prog,
               const MathematicalProgramSolverInterface& solver) {
  if (solver.available()) {
    SolutionResult result = solver.Solve(*prog);
    SolverType solver_type;
    int solver_status;
    prog->GetSolverResult(&solver_type, &solver_status);
    EXPECT_EQ(result, SolutionResult::kSolutionFound)
        << "Solver " << solver_type << " fails to find the solution."
        << std::endl;
  }
}

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
