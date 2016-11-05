#pragma once

#include <list>
#include <string>

#include "gtest/gtest.h"

#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
namespace test {
void RunSolver(MathematicalProgram* prog,
               const MathematicalProgramSolverInterface& solver) {
  if (solver.available()) {
    SolutionResult result = solver.Solve(*prog);
    std::string solver_name;
    int solver_status;
    prog->GetSolverResult(&solver_name, &solver_status);
    EXPECT_EQ(result, SolutionResult::kSolutionFound)
        << "Solver " << solver_name << " fails to find the solution."
        << std::endl;
  }
}

void AddSolverIfAvailable(
    const std::string &solver_name,
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>> *
    solver_list) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> all_solvers;
  auto gurobi_solver = std::make_unique<GurobiSolver>();
  all_solvers.push_back(std::move(gurobi_solver));
  auto mosek_solver = std::make_unique<MosekSolver>();
  all_solvers.push_back(std::move(mosek_solver));
  auto snopt_solver = std::make_unique<SnoptSolver>();
  all_solvers.push_back(std::move(snopt_solver));

  for (auto& solver : all_solvers) {
    if (solver->SolverName() == solver_name) {
      if (solver->available()) {
        solver_list->push_back(std::move(solver));
      }
      return;
    }
  }
  throw std::runtime_error(solver_name + " is not supported");
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
