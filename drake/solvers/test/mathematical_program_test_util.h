#pragma once

#include <list>
#include <string>

#include "gtest/gtest.h"

#include "drake/solvers/mathematical_program.h"

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

void AddSolverToListIfAvailable(
    const std::string &solver_name,
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>> *
    solver_list) {
  if (solver_name == "Gurobi") {
    auto gurobi_solver = std::make_unique<GurobiSolver>();
    if (gurobi_solver->available()) {
      solver_list->push_back(std::move(gurobi_solver));
    }
  } else if (solver_name == "Mosek") {
    auto mosek_solver = std::make_unique<MosekSolver>();
    if (mosek_solver->available()) {
      solver_list->push_back(std::move(mosek_solver));
    }
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
