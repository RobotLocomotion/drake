#pragma once

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver_test.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
namespace test {
void CheckSolver(const MathematicalProgram& prog,
                 SolverType desired_solver_type);

void RunSolver(MathematicalProgram* prog,
               const MathematicalProgramSolverInterface& solver);

void AddSolverIfAvailable(
    SolverType solver_type,
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>*
        solver_list);

}  // namespace test
}  // namespace solvers
}  // namespace drake
