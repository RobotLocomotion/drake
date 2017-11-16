#include <iostream>
#include <list>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/test/add_solver_util.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using std::make_shared;

using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix4d;
using Eigen::RowVector2d;

namespace drake {
namespace solvers {
namespace test {
namespace {
void GetSecondOrderConicProgramSolvers(
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>* solvers) {
  AddSolverIfAvailable<GurobiSolver>(solvers);
  AddSolverIfAvailable<MosekSolver>(solvers);
}

void GetSemidefiniteProgramSolvers(
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>* solvers) {
  AddSolverIfAvailable<MosekSolver>(solvers);
}
}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
