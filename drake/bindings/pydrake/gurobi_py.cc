#include <pybind11/pybind11.h>

#include "drake/solvers/gurobi_solver.h"

namespace py = pybind11;

PYBIND11_MODULE(gurobi, m) {
  using drake::solvers::GurobiSolver;

  m.doc() = "Gurobi solver bindings for MathematicalProgram";

  py::object solverinterface =
      py::module::import("pydrake.solvers.mathematicalprogram").attr(
          "MathematicalProgramSolverInterface");

  py::class_<GurobiSolver>(m, "GurobiSolver", solverinterface)
    .def(py::init<>());
}
