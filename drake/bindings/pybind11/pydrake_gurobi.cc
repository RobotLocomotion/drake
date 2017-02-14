#include "drake/solvers/gurobi_solver.h"

#include <pybind11/pybind11.h>


namespace py = pybind11;

PYBIND11_PLUGIN(gurobi) {
  using drake::solvers::GurobiSolver;

  py::module m("gurobi", "Gurobi solver bindings for MathematicalProgram");

  py::object solverinterface =
    (py::object) py::module::import("pydrake.solvers.mathematicalprogram").attr(
      "MathematicalProgramSolverInterface");

  py::class_<GurobiSolver>(m, "GurobiSolver", solverinterface)
    .def(py::init<>());

  return m.ptr();
}
