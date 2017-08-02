#include <pybind11/pybind11.h>

#include "drake/solvers/mosek_solver.h"

namespace py = pybind11;

PYBIND11_PLUGIN(mosek) {
  using drake::solvers::MosekSolver;

  py::module m("mosek", "Mosek solver bindings for MathematicalProgram");

  py::object solverinterface =
    (py::object) py::module::import("pydrake.solvers.mathematicalprogram").attr(
      "MathematicalProgramSolverInterface");

  py::class_<MosekSolver>(m, "MosekSolver", solverinterface)
    .def(py::init<>());

  return m.ptr();
}
