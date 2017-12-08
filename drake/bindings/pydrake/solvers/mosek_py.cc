#include <pybind11/pybind11.h>

#include "drake/solvers/mosek_solver.h"

namespace py = pybind11;

PYBIND11_MODULE(mosek, m) {
  using drake::solvers::MosekSolver;

  m.doc() = "Mosek solver bindings for MathematicalProgram";

  py::object solverinterface =
      py::module::import("pydrake.solvers.mathematicalprogram").attr(
          "MathematicalProgramSolverInterface");

  py::class_<MosekSolver>(m, "MosekSolver", solverinterface)
    .def(py::init<>());
}
