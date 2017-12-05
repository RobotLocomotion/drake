#include <pybind11/pybind11.h>

#include "drake/solvers/ipopt_solver.h"

namespace py = pybind11;

PYBIND11_MODULE(ipopt, m) {
  using drake::solvers::IpoptSolver;

  m.doc() = "Ipopt solver bindings for MathematicalProgram";

  py::object solverinterface =
      py::module::import("pydrake.solvers.mathematicalprogram").attr(
          "MathematicalProgramSolverInterface");

  py::class_<IpoptSolver>(m, "IpoptSolver", solverinterface)
    .def(py::init<>());
}
