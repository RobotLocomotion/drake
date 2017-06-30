#include "drake/solvers/ipopt_solver.h"

#include <pybind11/pybind11.h>


namespace py = pybind11;

PYBIND11_PLUGIN(ipopt) {
  using drake::solvers::IpoptSolver;

  py::module m("ipopt", "Ipopt solver bindings for MathematicalProgram");

  py::object solverinterface =
    (py::object) py::module::import("pydrake.solvers.mathematicalprogram").attr(
      "MathematicalProgramSolverInterface");

  py::class_<IpoptSolver>(m, "IpoptSolver", solverinterface)
    .def(py::init<>());

  return m.ptr();
}
