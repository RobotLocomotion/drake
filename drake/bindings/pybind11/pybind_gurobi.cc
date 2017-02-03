#include <pybind11/pybind11.h>


#include "drake/solvers/gurobi_solver.h"

namespace py = pybind11;

PYBIND11_PLUGIN(gurobi) {
  using drake::solvers::GurobiSolver;

  py::module m("gurobi", "Gurobi solver bindings for MathematicalProgram");

  py::class_<GurobiSolver>(m, "GurobiSolver")
    .def(py::init<>())
    .def("available", &GurobiSolver::available)
    .def("SolverName", &GurobiSolver::SolverName)
    .def("Solve", &GurobiSolver::Solve);

  return m.ptr();
}
