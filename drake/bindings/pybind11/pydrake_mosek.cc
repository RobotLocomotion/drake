#include <pybind11/pybind11.h>


#include "drake/solvers/mosek_solver.h"

namespace py = pybind11;

PYBIND11_PLUGIN(mosek) {
  using drake::solvers::MosekSolver;

  py::module m("mosek", "Mosek solver bindings for MathematicalProgram");

  py::class_<MosekSolver>(m, "MosekSolver")
    .def(py::init<>())
    .def("available", &MosekSolver::available)
    .def("SolverName", &MosekSolver::SolverName)
    .def("Solve", &MosekSolver::Solve);

  return m.ptr();
}
