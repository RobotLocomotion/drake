#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/ipopt_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(ipopt, m) {
  using drake::solvers::IpoptSolver;

  m.doc() = "Ipopt solver bindings for MathematicalProgram";

  py::object solverinterface =
      py::module::import("pydrake.solvers.mathematicalprogram").attr(
          "MathematicalProgramSolverInterface");

  py::class_<IpoptSolver>(m, "IpoptSolver", solverinterface)
    .def(py::init<>());
}

}  // namespace pydrake
}  // namespace drake
