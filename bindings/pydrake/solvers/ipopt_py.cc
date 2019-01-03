#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/ipopt_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(ipopt, m) {
  using drake::solvers::IpoptSolver;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "Ipopt solver bindings for MathematicalProgram";

  py::object solverinterface =
      py::module::import("pydrake.solvers.mathematicalprogram")
          .attr("MathematicalProgramSolverInterface");

  py::class_<IpoptSolver>(
      m, "IpoptSolver", solverinterface, doc.IpoptSolver.doc)
      .def(py::init<>(), doc.IpoptSolver.ctor.doc);
}

}  // namespace pydrake
}  // namespace drake
