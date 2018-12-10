#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/osqp_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(osqp, m) {
  using drake::solvers::OsqpSolver;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "OSQP solver bindings for MathematicalProgram";

  py::object solverinterface =
      py::module::import("pydrake.solvers.mathematicalprogram")
          .attr("MathematicalProgramSolverInterface");

  py::class_<OsqpSolver>(m, "OsqpSolver", solverinterface, doc.OsqpSolver.doc)
      .def(py::init<>(), doc.OsqpSolver.ctor.doc_0args);
}

}  // namespace pydrake
}  // namespace drake
