#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(gurobi, m) {
  using drake::solvers::GurobiSolver;

  m.doc() = "Gurobi solver bindings for MathematicalProgram";
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::object solverinterface =
      py::module::import("pydrake.solvers.mathematicalprogram")
          .attr("MathematicalProgramSolverInterface");

  py::class_<GurobiSolver>(
      m, "GurobiSolver", solverinterface, doc.GurobiSolver.doc)
      .def(py::init<>(), doc.GurobiSolver.ctor.doc);
}

}  // namespace pydrake
}  // namespace drake
