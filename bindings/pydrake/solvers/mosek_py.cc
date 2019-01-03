#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(mosek, m) {
  using drake::solvers::MosekSolver;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "Mosek solver bindings for MathematicalProgram";

  py::object solverinterface =
      py::module::import("pydrake.solvers.mathematicalprogram")
          .attr("MathematicalProgramSolverInterface");

  py::class_<MosekSolver>(
      m, "MosekSolver", solverinterface, doc.MosekSolver.doc)
      .def(py::init<>(), doc.MosekSolver.ctor.doc);
}

}  // namespace pydrake
}  // namespace drake
