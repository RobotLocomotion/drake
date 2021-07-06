#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_pybind.h"
#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(gurobi, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "Gurobi solver bindings for MathematicalProgram";

  py::module::import("pydrake.common.value");
  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<GurobiSolver, SolverInterface> cls(
      m, "GurobiSolver", doc.GurobiSolver.doc);
  cls.def(py::init<>(), doc.GurobiSolver.ctor.doc)
      .def_static("id", &GurobiSolver::id, doc.GurobiSolver.id.doc);
  pysolvers::BindAcquireLicense(&cls, doc.GurobiSolver);

  py::class_<GurobiSolverDetails>(
      m, "GurobiSolverDetails", doc.GurobiSolverDetails.doc)
      .def_readonly("optimizer_time", &GurobiSolverDetails::optimizer_time,
          doc.GurobiSolverDetails.optimizer_time.doc)
      .def_readonly("error_code", &GurobiSolverDetails::error_code,
          doc.GurobiSolverDetails.error_code.doc)
      .def_readonly("optimization_status",
          &GurobiSolverDetails::optimization_status,
          doc.GurobiSolverDetails.optimization_status.doc)
      .def_readonly("objective_bound", &GurobiSolverDetails::objective_bound,
          doc.GurobiSolverDetails.objective_bound.doc);
  AddValueInstantiation<GurobiSolverDetails>(m);
}

}  // namespace pydrake
}  // namespace drake
