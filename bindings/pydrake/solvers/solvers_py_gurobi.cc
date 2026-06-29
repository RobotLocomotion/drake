#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/bindings/pydrake/solvers/solvers_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversGurobi(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;

  class_<GurobiSolver, SolverInterface> cls(
      m, "GurobiSolver", doc.GurobiSolver.doc);
  cls  // BR
      .def(py::init<>(), doc.GurobiSolver.ctor.doc)
      .def_static("id", &GurobiSolver::id, doc.GurobiSolver.id.doc);
  pysolvers::BindAcquireLicense(&cls, doc.GurobiSolver);

  class_<GurobiSolverDetails>(
      m, "GurobiSolverDetails", doc.GurobiSolverDetails.doc)
      .def_ro("optimizer_time", &GurobiSolverDetails::optimizer_time,
          doc.GurobiSolverDetails.optimizer_time.doc)
      .def_ro("error_code", &GurobiSolverDetails::error_code,
          doc.GurobiSolverDetails.error_code.doc)
      .def_ro("optimization_status", &GurobiSolverDetails::optimization_status,
          doc.GurobiSolverDetails.optimization_status.doc)
      .def_ro("objective_bound", &GurobiSolverDetails::objective_bound,
          doc.GurobiSolverDetails.objective_bound.doc);
  AddValueInstantiation<GurobiSolverDetails>(m);

  class_<GurobiSolver::SolveStatusInfo>(
      cls, "SolveStatusInfo", doc.GurobiSolver.SolveStatusInfo.doc)
      .def_ro("reported_runtime",
          &GurobiSolver::SolveStatusInfo::reported_runtime,
          doc.GurobiSolver.SolveStatusInfo.reported_runtime.doc)
      .def_ro("current_objective",
          &GurobiSolver::SolveStatusInfo::current_objective,
          doc.GurobiSolver.SolveStatusInfo.current_objective.doc)
      .def_ro("best_objective", &GurobiSolver::SolveStatusInfo::best_objective,
          doc.GurobiSolver.SolveStatusInfo.best_objective.doc)
      .def_ro("best_bound", &GurobiSolver::SolveStatusInfo::best_bound,
          doc.GurobiSolver.SolveStatusInfo.best_bound.doc)
      .def_ro("explored_node_count",
          &GurobiSolver::SolveStatusInfo::explored_node_count,
          doc.GurobiSolver.SolveStatusInfo.explored_node_count.doc)
      .def_ro("feasible_solutions_count",
          &GurobiSolver::SolveStatusInfo::feasible_solutions_count,
          doc.GurobiSolver.SolveStatusInfo.feasible_solutions_count.doc);

  cls  // BR
      .def("AddMipNodeCallback", &GurobiSolver::AddMipNodeCallback,
          py::arg("callback"), doc.GurobiSolver.AddMipNodeCallback.doc)
      .def("AddMipSolCallback", &GurobiSolver::AddMipSolCallback,
          py::arg("callback"), doc.GurobiSolver.AddMipSolCallback.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
