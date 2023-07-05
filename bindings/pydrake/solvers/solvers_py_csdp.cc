#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/sdpa_free_format.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversCsdp(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::class_<CsdpSolver, SolverInterface> csdp_cls(
      m, "CsdpSolver", doc.CsdpSolver.doc);
  csdp_cls.def(py::init<>(), doc.CsdpSolver.ctor.doc)
      .def_static("id", &CsdpSolver::id, doc.CsdpSolver.id.doc);

  py::class_<CsdpSolverDetails>(
      m, "CsdpSolverDetails", doc.CsdpSolverDetails.doc)
      .def_readonly("return_code", &CsdpSolverDetails::return_code,
          doc.CsdpSolverDetails.return_code.doc)
      .def_readonly("primal_objective", &CsdpSolverDetails::primal_objective,
          doc.CsdpSolverDetails.primal_objective.doc)
      .def_readonly("dual_objective", &CsdpSolverDetails::dual_objective,
          doc.CsdpSolverDetails.dual_objective.doc)
      .def_readonly(
          "y_val", &CsdpSolverDetails::y_val, doc.CsdpSolverDetails.y_val.doc)
      .def_readonly(
          "Z_val", &CsdpSolverDetails::Z_val, doc.CsdpSolverDetails.Z_val.doc);
  AddValueInstantiation<CsdpSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
