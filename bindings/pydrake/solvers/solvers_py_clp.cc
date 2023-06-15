#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/clp_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversClp(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::class_<ClpSolver, SolverInterface>(m, "ClpSolver", doc.ClpSolver.doc)
      .def(py::init<>(), doc.ClpSolver.ctor.doc)
      .def_static("id", &ClpSolver::id, doc.ClpSolver.id.doc);

  py::class_<ClpSolverDetails>(m, "ClpSolverDetails", doc.ClpSolverDetails.doc)
      .def_readonly(
          "status", &ClpSolverDetails::status, doc.ClpSolverDetails.status.doc);
  AddValueInstantiation<ClpSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
