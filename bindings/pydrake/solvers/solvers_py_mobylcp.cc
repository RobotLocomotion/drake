#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/moby_lcp_solver.h"

namespace drake {
namespace pydrake {
namespace internal {
void DefineSolversMobyLCP(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;
  py::class_<MobyLcpSolver, SolverInterface>(
      m, "MobyLcpSolver", doc.MobyLcpSolver.doc)
      .def(py::init<>(), doc.MobyLcpSolver.ctor.doc)
      .def_static("id", &MobyLcpSolver::id, doc.MobyLcpSolver.id.doc);
}
}  // namespace internal
}  // namespace pydrake
}  // namespace drake
