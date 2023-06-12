#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/moby_lcp_solver.h"

namespace drake {
namespace pydrake {
namespace internal {
void DefineSolversMobyLCP(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;
  py::class_<MobyLCPSolver<double>, SolverInterface>(
      m, "MobyLCPSolver", doc.MobyLCPSolver.doc)
      .def(py::init<>(), doc.MobyLCPSolver.ctor.doc)
      .def_static("id", &MobyLCPSolver<double>::id, doc.MobyLCPSolver.id.doc);
}
}  // namespace internal
}  // namespace pydrake
}  // namespace drake
