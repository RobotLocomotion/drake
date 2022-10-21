#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/ibex_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void DefineSolversIbex(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::class_<IbexSolver, SolverInterface>(
      m, "IbexSolver", doc.IbexSolver.doc_deprecated)
      .def(py_init_deprecated<IbexSolver>(doc.IbexSolver.ctor.doc_deprecated),
          doc.IbexSolver.ctor.doc_deprecated)
      .def_static("id",
          WrapDeprecated(doc.IbexSolver.id.doc_deprecated, &IbexSolver::id),
          doc.IbexSolver.id.doc_deprecated);
}
#pragma GCC diagnostic pop

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
