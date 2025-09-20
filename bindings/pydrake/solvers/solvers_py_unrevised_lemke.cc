#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/unrevised_lemke_solver.h"

namespace drake {
namespace pydrake {
namespace internal {
void DefineSolversUnrevisedLemke(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;
  py::class_<UnrevisedLemkeSolver<double>, SolverInterface>(
      m, "UnrevisedLemkeSolver", doc.UnrevisedLemkeSolver.doc)
      .def(py::init<>(), doc.UnrevisedLemkeSolver.ctor.doc)
      .def_static("id", &UnrevisedLemkeSolver<double>::id,
          doc.UnrevisedLemkeSolver.id.doc);
}
}  // namespace internal
}  // namespace pydrake
}  // namespace drake
