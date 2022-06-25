#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/ibex_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversIbex(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::class_<IbexSolver, SolverInterface>(m, "IbexSolver", doc.IbexSolver.doc)
      .def(py::init<>(), doc.IbexSolver.ctor.doc)
      .def_static("id", &IbexSolver::id, doc.IbexSolver.id.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
