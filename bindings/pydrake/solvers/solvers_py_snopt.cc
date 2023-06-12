#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversSnopt(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::class_<SnoptSolver, SolverInterface>(
      m, "SnoptSolver", doc.SnoptSolver.doc)
      .def(py::init<>(), doc.SnoptSolver.ctor.doc)
      .def_static("id", &SnoptSolver::id, doc.SnoptSolver.id.doc);

  py::class_<SnoptSolverDetails>(
      m, "SnoptSolverDetails", doc.SnoptSolverDetails.doc)
      .def_readonly(
          "info", &SnoptSolverDetails::info, doc.SnoptSolverDetails.info.doc)
      .def_readonly(
          "xmul", &SnoptSolverDetails::xmul, doc.SnoptSolverDetails.xmul.doc)
      .def_readonly("F", &SnoptSolverDetails::F, doc.SnoptSolverDetails.F.doc)
      .def_readonly(
          "Fmul", &SnoptSolverDetails::Fmul, doc.SnoptSolverDetails.Fmul.doc);
  AddValueInstantiation<SnoptSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
