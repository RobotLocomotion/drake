#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversSnopt(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;

  py::class_<SnoptSolver, SolverInterface>(
      m, "SnoptSolver", doc.SnoptSolver.doc)
      .def(py::init<>(), doc.SnoptSolver.ctor.doc)
      .def_static("id", &SnoptSolver::id, doc.SnoptSolver.id.doc);

  py::class_<SnoptSolverDetails>(
      m, "SnoptSolverDetails", doc.SnoptSolverDetails.doc)
      .def_ro(
          "info", &SnoptSolverDetails::info, doc.SnoptSolverDetails.info.doc)
      .def_ro(
          "xmul", &SnoptSolverDetails::xmul, doc.SnoptSolverDetails.xmul.doc)
      .def_ro("F", &SnoptSolverDetails::F, doc.SnoptSolverDetails.F.doc)
      .def_ro(
          "Fmul", &SnoptSolverDetails::Fmul, doc.SnoptSolverDetails.Fmul.doc)
      .def_ro("solve_time", &SnoptSolverDetails::solve_time,
          doc.SnoptSolverDetails.solve_time.doc);
  AddValueInstantiation<SnoptSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
