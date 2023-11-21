#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/clarabel_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversClarabel(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::class_<ClarabelSolver, SolverInterface>(
      m, "ClarabelSolver", doc.ClarabelSolver.doc)
      .def(py::init<>(), doc.ClarabelSolver.ctor.doc)
      .def_static("id", &ClarabelSolver::id, doc.ClarabelSolver.id.doc);

  py::class_<ClarabelSolverDetails>(
      m, "ClarabelSolverDetails", doc.ClarabelSolverDetails.doc)
      .def_readonly("solve_time", &ClarabelSolverDetails::solve_time,
          doc.ClarabelSolverDetails.solve_time.doc)
      .def_readonly("iterations", &ClarabelSolverDetails::iterations,
          doc.ClarabelSolverDetails.iterations.doc)
      .def_readonly("status", &ClarabelSolverDetails::status,
          doc.ClarabelSolverDetails.status.doc);
  AddValueInstantiation<ClarabelSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
