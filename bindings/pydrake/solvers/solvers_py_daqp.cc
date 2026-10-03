#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/daqp_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversDaqp(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;

  class_<DaqpSolver, SolverInterface>(m, "DaqpSolver", doc.DaqpSolver.doc)
      .def(py::init<>(), doc.DaqpSolver.ctor.doc)
      .def_static("id", &DaqpSolver::id, doc.DaqpSolver.id.doc);

  class_<DaqpSolverDetails>(m, "DaqpSolverDetails", doc.DaqpSolverDetails.doc)
      .def_ro("exitflag", &DaqpSolverDetails::exitflag,
          doc.DaqpSolverDetails.exitflag.doc)
      .def_ro("iterations", &DaqpSolverDetails::iterations,
          doc.DaqpSolverDetails.iterations.doc)
      .def_ro("setup_time", &DaqpSolverDetails::setup_time,
          doc.DaqpSolverDetails.setup_time.doc)
      .def_ro("solve_time", &DaqpSolverDetails::solve_time,
          doc.DaqpSolverDetails.solve_time.doc)
      .def_ro("multipliers", &DaqpSolverDetails::multipliers,
          doc.DaqpSolverDetails.multipliers.doc);
  AddValueInstantiation<DaqpSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
