#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/osqp_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversOsqp(py::module_ m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;

  class_<OsqpSolver, SolverInterface>(m, "OsqpSolver", doc.OsqpSolver.doc)
      .def(py::init<>(), doc.OsqpSolver.ctor.doc)
      .def_static("id", &OsqpSolver::id, doc.OsqpSolver.id.doc);

  class_<OsqpSolverDetails>(m, "OsqpSolverDetails", doc.OsqpSolverDetails.doc)
      .def_ro("iter", &OsqpSolverDetails::iter, doc.OsqpSolverDetails.iter.doc)
      .def_ro("status_val", &OsqpSolverDetails::status_val,
          doc.OsqpSolverDetails.status_val.doc)
      .def_ro("primal_res", &OsqpSolverDetails::primal_res,
          doc.OsqpSolverDetails.primal_res.doc)
      .def_ro("dual_res", &OsqpSolverDetails::dual_res,
          doc.OsqpSolverDetails.dual_res.doc)
      .def_ro("setup_time", &OsqpSolverDetails::setup_time,
          doc.OsqpSolverDetails.setup_time.doc)
      .def_ro("solve_time", &OsqpSolverDetails::solve_time,
          doc.OsqpSolverDetails.solve_time.doc)
      .def_ro("polish_time", &OsqpSolverDetails::polish_time,
          doc.OsqpSolverDetails.polish_time.doc)
      .def_ro("run_time", &OsqpSolverDetails::run_time,
          doc.OsqpSolverDetails.run_time.doc)
      .def_ro("y", &OsqpSolverDetails::y, doc.OsqpSolverDetails.y.doc);
  AddValueInstantiation<OsqpSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
