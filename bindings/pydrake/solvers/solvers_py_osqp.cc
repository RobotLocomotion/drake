#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/osqp_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversOsqp(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::class_<OsqpSolver, SolverInterface>(m, "OsqpSolver", doc.OsqpSolver.doc)
      .def(py::init<>(), doc.OsqpSolver.ctor.doc)
      .def_static("id", &OsqpSolver::id, doc.OsqpSolver.id.doc);

  py::class_<OsqpSolverDetails>(
      m, "OsqpSolverDetails", doc.OsqpSolverDetails.doc)
      .def_readonly(
          "iter", &OsqpSolverDetails::iter, doc.OsqpSolverDetails.iter.doc)
      .def_readonly("status_val", &OsqpSolverDetails::status_val,
          doc.OsqpSolverDetails.status_val.doc)
      .def_readonly("primal_res", &OsqpSolverDetails::primal_res,
          doc.OsqpSolverDetails.primal_res.doc)
      .def_readonly("dual_res", &OsqpSolverDetails::dual_res,
          doc.OsqpSolverDetails.dual_res.doc)
      .def_readonly("setup_time", &OsqpSolverDetails::setup_time,
          doc.OsqpSolverDetails.setup_time.doc)
      .def_readonly("solve_time", &OsqpSolverDetails::solve_time,
          doc.OsqpSolverDetails.solve_time.doc)
      .def_readonly("polish_time", &OsqpSolverDetails::polish_time,
          doc.OsqpSolverDetails.polish_time.doc)
      .def_readonly("run_time", &OsqpSolverDetails::run_time,
          doc.OsqpSolverDetails.run_time.doc)
      .def_readonly("y", &OsqpSolverDetails::y, doc.OsqpSolverDetails.y.doc);
  AddValueInstantiation<OsqpSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
