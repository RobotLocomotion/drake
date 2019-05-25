#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/osqp_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(osqp, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "OSQP solver bindings for MathematicalProgram";

  py::module::import("pydrake.solvers.mathematicalprogram");
  py::module::import("pydrake.systems.framework").attr("AbstractValue");

  py::class_<OsqpSolver, SolverInterface>(m, "OsqpSolver", doc.OsqpSolver.doc)
      .def(py::init<>(), doc.OsqpSolver.ctor.doc);

  py::class_<OsqpSolverDetails>(
      m, "OsqpSolverDetails", doc.OsqpSolverDetails.doc)
      .def_readwrite(
          "iter", &OsqpSolverDetails::iter, doc.OsqpSolverDetails.iter.doc)
      .def_readwrite("status_val", &OsqpSolverDetails::status_val,
          doc.OsqpSolverDetails.status_val.doc)
      .def_readwrite("primal_res", &OsqpSolverDetails::primal_res,
          doc.OsqpSolverDetails.primal_res.doc)
      .def_readwrite("dual_res", &OsqpSolverDetails::dual_res,
          doc.OsqpSolverDetails.dual_res.doc)
      .def_readwrite("setup_time", &OsqpSolverDetails::setup_time,
          doc.OsqpSolverDetails.setup_time.doc)
      .def_readwrite("solve_time", &OsqpSolverDetails::solve_time,
          doc.OsqpSolverDetails.solve_time.doc)
      .def_readwrite("polish_time", &OsqpSolverDetails::polish_time,
          doc.OsqpSolverDetails.polish_time.doc)
      .def_readwrite("run_time", &OsqpSolverDetails::run_time,
          doc.OsqpSolverDetails.run_time.doc);
  AddValueInstantiation<OsqpSolverDetails>(m);
}

}  // namespace pydrake
}  // namespace drake
