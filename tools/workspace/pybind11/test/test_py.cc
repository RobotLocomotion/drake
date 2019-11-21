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

  constexpr auto& cls_doc = doc.OsqpSolver;
  py::class_<OsqpSolver, SolverInterface>(m, "OsqpSolver", cls_doc.doc)
      .def(py::init<>(), cls_doc.ctor.doc);

  constexpr auto& cls_doc = doc.
      OsqpSolverDetails;

  py::class_<OsqpSolverDetails>(
      m, "OsqpSolverDetails", cls_doc.doc)
      .def_readwrite(
          "iter", &OsqpSolverDetails::iter, cls_doc.iter.doc)
      .def_readwrite("status_val", &OsqpSolverDetails::status_val,
          cls_doc.
          status_val.doc)
      .def_readwrite("primal_res", &OsqpSolverDetails::primal_res,
          cls_doc.primal_res
          .doc)
      .def_readwrite("dual_res", &OsqpSolverDetails::dual_res,
          cls_doc.dual_res.doc)
      .def_readwrite("setup_time", &OsqpSolverDetails::setup_time,
          cls_doc.  // BR Comment to test
          setup_time.doc)
      .def_readwrite("solve_time", &OsqpSolverDetails::solve_time,
          cls_doc.solve_time.doc)
      .def_readwrite("polish_time", &OsqpSolverDetails::polish_time,
          cls_doc.polish_time.doc)
      .def_readwrite("run_time", &OsqpSolverDetails::run_time,
          cls_doc.run_time.doc);
  AddValueInstantiation<OsqpSolverDetails>(m);
}

}  // namespace pydrake
}  // namespace drake
