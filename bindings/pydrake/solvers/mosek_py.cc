#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_pybind.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(mosek, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "Mosek solver bindings for MathematicalProgram";

  py::module::import("pydrake.common.value");
  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<MosekSolver, SolverInterface> cls(
      m, "MosekSolver", doc.MosekSolver.doc);
  cls.def(py::init<>(), doc.MosekSolver.ctor.doc)
      .def_static("id", &MosekSolver::id, doc.MosekSolver.id.doc);
  pysolvers::BindAcquireLicense(&cls, doc.MosekSolver);

  py::class_<MosekSolverDetails>(
      m, "MosekSolverDetails", doc.MosekSolverDetails.doc)
      .def_readonly("optimizer_time", &MosekSolverDetails::optimizer_time,
          doc.MosekSolverDetails.optimizer_time.doc)
      .def_readonly("rescode", &MosekSolverDetails::rescode,
          doc.MosekSolverDetails.rescode.doc)
      .def_readonly("solution_status", &MosekSolverDetails::solution_status,
          doc.MosekSolverDetails.solution_status.doc);
  AddValueInstantiation<MosekSolverDetails>(m);
}

}  // namespace pydrake
}  // namespace drake
