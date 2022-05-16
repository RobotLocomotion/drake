#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/scs_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(scs, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "SCS solver bindings for MathematicalProgram";

  py::module::import("pydrake.common.value");
  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<ScsSolver, SolverInterface>(m, "ScsSolver", doc.ScsSolver.doc)
      .def(py::init<>(), doc.ScsSolver.ctor.doc)
      .def_static("id", &ScsSolver::id, doc.ScsSolver.id.doc);

  py::class_<ScsSolverDetails>(m, "ScsSolverDetails", doc.ScsSolverDetails.doc)
      .def_readonly("scs_status", &ScsSolverDetails::scs_status,
          doc.ScsSolverDetails.scs_status.doc)
      .def_readonly(
          "iter", &ScsSolverDetails::iter, doc.ScsSolverDetails.iter.doc)
      .def_readonly("primal_objective", &ScsSolverDetails::primal_objective,
          doc.ScsSolverDetails.primal_objective.doc)
      .def_readonly("dual_objective", &ScsSolverDetails::dual_objective,
          doc.ScsSolverDetails.dual_objective.doc)
      .def_readonly("primal_residue", &ScsSolverDetails::primal_residue,
          doc.ScsSolverDetails.primal_residue.doc)
      .def_readonly("residue_infeasibility",
          &ScsSolverDetails::residue_infeasibility,
          doc.ScsSolverDetails.residue_infeasibility.doc)
      .def_readonly("residue_unbounded_a",
          &ScsSolverDetails::residue_unbounded_a,
          doc.ScsSolverDetails.residue_unbounded_a.doc)
      .def_readonly("residue_unbounded_p",
          &ScsSolverDetails::residue_unbounded_p,
          doc.ScsSolverDetails.residue_unbounded_p.doc)
      .def_readonly("duality_gap", &ScsSolverDetails::duality_gap,
          doc.ScsSolverDetails.duality_gap.doc)
      .def_readonly("scs_setup_time", &ScsSolverDetails::scs_setup_time,
          doc.ScsSolverDetails.scs_setup_time.doc)
      .def_readonly("scs_solve_time", &ScsSolverDetails::scs_solve_time,
          doc.ScsSolverDetails.scs_solve_time.doc)
      .def_readonly("y", &ScsSolverDetails::y, doc.ScsSolverDetails.y.doc)
      .def_readonly("s", &ScsSolverDetails::s, doc.ScsSolverDetails.s.doc);
  AddValueInstantiation<ScsSolverDetails>(m);
}

}  // namespace pydrake
}  // namespace drake
