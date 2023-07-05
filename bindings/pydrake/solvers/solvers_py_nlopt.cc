#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/nlopt_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversNlopt(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::class_<NloptSolver, SolverInterface>(
      m, "NloptSolver", doc.NloptSolver.doc)
      .def(py::init<>(), doc.NloptSolver.ctor.doc)
      .def_static("id", &NloptSolver::id, doc.NloptSolver.id.doc)
      .def_static("ConstraintToleranceName",
          &NloptSolver::ConstraintToleranceName,
          doc.NloptSolver.ConstraintToleranceName.doc)
      .def_static("XRelativeToleranceName",
          &NloptSolver::XRelativeToleranceName,
          doc.NloptSolver.XRelativeToleranceName.doc)
      .def_static("XAbsoluteToleranceName",
          &NloptSolver::XAbsoluteToleranceName,
          doc.NloptSolver.XAbsoluteToleranceName.doc)
      .def_static("MaxEvalName", &NloptSolver::MaxEvalName,
          doc.NloptSolver.MaxEvalName.doc)
      .def_static("AlgorithmName", &NloptSolver::AlgorithmName,
          doc.NloptSolver.AlgorithmName.doc);

  py::class_<NloptSolverDetails>(
      m, "NloptSolverDetails", doc.NloptSolverDetails.doc)
      .def_readonly("status", &NloptSolverDetails::status,
          doc.NloptSolverDetails.status.doc);
  AddValueInstantiation<NloptSolverDetails>(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
