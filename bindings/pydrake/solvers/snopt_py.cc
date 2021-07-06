#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(snopt, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "SNOPT solver bindings for MathematicalProgram";

  py::module::import("pydrake.common.value");
  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<SnoptSolver, SolverInterface>(
      m, "SnoptSolver", doc.SnoptSolver.doc)
      .def(py::init<>(), doc.SnoptSolver.ctor.doc)
      .def_static("id", &SnoptSolver::id, doc.SnoptSolver.id.doc);

  py::class_<SnoptSolverDetails>(
      m, "SnoptSolverDetails", doc.SnoptSolverDetails.doc)
      .def_readonly(
          "info", &SnoptSolverDetails::info, doc.SnoptSolverDetails.info.doc)
      .def_readonly(
          "xmul", &SnoptSolverDetails::xmul, doc.SnoptSolverDetails.xmul.doc)
      .def_readonly("F", &SnoptSolverDetails::F, doc.SnoptSolverDetails.F.doc)
      .def_readonly(
          "Fmul", &SnoptSolverDetails::Fmul, doc.SnoptSolverDetails.Fmul.doc);
  AddValueInstantiation<SnoptSolverDetails>(m);
}

}  // namespace pydrake
}  // namespace drake
