#include "pybind11/pybind11.h"

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

  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<SnoptSolver, SolverInterface>(
      m, "SnoptSolver", doc.SnoptSolver.doc)
      .def(py::init<>(), doc.SnoptSolver.ctor.doc);
}

}  // namespace pydrake
}  // namespace drake
