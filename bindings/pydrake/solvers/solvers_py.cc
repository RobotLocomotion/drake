#include "drake/bindings/pydrake/solvers/solvers_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(solvers, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
Bindings for Solving Mathematical Programs.

If you are formulating constraints using symbolic formulas, please review the
top-level documentation for :py:mod:`pydrake.math`.
)""";

  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.common.value");
  py::module::import("pydrake.symbolic");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineSolversEvaluators(m);
  internal::DefineSolversMathematicalProgram(m);
  internal::DefineSolversAugmentedLagrangian(m);
  internal::DefineSolversBranchAndBound(m);
  internal::DefineSolversMixedIntegerOptimizationUtil(m);
  internal::DefineSolversMixedIntegerRotationConstraint(m);
  internal::DefineSolversSdpaFreeFormat(m);
  internal::DefineSolversSemidefiniteRelaxation(m);
  internal::DefineSolversClarabel(m);
  internal::DefineSolversClp(m);
  internal::DefineSolversCsdp(m);
  internal::DefineSolversGurobi(m);
  internal::DefineSolversIpopt(m);
  internal::DefineSolversMobyLCP(m);
  internal::DefineSolversMosek(m);
  internal::DefineSolversNlopt(m);
  internal::DefineSolversOsqp(m);
  internal::DefineSolversScs(m);
  internal::DefineSolversSnopt(m);
  internal::DefineSolversUnrevisedLemke(m);
}

}  // namespace pydrake
}  // namespace drake
