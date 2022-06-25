#include "drake/bindings/pydrake/solvers/solvers_py.h"

// #include "pybind11/eval.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(solvers, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
Bindings for Mathematical Programming.

If you are formulating constraints using symbolic formulas, please review the
top-level documentation for :py:mod:`pydrake.math`.
)""";

  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.common.value");
  py::module::import("pydrake.symbolic");

  /* The order of execution matters -- a module may rely on the definition
   of bindings executed prior to it. */
  internal::DefineSolversMathematicalProgram(m);
  internal::DefineSolversAugmentedLagrangian(m);
  internal::DefineSolversBranchAndBound(m);
  internal::DefineSolversMixedIntegerOptimizationUtil(m);
  internal::DefineSolversMixedIntegerRotationConstraint(m);
  internal::DefineSolversSdpaFreeFormat(m);
  internal::DefineSolversClp(m);
  internal::DefineSolversCsdp(m);
  internal::DefineSolversDreal(m);
  internal::DefineSolversGurobi(m);
  internal::DefineSolversIbex(m);
  internal::DefineSolversIpopt(m);
  internal::DefineSolversMosek(m);
  internal::DefineSolversNlopt(m);
  internal::DefineSolversOsqp(m);
  internal::DefineSolversScs(m);
  internal::DefineSolversSnopt(m);
}

}  // namespace pydrake
}  // namespace drake
