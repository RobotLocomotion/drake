#pragma once

/* This file declares the functions that bind the drake::solvers namespace.
These functions form a complete partition of the drake::solvers bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per solvers_py_augmented_lagrangian.cc. */
void DefineSolversAugmentedLagrangian(py::module m);

/* Defines bindings per solvers_py_branch_and_bound.cc. */
void DefineSolversBranchAndBound(py::module m);

/* Defines the Clarabel bindings. See solvers_py_clarabel.cc. */
void DefineSolversClarabel(py::module m);

/* Defines the CLP bindings. See solvers_py_clp.cc. */
void DefineSolversClp(py::module m);

/* Defines the CSDP bindings. See solvers_py_csdp.cc. */
void DefineSolversCsdp(py::module m);

/* Defines the evaluators. See solvers_py_evaluator.cc. */
void DefineSolversEvaluators(py::module m);

/* Defines the GUROBI bindings. See solvers_py_gurobi.cc. */
void DefineSolversGurobi(py::module m);

/* Defines the IPOPT bindings. See solvers_py_ipopt.cc. */
void DefineSolversIpopt(py::module m);

/* Defines the cost, constraint, mathematical program, etc. bindings.
See solvers_py_mathematicalprogram.cc.
TODO(jwnimmer-tri) Split this into smaller pieces.
*/
void DefineSolversMathematicalProgram(py::module m);

/* Defines bindings per solvers_py_mixed_integer_optimization_util.cc */
void DefineSolversMixedIntegerOptimizationUtil(py::module m);

/* Defines bindings per solvers_py_mixed_integer_rotation_constraint.cc */
void DefineSolversMixedIntegerRotationConstraint(py::module m);

/* Defines the MobyLCP bindings. See solvers_py_mobylcp.cc. */
void DefineSolversMobyLCP(py::module m);

/* Defines the MOSEK™ bindings. See solvers_py_mosek.cc. */
void DefineSolversMosek(py::module m);

/* Defines the NLOPT bindings. See solvers_py_nlopt.cc. */
void DefineSolversNlopt(py::module m);

/* Defines the OSQP bindings. See solvers_py_osqp.cc. */
void DefineSolversOsqp(py::module m);

/* Defines the SCS bindings. See solvers_py_scs.cc. */
void DefineSolversScs(py::module m);

/* Defines bindings per solvers_py_sdpa_free_format.cc. */
void DefineSolversSdpaFreeFormat(py::module m);

/* Defines bindings per solvers_py_semidefinite_relaxation.cc. */
void DefineSolversSemidefiniteRelaxation(py::module m);

/* Defines the SNOPT bindings. See solvers_py_snopt.cc. */
void DefineSolversSnopt(py::module m);

/* Defines the UnrevisedLemkeSolver bindings. See solvers_py_unrevised_lemke.cc.
 */
void DefineSolversUnrevisedLemke(py::module m);
}  // namespace internal
}  // namespace pydrake
}  // namespace drake
