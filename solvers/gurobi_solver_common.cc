/* clang-format off to disable clang-format-includes */
#include "drake/solvers/gurobi_solver.h"
/* clang-format on */

#include <cstdlib>
#include <cstring>

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

namespace drake {
namespace solvers {

GurobiSolver::GurobiSolver()
    : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

GurobiSolver::~GurobiSolver() = default;

SolverId GurobiSolver::id() {
  static const never_destroyed<SolverId> singleton{"Gurobi"};
  return singleton.access();
}

bool GurobiSolver::is_enabled() {
  const char* grb_license_file = std::getenv("GRB_LICENSE_FILE");
  return ((grb_license_file != nullptr) &&
          (std::strlen(grb_license_file) > 0));
}

namespace {
// If the program is compatible with this solver, returns true and clears the
// explanation.  Otherwise, returns false and sets the explanation.  In either
// case, the explanation can be nullptr in which case it is ignored.
bool CheckAttributes(const MathematicalProgram& prog,
                     std::string* explanation) {
  // TODO(hongkai.dai): Gurobi supports callback. Add callback capability.
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kBinaryVariable});
  return internal::CheckConvexSolverAttributes(
      prog, solver_capabilities.access(), "GurobiSolver", explanation);
}
}  // namespace

bool GurobiSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string GurobiSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}
}  // namespace solvers
}  // namespace drake
