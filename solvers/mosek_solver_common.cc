/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#ifdef DRAKE_MOSEK_LAZY_LOAD_LIBRARY
#include <dlfcn.h>
#endif

#include <cstdlib>
#include <cstring>
#include <string>

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

MosekSolver::MosekSolver()
    : SolverBase(id(), &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

MosekSolver::~MosekSolver() = default;

SolverId MosekSolver::id() {
  static const never_destroyed<SolverId> singleton{"MOSEK"};
  return singleton.access();
}

#ifdef DRAKE_MOSEK_LAZY_LOAD_LIBRARY
extern "C" void* drake_dlopen_mosek(const char* lib_name, bool quiet);
#endif

bool MosekSolver::is_enabled() {
  static const bool latched_result = []() {
#ifdef DRAKE_MOSEK_LAZY_LOAD_LIBRARY
    void* dlhandle = drake_dlopen_mosek(DRAKE_MOSEK_LAZY_LOAD_LIBRARY, true);
    if (dlhandle != nullptr) {
      // Call a MOSEK API so that dlclose decrements the reference count without
      // unloading the library.
      MSKint32t major, minor, revision;
      MSK_getversion(&major, &minor, &revision);
      dlclose(dlhandle);
      const char* moseklm_license_file = std::getenv("MOSEKLM_LICENSE_FILE");
      return ((moseklm_license_file != nullptr) &&
              (std::strlen(moseklm_license_file) > 0));
      return true;
    } else {
      return false;
    }
#else
    return true;
#endif
  }();
  return latched_result;
}

namespace {
// If the program is compatible with this solver, returns true and clears the
// explanation.  Otherwise, returns false and sets the explanation.  In either
// case, the explanation can be nullptr in which case it is ignored.
bool CheckAttributes(const MathematicalProgram& prog,
                     std::string* explanation) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kQuadraticConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kPositiveSemidefiniteConstraint,
          ProgramAttribute::kExponentialConeConstraint,
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kL2NormCost, ProgramAttribute::kBinaryVariable});
  return internal::CheckConvexSolverAttributes(
      prog, solver_capabilities.access(), "MosekSolver", explanation);
}
}  // namespace

bool MosekSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string MosekSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}

}  // namespace solvers
}  // namespace drake
