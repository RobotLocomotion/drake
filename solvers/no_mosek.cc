/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mosek_solver.h"
/* clang-format on */

#ifdef DRAKE_MOSEK_LAZY_LOAD_LIBRARY
#include <dlfcn.h>
#endif

#include <memory>
#include <stdexcept>

using std::runtime_error;
using std::shared_ptr;

namespace drake {
namespace solvers {

shared_ptr<MosekSolver::License> MosekSolver::AcquireLicense() {
  return shared_ptr<MosekSolver::License>();
}

bool MosekSolver::is_available() {
  return false;
}

#ifdef DRAKE_MOSEK_LAZY_LOAD_LIBRARY
extern "C" void* drake_dlopen_mosek(const char* lib_name, bool quiet);
#endif

bool MosekSolver::is_enabled() {
  if (!has_license_file()) {
    return false;
  }
  static const bool latched_result = []() {
#ifdef DRAKE_MOSEK_LAZY_LOAD_LIBRARY
    void* dlhandle = drake_dlopen_mosek(DRAKE_MOSEK_LAZY_LOAD_LIBRARY, true);
    if (dlhandle != nullptr) {
      dlclose(dlhandle);
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

void MosekSolver::DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                           internal::SpecificOptions*,
                           MathematicalProgramResult*) const {
  throw runtime_error(
      "Mosek is not installed in your build. You'll need to use a different "
      "solver.");
}

}  // namespace solvers
}  // namespace drake
