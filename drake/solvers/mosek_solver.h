#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

/**
 * Permits controlling the scope (RAII) of a MOSEK license.
 * This attempts to obtain a MOSEK license session if it is the first one.
 * If it fails, it throws a runtime_error.
 * Once no scope objects are alive, the MOSEK license session is released.
 * If a scope object is declared in a main() function, then the MOSEK
 * license lives for the duration of the program.
 */
class MosekLicenseScope {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekLicenseScope)

  MosekLicenseScope();
  ~MosekLicenseScope();

  // TODO(eric.cousineau): Consider using a MosekSolver class itself, with
  // a ::Prelock() method, to control shared license scoping.
  class Impl;
  Impl* impl() const;
 private:
  std::unique_ptr<Impl> impl_;
};

class MosekSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekSolver)

  MosekSolver() : MathematicalProgramSolverInterface(SolverType::kMosek) {}

  /**
   * Defined true if Mosek was included during compilation, false otherwise.
   */
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

 private:
  // Note that this is mutable to allow latching the allocation of mosek_env_
  // during the first call of Solve() (which avoids grabbing a Mosek license
  // before we know that we actually want one).
  mutable std::unique_ptr<MosekLicenseScope> license_scope_;
};

}  // namespace solvers
}  // namespace drake
