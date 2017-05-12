#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

/*
 * Permit a scoped lock (RAII) of a MOSEK license.
 * This will attempt to obtain a MOSEK license session if it is the first one.
 * If it fails, it will throw a runtime_error.
 * Once there are no locks in scope, the MOSEK license session will be
 * released.
 * If a lock is declared in a main() function, then a lock on the MOSEK license
 * will be obtained for the entire duration of the program.
 */
class MosekLicenseLock {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekLicenseLock)

  MosekLicenseLock();
  ~MosekLicenseLock();

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
  mutable std::unique_ptr<MosekLicenseLock> license_lock_;
};

}  // namespace solvers
}  // namespace drake
