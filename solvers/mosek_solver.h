#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class MosekSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekSolver)

  MosekSolver() = default;
  ~MosekSolver() override = default;

  /**
   * Defined true if Mosek was included during compilation, false otherwise.
   */
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  /**
   * This type contains a valid MOSEK license environment, and is only to be
   * used from AcquireLicense().
   */
  class License;

  /**
   * This acquires a MOSEK license environment shared among all MosekSolver
   * instances; the environment will stay valid as long as at least one
   * shared_ptr returned by this function is alive.
   * Call this ONLY if you must use different MathematicalProgram
   * instances at different instances in time, and repeatedly acquiring the
   * license is costly (e.g., requires contacting a license server).
   * @return A shared pointer to a license environment that will stay valid
   * as long as any shared_ptr returned by this function is alive. If MOSEK is
   * not available in your build, this will return a null (empty) shared_ptr.
   * @throws std::runtime_error if MOSEK is available but a license cannot be
   * obtained.
   */
  static std::shared_ptr<License> AcquireLicense();

 private:
  // Note that this is mutable to allow latching the allocation of mosek_env_
  // during the first call of Solve() (which avoids grabbing a Mosek license
  // before we know that we actually want one).
  mutable std::shared_ptr<License> license_;
};

}  // namespace solvers
}  // namespace drake
