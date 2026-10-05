#pragma once

#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {
/**
 * The CLP solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<ClpSolver>() to obtain the
 * details.
 */
struct ClpSolverDetails {
  /** The CLP_VERSION from the Clp build. */
  std::string clp_version;

  /** Refer to ClpModel::status() function for the meaning of the status code.
   * - -1: unknown error.
   * - 0: optimal.
   * - 1: primal infeasible
   * - 2: dual infeasible
   * - 3: stopped on iterations or time.
   * - 4: stopped due to errors
   * - 5: stopped by event handler
   */
  int status{-1};
};

/**
 * A wrapper to call CLP using Drake's MathematicalProgram.
 * @note Currently our ClpSolver has a memory issue when solving a QP. The user
 * should be aware of this risk.
 * @note The authors can adjust the problem scaling option by setting "scaling"
 as mentioned in
 https://github.com/coin-or/Clp/blob/43129ba1a7fd66ce70fe0761fcd696951917ed2e/src/ClpModel.hpp#L705-L706
 * For example
 * prog.SetSolverOption(ClpSolver::id(), "scaling", 0);
 * will do "no scaling". The default is 1.
 */
class ClpSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ClpSolver);

  using Details = ClpSolverDetails;

  ClpSolver();

  ~ClpSolver() final;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  static std::string UnsatisfiedProgramAttributes(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                internal::SpecificOptions*,
                MathematicalProgramResult*) const final;
};
}  // namespace solvers
}  // namespace drake
