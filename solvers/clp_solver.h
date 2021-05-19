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
  // Refer to ClpModel::status() function for the meaning of the status code.
  // -1: unknown error.
  // 0: optimal.
  // 1: primal infeasible
  // 2: dual infeasible
  // 3: stopped on iterations or time.
  // 4: stopped due to errors
  // 5: stopped by event handler
  int status;
};

/**
 * A wrapper to call CLP using Drake's MathematicalProgram.
 * @note Currently our ClpSolver has a memory issue when solving a QP. The user
 * should be aware of this risk.
 */
class ClpSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ClpSolver)

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
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};
}  // namespace solvers
}  // namespace drake
