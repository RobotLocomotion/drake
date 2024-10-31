#pragma once

#include <ostream>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/**
 * The Ipopt solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<IpoptSolver>() to obtain the
 * details.
 */
struct IpoptSolverDetails {
  /**
   * The final status of the solver. Please refer to section 6 in
   * Introduction to Ipopt: A tutorial for downloading, installing, and using
   * Ipopt.
   * You could also find the meaning of the status as Ipopt::SolverReturn
   * defined in IpAlgTypes.hpp
   */
  int status{};
  /// The final value for the lower bound multiplier.
  Eigen::VectorXd z_L;
  /// The final value for the upper bound multiplier.
  Eigen::VectorXd z_U;
  /// The final value for the constraint function.
  Eigen::VectorXd g;
  /// The final value for the constraint multiplier.
  Eigen::VectorXd lambda;

  /** Convert status field to string. This function is useful if you want to
   * interpret the meaning of status.
   */
  const char* ConvertStatusToString() const;
};

/**
 * A wrapper to call
 * <a href="https://coin-or.github.io/Ipopt/">Ipopt</a>
 * using Drake's MathematicalProgram.
 *
 * The IpoptSolver is NOT threadsafe to call in parallel. This is due to Ipopt's
 * reliance on the MUMPs linear solver which is not safe to call concurrently
 * (see https://github.com/coin-or/Ipopt/issues/733). This can be resolved by
 * enabling the SPRAL solver (see Drake issue
 * https://github.com/RobotLocomotion/drake/issues/21476).
 */
class IpoptSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IpoptSolver);

  /// Type of details stored in MathematicalProgramResult.
  using Details = IpoptSolverDetails;

  IpoptSolver();
  ~IpoptSolver() final;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
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
