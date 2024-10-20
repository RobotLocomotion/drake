#pragma once

#include <ostream>
#include <set>
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

class IpoptSolver final : public SolverBase {
  /**
   * The IpoptSolver is NOT threadsafe to call in parallel. This is due to the
   * reliance on the MUMPs solver which is not safe to call concurrently (see
   * https://github.com/coin-or/Ipopt/issues/733). This can be resolved by
   * enabling the SPRAL solver (see Drake issue
   * https://github.com/RobotLocomotion/drake/issues/21476).
   */
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
  /// Returns true if the linear solver name is known to be thread safe.
  static bool IsThreadSafeLinearSolver(const std::string& solver_name);
  //@}

  /// Some of the linear systems solvers in IPOPT are not threadsafe. Here we
  /// collect the ones that are known to be.
  static const std::set<std::string> known_threadsafe_linear_solvers;

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};

}  // namespace solvers
}  // namespace drake
