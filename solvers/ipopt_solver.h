#pragma once

#include <ostream>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/**
 * The details of IPOPT solvers after calling Solve function. The users can get
 * the details by
 * MathematicalProgramResult::get_solver_details().GetValue<IpoptSolverDetails>();
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
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IpoptSolver)

  IpoptSolver();
  ~IpoptSolver() final;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};

}  // namespace solvers
}  // namespace drake
