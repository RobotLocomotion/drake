#pragma once

#include <ostream>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

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

class IpoptSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IpoptSolver)

  IpoptSolver() = default;
  ~IpoptSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // Ipopt was available during compilation.
  bool available() const override { return is_available(); };

  static bool is_available();

  SolutionResult Solve(MathematicalProgram& prog) const override;

  void Solve(const MathematicalProgram& prog,
             const optional<Eigen::VectorXd>& initial_guess,
             const optional<SolverOptions>& solver_options,
             MathematicalProgramResult* result) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const override;

  static bool ProgramAttributesSatisfied(const MathematicalProgram& prog);
};

}  // namespace solvers
}  // namespace drake
