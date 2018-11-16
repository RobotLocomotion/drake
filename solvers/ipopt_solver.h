#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {
/**
 * This enum class is copied from IpAlgTypes.hpp in the Ipopt source code. We provide this enum class here so that the user can understand the status in IpoptSolverDetails.
 */
enum class IpoptSolverReturn {
  SUCCESS,
  MAXITER_EXCEEDED,
  CPUTIME_EXCEEDED,
  STOP_AT_TINY_STEP,
  STOP_AT_ACCEPTABLE_POINT,
  LOCAL_INFEASIBILITY,
  USER_REQUESTED_STOP,
  FEASIBLE_POINT_FOUND,
  DIVERGING_ITERATES,
  RESTORATION_FAILURE,
  ERROR_IN_STEP_COMPUTATION,
  INVALID_NUMBER_DETECTED,
  TOO_FEW_DEGREES_OF_FREEDOM,
  INVALID_OPTION,
  OUT_OF_MEMORY,
  INTERNAL_ERROR,
  UNASSIGNED
};

std::string to_string(IpoptSolverReturn status);

/**
 * The details of IPOPT solvers after calling Solve function. The users can get
 * the details by
 * MathematicalProgramResult::get_solver_details().GetValue<IpoptSolverDetails>();
 */
struct IpoptSolverDetails {
  /**
   * The final status of the solver. Please refer to 
   */
  IpoptSolverReturn status;
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
