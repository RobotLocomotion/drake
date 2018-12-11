#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {
/**
 * The NLopt solver details after calling Solve() function. The users can call
 * MathematicalProgramResult::get_solver_details().GetValue<NloptSolverDetails>()
 * to obtain the details.
 */
struct NloptSolverDetails {
  // The return status of NLopt solver. Please refer to
  // https://nlopt.readthedocs.io/en/latest/NLopt_Reference/#return-values.
  int status{};
};

class NloptSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NloptSolver)

  NloptSolver() = default;
  ~NloptSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // NLOpt was available during compilation.
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

  /** The key name for the double-valued constraint tolerance.*/
  static std::string ConstraintToleranceName();

  /** The key name for double-valued x relative tolerance.*/
  static std::string XRelativeToleranceName();

  /** The key name for double-valued x absolute tolerance.*/
  static std::string XAbsoluteToleranceName();

  /** The key name for int-valued maximum number of evaluations. */
  static std::string MaxEvalName();
};

}  // namespace solvers
}  // namespace drake
