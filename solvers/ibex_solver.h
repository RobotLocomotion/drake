#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solver_base.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace solvers {
/// An implementation of SolverInterface for the IBEX solver
/// (http://www.ibex-lib.org/)
///
/// For now, this solver only supports the following constraints / costs:
///  - kGenericConstraint
///  - kLinearConstraint
///  - kGenericCost
///  - kQuadraticCost
///
/// @note Only costs and constraints that support symbolic evaluation are
/// compatible with this solver.
///
/// TODO(soonho): Add and enumerate supported options here.
class IbexSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IbexSolver)

  IbexSolver();
  ~IbexSolver() final;

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};

}  // namespace solvers
}  // namespace drake
