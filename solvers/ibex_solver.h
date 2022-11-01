#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solver_base.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace solvers {
/// (Deprecated.) IBEX support is being withdrawn from Drake; for details, see
/// https://github.com/RobotLocomotion/drake/pull/18156. This class will be
/// removed from Drake on or after 2023-02-01.
///
/// An implementation of SolverInterface for the IBEX solver
/// (http://www.ibex-lib.org).
///
/// This solver supports the following constraints / costs:
///  - ExponentialConeConstraint
///  - GenericConstraint
///  - LinearComplementarityConstraint
///  - LinearConstraint
///  - LinearEqualityConstraint
///  - LorentzConeConstraint
///  - RotatedLorentzConeConstraint
///  - GenericCost
///  - LinearCost
///  - QuadraticCost
///
/// @note Only costs and constraints that support symbolic evaluation are
/// compatible with this solver.
///
/// Currently this implementation supports the following options:
///  - rel_eps_f <double>: Relative precision (∈ ℝ⁺) on the objective.
///  - abs_eps_f <double>: Absolute precision (∈ ℝ⁺) on the objective.
///  - eps_h <double>: Equality relaxation value (∈ ℝ⁺).
///  - rigor <int>: Activate rigor mode (certify feasibility of equations).
///                 0 --> rigor off, != 0 --> rigor on.
///  - random_seed <double>: Random seed (useful for reproducibility).
///  - eps_x <double>: Precision on the variable (∈ ℝ⁺).
///  - trace <int>: Activate trace. Updates of loup/uplo are printed while
///                 minimizing. 0 - nothing is printed. 1 - prints every
///                 loup/uplo update. 2 - prints also each handled node.
///                 Note that trace = 1 if kPrintToConsole = true.
///  - timeout <double>: Timeout (time in seconds). 0.0 indicates +∞.
///
/// See http://www.ibex-lib.org/doc/optim.html#options for more information.
class DRAKE_DEPRECATED("2023-02-01",
    "IBEX support is being withdrawn from Drake; for details, see "
    "https://github.com/RobotLocomotion/drake/pull/18156")
IbexSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IbexSolver)

  DRAKE_DEPRECATED("2023-02-01",
      "IBEX support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  IbexSolver();

  ~IbexSolver() final;

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

  /// @name Static versions of the instance methods with similar names.
  //@{
  DRAKE_DEPRECATED("2023-02-01",
      "IBEX support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  static SolverId id();
  DRAKE_DEPRECATED("2023-02-01",
      "IBEX support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  static bool is_available();
  DRAKE_DEPRECATED("2023-02-01",
      "IBEX support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  static bool is_enabled();
  DRAKE_DEPRECATED("2023-02-01",
      "IBEX support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};

}  // namespace solvers
}  // namespace drake
