#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {
/**
 * The NLopt solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<NloptSolver>() to obtain the
 * details.
 */
struct NloptSolverDetails {
  /// The return status of NLopt solver. Please refer to
  /// https://nlopt.readthedocs.io/en/latest/NLopt_Reference/#return-values.
  int status{};
};

class NloptSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NloptSolver);

  /// Type of details stored in MathematicalProgramResult.
  using Details = NloptSolverDetails;

  NloptSolver();
  ~NloptSolver() final;

  /** The key name for the double-valued constraint tolerance.*/
  static std::string ConstraintToleranceName();

  /** The key name for double-valued x relative tolerance.*/
  static std::string XRelativeToleranceName();

  /** The key name for double-valued x absolute tolerance.*/
  static std::string XAbsoluteToleranceName();

  /** The key name for the double-valued relative tolerance on the objective
   * function value. The default value is 0, which disables this stopping
   * criterion. */
  static std::string FRelativeToleranceName();

  /** The key name for the double-valued absolute tolerance on the objective
   * function value. The default value is 0, which disables this stopping
   * criterion. */
  static std::string FAbsoluteToleranceName();

  /** The key name for int-valued maximum number of evaluations. */
  static std::string MaxEvalName();

  /** The key name for the maximum runtime. By default, there is no maximum
   * runtime. A nonpositive value will be interpreted as no maximum runtime. */
  static std::string MaxTimeName();

  /** The key name for the double-valued target objective value. Because
   * Drake always minimizes, the solve stops as soon as it finds a point whose
   * cost is less than or equal to this value; it is a "good enough, stop
   * here" target, not a bound that the solver enforces. The default value is
   * negative infinity, so that the criterion never triggers. There is no
   * local (inner) optimizer counterpart, because the algorithms that use a
   * local optimizer derive its target from this option. */
  static std::string StopValName();

  /** The key name for the string-valued algorithm. */
  static std::string AlgorithmName();

  /** The key name for the string-valued algorithm of the local (inner)
   * optimizer. Some NLopt algorithms -- notably the augmented Lagrangian
   * family (e.g. LD_AUGLAG_EQ) and the multi-level single-linkage family --
   * work by handing a sequence of subproblems to a separate "local"
   * optimizer; this option chooses that optimizer's algorithm. The default
   * value is the empty string, which leaves NLopt's own default in place.
   * Algorithms that do not use a local optimizer ignore this option, as do
   * the other LocalOptimizer... options below whenever this one is empty. */
  static std::string LocalOptimizerAlgorithmName();

  /** The key name for the double-valued x relative tolerance of the local
   * (inner) optimizer. */
  static std::string LocalOptimizerXRelativeToleranceName();

  /** The key name for the double-valued x absolute tolerance of the local
   * (inner) optimizer. */
  static std::string LocalOptimizerXAbsoluteToleranceName();

  /** The key name for the double-valued relative tolerance on the objective
   * function value of the local (inner) optimizer. The default value is 0,
   * which disables this stopping criterion. */
  static std::string LocalOptimizerFRelativeToleranceName();

  /** The key name for the double-valued absolute tolerance on the objective
   * function value of the local (inner) optimizer. The default value is 0,
   * which disables this stopping criterion. */
  static std::string LocalOptimizerFAbsoluteToleranceName();

  /** The key name for the int-valued maximum number of evaluations of the
   * local (inner) optimizer. By default there is no maximum; a nonpositive
   * value means no maximum. Setting a positive value truncates each
   * subproblem solve, which lets the outer algorithm make progress more often
   * instead of solving the first subproblem to convergence. */
  static std::string LocalOptimizerMaxEvalName();

  /** The key name for the maximum runtime of the local (inner) optimizer. By
   * default there is no maximum runtime. A nonpositive value will be
   * interpreted as no maximum runtime. */
  static std::string LocalOptimizerMaxTimeName();

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
