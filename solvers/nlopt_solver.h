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

  /** The key name for int-valued maximum number of evaluations. */
  static std::string MaxEvalName();

  /** The key name for the maximum runtime. By default, there is no maximum
   * runtime. A nonpositive value will be interpreted as no maximum runtime. */
  static std::string MaxTimeName();

  /** The key name for the string-valued algorithm. */
  static std::string AlgorithmName();

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
