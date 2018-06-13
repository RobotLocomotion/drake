#pragma once

#include <string>
#include <unordered_map>

#include <dreal/dreal.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class DrealSolver : public MathematicalProgramSolverInterface {
 public:
  using Interval = dreal::Box::Interval;
  using IntervalBox = std::unordered_map<symbolic::Variable, Interval>;

  /// Indicates whether to use dReal's --local-optimization option or not.
  enum class LocalOptimization {
    kUse,     ///< Use "--local-optimization" option.
    kNotUse,  ///< Do not use "--local-optimization" option.
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrealSolver)

  DrealSolver() = default;
  ~DrealSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // Dreal was available during compilation.
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  /// Checks the satisfiability of a given formula @p f with a given precision
  /// @p delta.
  ///
  /// @returns a model, a mapping from a variable to an interval, if @p f is
  /// δ-satisfiable.
  /// @returns a nullopt, if @p is unsatisfiable.
  static optional<IntervalBox> CheckSatisfiability(const symbolic::Formula& f,
                                                   double delta);

  /// Finds a solution to minimize @p objective function while satisfying a
  /// given @p constraint using @p delta. When @p local_optimization is
  /// Localoptimization::kUse, enable "--local-optimization" dReal option which
  /// uses NLopt's local-optimization algorithms to refine counterexamples in
  /// the process of global optimization.
  ///
  /// @returns a model, a mapping from a variable to an interval, if a solution
  /// exists.
  /// @returns nullopt, if there is no solution.
  static optional<IntervalBox> Minimize(const symbolic::Expression& objective,
                                        const symbolic::Formula& constraint,
                                        double delta,
                                        LocalOptimization local_optimization);
};

}  // namespace solvers
}  // namespace drake
