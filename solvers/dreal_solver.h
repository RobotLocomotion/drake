#pragma once

#include <string>
#include <unordered_map>

#include <dreal/dreal.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

class DrealSolver final : public SolverBase {
 public:
  using Interval = dreal::Box::Interval;
  using IntervalBox = std::unordered_map<symbolic::Variable, Interval>;

  /// Indicates whether to use dReal's --local-optimization option or not.
  enum class LocalOptimization {
    kUse,     ///< Use "--local-optimization" option.
    kNotUse,  ///< Do not use "--local-optimization" option.
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrealSolver)

  DrealSolver();
  ~DrealSolver() final;

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
