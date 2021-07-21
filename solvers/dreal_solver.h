#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/// An implementation of SolverInterface for the dReal4 solver
/// (https://github.com/dreal/dreal4).
///
/// Currently this implementation supports the following options:
///  * precision <double>: This value is used as a termination condition. When
///                        the ICP algorithm finds an interval box whose width
///                        is smaller than this value, it concludes that the
///                        query is delta-satisfiable and provides the interval
///                        box as a witness. Default value = 0.001.
///
///  * use_local_optimization <bool>: Use local-optimization algorithms in
///                                   solving exists-forall problem. For now,
///                                   the solver is using NLopt. See
///                                   https://link.springer.com/chapter/10.1007%2F978-3-319-96142-2_15
///                                   for details. Default value = True.
///
/// To see the full list of dReal4 options, visit
/// https://github.com/dreal/dreal4#command-line-options.
class DrealSolver final : public SolverBase {
 public:
  /// Class representing an interval of doubles.
  class Interval {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Interval)

    /// Constructs an interval [low, high].
    ///
    /// @pre Its lower bound @p low must be less than or equal to its upper
    /// bound @p high.
    Interval(double low, double high) : low_{low}, high_{high} {
      DRAKE_DEMAND(low <= high);
    }

    /// Returns its diameter.
    double diam() const { return high_ - low_; }

    /// Returns its mid-point.
    double mid() const { return high_ / 2 + low_ / 2; }

    /// Returns its lower bound.
    double low() const { return low_; }

    /// Returns its upper bound.
    double high() const { return high_; }

   private:
    double low_{};
    double high_{};
  };

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
  static std::optional<IntervalBox> CheckSatisfiability(
      const symbolic::Formula& f,
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
  static std::optional<IntervalBox> Minimize(
      const symbolic::Expression& objective,
      const symbolic::Formula& constraint,
      double delta,
      LocalOptimization local_optimization);

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
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;
};

}  // namespace solvers
}  // namespace drake
