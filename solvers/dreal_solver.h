#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic/expression.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/// (Deprecated.) dReal support is being withdrawn from Drake; for details, see
/// https://github.com/RobotLocomotion/drake/pull/18156. This class will be
/// removed from Drake on or after 2023-02-01.
///
/// An implementation of SolverInterface for the dReal4 solver
/// (https://github.com/dreal/dreal4).
class DRAKE_DEPRECATED("2023-02-01",
    "dReal support is being withdrawn from Drake; for details, see "
    "https://github.com/RobotLocomotion/drake/pull/18156")
DrealSolver final : public SolverBase {
 public:
  /// (Deprecated.) dReal support is being withdrawn from Drake; for details,
  /// see https://github.com/RobotLocomotion/drake/pull/18156. This class will
  /// be removed from Drake on or after 2023-02-01.
  ///
  /// Class representing an interval of doubles.
  class DRAKE_DEPRECATED("2023-02-01",
      "dReal support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  Interval {
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /// (Deprecated.) dReal support is being withdrawn from Drake; for details,
  /// see https://github.com/RobotLocomotion/drake/pull/18156. This class will
  /// be removed from Drake on or after 2023-02-01.
  using IntervalBox = std::unordered_map<symbolic::Variable, Interval>;
#pragma GCC diagnostic pop

  /// (Deprecated.) dReal support is being withdrawn from Drake; for details,
  /// see https://github.com/RobotLocomotion/drake/pull/18156. This class will
  /// be removed from Drake on or after 2023-02-01.
  enum class LocalOptimization {
    kUse,     ///< Use "--local-optimization" option.
    kNotUse,  ///< Do not use "--local-optimization" option.
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrealSolver)

  DRAKE_DEPRECATED("2023-02-01",
      "dReal support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  DrealSolver();

  ~DrealSolver() final;

  DRAKE_DEPRECATED("2023-02-01",
      "dReal support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  static std::optional<IntervalBox> CheckSatisfiability(
      const symbolic::Formula& f,
      double delta);

  DRAKE_DEPRECATED("2023-02-01",
      "dReal support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  static std::optional<IntervalBox> Minimize(
      const symbolic::Expression& objective,
      const symbolic::Formula& constraint,
      double delta,
      LocalOptimization local_optimization);

  /// @name Static versions of the instance methods with similar names.
  //@{
  DRAKE_DEPRECATED("2023-02-01",
      "dReal support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  static SolverId id();
  DRAKE_DEPRECATED("2023-02-01",
      "dReal support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  static bool is_available();
  DRAKE_DEPRECATED("2023-02-01",
      "dReal support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
  static bool is_enabled();
  DRAKE_DEPRECATED("2023-02-01",
      "dReal support is being withdrawn from Drake; for details, see "
      "https://github.com/RobotLocomotion/drake/pull/18156")
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
