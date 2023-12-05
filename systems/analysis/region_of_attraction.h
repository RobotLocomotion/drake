#pragma once

#include <optional>

#include "drake/common/name_value.h"
#include "drake/common/symbolic/expression.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_options.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace analysis {

/**
 * Consolidates the many possible options to be passed to the region of
 * attraction algorithm.
 */
struct RegionOfAttractionOptions {
  RegionOfAttractionOptions() = default;

  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(lyapunov_candidate));
    a->Visit(DRAKE_NVP(state_variables));
    a->Visit(DRAKE_NVP(use_implicit_dynamics));
    a->Visit(DRAKE_NVP(solver_id));
    a->Visit(DRAKE_NVP(solver_options));
  }

  /** A candidate Lyapunov function using the symbolic Variables named
   * x0, x1, ..., where the order matches the continuous state vector of the
   * system being evaluated (or the vector state_variables).
   */
  symbolic::Expression lyapunov_candidate{};

  /** If non-empty, a list of Variable that associates the variable name with
   * the elements of the System's continuous state vector.  Must be empty
   * or have size equal to the number of continuous state variables in the
   * system.
   */
  VectorX<symbolic::Variable> state_variables{};

  /** If true, the system dynamics will be evaluated using
   * CalcImplicitTimeDerivativesResidual instead of CalcTimeDerivatives to
   * obtain g(x,ẋ) = 0 (instead of ẋ = f(x)).  The Lyapunov conditions will
   * also be evaluated in the implicit form. This is more expensive than
   * analysis in the explicit form, as it requires more indeterminates, but it
   * enables analysis of systems with rational polynomial dynamics.
   *
   * See https://underactuated.csail.mit.edu/lyapunov.html#ex:implicit for more
   * details.
   */
  bool use_implicit_dynamics{false};

  /** If not std::nullopt, then we will solve the optimization problem using the
   * specified solver; otherwise Drake will choose a solver.
   */
  std::optional<solvers::SolverId> solver_id{std::nullopt};

  /** The solver options used in the optimization problem. */
  std::optional<solvers::SolverOptions> solver_options{std::nullopt};
};

/**
 * Estimates the region of attraction of the time-invariant @p system at the
 * fixed point defined by @p context.
 *
 * This implementation only searches for the largest level set of the
 * `lyapunov_candidate` function from @p options (or a candidate obtained
 * from solving the Lyapunov equation on the linearization).
 *
 * @param system a time-invariant continuous-time System that supports
 * scalar-type conversion to symbolic::Expression.  The dynamics of the
 * system must be polynomial.
 *
 * @param context a Context that defines the parameters of the system and
 * the fixed-point about which we are analyzing the regional stability.
 *
 * @param options provides a variety of configuration options.  @see
 * RegionOfAttractionOptions.
 *
 * @returns a symbolic::Expression representing a Lyapunov function using
 * the symbolic Variables named x0, x1..., where the order matches the
 * continuous state vector in the @p context, or the vector state_variables
 * passed in through the options structure (if it is non-empty).  The level set
 * {x | V(x)<=1} containing the fixed-point in @p context represents the region
 * of attraction.
 *
 * @pre For the given @p system and @p context, any required input ports on @p
 * system must be "defined", i.e., connected to other systems in a larger
 * diagram or holding fixed values; see System::FixInputPortsFrom for possible
 * caveats. Analyzing a closed-loop system would typically be accomplished by
 * having both the plant and the controller in a diagram (which then has no
 * input ports), and passing the diagram into this method as @p system.
 *
 * Note: There are more numerical recipes for region of attraction analysis that
 * could extend the current implementation. Do report an issue if you discover a
 * system for which this code does not perform well.
 *
 * @ingroup analysis
 */
symbolic::Expression RegionOfAttraction(
    const System<double>& system, const Context<double>& context,
    const RegionOfAttractionOptions& options = RegionOfAttractionOptions());

}  // namespace analysis
}  // namespace systems
}  // namespace drake
