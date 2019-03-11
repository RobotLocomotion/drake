#pragma once

#include "drake/common/symbolic.h"
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
 * Note: There are more numerical techniques that we know how to apply here.
 * Do report an issue if you discover a system for which this code does not
 * perform well.
 *
 * @ingroup analysis
 */
symbolic::Expression RegionOfAttraction(
    const System<double>& system, const Context<double>& context,
    const RegionOfAttractionOptions& options = RegionOfAttractionOptions());

}  // namespace analysis
}  // namespace systems
}  // namespace drake
