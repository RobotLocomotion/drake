#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/// An interface for continuous extension of ODE and DAE solutions, to
/// efficiently approximate them in between integration steps when solving
/// them numerically (see IntegratorBase class documentation).
///
/// This _continuous extension_ (see [Engquist, 2015]) concept can be formally
/// stated as follows: given a solution 𝐱(t) ∈ ℝⁿ to an ODE or DAE system that
/// is approximated at a discrete set of points 𝐲(tₖ) ∈ ℝⁿ where
/// tₖ ∈ {t₁, ..., tᵢ} with tᵢ ∈ ℝ (e.g. as a result of numerical integration),
/// a continuous extension of 𝐱(t) is another function 𝐳(t) ∈ ℝⁿ defined for
/// t ∈ [t₁, tᵢ] such that 𝐳(tⱼ) = 𝐲(tⱼ) for all tⱼ ∈ {t₁, ..., tᵢ} and that
/// approximates 𝐱(t) for every value in the closed interval [t₁, tᵢ].
///
/// - [Engquist, 2105] B. Engquist. Encyclopedia of Applied and Computational
///                    Mathematics, p. 339, Springer, 2015.
/// @tparam T A valid Eigen scalar type.
template <typename T>
class ContinuousExtension {
 public:
  virtual ~ContinuousExtension() {}

  /// Evaluates the extension function at the given time @p t.
  /// @param t Time to evaluate extension at.
  /// @return Extension function vector value.
  /// @pre Extension is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions are not met.
  /// @throw std::runtime_error if the extension is not defined for the
  ///                           given @p t.
  virtual VectorX<T> Evaluate(const T& t) const = 0;

  /// Returns the extension dimension `n`.
  /// @pre Extension is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual int get_dimensions() const = 0;

  /// Checks whether the extension is empty or not.
  virtual bool is_empty() const = 0;

  /// Returns extension's start time i.e. the oldest time `t`
  /// that it can be evaluated at e.g. via Evaluate().
  /// @pre Extension is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual const T& get_start_time() const = 0;

  /// Returns extension's end time i.e. the newest time `t`
  /// that it can be evaluated at e.g. via Evaluate().
  /// @pre Extension is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual const T& get_end_time() const = 0;
};

}  // namespace systems
}  // namespace drake
