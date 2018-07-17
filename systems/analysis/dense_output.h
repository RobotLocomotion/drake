#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/// An interface for dense output of ODEs solutions, to efficiently approximate
/// them at arbitrarily many points when solving them numerically (see
/// IntegratorBase class documentation).
///
/// Multiple definitions of _dense output_ may be found in literature. For some
/// authors, it refers to the process of repeatedly adjusting the integration
/// step size so that all points to be approximated are directly provided by the
/// integrator (see [Engquist, 2015]). For others, it stands for any numerical
/// approximation technique used to determine the solution in between steps
/// (see [Hairer, 1993]). Despite this caveat, it is common terminology in IVP
/// literature and thus its imparted functionality is immediately clear.
///
/// Herein, the concept in use may be formally stated as follows: given a
/// solution 𝐱(t) ∈ ℝⁿ to an ODE or DAE system that is approximated at a
/// discrete set of points 𝐲(tₖ) ∈ ℝⁿ where tₖ ∈ {t₁, ..., tᵢ} with
/// tᵢ ∈ ℝ (e.g. as a result of numerical integration), a dense outpupt of
/// 𝐱(t) is another function 𝐳(t) ∈ ℝⁿ defined for t ∈ [t₁, tᵢ] such that
/// 𝐳(tⱼ) = 𝐲(tⱼ) for all tⱼ ∈ {t₁, ..., tᵢ} and that approximates 𝐱(t) for
/// every value in the closed interval [t₁, tᵢ].
///
/// - [Engquist, 2105] B. Engquist. Encyclopedia of Applied and Computational
///                    Mathematics, p. 339, Springer, 2015.
/// - [Hairer, 1993] E. Hairer, S. Nørsett and G. Wanner. Solving Ordinary
///                  Differential Equations I (Nonstiff Problems), p.188,
///                  Springer, 1993.
/// @tparam T A valid Eigen scalar type.
template <typename T>
class DenseOutput {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseOutput)

  virtual ~DenseOutput() = default;

  /// Evaluates the output at the given time @p t.
  /// @param t Time to evaluate output at.
  /// @return Output vector value.
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions are not met.
  /// @throw std::runtime_error if the output is not defined for the
  ///                           given @p t.
  virtual VectorX<T> Evaluate(const T& t) const = 0;

  /// Evaluates the output's @p dimension at the given time @p t.
  /// @note On some implementations, the computational cost of this
  ///       method may be lower than that of of indexing an
  ///       Evaluate(const T&) call return vector value.
  /// @param t Time to evaluate output at.
  /// @param dimension Dimension to evaluate.
  /// @return Output @p dimension scalar value .
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions are not met.
  /// @throw std::runtime_error if the output is not defined for the
  ///                           given @p t.
  /// @throw std::runtime_error if given @p dimension is not valid
  ///                           i.e. 0 <= @p dimension < get_dimensions().
  virtual T Evaluate(const T& t, int dimension) const = 0;

  /// Returns the output dimension `n`.
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual int get_dimensions() const = 0;

  /// Checks whether the output is empty or not.
  virtual bool is_empty() const = 0;

  /// Returns output's start time, or in other words, the oldest time
  /// `t` that it can be evaluated at e.g. via Evaluate().
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual const T& get_start_time() const = 0;

  /// Returns output's end time, or in other words, the newest time
  /// `t` that it can be evaluated at e.g. via Evaluate().
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual const T& get_end_time() const = 0;

 protected:
  DenseOutput() = default;
};

}  // namespace systems
}  // namespace drake
