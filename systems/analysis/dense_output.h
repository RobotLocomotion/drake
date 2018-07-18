#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/// An interface for dense output of ODE solutions, to efficiently approximate
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
  VectorX<T> Evaluate(const T& t) const {
    if (is_empty()) {
      throw std::logic_error("Empty dense output cannot be evaluated.");
    }
    if (t < this->do_get_start_time() || t > this->do_get_end_time()) {
      throw std::runtime_error("Dense output is not defined for given time.");
    }
    return this->DoEvaluate(t);
  }

  /// Evaluates the output's @p dimension at the given time @p t.
  /// @note On some implementations, the computational cost of this
  ///       method may be lower than that of of indexing an
  ///       Evaluate(const T&) call return vector value, thus making
  ///       it the preferred mechanism when targeting a single dimension.
  /// @param t Time to evaluate output at.
  /// @param dimension Dimension to evaluate.
  /// @return Output @p dimension scalar value .
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions are not met.
  /// @throw std::runtime_error if the output is not defined for the
  ///                           given @p t.
  /// @throw std::runtime_error if given @p dimension is not valid
  ///                           i.e. 0 <= @p dimension < get_dimensions().
  T Evaluate(const T& t, int dimension) const {
    if (is_empty()) {
      throw std::logic_error("Empty dense output cannot be evaluated.");
    }
    if (dimension < 0 || this->do_get_dimensions() <= dimension) {
      throw std::runtime_error("Invalid dimension for dense output.");
    }
    if (t < this->do_get_start_time() || t > this->do_get_end_time()) {
      throw std::runtime_error("Dense output is not defined for given time.");
    }
    return this->DoEvaluate(t, dimension);
  }

  /// Returns the output dimension `n`.
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  int get_dimensions() const {
    if (is_empty()) {
      throw std::logic_error("Dimension is not defined for"
                             " an empty dense output.");
    }
    return this->do_get_dimensions();
  }

  /// Checks whether the output is empty or not.
  bool is_empty() const { return this->do_is_empty(); }

  /// Returns output's start time, or in other words, the oldest time
  /// `t` that it can be evaluated at e.g. via Evaluate().
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  const T& get_start_time() const {
    if (is_empty()) {
      throw std::logic_error("Start time is not defined for"
                             " an empty dense output.");
    }
    return this->do_get_start_time();
  }

  /// Returns output's end time, or in other words, the newest time
  /// `t` that it can be evaluated at e.g. via Evaluate().
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  const T& get_end_time() const {
    if (is_empty()) {
      throw std::logic_error("End time is not defined for"
                             " an empty dense output.");
    }
    return this->do_get_end_time();
  }

 protected:
  DenseOutput() = default;

 private:
  // @see Evaluate(const T&)
  virtual VectorX<T> DoEvaluate(const T& t) const = 0;

  // @remarks The computational cost of this method must
  //          be less than or equal to that of indexing
  //          DoEvaluate(const T&) return value.
  // @see Evaluate(const T&, int)
  virtual T DoEvaluate(const T& t, int dimension) const {
    return this->DoEvaluate(t)(dimension);
  }

  // @see is_empty()
  virtual bool do_is_empty() const = 0;

  // @see get_dimensions()
  virtual int do_get_dimensions() const = 0;

  // @see get_start_time()
  virtual const T& do_get_start_time() const = 0;

  // @see get_end_time()
  virtual const T& do_get_end_time() const = 0;
};

}  // namespace systems
}  // namespace drake
