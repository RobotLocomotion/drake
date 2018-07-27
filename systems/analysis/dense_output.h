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
/// solution 𝐱(t) ∈ ℝⁿ to an ODE system that is approximated at a discrete
/// set of points 𝐲(tₖ) ∈ ℝⁿ where tₖ ∈ {t₁, ..., tᵢ} with tᵢ ∈ ℝ (e.g. as
/// a result of numerical integration), a dense output of 𝐱(t) is another
/// function 𝐳(t) ∈ ℝⁿ defined for t ∈ [t₁, tᵢ] such that 𝐳(tⱼ) = 𝐲(tⱼ) for
/// all tⱼ ∈ {t₁, ..., tᵢ} and that approximates 𝐱(t) for every value in the
/// closed interval [t₁, tᵢ].
///
/// @warning Note that dense outputs are not bound to enforce the same
///          constraints that the approximated solution does, and therefore
///          these may be violated.
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
  /// @param t Time at which to evaluate output.
  /// @returns Output vector value.
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throws std::logic_error if any of the preconditions are not met.
  /// @throws std::runtime_error if the output is not defined for the
  ///                            given @p t.
  VectorX<T> Evaluate(const T& t) const {
    if (is_empty()) {
      throw std::logic_error("Empty dense output cannot be evaluated.");
    }
    if (t < this->do_start_time() || t > this->do_end_time()) {
      throw std::runtime_error("Dense output is not defined for given time.");
    }
    return this->DoEvaluate(t);
  }

  /// Evaluates the output's @p dimension at the given time @p t.
  /// @note On some implementations, the computational cost of this
  ///       method may be lower than that of of indexing an
  ///       Evaluate(const T&) call return vector value, thus making
  ///       it the preferred mechanism when targeting a single dimension.
  /// @param t Time at which to evaluate output.
  /// @param dimension Dimension to evaluate.
  /// @returns Output @p dimension scalar value.
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throws std::logic_error if any of the preconditions are not met.
  /// @throws std::runtime_error if the output is not defined for the
  ///                            given @p t.
  /// @throws std::runtime_error if given @p dimension is not valid
  ///                            i.e. 0 <= @p dimension < size().
  T Evaluate(const T& t, int dimension) const {
    if (is_empty()) {
      throw std::logic_error("Empty dense output cannot be evaluated.");
    }
    if (dimension < 0 || this->do_size() <= dimension) {
      throw std::runtime_error("Invalid dimension for dense output.");
    }
    if (t < this->do_start_time() || t > this->do_end_time()) {
      throw std::runtime_error("Dense output is not defined for given time.");
    }
    return this->DoEvaluate(t, dimension);
  }

  /// Returns the output size (i.e. the number of elements in an
  /// output value).
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  int size() const {
    if (is_empty()) {
      throw std::logic_error("Size is not defined for"
                             " an empty dense output.");
    }
    return this->do_size();
  }

  /// Checks whether the output is empty or not.
  bool is_empty() const { return this->do_is_empty(); }

  /// Returns output's start time, or in other words, the oldest time
  /// `t` that it can be evaluated at e.g. via Evaluate().
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throws std::logic_error if any of the preconditions is not met.
  const T& start_time() const {
    if (is_empty()) {
      throw std::logic_error("Start time is not defined for"
                             " an empty dense output.");
    }
    return this->do_start_time();
  }

  /// Returns output's end time, or in other words, the newest time
  /// `t` that it can be evaluated at e.g. via Evaluate().
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throws std::logic_error if any of the preconditions is not met.
  const T& end_time() const {
    if (is_empty()) {
      throw std::logic_error("End time is not defined for"
                             " an empty dense output.");
    }
    return this->do_end_time();
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

  // @see size()
  virtual int do_size() const = 0;

  // @see start_time()
  virtual const T& do_start_time() const = 0;

  // @see end_time()
  virtual const T& do_end_time() const = 0;
};

}  // namespace systems
}  // namespace drake
