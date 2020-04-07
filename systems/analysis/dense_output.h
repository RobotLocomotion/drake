#pragma once

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/default_scalars.h"
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
/// @warning Dense outputs are, in general, not bound to attain the same
///          accuracy that error-controlled integration schemes do. Check
///          each subclass documentation for further specification.
/// @warning Note that dense outputs do not enforce any algebraic constraints
///          on the solution that integrators might enforce.
///
/// - [Engquist, 2105] B. Engquist. Encyclopedia of Applied and Computational
///                    Mathematics, p. 339, Springer, 2015.
/// - [Hairer, 1993] E. Hairer, S. Nørsett and G. Wanner. Solving Ordinary
///                  Differential Equations I (Nonstiff Problems), p.188,
///                  Springer, 1993.
/// @tparam_default_scalar
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
  /// @throws std::runtime_error if given @p t is not within output's domain
  ///                            i.e. @p t ∉ [start_time(), end_time()].
  VectorX<T> Evaluate(const T& t) const {
    ThrowIfOutputIsEmpty(__func__);
    ThrowIfTimeIsInvalid(__func__, t);
    return this->DoEvaluate(t);
  }

  /// Evaluates the output value's `n`th scalar element (0-indexed) at the
  /// given time @p t.
  /// @note On some implementations, the computational cost of this
  ///       method may be lower than that of indexing an Evaluate(const T&)
  ///       call return vector value, thus making it the preferred mechanism
  ///       when targeting a single dimension.
  /// @param t Time at which to evaluate output.
  /// @param n The nth scalar element (0-indexed) of the output
  ///          value to evaluate.
  /// @returns Output value's `n`th scalar element (0-indexed).
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throws std::logic_error if any of the preconditions are not met.
  /// @throws std::runtime_error if given @p t is not within output's domain
  ///                            i.e. @p t ∉ [start_time(), end_time()].
  /// @throws std::runtime_error if given @p n does not refer to a valid
  ///                            output dimension i.e. @p n ∉ [0, size()).
  T EvaluateNth(const T& t, int n) const {
    ThrowIfOutputIsEmpty(__func__);
    ThrowIfNthElementIsInvalid(__func__, n);
    ThrowIfTimeIsInvalid(__func__, t);
    return this->DoEvaluateNth(t, n);
  }

  /// Returns the output size (i.e. the number of elements in an
  /// output value).
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throws std::logic_error if any of the preconditions is not met.
  int size() const {
    ThrowIfOutputIsEmpty(__func__);
    return this->do_size();
  }

  /// Checks whether the output is empty or not.
  bool is_empty() const { return this->do_is_empty(); }

  /// Returns output's start time, or in other words, the oldest time
  /// `t` that it can be evaluated at e.g. via Evaluate().
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throws std::logic_error if any of the preconditions is not met.
  const T& start_time() const {
    ThrowIfOutputIsEmpty(__func__);
    return this->do_start_time();
  }

  /// Returns output's end time, or in other words, the newest time
  /// `t` that it can be evaluated at e.g. via Evaluate().
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throws std::logic_error if any of the preconditions is not met.
  const T& end_time() const {
    ThrowIfOutputIsEmpty(__func__);
    return this->do_end_time();
  }

 protected:
  DenseOutput() = default;

  // @see Evaluate(const T&)
  virtual VectorX<T> DoEvaluate(const T& t) const = 0;

  // @remarks The computational cost of this method must
  //          be less than or equal to that of indexing
  //          DoEvaluate(const T&) return value.
  // @see Evaluate(const T&, int)
  virtual T DoEvaluateNth(const T& t, int n) const {
    return this->DoEvaluate(t)(n);
  }

  // @see is_empty()
  virtual bool do_is_empty() const = 0;

  // @see size()
  virtual int do_size() const = 0;

  // @see start_time()
  virtual const T& do_start_time() const = 0;

  // @see end_time()
  virtual const T& do_end_time() const = 0;

  // Asserts that this dense output is not empty.
  // @param func_name Call site name for error message clarity (i.e. __func__).
  // @throws std::logic_error if output is empty i.e. is_empty() equals false.
  void ThrowIfOutputIsEmpty(const char* func_name) const {
    if (is_empty()) {
      throw std::logic_error(fmt::format(
          "{}(): Dense output is empty.", func_name));
    }
  }

  // Asserts that the given element index @p n is valid for this dense output.
  // @param func_name Call site name for error message clarity (i.e. __func__).
  // @param n The nth scalar element (0-indexed) to be checked.
  // @throws std::runtime_error if given @p n does not refer to a valid
  //                            output dimension i.e. @p n ∉ [0, size()).
  void ThrowIfNthElementIsInvalid(const char* func_name, int n) const {
    if (n < 0 || this->do_size() <= n) {
      throw std::runtime_error(fmt::format(
          "{}(): Index {} out of dense output [0, {}) range.",
          func_name, n, this->do_size()));
    }
  }

  // Asserts that the given given time @p t is valid for this dense output.
  // @param func_name Call site name for error message clarity (i.e. __func__).
  // @param t Time to be checked.
  // @throws std::runtime_error if given @p t is not within output's domain
  //                            i.e. @p t ∉ [start_time(), end_time()].
  void ThrowIfTimeIsInvalid(const char* func_name, const T& t) const {
    if (t < this->do_start_time() || t > this->do_end_time()) {
      throw std::runtime_error(fmt::format(
          "{}(): Time {} out of dense output [{}, {}] domain.",
          func_name, t, this->do_start_time(), this->do_end_time()));
    }
  }
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::DenseOutput)
