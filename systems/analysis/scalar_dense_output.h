#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {

/// An interface for dense output of scalar ODE solutions.
///
/// See DenseOutput class documentation for further details.
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
class ScalarDenseOutput {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScalarDenseOutput)

  virtual ~ScalarDenseOutput() = default;

  /// Evaluates output at the given time @p t.
  /// @param t Time to evaluate output at.
  /// @return Output scalar value.
  /// @pre Output is not empty i.e. is_empty() is false.
  /// @throw std::logic_error if any of the preconditions is not met.
  /// @throw std::runtime_error if the output is not defined for the
  ///                           given @p t.
  T Evaluate(const T& t) const {
    if (is_empty()) {
      throw std::logic_error("Empty scalar dense output"
                             " cannot be evaluated.");
    }
    if (t < this->do_get_start_time() || t > this->do_get_end_time()) {
      throw std::runtime_error("Scalar dense output is not"
                               " defined for given time.");
    }
    return this->DoEvaluate(t);
  }

  /// Checks whether the output is empty or not.
  virtual bool is_empty() const { return do_is_empty(); }

  /// Returns output's start time, or in other words, the oldest time
  /// `t` that it can be evaluated at e.g. via Evaluate().
  /// @pre Output is not empty i.e. is_empty() equals false.
  /// @throw std::logic_error if any of the preconditions is not met.
  const T& get_start_time() const {
    if (is_empty()) {
      throw std::logic_error("Start time is not defined for"
                             " an empty scalar dense output.");
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
                             " an empty scalar dense output.");
    }
    return this->do_get_end_time();
  }

 protected:
  ScalarDenseOutput() = default;

 private:
  // @see Evaluate(const T&)
  virtual T DoEvaluate(const T& t) const = 0;

  // @see is_empty()
  virtual bool do_is_empty() const = 0;

  // @see get_start_time()
  virtual const T& do_get_start_time() const = 0;

  // @see get_end_time()
  virtual const T& do_get_end_time() const = 0;
};

}  // namespace systems
}  // namespace drake
