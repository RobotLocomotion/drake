#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {

/// An interface for dense output of scalar ODE and DAE solutions.
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
  /// @param t Time to evaluate extension at.
  /// @return Output scalar value.
  /// @pre Output is not empty i.e. is_empty() is false.
  /// @throw std::logic_error if any of the preconditions is not met.
  /// @throw std::runtime_error if the extension is not defined for the
  ///                           given @p t.
  virtual T Evaluate(const T& t) const = 0;

  /// Checks whether the extension is empty or not.
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
  ScalarDenseOutput() = default;
};

}  // namespace systems
}  // namespace drake
